#include <rtems.h>

#define CONFIGURE_INIT

#include <bsp.h> /* for device driver prototypes */

/* configuration information */
rtems_task Init(rtems_task_argument argument); /* forward declaration needed */

#define GRSPW_DEVICE_NAME "/dev/grspw0"

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS 23 // number of tasks concurrently active including INIT
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY 1 // instead of 100
#define CONFIGURE_INIT_TASK_MODE (RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT)
#define CONFIGURE_INIT_TASK_ATTRIBUTES (RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT)
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 6 // [hous] [load] [avgv]
#define CONFIGURE_MAXIMUM_TIMERS 6 // [spiq] [link] [spacewire_reset_link]
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 5
#ifdef PRINT_STACK_REPORT
#define CONFIGURE_STACK_CHECKER_ENABLED
#endif

#include <rtems/confdefs.h>

/* If --drvmgr was enabled during the configuration of the RTEMS kernel */
#ifdef RTEMS_DRVMGR_STARTUP
#ifdef LEON3
/* Add Timer and UART Driver */

#ifdef CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_DRIVER_AMBAPP_GAISLER_GPTIMER
#endif

#ifdef CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_DRIVER_AMBAPP_GAISLER_APBUART
#endif

#endif
#define CONFIGURE_DRIVER_AMBAPP_GAISLER_GRSPW /* GRSPW Driver */
#include <drvmgr/drvmgr_confdefs.h>
#endif

#include "GscMemoryLPP.hpp"

void initCache()
{
    // ASI 2 contains a few control registers that have not been assigned as ancillary state
    // registers. These should only be read and written using 32-bit LDA/STA instructions. All cache
    // registers are accessed through load/store operations to the alternate address space
    // (LDA/STA), using ASI = 2. The table below shows the register addresses:
    //      0x00 Cache control register
    //      0x04 Reserved
    //      0x08 Instruction cache configuration register
    //      0x0C Data cache configuration register

    // Cache Control Register Leon3 / Leon3FT
    // 31..30  29   28  27..24  23  22  21  20..19  18  17  16
    //         RFT  PS  TB      DS  FD  FI  FT          ST  IB
    // 15  14  13..12  11..10  9..8  7..6  5   4   3..2  1..0
    // IP  DP  ITE     IDE     DTE   DDE   DF  IF  DCS   ICS

    unsigned int cacheControlRegister;

    CCR_resetCacheControlRegister();
    ASR16_resetRegisterProtectionControlRegister();

    cacheControlRegister = CCR_getValue();

    CCR_enableInstructionCache(); // ICS bits
    CCR_enableDataCache(); // DCS bits
    CCR_enableInstructionBurstFetch(); // IB  bit

    faultTolerantScheme();

    cacheControlRegister = CCR_getValue();
}

#include "processing/ASM/spectralmatrices.h"

void fill_matrix(float* matrix, int size)
{
    while (size)
    {
        *matrix = (float)size;
        matrix++;
        size--;
    }
}

unsigned int get_timetag_counter()
{
    return *((volatile int*)0x90000008);
}

volatile int duration = 0;
volatile int duration2 = 0;

#define ARG(...) (__VA_ARGS__)

#define BENCH(code, duration)                                                                      \
    {                                                                                              \
        int __computation_begin = get_timetag_counter();                                           \
        for (int i = 0; i < 8; i++)                                                                \
        {                                                                                          \
            code;                                                                                  \
        }                                                                                          \
        int __computation_end = get_timetag_counter();                                             \
        duration = (__computation_end - __computation_begin) / (8);                                \
    }

volatile float input_matrix[25 * 128];
volatile float output_matrix[25 * 128];
volatile float b_trans[3 * 3 * 2 * 128];
volatile float e_trans[2 * 2 * 2 * 128];

rtems_task Init(rtems_task_argument ignored)
{
    (void)ignored;
    /** This is the RTEMS INIT taks, it is the first task launched by the system.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The INIT task create and run all other RTEMS tasks.
     *
     */

    initCache();


    fill_matrix(input_matrix, 25 * 128);
    fill_matrix(output_matrix, 25 * 128);
    fill_matrix(b_trans, 3 * 3 * 2 * 128);
    fill_matrix(e_trans, 2 * 2 * 2 * 128);

    BENCH(ARG(
              {
                  for (int freq = 0; freq < 128; freq++)
                      Matrix_change_of_basis(input_matrix + freq * 25, b_trans + freq * 3 * 3 * 2,
                          e_trans + freq * 2 * 2 * 2, output_matrix + freq * 25);
              }),
        duration);
    BENCH(ARG(
              {
                  for (int freq = 0; freq < 128; freq++)
                      Matrix_change_of_basis_old(input_matrix + freq * 25,
                          b_trans + freq * 3 * 3 * 2, e_trans + freq * 2 * 2 * 2,
                          output_matrix + freq * 25);
              }),
        duration2);
    while (1)
        ;
}

#pragma once
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
#define CONFIGURE_EXTRA_TASK_STACKS              (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY             1 // instead of 100
#define CONFIGURE_INIT_TASK_MODE                 (RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT)
#define CONFIGURE_INIT_TASK_ATTRIBUTES           (RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT)
#define CONFIGURE_MAXIMUM_DRIVERS                16
#define CONFIGURE_MAXIMUM_PERIODS                6 // [hous] [load] [avgv]
#define CONFIGURE_MAXIMUM_TIMERS                 6 // [spiq] [link] [spacewire_reset_link]
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES         5
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
    // ASI 2 contains a few control registers that have not been assigned as
    // ancillary state registers. These should only be read and written using
    // 32-bit LDA/STA instructions. All cache registers are accessed through
    // load/store operations to the alternate address space (LDA/STA), using ASI
    // = 2. The table below shows the register addresses:
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

#define ARG(...) (__VA_ARGS__)

#define BENCH(code, duration, iterations)                                                          \
    {                                                                                              \
        register int __computation_begin = get_timetag_counter();                                  \
        for (volatile int i = 0; i < iterations; i++)                                              \
        {                                                                                          \
            code;                                                                                  \
        }                                                                                          \
        register int __computation_end = get_timetag_counter();                                    \
        duration = (__computation_end - __computation_begin) / (iterations);                       \
    }

#include "processing/ASM/spectralmatrices.h"

void fill_matrix(volatile float* matrix, int size)
{
    while (size)
    {
        *matrix = (float)size;
        matrix++;
        size--;
    }
}

static inline unsigned int get_timetag_counter()
{
#ifndef TSIM
    return *((volatile int*)0x90000008);
#else
    return 0;
#endif
}

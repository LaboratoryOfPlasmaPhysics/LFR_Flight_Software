//*************************
// GPL reminder to be added
//*************************

#include <rtems.h>

/* configuration information */

#define CONFIGURE_INIT

#include <bsp.h> /* for device driver prototypes */

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS             10
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS         (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY	100
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 1

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
    #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GRSPW   /* GRSPW Driver */
    #include <drvmgr/drvmgr_confdefs.h>
#endif

#include <fsw_init.h>
#include <fsw_config.c>

int fdSPW;
int fdUART;

char *link_status(int status);

char *lstates[6] = {"Error-reset",
                    "Error-wait",
                    "Ready",
                    "Started",
                    "Connecting",
                    "Run"
};

rtems_id   Task_id[10];         /* array of task ids */
rtems_name Task_name[10];       /* array of task names */

rtems_task Init( rtems_task_argument ignored )
{
    rtems_status_code status;

    //send_console_outputs_on_serial_port();

    InitLookUpTableForCRC(); // in tc_handler.h

    create_all_tasks();
    start_all_tasks();

    configure_spw_link();
    // configure timer for spectral matrices simulation
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR, CLKDIV_SM_SIMULATOR,
                    IRQ_SPARC_SM, spectral_matrices_isr );
    // configure timer for waveforms simulation
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR, CLKDIV_WF_SIMULATOR,
                    IRQ_SPARC_WF, waveforms_isr );

    LEON_Unmask_interrupt( IRQ_SM );
    LEON_Unmask_interrupt( IRQ_WF );

    status = rtems_task_delete(RTEMS_SELF);
}

rtems_task spw_spiq_task(rtems_task_argument unused)
{
    rtems_event_set event_out;
    struct grspw_regs_str *grspw_regs;
    grspw_regs = (struct grspw_regs_str *) REGS_ADDR_GRSPW;

    while(1){
        PRINTF("In SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        if (rtems_task_suspend(Task_id[1])!=RTEMS_SUCCESSFUL)   // suspend RECV task
            PRINTF("In SPIQ *** Error suspending RECV Task\n")

        configure_spw_link();

        if (rtems_task_restart(Task_id[1], 1)!=RTEMS_SUCCESSFUL) // restart RECV task
            PRINTF("In SPIQ *** Error resume RECV Task\n")
    }
}

int create_all_tasks()
{
    rtems_status_code status;

    Task_name[1] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[3] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[4] = rtems_build_name( 'S', 'M', 'I', 'Q' );
    Task_name[5] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[6] = rtems_build_name( 'A', 'V', 'F', '0' );
    Task_name[7] = rtems_build_name( 'B', 'P', 'F', '0' );
    Task_name[8] = rtems_build_name( 'W', 'F', 'R', 'M' );

    // RECV
    status = rtems_task_create(
        Task_name[1], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[1]
    );
    // SPIQ
    status = rtems_task_create(
        Task_name[3], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[3]
    );
    // SMIQ
    status = rtems_task_create(
        Task_name[4], 10, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[4]
    );
    // STAT
    status = rtems_task_create(
        Task_name[5], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[5]
    );
    // AVF0
    status = rtems_task_create(
        Task_name[6], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES  | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[6]
    );
    // BPF0
    status = rtems_task_create(
        Task_name[7], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[7]
    );
    // WFRM
    status = rtems_task_create(
        Task_name[8], 100, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[8]
    );

    return 0;
}

int start_all_tasks()
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[3], spw_spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_SPIQ\n")

    status = rtems_task_start( Task_id[1], spw_recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_RECV\n")

    //status = rtems_task_start( Task_id[4], spw_bppr_task_rate_monotonic, 1 );
    status = rtems_task_start( Task_id[4], spw_smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPPR\n")

    status = rtems_task_start( Task_id[5], spw_stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_STAT\n")

    status = rtems_task_start( Task_id[6], spw_avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_AVF0\n")

    status = rtems_task_start( Task_id[7], spw_bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPF0\n")

    status = rtems_task_start( Task_id[8], spw_wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_WFRM\n")

    return 0;
}

int configure_spw_link()
{
    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
    if (fdSPW<0) PRINTF("In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, 0) != RTEMS_SUCCESSFUL){
        PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" not started, retry\n")
        close(fdSPW); // close the device
        fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
        if (fdSPW<0) PRINTF("In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
        rtems_task_wake_after(100);
    }

    PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    // sets a few parameters of the link
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RMAPEN, 1);      // sets the RMAP enable bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_RMAPEN\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[3]);    // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 1);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    //status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DESTKEY, CCSDS_DESTINATION_ID);  // sets the destination key
    //if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

char *link_status(int status){
        return lstates[status];
}

rtems_status_code write_spw(spw_ioctl_pkt_send* spw_ioctl_send)
{
    rtems_status_code status;
    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, spw_ioctl_send );
    if (status!=RTEMS_SUCCESSFUL) printf("In write_spw *** Error SPACEWIRE_IOCTRL_SEND\n");
    return status;
}



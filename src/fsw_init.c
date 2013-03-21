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
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 1

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

char *link_status(int status);

char *lstates[6] = {"Error-reset",
                    "Error-wait",
                    "Ready",
                    "Started",
                    "Connecting",
                    "Run"
};

rtems_task Init( rtems_task_argument ignored )
{
    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    //send_console_outputs_on_serial_port();

    initLookUpTableForCRC(); // in tc_handler.h
    init_default_mode_parameters();
    create_message_queue();
    create_all_tasks();
    start_all_tasks();

    configure_spw_link();
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR, CLKDIV_SM_SIMULATOR,
                    IRQ_SPARC_SM, spectral_matrices_isr );
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR, CLKDIV_WF_SIMULATOR,
                    IRQ_SPARC_WF, waveforms_isr );

    // irq handling of the time management unit
    status = rtems_interrupt_catch( commutation_isr1,
                                   IRQ_SPARC_TIME1,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL)
        PRINTF("commutation_isr1 *** rtems_interrupt_catch successfullly configured\n")

    status = rtems_interrupt_catch( commutation_isr2,
                                   IRQ_SPARC_TIME2,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL)
        PRINTF("commutation_isr2 *** rtems_interrupt_catch successfullly configured\n")

    LEON_Unmask_interrupt( IRQ_TIME1 );
    LEON_Unmask_interrupt( IRQ_TIME2 );

    status = rtems_task_delete(RTEMS_SELF);
}

rtems_task spiq_task(rtems_task_argument unused)
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

void init_default_mode_parameters()
{
    // COMMON PARAMETERS
    param_common[0] = 0x00;
    param_common[1] = 0x10;             // default value 0 0 0 1 0 0 0 0
    // NORMAL MODE
    param_norm.sy_lfr_n_swf_l = 2048;   // nb sample
    param_norm.sy_lfr_n_swf_p = 300;    // sec
    param_norm.sy_lfr_n_asm_p = 3600;   // sec
    param_norm.sy_lfr_n_bp_p0 = 4;      // sec
    param_norm.sy_lfr_n_bp_p1 = 20;     // sec
    // BURST MODE
    param_burst.sy_lfr_b_bp_p0 = 1;     // sec
    param_burst.sy_lfr_b_bp_p1 = 5;     // sec
    // SBM1 MODE
    param_sbm1.sy_lfr_s1_bp_p0 = 1;     // sec
    param_sbm1.sy_lfr_s1_bp_p1 = 1;     // sec
    // SBM2 MODE
    param_sbm2.sy_lfr_s2_bp_p0 = 1;     // sec
    param_sbm2.sy_lfr_s2_bp_p0 = 5;     // sec
}

int create_all_tasks()
{
    rtems_status_code status;

    Task_name[1] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[2] = rtems_build_name( 'A', 'C', 'T', 'N' );
    Task_name[3] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[4] = rtems_build_name( 'S', 'M', 'I', 'Q' );
    Task_name[5] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[6] = rtems_build_name( 'A', 'V', 'F', '0' );
    Task_name[7] = rtems_build_name( 'B', 'P', 'F', '0' );
    Task_name[8] = rtems_build_name( 'W', 'F', 'R', 'M' );
    Task_name[9] = rtems_build_name( 'D', 'U', 'M', 'B' );

    // RECV
    status = rtems_task_create(
        Task_name[1], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[1]
    );
    // ACTN
    status = rtems_task_create(
        Task_name[2], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[2]
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
        Task_name[5], 150, RTEMS_MINIMUM_STACK_SIZE * 2,
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
    // DUMB
    status = rtems_task_create(
        Task_name[9], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[9]
    );

    return 0;
}

int start_all_tasks()
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[3], spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_SPIQ\n")

    status = rtems_task_start( Task_id[1], recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_RECV\n")

    status = rtems_task_start( Task_id[2], actn_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_ACTN\n")

    status = rtems_task_start( Task_id[4], smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPPR\n")

    status = rtems_task_start( Task_id[5], stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_STAT\n")

    status = rtems_task_start( Task_id[6], avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_AVF0\n")

    status = rtems_task_start( Task_id[7], bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPF0\n")

    status = rtems_task_start( Task_id[8], wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_WFRM\n")

    status = rtems_task_start( Task_id[9], dumb_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_DUMB\n")

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
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DESTKEY, CCSDS_DESTINATION_ID);  // sets the destination key
    PRINTF1("destination address set to: %d\n", CCSDS_DESTINATION_ID)
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_NODEADDR, CCSDS_NODE_ADDRESS);  // sets the destination key
    PRINTF1("node address set to: %d\n", CCSDS_NODE_ADDRESS)
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")

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



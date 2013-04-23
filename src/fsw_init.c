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

#define CONFIGURE_MAXIMUM_TASKS             15
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS         (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY	100
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 5
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

    PRINTF("\n\n\n\n\n")
    PRINTF("***************************\n")
    PRINTF("** START Flight Software **\n")
    PRINTF("***************************\n")
    PRINTF("\n\n")

    //send_console_outputs_on_serial_port();
    set_apbuart_scaler_reload_register(REGS_ADDR_APBUART, APBUART_SCALER_RELOAD_VALUE);

    initLookUpTableForCRC(); // in tc_handler.h
    init_default_mode_parameters();
    create_message_queue();

    create_names();
    create_all_tasks();
    start_all_tasks();

    grspw_timecode_callback = &timecode_irq_handler;

    configure_spw_link();

    init_waveforms();

    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR, CLKDIV_SM_SIMULATOR,
                    IRQ_SPARC_SM, spectral_matrices_isr );
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR, CLKDIV_WF_SIMULATOR,
                    IRQ_SPARC_WF, waveforms_isr );

    // irq handling of the time management unit
    status = rtems_interrupt_catch( commutation_isr1,
                                   IRQ_SPARC_TIME1,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL)
        PRINTF("OK  *** commutation_isr1 *** rtems_interrupt_catch successfullly configured\n")

    status = rtems_interrupt_catch( commutation_isr2,
                                   IRQ_SPARC_TIME2,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL)
        PRINTF("OK  *** commutation_isr2 *** rtems_interrupt_catch successfullly configured\n")

    LEON_Unmask_interrupt( IRQ_TIME1 );
    LEON_Unmask_interrupt( IRQ_TIME2 );

    status = rtems_task_delete(RTEMS_SELF);
}

rtems_task spiq_task(rtems_task_argument unused)
{
    rtems_event_set event_out;
    struct grspw_regs_str *grspw_regs;
    grspw_regs = (struct grspw_regs_str *) REGS_ADDR_GRSPW;
    rtems_status_code status;

    while(1){
        PRINTF("in SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        if (rtems_task_suspend(Task_id[TASKID_RECV])!=RTEMS_SUCCESSFUL)   // suspend RECV task
            PRINTF("in SPIQ *** Error suspending RECV Task\n")
        if (rtems_task_suspend(Task_id[TASKID_HOUS])!=RTEMS_SUCCESSFUL)   // suspend HOUS task
            PRINTF("in SPIQ *** Error suspending HOUS Task\n")

        configure_spw_link();

        status = rtems_task_restart( Task_id[TASKID_HOUS], 1 );
        if (status!=RTEMS_SUCCESSFUL)
        PRINTF1("in SPIQ *** Error restarting HOUS Task *** code %d\n", status)

        if (rtems_task_restart(Task_id[TASKID_RECV], 1)!=RTEMS_SUCCESSFUL) // restart RECV task
            PRINTF("in SPIQ *** Error restarting RECV Task\n")
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

int create_names()
{
    // task names
    Task_name[TASKID_RECV] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[TASKID_ACTN] = rtems_build_name( 'A', 'C', 'T', 'N' );
    Task_name[TASKID_SPIQ] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[TASKID_SMIQ] = rtems_build_name( 'S', 'M', 'I', 'Q' );
    Task_name[TASKID_STAT] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[TASKID_AVF0] = rtems_build_name( 'A', 'V', 'F', '0' );
    Task_name[TASKID_BPF0] = rtems_build_name( 'B', 'P', 'F', '0' );
    Task_name[TASKID_WFRM] = rtems_build_name( 'W', 'F', 'R', 'M' );
    Task_name[TASKID_DUMB] = rtems_build_name( 'D', 'U', 'M', 'B' );
    Task_name[TASKID_HOUS] = rtems_build_name( 'H', 'O', 'U', 'S' );

    // rate monotonic period name
    HK_name = rtems_build_name( 'H', 'O', 'U', 'S' );

    return 0;
}

int create_all_tasks()
{
    rtems_status_code status;

    // RECV
    status = rtems_task_create(
        Task_name[TASKID_RECV], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_RECV]
    );
    // ACTN
    status = rtems_task_create(
        Task_name[TASKID_ACTN], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_ACTN]
    );
    // SPIQ
    status = rtems_task_create(
        Task_name[TASKID_SPIQ], 5, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SPIQ]
    );
    // SMIQ
    status = rtems_task_create(
        Task_name[TASKID_SMIQ], 10, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SMIQ]
    );
    // STAT
    status = rtems_task_create(
        Task_name[TASKID_STAT], 150, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_STAT]
    );
    // AVF0
    status = rtems_task_create(
        Task_name[TASKID_AVF0], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES  | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVF0]
    );
    // BPF0
    status = rtems_task_create(
        Task_name[TASKID_BPF0], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_BPF0]
    );
    // WFRM
    status = rtems_task_create(
        Task_name[TASKID_WFRM], 100, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_WFRM]
    );
    // DUMB
    status = rtems_task_create(
        Task_name[TASKID_DUMB], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_DUMB]
    );
    // HOUS
    status = rtems_task_create(
        Task_name[TASKID_HOUS], 199, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_HOUS]
    );

    return 0;
}

int start_all_tasks()
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[TASKID_SPIQ], spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_SPIQ\n")

    status = rtems_task_start( Task_id[TASKID_RECV], recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_RECV\n")

    status = rtems_task_start( Task_id[TASKID_ACTN], actn_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_ACTN\n")

    status = rtems_task_start( Task_id[TASKID_SMIQ], smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPPR\n")

    status = rtems_task_start( Task_id[TASKID_STAT], stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_STAT\n")

    status = rtems_task_start( Task_id[TASKID_AVF0], avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_AVF0\n")

    status = rtems_task_start( Task_id[TASKID_BPF0], bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPF0\n")

    status = rtems_task_start( Task_id[TASKID_WFRM], wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_WFRM\n")

    status = rtems_task_start( Task_id[TASKID_DUMB], dumb_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_DUMB\n")

    status = rtems_task_start( Task_id[TASKID_HOUS], hous_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_HOUS\n")

    return 0;
}

int configure_spw_link()
{
    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    PRINTF("OK  *** in configure_spw_link *** try to open "GRSPW_DEVICE_NAME"\n")
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
    if (fdSPW<0) PRINTF("ERR *** in configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, 0) != RTEMS_SUCCESSFUL){
        PRINTF(".")
        fflush(stdout);
        close(fdSPW); // close the device
        fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
        if (fdSPW<0) PRINTF("ERR *** In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
        rtems_task_wake_after(100);
    }

    PRINTF("OK  *** In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    configure_spacewire_set_NP(1, REGS_ADDR_GRSPW); // No Port force
    configure_spacewire_set_RE(1, REGS_ADDR_GRSPW); // the dedicated call seems to  break the no port force configuration

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 1);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909);
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")

    PRINTF("OK  *** in configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

void configure_spacewire_set_NP(unsigned char val, unsigned int regAddr) // No Port force
{
    unsigned int *spwptr;
    spwptr = (unsigned int*) regAddr;
    if (val == 1)
    {
        *spwptr = *spwptr | 0x00100000; // [NP] set the No port force bit
    }
    if (val== 0)
    {
        *spwptr = *spwptr & 0xffdfffff;
    }
}

void configure_spacewire_set_RE(unsigned char val, unsigned int regAddr) // RMAP Enable
{
    unsigned int *spwptr;
    spwptr = (unsigned int*) regAddr;
    if (val == 1)
    {
        *spwptr = *spwptr | 0x00010000; // [NP] set the No port force bit
    }
    if (val== 0)
    {
        *spwptr = *spwptr & 0xfffdffff;
    }
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

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc)
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        printf("In timecode_irq_handler *** Error sending event to DUMB\n");
}


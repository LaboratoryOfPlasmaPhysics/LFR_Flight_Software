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

#define CONFIGURE_MAXIMUM_TASKS 15
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY 100
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

    //send_console_outputs_on_apbuart_port();
    set_apbuart_scaler_reload_register(REGS_ADDR_APBUART, APBUART_SCALER_RELOAD_VALUE);

    initLookUpTableForCRC(); // in tc_handler.h
    init_parameter_dump();
    init_local_mode_parameters();
    init_housekeeping_parameters();
    create_message_queue();

    create_names();         // create all names
    create_all_tasks();     // create all tasks
    start_all_tasks();      // start all tasks
    stop_current_mode();    // go in STANDBY mode

    grspw_timecode_callback = &timecode_irq_handler;

    spacewire_configure_link();

    //****************************
    // Spectral Matrices simulator
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR, CLKDIV_SM_SIMULATOR,
                    IRQ_SPARC_SM, spectral_matrices_isr );

    //**********
    // WAVEFORMS
    // simulator

#ifdef GSA
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR, CLKDIV_WF_SIMULATOR,
                    IRQ_SPARC_WF, waveforms_simulator_isr );
#else
    // configure the registers of the waveform picker
    reset_wfp_regs();
    // configure the waveform picker interrupt service routine
    status = rtems_interrupt_catch( waveforms_isr,
                                   IRQ_SPARC_WAVEFORM_PICKER,
                                   &old_isr_handler) ;
    LEON_Mask_interrupt( IRQ_WAVEFORM_PICKER );
#endif

    //**********

    //*****************************************
    // irq handling of the time management unit
    status = rtems_interrupt_catch( commutation_isr1,
                                   IRQ_SPARC_TIME1,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL) {
        PRINTF("OK  *** commutation_isr1 *** rtems_interrupt_catch successfullly configured\n")
    }

    status = rtems_interrupt_catch( commutation_isr2,
                                   IRQ_SPARC_TIME2,
                                   &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status==RTEMS_SUCCESSFUL) {
        PRINTF("OK  *** commutation_isr2 *** rtems_interrupt_catch successfullly configured\n")
    }

    LEON_Unmask_interrupt( IRQ_TIME1 );
    LEON_Unmask_interrupt( IRQ_TIME2 );

#ifdef GSA
    //if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
    //    printf("in INIT *** Error sending event to WFRM\n");
    //}
#endif

    status = rtems_task_delete(RTEMS_SELF);

}

rtems_task spiq_task(rtems_task_argument unused)
{
    rtems_event_set event_out;
    rtems_status_code status;
    unsigned char lfrMode;

    while(true){
        PRINTF("in SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4; // get the current mode

        status = spacewire_wait_for_link();

        if (status != RTEMS_SUCCESSFUL)
        {
            //****************
            // STOP THE SYSTEM
            spacewire_compute_stats_offsets();
            stop_current_mode();
            if (rtems_task_suspend(Task_id[TASKID_RECV])!=RTEMS_SUCCESSFUL) {   // suspend RECV task
                PRINTF("in SPIQ *** Error suspending RECV Task\n")
            }
            if (rtems_task_suspend(Task_id[TASKID_HOUS])!=RTEMS_SUCCESSFUL) {   // suspend HOUS task
                PRINTF("in SPIQ *** Error suspending HOUS Task\n")
            }

            //***************************
            // RESTART THE SPACEWIRE LINK
            spacewire_configure_link();

            //*******************
            // RESTART THE SYSTEM
            //ioctl(fdSPW, SPACEWIRE_IOCTRL_CLR_STATISTICS);   // clear statistics
            status = rtems_task_restart( Task_id[TASKID_HOUS], 1 );
            if (status != RTEMS_SUCCESSFUL) {
                PRINTF1("in SPIQ *** Error restarting HOUS Task *** code %d\n", status)
            }
            if (rtems_task_restart(Task_id[TASKID_RECV], 1) != RTEMS_SUCCESSFUL) {    // restart RECV task
                PRINTF("in SPIQ *** Error restarting RECV Task\n")
            }
            //enter_mode(lfrMode, NULL); // enter the mode that was running before the SpaceWire interruption
        }
    }
}

void init_parameter_dump(void)
{
    parameter_dump_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    parameter_dump_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    parameter_dump_packet.reserved = CCSDS_RESERVED;
    parameter_dump_packet.userApplication = CCSDS_USER_APP;
    parameter_dump_packet.packetID[0] = (unsigned char) (TM_PACKET_ID_PARAMETER_DUMP >> 8);
    parameter_dump_packet.packetID[1] = (unsigned char) TM_PACKET_ID_PARAMETER_DUMP;
    parameter_dump_packet.packetSequenceControl[0] = (unsigned char) (TM_PACKET_SEQ_CTRL_STANDALONE << 6);
    parameter_dump_packet.packetSequenceControl[1] = 0x00;
    parameter_dump_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_PARAMETER_DUMP >> 8);
    parameter_dump_packet.packetLength[1] = (unsigned char) PACKET_LENGTH_PARAMETER_DUMP;
    // DATA FIELD HEADER
    parameter_dump_packet.spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    parameter_dump_packet.serviceType = TM_TYPE_PARAMETER_DUMP;
    parameter_dump_packet.serviceSubType = TM_SUBTYPE_PARAMETER_DUMP;
    parameter_dump_packet.destinationID = TM_DESTINATION_ID_GROUND;
    parameter_dump_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    parameter_dump_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    parameter_dump_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    parameter_dump_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
    parameter_dump_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    parameter_dump_packet.time[5] = (unsigned char) (time_management_regs->fine_time);
    parameter_dump_packet.sid = SID_PARAMETER_DUMP;

    //******************
    // COMMON PARAMETERS
    parameter_dump_packet.unused0 = DEFAULT_SY_LFR_COMMON0;
    parameter_dump_packet.bw_sp0_sp1_r0_r1 = DEFAULT_SY_LFR_COMMON1;

    //******************
    // NORMAL PARAMETERS
    parameter_dump_packet.sy_lfr_n_swf_l[0] = (unsigned char) (DEFAULT_SY_LFR_N_SWF_L >> 8);
    parameter_dump_packet.sy_lfr_n_swf_l[1] = (unsigned char) DEFAULT_SY_LFR_N_SWF_L;
    parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char) (DEFAULT_SY_LFR_N_SWF_P >> 8);
    parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char) DEFAULT_SY_LFR_N_SWF_P;
    parameter_dump_packet.sy_lfr_n_asm_p[0] = (unsigned char) (DEFAULT_SY_LFR_N_ASM_P >> 8);
    parameter_dump_packet.sy_lfr_n_asm_p[1] = (unsigned char) DEFAULT_SY_LFR_N_ASM_P;
    parameter_dump_packet.sy_lfr_n_bp_p0 = (unsigned char) DEFAULT_SY_LFR_N_BP_P0;
    parameter_dump_packet.sy_lfr_n_bp_p1 = (unsigned char) DEFAULT_SY_LFR_N_BP_P1;

    //*****************
    // BURST PARAMETERS
    parameter_dump_packet.sy_lfr_b_bp_p0 = (unsigned char) DEFAULT_SY_LFR_B_BP_P0;
    parameter_dump_packet.sy_lfr_b_bp_p1 = (unsigned char) DEFAULT_SY_LFR_B_BP_P1;

    //****************
    // SBM1 PARAMETERS
    parameter_dump_packet.sy_lfr_s1_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P0;
    parameter_dump_packet.sy_lfr_s1_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P0;

    //****************
    // SBM2 PARAMETERS
    parameter_dump_packet.sy_lfr_s2_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P0;
    parameter_dump_packet.sy_lfr_s2_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P0;
}

void init_local_mode_parameters(void)
{
    // LOCAL PARAMETERS
    // (2 snapshots of 2048 points per seconds) * (period of the NORM snashots)
    param_local.local_sbm1_nb_cwf_max = 2 * (
                parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
                + parameter_dump_packet.sy_lfr_n_swf_p[1]
             );
    // (period of the NORM snashots) / (8 seconds per snapshot at f2 = 256 Hz)
    param_local.local_sbm2_nb_cwf_max = (
                parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
                + parameter_dump_packet.sy_lfr_n_swf_p[1]
             )/ 8;

    PRINTF1("local_sbm1_nb_cwf_max %d \n", param_local.local_sbm1_nb_cwf_max)
    PRINTF1("local_sbm2_nb_cwf_max %d \n", param_local.local_sbm2_nb_cwf_max)

    param_local.local_sbm1_nb_cwf_sent = 0;
    param_local.local_sbm2_nb_cwf_sent = 0;
}

void init_housekeeping_parameters(void)
{
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int k = 0;
    char *parameters;

    parameters = (char*) &housekeeping_packet.lfr_status_word;
    for(i = 0; i< SIZE_HK_PARAMETERS; i++)
    {
        parameters[i] = 0x00;
    }
    // init status word
    housekeeping_packet.lfr_status_word[0] = 0x00;
    housekeeping_packet.lfr_status_word[1] = 0x00;
    // init software version
    housekeeping_packet.lfr_sw_version[0] = SW_VERSION_N1;
    housekeeping_packet.lfr_sw_version[1] = SW_VERSION_N2;
    housekeeping_packet.lfr_sw_version[2] = SW_VERSION_N3;
    housekeeping_packet.lfr_sw_version[3] = SW_VERSION_N4;
    // init sequence counters
    for (i = 0; i<SEQ_CNT_NB_PID; i++)
    {
        for(j = 0; j<SEQ_CNT_NB_CAT; j++)
        {
            for(k = 0; k<SEQ_CNT_NB_DEST_ID; k++)
            {
                sequenceCounters[i][j][k] = 0x00;
            }
        }
    }
}

int create_names( void )
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

int create_all_tasks( void )
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
        Task_name[TASKID_ACTN], 100, RTEMS_MINIMUM_STACK_SIZE * 2,
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
        Task_name[TASKID_WFRM], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
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

int start_all_tasks( void )
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[TASKID_SPIQ], spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_SPIQ\n")
    }

    status = rtems_task_start( Task_id[TASKID_RECV], recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_RECV\n")
    }

    status = rtems_task_start( Task_id[TASKID_ACTN], actn_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_ACTN\n")
    }

    status = rtems_task_start( Task_id[TASKID_SMIQ], smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_BPPR\n")
    }

    status = rtems_task_start( Task_id[TASKID_STAT], stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_STAT\n")
    }

    status = rtems_task_start( Task_id[TASKID_AVF0], avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_AVF0\n")
    }

    status = rtems_task_start( Task_id[TASKID_BPF0], bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_BPF0\n")
    }

    status = rtems_task_start( Task_id[TASKID_WFRM], wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_WFRM\n")
    }

    status = rtems_task_start( Task_id[TASKID_DUMB], dumb_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_DUMB\n")
    }

    status = rtems_task_start( Task_id[TASKID_HOUS], hous_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("In INIT *** Error starting TASK_HOUS\n")
    }

    return 0;
}

int spacewire_configure_link( void )
{
    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    PRINTF("OK  *** in configure_spw_link *** try to open "GRSPW_DEVICE_NAME"\n")
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
    if ( fdSPW<0 ) {
        PRINTF("ERR *** in configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
    }

    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, -1) != RTEMS_SUCCESSFUL){
        PRINTF(".")
        fflush( stdout );
        close( fdSPW ); // close the device
        fdSPW = open( GRSPW_DEVICE_NAME, O_RDWR ); // open the device. the open call reset the hardware
        if (fdSPW<0) {
            PRINTF("ERR *** In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
        }
        rtems_task_wake_after(100);
    }

    PRINTF("OK  *** In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    spacewire_set_NP(1, REGS_ADDR_GRSPW); // No Port force
    spacewire_set_RE(1, REGS_ADDR_GRSPW); // the dedicated call seems to  break the no port force configuration

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    //status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    //if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 0);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 0);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK, 0);             // transmission blocks
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 0);      // transmission blocks on full
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909);
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")

    PRINTF("OK  *** in configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

int spacewire_wait_for_link(void)
{
    unsigned int i;
    int linkStatus;
    rtems_status_code status = RTEMS_UNSATISFIED;

    for(i = 0; i< 10; i++){
        PRINTF(".")
        fflush( stdout );
        ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);   // get the link status
        PRINTF1("in spacewire_wait_for_link *** link status is: %s\n", lstates[linkStatus])
        if ( linkStatus == 5) {
            PRINTF("in spacewire_wait_for_link *** link is running\n")
            status = RTEMS_SUCCESSFUL;
            break;
        }
        rtems_task_wake_after(100);
    }

    return status;
}

void spacewire_set_NP(unsigned char val, unsigned int regAddr) // [N]o [P]ort force
{
    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1) {
        *spwptr = *spwptr | 0x00100000; // [NP] set the No port force bit
    }
    if (val== 0) {
        *spwptr = *spwptr & 0xffdfffff;
    }
}

void spacewire_set_RE(unsigned char val, unsigned int regAddr) // [R]MAP [E]nable
{
    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1)
    {
        *spwptr = *spwptr | 0x00010000; // [RE] set the RMAP Enable bit
    }
    if (val== 0)
    {
        *spwptr = *spwptr & 0xfffdffff;
    }
}

void spacewire_compute_stats_offsets()
{
    spw_stats spacewire_stats_grspw;
    rtems_status_code status;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &spacewire_stats_grspw );

    spacewire_stats_backup.packets_received = spacewire_stats_grspw.packets_received
            + spacewire_stats.packets_received;
    spacewire_stats_backup.packets_sent = spacewire_stats_grspw.packets_sent
            + spacewire_stats.packets_sent;
    spacewire_stats_backup.parity_err = spacewire_stats_grspw.parity_err
            + spacewire_stats.parity_err;
    spacewire_stats_backup.disconnect_err = spacewire_stats_grspw.disconnect_err
            + spacewire_stats.disconnect_err;
    spacewire_stats_backup.escape_err = spacewire_stats_grspw.escape_err
            + spacewire_stats.escape_err;
    spacewire_stats_backup.credit_err = spacewire_stats_grspw.credit_err
            + spacewire_stats.credit_err;
    spacewire_stats_backup.write_sync_err = spacewire_stats_grspw.write_sync_err
            + spacewire_stats.write_sync_err;
    spacewire_stats_backup.rx_rmap_header_crc_err = spacewire_stats_grspw.rx_rmap_header_crc_err
            + spacewire_stats.rx_rmap_header_crc_err;
    spacewire_stats_backup.rx_rmap_data_crc_err = spacewire_stats_grspw.rx_rmap_data_crc_err
            + spacewire_stats.rx_rmap_data_crc_err;
    spacewire_stats_backup.early_ep = spacewire_stats_grspw.early_ep
            + spacewire_stats.early_ep;
    spacewire_stats_backup.invalid_address = spacewire_stats_grspw.invalid_address
            + spacewire_stats.invalid_address;
    spacewire_stats_backup.rx_eep_err = spacewire_stats_grspw.rx_eep_err
            + spacewire_stats.rx_eep_err;
    spacewire_stats_backup.rx_truncated = spacewire_stats_grspw.rx_truncated
            + spacewire_stats.rx_truncated;
}

rtems_status_code write_spw(spw_ioctl_pkt_send* spw_ioctl_send)
{
    rtems_status_code status;
    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, spw_ioctl_send );
    if (status != RTEMS_SUCCESSFUL){
        //PRINTF1("ERR *** in write_spw *** write operation failed with code: %d\n", status)
    }
    return status;
}

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc)
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_1 ) != RTEMS_SUCCESSFUL) {
        printf("In timecode_irq_handler *** Error sending event to DUMB\n");
    }
}

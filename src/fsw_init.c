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

#define CONFIGURE_MAXIMUM_TASKS 20
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY 5 // instead of 100
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 5
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 2

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

rtems_task Init( rtems_task_argument ignored )
{
    rtems_status_code status;
    rtems_isr_entry  old_isr_handler;

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
    create_message_queues();

    create_names();         // create all names
    create_all_tasks();     // create all tasks

    start_all_tasks();      // start all tasks
    stop_current_mode();    // go in STANDBY mode

    grspw_timecode_callback = &timecode_irq_handler;

    spacewire_configure_link();

#ifdef GSA
    // Spectral Matrices simulator
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR, CLKDIV_SM_SIMULATOR,
                    IRQ_SPARC_SM, spectral_matrices_isr );
    // WaveForms
    configure_timer((gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR, CLKDIV_WF_SIMULATOR,
                    IRQ_SPARC_WF, waveforms_simulator_isr );
    LEON_Mask_interrupt( IRQ_SM );
    LEON_Mask_interrupt( IRQ_WF );
#else
    // reset configuration registers
    reset_waveform_picker_regs();
    reset_spectral_matrix_regs();
    // configure IRQ handling for the waveform picker unit
    status = rtems_interrupt_catch( waveforms_isr,
                                   IRQ_SPARC_WAVEFORM_PICKER,
                                   &old_isr_handler) ;
    // configure IRQ handling for the spectral matrix unit
    status = rtems_interrupt_catch( spectral_matrices_isr,
                                   IRQ_SPARC_SPECTRAL_MATRIX,
                                   &old_isr_handler) ;
    // mask IRQ lines
    LEON_Mask_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Mask_interrupt( IRQ_SPECTRAL_MATRIX );
#endif

    status = rtems_task_delete(RTEMS_SELF);

}

void init_parameter_dump(void)
{
    parameter_dump_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    parameter_dump_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    parameter_dump_packet.reserved = CCSDS_RESERVED;
    parameter_dump_packet.userApplication = CCSDS_USER_APP;
    parameter_dump_packet.packetID[0] = (unsigned char) (TM_PACKET_ID_PARAMETER_DUMP >> 8);
    parameter_dump_packet.packetID[1] = (unsigned char) TM_PACKET_ID_PARAMETER_DUMP;
    parameter_dump_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    parameter_dump_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
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
    parameter_dump_packet.sy_lfr_n_swf_l[1] = (unsigned char) (DEFAULT_SY_LFR_N_SWF_L     );
    parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char) (DEFAULT_SY_LFR_N_SWF_P >> 8);
    parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char) (DEFAULT_SY_LFR_N_SWF_P     );
    parameter_dump_packet.sy_lfr_n_asm_p[0] = (unsigned char) (DEFAULT_SY_LFR_N_ASM_P >> 8);
    parameter_dump_packet.sy_lfr_n_asm_p[1] = (unsigned char) (DEFAULT_SY_LFR_N_ASM_P     );
    parameter_dump_packet.sy_lfr_n_bp_p0 = (unsigned char) DEFAULT_SY_LFR_N_BP_P0;
    parameter_dump_packet.sy_lfr_n_bp_p1 = (unsigned char) DEFAULT_SY_LFR_N_BP_P1;

    //*****************
    // BURST PARAMETERS
    parameter_dump_packet.sy_lfr_b_bp_p0 = (unsigned char) DEFAULT_SY_LFR_B_BP_P0;
    parameter_dump_packet.sy_lfr_b_bp_p1 = (unsigned char) DEFAULT_SY_LFR_B_BP_P1;

    //****************
    // SBM1 PARAMETERS
    parameter_dump_packet.sy_lfr_s1_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P0; // min value is 0.25 s for the period
    parameter_dump_packet.sy_lfr_s1_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P1;

    //****************
    // SBM2 PARAMETERS
    parameter_dump_packet.sy_lfr_s2_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P0;
    parameter_dump_packet.sy_lfr_s2_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P1;
}

void init_local_mode_parameters(void)
{
    // LOCAL PARAMETERS
    set_local_sbm1_nb_cwf_max();
    set_local_sbm2_nb_cwf_max();
    set_local_nb_interrupt_f0_MAX();

    PRINTF1("local_sbm1_nb_cwf_max %d \n", param_local.local_sbm1_nb_cwf_max)
    PRINTF1("local_sbm2_nb_cwf_max %d \n", param_local.local_sbm2_nb_cwf_max)
    PRINTF1("nb_interrupt_f0_MAX = %d\n", param_local.local_nb_interrupt_f0_MAX)

    reset_local_sbm1_nb_cwf_sent();
    reset_local_sbm2_nb_cwf_sent();
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
    updateLFRCurrentMode();
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
    Task_name[TASKID_MATR] = rtems_build_name( 'M', 'A', 'T', 'R' );
    Task_name[TASKID_CWF3] = rtems_build_name( 'C', 'W', 'F', '3' );
    Task_name[TASKID_CWF2] = rtems_build_name( 'C', 'W', 'F', '2' );
    Task_name[TASKID_CWF1] = rtems_build_name( 'C', 'W', 'F', '1' );
    Task_name[TASKID_SEND] = rtems_build_name( 'S', 'E', 'N', 'D' );

    // rate monotonic period name
    HK_name = rtems_build_name( 'H', 'O', 'U', 'S' );

    return 0;
}

int create_all_tasks( void )
{
    rtems_status_code status;

    // RECV
    status = rtems_task_create(
        Task_name[TASKID_RECV], TASK_PRIORITY_RECV, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_RECV]
    );
    // ACTN
    status = rtems_task_create(
        Task_name[TASKID_ACTN], TASK_PRIORITY_ACTN, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_ACTN]
    );
    // SPIQ
    status = rtems_task_create(
        Task_name[TASKID_SPIQ], TASK_PRIORITY_SPIQ, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SPIQ]
    );
    // SMIQ
    status = rtems_task_create(
        Task_name[TASKID_SMIQ], TASK_PRIORITY_SMIQ, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SMIQ]
    );
    // STAT
    status = rtems_task_create(
        Task_name[TASKID_STAT], TASK_PRIORITY_STAT, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_STAT]
    );
    // AVF0
    status = rtems_task_create(
        Task_name[TASKID_AVF0], TASK_PRIORITY_AVF0, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES  | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVF0]
    );
    // BPF0
    status = rtems_task_create(
        Task_name[TASKID_BPF0], TASK_PRIORITY_BPF0, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_BPF0]
    );
    // WFRM
    status = rtems_task_create(
        Task_name[TASKID_WFRM], TASK_PRIORITY_WFRM, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_WFRM]
    );
    // DUMB
    status = rtems_task_create(
        Task_name[TASKID_DUMB], TASK_PRIORITY_DUMB, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_DUMB]
    );
    // HOUS
    status = rtems_task_create(
        Task_name[TASKID_HOUS], TASK_PRIORITY_HOUS, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_HOUS]
    );
    // MATR
    status = rtems_task_create(
        Task_name[TASKID_MATR], TASK_PRIORITY_MATR, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_MATR]
    );
    // CWF3
    status = rtems_task_create(
        Task_name[TASKID_CWF3], TASK_PRIORITY_CWF3, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_CWF3]
    );
    // CWF2
    status = rtems_task_create(
        Task_name[TASKID_CWF2], TASK_PRIORITY_CWF2, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_CWF2]
    );
    // CWF1
    status = rtems_task_create(
        Task_name[TASKID_CWF1], TASK_PRIORITY_CWF1, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_CWF1]
    );
    // SEND
    status = rtems_task_create(
        Task_name[TASKID_SEND], TASK_PRIORITY_SEND, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SEND]
    );

    return 0;
}

int start_all_tasks( void )
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[TASKID_SPIQ], spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_SPIQ\n")
    }

    status = rtems_task_start( Task_id[TASKID_RECV], recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_RECV\n")
    }

    status = rtems_task_start( Task_id[TASKID_ACTN], actn_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_ACTN\n")
    }

    status = rtems_task_start( Task_id[TASKID_SMIQ], smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_BPPR\n")
    }

    status = rtems_task_start( Task_id[TASKID_STAT], stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_STAT\n")
    }

    status = rtems_task_start( Task_id[TASKID_AVF0], avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_AVF0\n")
    }

    status = rtems_task_start( Task_id[TASKID_BPF0], bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_BPF0\n")
    }

    status = rtems_task_start( Task_id[TASKID_WFRM], wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_WFRM\n")
    }

    status = rtems_task_start( Task_id[TASKID_DUMB], dumb_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_DUMB\n")
    }

    status = rtems_task_start( Task_id[TASKID_HOUS], hous_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_HOUS\n")
    }

    status = rtems_task_start( Task_id[TASKID_MATR], matr_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_MATR\n")
    }

    status = rtems_task_start( Task_id[TASKID_CWF3], cwf3_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_CWF3\n")
    }

    status = rtems_task_start( Task_id[TASKID_CWF2], cwf2_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_CWF2\n")
    }

    status = rtems_task_start( Task_id[TASKID_CWF1], cwf1_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_CWF1\n")
    }
    status = rtems_task_start( Task_id[TASKID_SEND], send_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in INIT *** Error starting TASK_SEND\n")
    }

    return 0;
}

int create_message_queues( void )
{
    rtems_status_code status;

    misc_name[0] = rtems_build_name( 'Q', 'U', 'E', 'U' );
    misc_name[1] = rtems_build_name( 'P', 'K', 'T', 'S' );

    status = rtems_message_queue_create( misc_name[0], ACTION_MSG_QUEUE_COUNT, CCSDS_TC_PKT_MAX_SIZE,
                                                 RTEMS_FIFO | RTEMS_LOCAL, &misc_id[0] );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in create_message_queues *** error creating QUEU\n")
    }

    status = rtems_message_queue_create( misc_name[1], ACTION_MSG_PKTS_COUNT, sizeof(spw_ioctl_pkt_send),
                                                 RTEMS_FIFO | RTEMS_LOCAL, &misc_id[1] );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in create_message_queues *** error creating PKTS\n")
    }

    return 0;
}

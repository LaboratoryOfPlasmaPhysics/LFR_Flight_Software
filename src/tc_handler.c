/** Functions and tasks related to TeleCommand handling.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle TeleCommands:\n
 * action launching\n
 * TC parsing\n
 * ...
 *
 */

#include "tc_handler.h"

//***********
// RTEMS TASK

rtems_task actn_task( rtems_task_argument unused )
{
    /** This RTEMS task is responsible for launching actions upton the reception of valid TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The ACTN task waits for data coming from an RTEMS msesage queue. When data arrives, it launches specific actions depending
     * on the incoming TeleCommand.
     *
     */

    int result;
    rtems_status_code status;       // RTEMS status code
    ccsdsTelecommandPacket_t TC;    // TC sent to the ACTN task
    size_t size;                    // size of the incoming TC packet
    unsigned char subtype;          // subtype of the current TC packet
    unsigned char time[6];
    rtems_id queue_rcv_id;
    rtems_id queue_snd_id;

    status =  get_message_queue_id_recv( &queue_rcv_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR get_message_queue_id_recv %d\n", status)
    }

    status =  get_message_queue_id_send( &queue_snd_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR get_message_queue_id_send %d\n", status)
    }

    result = LFR_SUCCESSFUL;
    subtype = 0;          // subtype of the current TC packet

    BOOT_PRINTF("in ACTN *** \n")

    while(1)
    {
        status = rtems_message_queue_receive( queue_rcv_id, (char*) &TC, &size,
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        getTime( time );    // set time to the current time
        if (status!=RTEMS_SUCCESSFUL)
        {
            PRINTF1("ERR *** in task ACTN *** error receiving a message, code %d \n", status)
        }
        else
        {
            subtype = TC.serviceSubType;
            switch(subtype)
            {
                case TC_SUBTYPE_RESET:
                    result = action_reset( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_COMM:
                    result = action_load_common_par( &TC );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_NORM:
                    result = action_load_normal_par( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_BURST:
                    result = action_load_burst_par( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM1:
                    result = action_load_sbm1_par( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM2:
                    result = action_load_sbm2_par( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_DUMP:
                    result = action_dump_par( queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_ENTER:
                    result = action_enter_mode( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_UPDT_INFO:
                    result = action_update_info( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_EN_CAL:
                    result = action_enable_calibration( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_DIS_CAL:
                    result = action_disable_calibration( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_UPDT_TIME:
                    result = action_update_time( &TC );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                default:
                    break;
            }
        }
    }
}

//***********
// TC ACTIONS

int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function executes specific actions when a TC_LFR_RESET TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    send_tm_lfr_tc_exe_not_implemented( TC, queue_id, time );
    return LFR_DEFAULT;
}

int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    /** This function executes specific actions when a TC_LFR_ENTER_MODE TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    rtems_status_code status;
    unsigned char requestedMode;
    unsigned int *transitionCoarseTime_ptr;
    unsigned int transitionCoarseTime;
    unsigned char * bytePosPtr;

    bytePosPtr = (unsigned char *) &TC->packetID;

    requestedMode = bytePosPtr[ BYTE_POS_CP_MODE_LFR_SET ];
    transitionCoarseTime_ptr = (unsigned int *) ( &bytePosPtr[ BYTE_POS_CP_LFR_ENTER_MODE_TIME ] );
    transitionCoarseTime = (*transitionCoarseTime_ptr) & 0x7fffffff;

    status = check_mode_value( requestedMode );

    if ( status != LFR_SUCCESSFUL )     // the mode value is inconsistent
    {
        send_tm_lfr_tc_exe_inconsistent( TC, queue_id, BYTE_POS_CP_MODE_LFR_SET, requestedMode );
    }
    else                                // the mode value is consistent, check the transition
    {
        status = check_mode_transition(requestedMode);
        if (status != LFR_SUCCESSFUL)
        {
            PRINTF("ERR *** in action_enter_mode *** check_mode_transition\n")
            send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        }
    }

    if ( status == LFR_SUCCESSFUL )     // the transition is valid, enter the mode
    {
        status = check_transition_date( transitionCoarseTime );
        if (status != LFR_SUCCESSFUL)
        {
            PRINTF("ERR *** in action_enter_mode *** check_transition_date\n")
            send_tm_lfr_tc_exe_inconsistent( TC, queue_id,
                                             BYTE_POS_CP_LFR_ENTER_MODE_TIME,
                                             bytePosPtr[ BYTE_POS_CP_LFR_ENTER_MODE_TIME + 3 ] );
        }
    }

    if ( status == LFR_SUCCESSFUL )     // the date is valid, enter the mode
    {
        PRINTF1("OK  *** in action_enter_mode *** enter mode %d\n", requestedMode);
        status = enter_mode( requestedMode, transitionCoarseTime );
    }

    return status;
}

int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_INFO TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     * @return LFR directive status code:
     * - LFR_DEFAULT
     * - LFR_SUCCESSFUL
     *
     */

    unsigned int val;
    int result;
    unsigned int status;
    unsigned char mode;
    unsigned char * bytePosPtr;

    bytePosPtr = (unsigned char *) &TC->packetID;

    // check LFR mode
    mode = (bytePosPtr[ BYTE_POS_UPDATE_INFO_PARAMETERS_SET5 ] & 0x1e) >> 1;
    status = check_update_info_hk_lfr_mode( mode );
    if (status == LFR_SUCCESSFUL)  // check TDS mode
    {
        mode = (bytePosPtr[ BYTE_POS_UPDATE_INFO_PARAMETERS_SET6 ] & 0xf0) >> 4;
        status = check_update_info_hk_tds_mode( mode );
    }
    if (status == LFR_SUCCESSFUL)  // check THR mode
    {
        mode = (bytePosPtr[ BYTE_POS_UPDATE_INFO_PARAMETERS_SET6 ] & 0x0f);
        status = check_update_info_hk_thr_mode( mode );
    }
    if (status == LFR_SUCCESSFUL)  // if the parameter check is successful
    {
        val = housekeeping_packet.hk_lfr_update_info_tc_cnt[0] * 256
                + housekeeping_packet.hk_lfr_update_info_tc_cnt[1];
        val++;
        housekeeping_packet.hk_lfr_update_info_tc_cnt[0] = (unsigned char) (val >> 8);
        housekeeping_packet.hk_lfr_update_info_tc_cnt[1] = (unsigned char) (val);
    }

    result = status;

    return result;
}

int action_enable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function executes specific actions when a TC_LFR_ENABLE_CALIBRATION TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    send_tm_lfr_tc_exe_not_implemented( TC, queue_id, time );
    result = LFR_DEFAULT;

    return result;
}

int action_disable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function executes specific actions when a TC_LFR_DISABLE_CALIBRATION TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    send_tm_lfr_tc_exe_not_implemented( TC, queue_id, time );
    result = LFR_DEFAULT;

    return result;
}

int action_update_time(ccsdsTelecommandPacket_t *TC)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_TIME TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     * @return LFR_SUCCESSFUL
     *
     */

    unsigned int val;

    time_management_regs->coarse_time_load = (TC->dataAndCRC[0] << 24)
                                                + (TC->dataAndCRC[1] << 16)
                                                + (TC->dataAndCRC[2] << 8)
                                                + TC->dataAndCRC[3];

    PRINTF1("time received: %x\n", time_management_regs->coarse_time_load)

    val = housekeeping_packet.hk_lfr_update_time_tc_cnt[0] * 256
            + housekeeping_packet.hk_lfr_update_time_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_update_time_tc_cnt[0] = (unsigned char) (val >> 8);
    housekeeping_packet.hk_lfr_update_time_tc_cnt[1] = (unsigned char) (val);
//    time_management_regs->ctrl = time_management_regs->ctrl | 1;    // force tick

    return LFR_SUCCESSFUL;
}

//*******************
// ENTERING THE MODES
int check_mode_value( unsigned char requestedMode )
{
    int status;

    if ( (requestedMode != LFR_MODE_STANDBY)
         && (requestedMode != LFR_MODE_NORMAL) && (requestedMode != LFR_MODE_BURST)
         && (requestedMode != LFR_MODE_SBM1) && (requestedMode != LFR_MODE_SBM2) )
    {
        status = LFR_DEFAULT;
    }
    else
    {
        status = LFR_SUCCESSFUL;
    }

    return status;
}

int check_mode_transition( unsigned char requestedMode )
{
    /** This function checks the validity of the transition requested by the TC_LFR_ENTER_MODE.
     *
     * @param requestedMode is the mode requested by the TC_LFR_ENTER_MODE
     *
     * @return LFR directive status codes:
     * - LFR_SUCCESSFUL - the transition is authorized
     * - LFR_DEFAULT - the transition is not authorized
     *
     */

    int status;

    switch (requestedMode)
    {
    case LFR_MODE_STANDBY:
        if ( lfrCurrentMode == LFR_MODE_STANDBY ) {
            status = LFR_DEFAULT;
        }
        else
        {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_NORMAL:
        if ( lfrCurrentMode == LFR_MODE_NORMAL ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_BURST:
        if ( lfrCurrentMode == LFR_MODE_BURST ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_SBM1:
        if ( lfrCurrentMode == LFR_MODE_SBM1 ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_SBM2:
        if ( lfrCurrentMode == LFR_MODE_SBM2 ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    default:
        status = LFR_DEFAULT;
        break;
    }

    return status;
}

int check_transition_date( unsigned int transitionCoarseTime )
{
    int status;
    unsigned int localCoarseTime;
    unsigned int deltaCoarseTime;

    status = LFR_SUCCESSFUL;

    if (transitionCoarseTime == 0)  // transition time = 0 means an instant transition
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        localCoarseTime = time_management_regs->coarse_time & 0x7fffffff;

        if ( transitionCoarseTime <= localCoarseTime )   // SSS-CP-EQS-322
        {
            status = LFR_DEFAULT;
            PRINTF2("ERR *** in check_transition_date *** transition = %x, local = %x\n", transitionCoarseTime, localCoarseTime)
        }

        if (status == LFR_SUCCESSFUL)
        {
            deltaCoarseTime = transitionCoarseTime - localCoarseTime;
            if ( deltaCoarseTime > 3 )                  // SSS-CP-EQS-323
            {
                status = LFR_DEFAULT;
                PRINTF1("ERR *** in check_transition_date *** deltaCoarseTime = %x\n", deltaCoarseTime)
            }
        }
    }

    return status;
}

int stop_current_mode( void )
{
    /** This function stops the current mode by masking interrupt lines and suspending science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_ALREADY_SUSPENDED - task already suspended
     *
     */

    rtems_status_code status;

    status = RTEMS_SUCCESSFUL;

    // (1) mask interruptions
    LEON_Mask_interrupt( IRQ_WAVEFORM_PICKER );     // mask waveform picker interrupt
    LEON_Mask_interrupt( IRQ_SPECTRAL_MATRIX );    // clear spectral matrix interrupt

    // (2) clear interruptions
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );    // clear waveform picker interrupt
    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );    // clear spectral matrix interrupt

    // (3) reset waveform picker registers
    reset_wfp_burst_enable();                       // reset burst and enable bits
    reset_wfp_status();                             // reset all the status bits

    // (4) reset spectral matrices registers
    set_irq_on_new_ready_matrix( 0 );               // stop the spectral matrices
    set_run_matrix_spectral( 0 );                   // run_matrix_spectral is set to 0
    reset_extractSWF();                             // reset the extractSWF flag to false

    // <Spectral Matrices simulator>
    LEON_Mask_interrupt( IRQ_SM_SIMULATOR );                  // mask spectral matrix interrupt simulator
    timer_stop( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR );
    LEON_Clear_interrupt( IRQ_SM_SIMULATOR );                 // clear spectral matrix interrupt simulator
    // </Spectral Matrices simulator>

    // suspend several tasks
    if (lfrCurrentMode != LFR_MODE_STANDBY) {
        status = suspend_science_tasks();
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in stop_current_mode *** in suspend_science_tasks *** ERR code: %d\n", status)
    }

    return status;
}

int enter_mode( unsigned char mode, unsigned int transitionCoarseTime )
{
    /** This function is launched after a mode transition validation.
     *
     * @param mode is the mode in which LFR will be put.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - the mode has been entered successfully
     * - RTEMS_NOT_SATISFIED - the mode has not been entered successfully
     *
     */

    rtems_status_code status;

    //**********************
    // STOP THE CURRENT MODE
    status = stop_current_mode();
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode *** stop_current_mode with mode = %d\n", mode)
    }

    //*************************
    // ENTER THE REQUESTED MODE
    if ( (mode == LFR_MODE_NORMAL) || (mode == LFR_MODE_BURST)
         || (mode == LFR_MODE_SBM1) || (mode == LFR_MODE_SBM2) )
    {
#ifdef PRINT_TASK_STATISTICS
        rtems_cpu_usage_reset();
        maxCount = 0;
#endif
        status = restart_science_tasks();
        launch_waveform_picker( mode, transitionCoarseTime );
        launch_spectral_matrix_simu( mode );
    }
    else if ( mode == LFR_MODE_STANDBY )
    {
#ifdef PRINT_TASK_STATISTICS
        rtems_cpu_usage_report();
#endif

#ifdef PRINT_STACK_REPORT
        rtems_stack_checker_report_usage();
#endif
        PRINTF1("maxCount = %d\n", maxCount)
    }
    else
    {
        status = RTEMS_UNSATISFIED;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode *** status = %d\n", status)
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int restart_science_tasks()
{
    /** This function is used to restart all science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * Science tasks are AVF0, BPF0, WFRM, CWF3, CW2, CWF1
     *
     */

    rtems_status_code status[6];
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status[0] = rtems_task_restart( Task_id[TASKID_AVF0], 1 );
    if (status[0] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 0 ERR %d\n", status[0])
    }

    status[2] = rtems_task_restart( Task_id[TASKID_WFRM],1 );
    if (status[2] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 2 ERR %d\n", status[2])
    }

    status[3] = rtems_task_restart( Task_id[TASKID_CWF3],1 );
    if (status[3] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 3 ERR %d\n", status[3])
    }

    status[4] = rtems_task_restart( Task_id[TASKID_CWF2],1 );
    if (status[4] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 4 ERR %d\n", status[4])
    }

    status[5] = rtems_task_restart( Task_id[TASKID_CWF1],1 );
    if (status[5] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 5 ERR %d\n", status[5])
    }

    if ( (status[0] != RTEMS_SUCCESSFUL) || (status[2] != RTEMS_SUCCESSFUL) ||
         (status[3] != RTEMS_SUCCESSFUL) || (status[4] != RTEMS_SUCCESSFUL) || (status[5] != RTEMS_SUCCESSFUL) )
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

int suspend_science_tasks()
{
    /** This function suspends the science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_ALREADY_SUSPENDED - task already suspended
     *
     */

    rtems_status_code status;

    status = rtems_task_suspend( Task_id[TASKID_AVF0] );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** AVF0 ERR %d\n", status)
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend WFRM
    {
        status = rtems_task_suspend( Task_id[TASKID_WFRM] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** WFRM ERR %d\n", status)
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend CWF3
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF3] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF3 ERR %d\n", status)
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend CWF2
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF2] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF2 ERR %d\n", status)
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend CWF1
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF1] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF1 ERR %d\n", status)
        }
    }

    return status;
}

void launch_waveform_picker( unsigned char mode, unsigned int transitionCoarseTime )
{
    reset_current_ring_nodes();
    reset_waveform_picker_regs();
    set_wfp_burst_enable_register( mode );

    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );

    waveform_picker_regs->run_burst_enable = waveform_picker_regs->run_burst_enable | 0x80; // [1000 0000]
    if (transitionCoarseTime == 0)
    {
        waveform_picker_regs->start_date = time_management_regs->coarse_time;
    }
    else
    {
        waveform_picker_regs->start_date = transitionCoarseTime;
    }
}

void launch_spectral_matrix( unsigned char mode )
{
    reset_nb_sm_f0();
    reset_current_sm_ring_nodes();
    reset_spectral_matrix_regs();

    struct grgpio_regs_str *grgpio_regs = (struct grgpio_regs_str *) REGS_ADDR_GRGPIO;
    grgpio_regs->io_port_direction_register =
            grgpio_regs->io_port_direction_register | 0x01; // [0001 1000], 0 = output disabled, 1 = output enabled
    grgpio_regs->io_port_output_register = grgpio_regs->io_port_output_register | 0x00; // set the bit 0 to 1
    set_irq_on_new_ready_matrix( 1 );
    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );
    LEON_Unmask_interrupt( IRQ_SPECTRAL_MATRIX );
    set_run_matrix_spectral( 1 );

}

void set_irq_on_new_ready_matrix( unsigned char value )
{
    if (value == 1)
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config | 0x01;
    }
    else
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config & 0xfffffffe;   // 1110
    }
}

void set_run_matrix_spectral( unsigned char value )
{
    if (value == 1)
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config | 0x4;  // [0100] set run_matrix spectral to 1
    }
    else
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config & 0xfffffffb;  // [1011] set run_matrix spectral to 0
    }
}

void launch_spectral_matrix_simu( unsigned char mode )
{
    reset_nb_sm_f0();
    reset_current_sm_ring_nodes();
    reset_spectral_matrix_regs();

    // Spectral Matrices simulator
    timer_start( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR );
    LEON_Clear_interrupt( IRQ_SM_SIMULATOR );
    LEON_Unmask_interrupt( IRQ_SM_SIMULATOR );
    set_local_nb_interrupt_f0_MAX();
}

//****************
// CLOSING ACTIONS
void update_last_TC_exe( ccsdsTelecommandPacket_t *TC, unsigned char * time )
{
    /** This function is used to update the HK packets statistics after a successful TC execution.
     *
     * @param TC points to the TC being processed
     * @param time is the time used to date the TC execution
     *
     */

    unsigned int val;

    housekeeping_packet.hk_lfr_last_exe_tc_id[0] = TC->packetID[0];
    housekeeping_packet.hk_lfr_last_exe_tc_id[1] = TC->packetID[1];
    housekeeping_packet.hk_lfr_last_exe_tc_type[0] = 0x00;
    housekeeping_packet.hk_lfr_last_exe_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[0] = 0x00;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_exe_tc_time[0] = time[0];
    housekeeping_packet.hk_lfr_last_exe_tc_time[1] = time[1];
    housekeeping_packet.hk_lfr_last_exe_tc_time[2] = time[2];
    housekeeping_packet.hk_lfr_last_exe_tc_time[3] = time[3];
    housekeeping_packet.hk_lfr_last_exe_tc_time[4] = time[4];
    housekeeping_packet.hk_lfr_last_exe_tc_time[5] = time[5];

    val = housekeeping_packet.hk_lfr_exe_tc_cnt[0] * 256 + housekeeping_packet.hk_lfr_exe_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_exe_tc_cnt[0] = (unsigned char) (val >> 8);
    housekeeping_packet.hk_lfr_exe_tc_cnt[1] = (unsigned char) (val);
}

void update_last_TC_rej(ccsdsTelecommandPacket_t *TC, unsigned char * time )
{
    /** This function is used to update the HK packets statistics after a TC rejection.
     *
     * @param TC points to the TC being processed
     * @param time is the time used to date the TC rejection
     *
     */

    unsigned int val;

    housekeeping_packet.hk_lfr_last_rej_tc_id[0] = TC->packetID[0];
    housekeeping_packet.hk_lfr_last_rej_tc_id[1] = TC->packetID[1];
    housekeeping_packet.hk_lfr_last_rej_tc_type[0] = 0x00;
    housekeeping_packet.hk_lfr_last_rej_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[0] = 0x00;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_rej_tc_time[0] = time[0];
    housekeeping_packet.hk_lfr_last_rej_tc_time[1] = time[1];
    housekeeping_packet.hk_lfr_last_rej_tc_time[2] = time[2];
    housekeeping_packet.hk_lfr_last_rej_tc_time[3] = time[3];
    housekeeping_packet.hk_lfr_last_rej_tc_time[4] = time[4];
    housekeeping_packet.hk_lfr_last_rej_tc_time[5] = time[5];

    val = housekeeping_packet.hk_lfr_rej_tc_cnt[0] * 256 + housekeeping_packet.hk_lfr_rej_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_rej_tc_cnt[0] = (unsigned char) (val >> 8);
    housekeeping_packet.hk_lfr_rej_tc_cnt[1] = (unsigned char) (val);
}

void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id )
{
    /** This function is the last step of the TC execution workflow.
     *
     * @param TC points to the TC being processed
     * @param result is the result of the TC execution (LFR_SUCCESSFUL / LFR_DEFAULT)
     * @param queue_id is the id of the RTEMS message queue used to send TM packets
     * @param time is the time used to date the TC execution
     *
     */

    unsigned char requestedMode;

    if (result == LFR_SUCCESSFUL)
    {
        if ( !( (TC->serviceType==TC_TYPE_TIME) & (TC->serviceSubType==TC_SUBTYPE_UPDT_TIME) )
             &
             !( (TC->serviceType==TC_TYPE_GEN) & (TC->serviceSubType==TC_SUBTYPE_UPDT_INFO))
             )
        {
            send_tm_lfr_tc_exe_success( TC, queue_id );
        }
        if ( (TC->serviceType == TC_TYPE_GEN) & (TC->serviceSubType == TC_SUBTYPE_ENTER) )
        {
            //**********************************
            // UPDATE THE LFRMODE LOCAL VARIABLE
            requestedMode = TC->dataAndCRC[1];
            housekeeping_packet.lfr_status_word[0] = (unsigned char) ((requestedMode << 4) + 0x0d);
            updateLFRCurrentMode();
        }
    }
    else if (result == LFR_EXE_ERROR)
    {
        send_tm_lfr_tc_exe_error( TC, queue_id );
    }
}

//***************************
// Interrupt Service Routines
rtems_isr commutation_isr1( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        printf("In commutation_isr1 *** Error sending event to DUMB\n");
    }
}

rtems_isr commutation_isr2( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        printf("In commutation_isr2 *** Error sending event to DUMB\n");
    }
}

//****************
// OTHER FUNCTIONS
void updateLFRCurrentMode()
{
    /** This function updates the value of the global variable lfrCurrentMode.
     *
     * lfrCurrentMode is a parameter used by several functions to know in which mode LFR is running.
     *
     */
    // update the local value of lfrCurrentMode with the value contained in the housekeeping_packet structure
    lfrCurrentMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;
}


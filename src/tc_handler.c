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
#include "math.h"

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

    BOOT_PRINTF("in ACTN *** \n");

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
            case TC_SUBTYPE_LOAD_COMM:
                result = action_load_common_par( &TC );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_NORM:
                result = action_load_normal_par( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_BURST:
                result = action_load_burst_par( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_SBM1:
                result = action_load_sbm1_par( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_SBM2:
                result = action_load_sbm2_par( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_DUMP:
                result = action_dump_par( &TC,  queue_snd_id );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_ENTER:
                result = action_enter_mode( &TC, queue_snd_id );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_UPDT_INFO:
                result = action_update_info( &TC, queue_snd_id );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_EN_CAL:
                result = action_enable_calibration( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_DIS_CAL:
                result = action_disable_calibration( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_K:
                result = action_load_kcoefficients( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_DUMP_K:
                result = action_dump_kcoefficients( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_FBINS:
                result = action_load_fbins_mask( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_LOAD_FILTER_PAR:
                result = action_load_filter_par( &TC, queue_snd_id, time );
                close_action( &TC, result, queue_snd_id );
                break;
            case TC_SUBTYPE_UPDT_TIME:
                result = action_update_time( &TC );
                close_action( &TC, result, queue_snd_id );
                break;
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

    PRINTF("this is the end!!!\n");
    exit(0);

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

    else                                // the mode value is valid, check the transition
    {
        status = check_mode_transition(requestedMode);
        if (status != LFR_SUCCESSFUL)
        {
            PRINTF("ERR *** in action_enter_mode *** check_mode_transition\n")
                    send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        }
    }

    if ( status == LFR_SUCCESSFUL )     // the transition is valid, check the date
    {
        status = check_transition_date( transitionCoarseTime );
        if (status != LFR_SUCCESSFUL)
        {
            PRINTF("ERR *** in action_enter_mode *** check_transition_date\n");
            send_tm_lfr_tc_exe_not_executable(TC, queue_id );
        }
    }

    if ( status == LFR_SUCCESSFUL )     // the date is valid, enter the mode
    {
        PRINTF1("OK  *** in action_enter_mode *** enter mode %d\n", requestedMode);

        switch(requestedMode)
        {
        case LFR_MODE_STANDBY:
            status = enter_mode_standby();
            break;
        case LFR_MODE_NORMAL:
            status = enter_mode_normal( transitionCoarseTime );
            break;
        case LFR_MODE_BURST:
            status = enter_mode_burst( transitionCoarseTime );
            break;
        case LFR_MODE_SBM1:
            status = enter_mode_sbm1( transitionCoarseTime );
            break;
        case LFR_MODE_SBM2:
            status = enter_mode_sbm2( transitionCoarseTime );
            break;
        default:
            break;
        }

        if (status != RTEMS_SUCCESSFUL)
        {
            status = LFR_EXE_ERROR;
        }
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

    // pa_bia_status_info
    // => pa_bia_mode_mux_set       3 bits
    // => pa_bia_mode_hv_enabled    1 bit
    // => pa_bia_mode_bias1_enabled 1 bit
    // => pa_bia_mode_bias2_enabled 1 bit
    // => pa_bia_mode_bias3_enabled 1 bit
    // => pa_bia_on_off (cp_dpu_bias_on_off)
    pa_bia_status_info = bytePosPtr[ BYTE_POS_UPDATE_INFO_PARAMETERS_SET2 ] & 0xfe; // [1111 1110]
    pa_bia_status_info = pa_bia_status_info
            | (bytePosPtr[ BYTE_POS_UPDATE_INFO_PARAMETERS_SET1 ] & 0x1);

    // REACTION_WHEELS_FREQUENCY, copy the incoming parameters in the local variable (to be copied in HK packets)

    cp_rpw_sc_rw_f_flags = bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW_F_FLAGS ];
    getReactionWheelsFrequencies( TC );
    build_sy_lfr_rw_masks();

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

    result = LFR_DEFAULT;

    setCalibration( true );

    result = LFR_SUCCESSFUL;

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

    result = LFR_DEFAULT;

    setCalibration( false );

    result = LFR_SUCCESSFUL;

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

    val = housekeeping_packet.hk_lfr_update_time_tc_cnt[0] * 256
            + housekeeping_packet.hk_lfr_update_time_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_update_time_tc_cnt[0] = (unsigned char) (val >> 8);
    housekeeping_packet.hk_lfr_update_time_tc_cnt[1] = (unsigned char) (val);

    oneTcLfrUpdateTimeReceived = 1;

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

void update_last_valid_transition_date( unsigned int transitionCoarseTime )
{
    if (transitionCoarseTime == 0)
    {
        lastValidEnterModeTime = time_management_regs->coarse_time + 1;
        PRINTF1("lastValidEnterModeTime = 0x%x (transitionCoarseTime = 0 => coarse_time+1)\n", lastValidEnterModeTime);
    }
    else
    {
        lastValidEnterModeTime = transitionCoarseTime;
        PRINTF1("lastValidEnterModeTime = 0x%x\n", transitionCoarseTime);
    }
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

        PRINTF2("localTime = %x, transitionTime = %x\n", localCoarseTime, transitionCoarseTime);

        if ( transitionCoarseTime <= localCoarseTime )   // SSS-CP-EQS-322
        {
            status = LFR_DEFAULT;
            PRINTF("ERR *** in check_transition_date *** transitionCoarseTime <= localCoarseTime\n");
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

int restart_asm_activities( unsigned char lfrRequestedMode )
{
    rtems_status_code status;

    status = stop_spectral_matrices();

    thisIsAnASMRestart = 1;

    status = restart_asm_tasks( lfrRequestedMode );

    launch_spectral_matrix();

    return status;
}

int stop_spectral_matrices( void )
{
    /** This function stops and restarts the current mode average spectral matrices activities.
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
    LEON_Mask_interrupt( IRQ_SPECTRAL_MATRIX );     // mask spectral matrix interrupt

    // (2) reset spectral matrices registers
    set_sm_irq_onNewMatrix( 0 );                    // stop the spectral matrices
    reset_sm_status();

    // (3) clear interruptions
    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );    // clear spectral matrix interrupt

    // suspend several tasks
    if (lfrCurrentMode != LFR_MODE_STANDBY) {
        status = suspend_asm_tasks();
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in stop_current_mode *** in suspend_science_tasks *** ERR code: %d\n", status)
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
    LEON_Mask_interrupt( IRQ_SPECTRAL_MATRIX );     // clear spectral matrix interrupt

    // (2) reset waveform picker registers
    reset_wfp_burst_enable();                       // reset burst and enable bits
    reset_wfp_status();                             // reset all the status bits

    // (3) reset spectral matrices registers
    set_sm_irq_onNewMatrix( 0 );                    // stop the spectral matrices
    reset_sm_status();

    // reset lfr VHDL module
    reset_lfr();

    reset_extractSWF();                             // reset the extractSWF flag to false

    // (4) clear interruptions
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );    // clear waveform picker interrupt
    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );    // clear spectral matrix interrupt

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

int enter_mode_standby( void )
{
    /** This function is used to put LFR in the STANDBY mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The STANDBY mode does not depends on a specific transition date, the effect of the TC_LFR_ENTER_MODE
     * is immediate.
     *
     */

    int status;

    status = stop_current_mode();       // STOP THE CURRENT MODE

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_report();
#endif

#ifdef PRINT_STACK_REPORT
    PRINTF("stack report selected\n")
    rtems_stack_checker_report_usage();
#endif

    return status;
}

int enter_mode_normal( unsigned int transitionCoarseTime )
{
    /** This function is used to start the NORMAL mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the NORMAL mode is started depends on the LFR current mode. If LFR is in SBM1 or SBM2,
     * the snapshots are not restarted, only ASM, BP and CWF data generation are affected.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch( lfrCurrentMode )
    {
    case LFR_MODE_STANDBY:
        status = restart_science_tasks( LFR_MODE_NORMAL ); // restart science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_NORMAL, transitionCoarseTime );
        }
        break;
    case LFR_MODE_BURST:
        status = stop_current_mode();           // stop the current mode
        status = restart_science_tasks( LFR_MODE_NORMAL ); // restart the science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_NORMAL, transitionCoarseTime );
        }
        break;
    case LFR_MODE_SBM1:
        status = restart_asm_activities( LFR_MODE_NORMAL ); // this is necessary to restart ASM tasks to update the parameters
        status = LFR_SUCCESSFUL;                   // lfrCurrentMode will be updated after the execution of close_action
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    case LFR_MODE_SBM2:
        status = restart_asm_activities( LFR_MODE_NORMAL ); // this is necessary to restart ASM tasks to update the parameters
        status = LFR_SUCCESSFUL;                   // lfrCurrentMode will be updated after the execution of close_action
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    default:
        break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode_normal *** status = %d\n", status)
                status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_burst( unsigned int transitionCoarseTime )
{
    /** This function is used to start the BURST mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the BURST mode is started does not depend on the LFR current mode.
     *
     */


    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = stop_current_mode();           // stop the current mode
    status = restart_science_tasks( LFR_MODE_BURST ); // restart the science tasks
    if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
    {
        launch_spectral_matrix( );
        launch_waveform_picker( LFR_MODE_BURST, transitionCoarseTime );
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode_burst *** status = %d\n", status)
                status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_sbm1( unsigned int transitionCoarseTime )
{
    /** This function is used to start the SBM1 mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the SBM1 mode is started depends on the LFR current mode. If LFR is in NORMAL or SBM2,
     * the snapshots are not restarted, only ASM, BP and CWF data generation are affected. In other
     * cases, the acquisition is completely restarted.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch( lfrCurrentMode )
    {
    case LFR_MODE_STANDBY:
        status = restart_science_tasks( LFR_MODE_SBM1 ); // restart science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_SBM1, transitionCoarseTime );
        }
        break;
    case LFR_MODE_NORMAL:                       // lfrCurrentMode will be updated after the execution of close_action
        status = restart_asm_activities( LFR_MODE_SBM1 );
        status = LFR_SUCCESSFUL;
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    case LFR_MODE_BURST:
        status = stop_current_mode();           // stop the current mode
        status = restart_science_tasks( LFR_MODE_SBM1 ); // restart the science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_SBM1, transitionCoarseTime );
        }
        break;
    case LFR_MODE_SBM2:
        status = restart_asm_activities( LFR_MODE_SBM1 );
        status = LFR_SUCCESSFUL;                // lfrCurrentMode will be updated after the execution of close_action
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    default:
        break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode_sbm1 *** status = %d\n", status);
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_sbm2( unsigned int transitionCoarseTime )
{
    /** This function is used to start the SBM2 mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the SBM2 mode is started depends on the LFR current mode. If LFR is in NORMAL or SBM1,
     * the snapshots are not restarted, only ASM, BP and CWF data generation are affected. In other
     * cases, the acquisition is completely restarted.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch( lfrCurrentMode )
    {
    case LFR_MODE_STANDBY:
        status = restart_science_tasks( LFR_MODE_SBM2 ); // restart science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_SBM2, transitionCoarseTime );
        }
        break;
    case LFR_MODE_NORMAL:
        status = restart_asm_activities( LFR_MODE_SBM2 );
        status = LFR_SUCCESSFUL;                // lfrCurrentMode will be updated after the execution of close_action
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    case LFR_MODE_BURST:
        status = stop_current_mode();           // stop the current mode
        status = restart_science_tasks( LFR_MODE_SBM2 ); // restart the science tasks
        if (status == RTEMS_SUCCESSFUL)         // relaunch spectral_matrix and waveform_picker modules
        {
            launch_spectral_matrix( );
            launch_waveform_picker( LFR_MODE_SBM2, transitionCoarseTime );
        }
        break;
    case LFR_MODE_SBM1:
        status = restart_asm_activities( LFR_MODE_SBM2 );
        status = LFR_SUCCESSFUL;                // lfrCurrentMode will be updated after the execution of close_action
        update_last_valid_transition_date( transitionCoarseTime );
        break;
    default:
        break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in enter_mode_sbm2 *** status = %d\n", status)
                status = RTEMS_UNSATISFIED;
    }

    return status;
}

int restart_science_tasks( unsigned char lfrRequestedMode )
{
    /** This function is used to restart all science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * Science tasks are AVF0, PRC0, WFRM, CWF3, CW2, CWF1
     *
     */

    rtems_status_code status[10];
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status[0] = rtems_task_restart( Task_id[TASKID_AVF0], lfrRequestedMode );
    if (status[0] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF0 ERR %d\n", status[0])
    }

    status[1] = rtems_task_restart( Task_id[TASKID_PRC0], lfrRequestedMode );
    if (status[1] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC0 ERR %d\n", status[1])
    }

    status[2] = rtems_task_restart( Task_id[TASKID_WFRM],1 );
    if (status[2] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** WFRM ERR %d\n", status[2])
    }

    status[3] = rtems_task_restart( Task_id[TASKID_CWF3],1 );
    if (status[3] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** CWF3 ERR %d\n", status[3])
    }

    status[4] = rtems_task_restart( Task_id[TASKID_CWF2],1 );
    if (status[4] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** CWF2 ERR %d\n", status[4])
    }

    status[5] = rtems_task_restart( Task_id[TASKID_CWF1],1 );
    if (status[5] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** CWF1 ERR %d\n", status[5])
    }

    status[6] = rtems_task_restart( Task_id[TASKID_AVF1], lfrRequestedMode );
    if (status[6] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF1 ERR %d\n", status[6])
    }

    status[7] = rtems_task_restart( Task_id[TASKID_PRC1],lfrRequestedMode );
    if (status[7] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC1 ERR %d\n", status[7])
    }

    status[8] = rtems_task_restart( Task_id[TASKID_AVF2], 1 );
    if (status[8] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF2 ERR %d\n", status[8])
    }

    status[9] = rtems_task_restart( Task_id[TASKID_PRC2], 1 );
    if (status[9] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC2 ERR %d\n", status[9])
    }

    if ( (status[0] != RTEMS_SUCCESSFUL) || (status[1] != RTEMS_SUCCESSFUL) ||
         (status[2] != RTEMS_SUCCESSFUL) || (status[3] != RTEMS_SUCCESSFUL) ||
         (status[4] != RTEMS_SUCCESSFUL) || (status[5] != RTEMS_SUCCESSFUL) ||
         (status[6] != RTEMS_SUCCESSFUL) || (status[7] != RTEMS_SUCCESSFUL) ||
         (status[8] != RTEMS_SUCCESSFUL) || (status[9] != RTEMS_SUCCESSFUL) )
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

int restart_asm_tasks( unsigned char lfrRequestedMode )
{
    /** This function is used to restart average spectral matrices tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * ASM tasks are AVF0, PRC0, AVF1, PRC1, AVF2 and PRC2
     *
     */

    rtems_status_code status[6];
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status[0] = rtems_task_restart( Task_id[TASKID_AVF0], lfrRequestedMode );
    if (status[0] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF0 ERR %d\n", status[0])
    }

    status[1] = rtems_task_restart( Task_id[TASKID_PRC0], lfrRequestedMode );
    if (status[1] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC0 ERR %d\n", status[1])
    }

    status[2] = rtems_task_restart( Task_id[TASKID_AVF1], lfrRequestedMode );
    if (status[2] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF1 ERR %d\n", status[2])
    }

    status[3] = rtems_task_restart( Task_id[TASKID_PRC1],lfrRequestedMode );
    if (status[3] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC1 ERR %d\n", status[3])
    }

    status[4] = rtems_task_restart( Task_id[TASKID_AVF2], 1 );
    if (status[4] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** AVF2 ERR %d\n", status[4])
    }

    status[5] = rtems_task_restart( Task_id[TASKID_PRC2], 1 );
    if (status[5] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** PRC2 ERR %d\n", status[5])
    }

    if ( (status[0] != RTEMS_SUCCESSFUL) || (status[1] != RTEMS_SUCCESSFUL) ||
         (status[2] != RTEMS_SUCCESSFUL) || (status[3] != RTEMS_SUCCESSFUL) ||
         (status[4] != RTEMS_SUCCESSFUL) || (status[5] != RTEMS_SUCCESSFUL) )
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

int suspend_science_tasks( void )
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

    PRINTF("in suspend_science_tasks\n")

            status = rtems_task_suspend( Task_id[TASKID_AVF0] );    // suspend AVF0
    if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
    {
        PRINTF1("in suspend_science_task *** AVF0 ERR %d\n", status)
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend PRC0
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC0] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC0 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend AVF1
    {
        status = rtems_task_suspend( Task_id[TASKID_AVF1] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** AVF1 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend PRC1
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC1] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC1 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend AVF2
    {
        status = rtems_task_suspend( Task_id[TASKID_AVF2] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** AVF2 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend PRC2
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC2] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC2 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend WFRM
    {
        status = rtems_task_suspend( Task_id[TASKID_WFRM] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** WFRM ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend CWF3
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF3] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** CWF3 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend CWF2
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF2] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** CWF2 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend CWF1
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF1] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** CWF1 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    return status;
}

int suspend_asm_tasks( void )
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

    PRINTF("in suspend_science_tasks\n")

            status = rtems_task_suspend( Task_id[TASKID_AVF0] );    // suspend AVF0
    if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
    {
        PRINTF1("in suspend_science_task *** AVF0 ERR %d\n", status)
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend PRC0
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC0] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC0 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend AVF1
    {
        status = rtems_task_suspend( Task_id[TASKID_AVF1] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** AVF1 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend PRC1
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC1] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC1 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend AVF2
    {
        status = rtems_task_suspend( Task_id[TASKID_AVF2] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** AVF2 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend PRC2
    {
        status = rtems_task_suspend( Task_id[TASKID_PRC2] );
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            PRINTF1("in suspend_science_task *** PRC2 ERR %d\n", status)
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    return status;
}

void launch_waveform_picker( unsigned char mode, unsigned int transitionCoarseTime )
{

    WFP_reset_current_ring_nodes();

    reset_waveform_picker_regs();

    set_wfp_burst_enable_register( mode );

    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );

    if (transitionCoarseTime == 0)
    {
        // instant transition means transition on the next valid date
        // this is mandatory to have a good snapshot period and a good correction of the snapshot period
        waveform_picker_regs->start_date = time_management_regs->coarse_time + 1;
    }
    else
    {
        waveform_picker_regs->start_date = transitionCoarseTime;
    }

    update_last_valid_transition_date(waveform_picker_regs->start_date);

}

void launch_spectral_matrix( void )
{
    SM_reset_current_ring_nodes();

    reset_spectral_matrix_regs();

    reset_nb_sm();

    set_sm_irq_onNewMatrix( 1 );

    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );
    LEON_Unmask_interrupt( IRQ_SPECTRAL_MATRIX );

}

void set_sm_irq_onNewMatrix( unsigned char value )
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

void set_sm_irq_onError( unsigned char value )
{
    if (value == 1)
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config | 0x02;
    }
    else
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config & 0xfffffffd;   // 1101
    }
}

//*****************************
// CONFIGURE CALIBRATION SIGNAL
void setCalibrationPrescaler( unsigned int prescaler )
{
    // prescaling of the master clock (25 MHz)
    // master clock is divided by 2^prescaler
    time_management_regs->calPrescaler = prescaler;
}

void setCalibrationDivisor( unsigned int divisionFactor )
{
    // division of the prescaled clock by the division factor
    time_management_regs->calDivisor = divisionFactor;
}

void setCalibrationData( void ){
    unsigned int k;
    unsigned short data;
    float val;
    float f0;
    float f1;
    float fs;
    float Ts;
    float scaleFactor;

    f0 = 625;
    f1 = 10000;
    fs = 160256.410;
    Ts = 1. / fs;
    scaleFactor = 0.250 / 0.000654; // 191, 500 mVpp, 2 sinus waves => 500 mVpp each, amplitude = 250 mV

    time_management_regs->calDataPtr = 0x00;

    // build the signal for the SCM calibration
    for (k=0; k<256; k++)
    {
        val = sin( 2 * pi * f0 * k * Ts )
                + sin(  2 * pi * f1 * k * Ts );
        data = (unsigned short) ((val * scaleFactor) + 2048);
        time_management_regs->calData = data & 0xfff;
    }
}

void setCalibrationDataInterleaved( void ){
    unsigned int k;
    float val;
    float f0;
    float f1;
    float fs;
    float Ts;
    unsigned short data[384];
    unsigned char *dataPtr;

    f0 = 625;
    f1 = 10000;
    fs = 240384.615;
    Ts = 1. / fs;

    time_management_regs->calDataPtr = 0x00;

    // build the signal for the SCM calibration
    for (k=0; k<384; k++)
    {
        val = sin( 2 * pi * f0 * k * Ts )
                + sin(  2 * pi * f1 * k * Ts );
        data[k] = (unsigned short) (val * 512 + 2048);
    }

    // write the signal in interleaved mode
    for (k=0; k<128; k++)
    {
        dataPtr = (unsigned char*) &data[k*3 + 2];
        time_management_regs->calData = (data[k*3] & 0xfff)
                + ( (dataPtr[0] & 0x3f) << 12);
        time_management_regs->calData = (data[k*3 + 1] & 0xfff)
                + ( (dataPtr[1] & 0x3f) << 12);
    }
}

void setCalibrationReload( bool state)
{
    if (state == true)
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl | 0x00000010;   // [0001 0000]
    }
    else
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl & 0xffffffef;   // [1110 1111]
    }
}

void setCalibrationEnable( bool state )
{
    // this bit drives the multiplexer
    if (state == true)
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl | 0x00000040;   // [0100 0000]
    }
    else
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl & 0xffffffbf; // [1011 1111]
    }
}

void setCalibrationInterleaved( bool state )
{
    // this bit drives the multiplexer
    if (state == true)
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl | 0x00000020;   // [0010 0000]
    }
    else
    {
        time_management_regs->calDACCtrl = time_management_regs->calDACCtrl & 0xffffffdf; // [1101 1111]
    }
}

void setCalibration( bool state )
{
    if (state == true)
    {
        setCalibrationEnable( true );
        setCalibrationReload( false );
        set_hk_lfr_calib_enable( true );
    }
    else
    {
        setCalibrationEnable( false );
        setCalibrationReload( true );
        set_hk_lfr_calib_enable( false );
    }
}

void configureCalibration( bool interleaved )
{
    setCalibration( false );
    if ( interleaved == true )
    {
        setCalibrationInterleaved( true );
        setCalibrationPrescaler( 0 );   // 25 MHz   => 25 000 000
        setCalibrationDivisor(  26 );   //          => 240 384
        setCalibrationDataInterleaved();
    }
    else
    {
        setCalibrationPrescaler( 0 );   // 25 MHz   => 25 000 000
        setCalibrationDivisor(  38 );   //          => 160 256 (39 - 1)
        setCalibrationData();
    }
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
            updateLFRCurrentMode( requestedMode );
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
        PRINTF("In commutation_isr1 *** Error sending event to DUMB\n")
    }
}

rtems_isr commutation_isr2( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        PRINTF("In commutation_isr2 *** Error sending event to DUMB\n")
    }
}

//****************
// OTHER FUNCTIONS
void updateLFRCurrentMode( unsigned char requestedMode )
{
    /** This function updates the value of the global variable lfrCurrentMode.
     *
     * lfrCurrentMode is a parameter used by several functions to know in which mode LFR is running.
     *
     */

    // update the local value of lfrCurrentMode with the value contained in the housekeeping_packet structure
    housekeeping_packet.lfr_status_word[0] = (unsigned char) ((requestedMode << 4) + 0x0d);
    lfrCurrentMode = requestedMode;
}

void set_lfr_soft_reset( unsigned char value )
{
    if (value == 1)
    {
        time_management_regs->ctrl = time_management_regs->ctrl | 0x00000004; // [0100]
    }
    else
    {
        time_management_regs->ctrl = time_management_regs->ctrl & 0xfffffffb; // [1011]
    }
}

void reset_lfr( void )
{
    set_lfr_soft_reset( 1 );

    set_lfr_soft_reset( 0 );

    set_hk_lfr_sc_potential_flag( true );
}

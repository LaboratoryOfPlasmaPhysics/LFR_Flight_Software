/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/
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

#include <math.h>
#include <string.h>

#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_housekeeping.h"
#include "hw/lfr_regs.h"
#include "tc_tm/tc_handler.h"

//***********
// RTEMS TASK

LFR_NO_RETURN rtems_task actn_task(rtems_task_argument unused)
{
    /** This RTEMS task is responsible for launching actions upton the reception of valid
     * TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The ACTN task waits for data coming from an RTEMS msesage queue. When data arrives, it
     * launches specific actions depending on the incoming TeleCommand.
     *
     */

    IGNORE_UNUSED_PARAMETER(unused);
    int result = LFR_SUCCESSFUL;
    rtems_status_code status; // RTEMS status code
    ccsdsTelecommandPacket_t __attribute__((aligned(4))) TC; // TC sent to the ACTN task
    size_t size = 0; // size of the incoming TC packet
    unsigned char subtype = 0; // subtype of the current TC packet
    unsigned char time[BYTES_PER_TIME];
    rtems_id queue_rcv_id = RTEMS_ID_NONE;
    rtems_id queue_snd_id = RTEMS_ID_NONE;

    memset(&TC, 0, sizeof(ccsdsTelecommandPacket_t));

    DEBUG_CHECK_STATUS(get_message_queue_id_recv(&queue_rcv_id));
    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_snd_id));

    BOOT_PRINTF("in ACTN *** \n");

    while (1)
    {
        status = rtems_message_queue_receive(
            queue_rcv_id, (char*)&TC, &size, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        DEBUG_CHECK_STATUS(status);
        getTime(time); // set time to the current time
        if (status == RTEMS_SUCCESSFUL)
        {
            subtype = TC.serviceSubType;
            switch (subtype)
            {
                case TC_SUBTYPE_RESET:
                    result = action_reset();
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_COMM:
                    result = action_load_common_par(&TC);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_NORM:
                    result = action_load_normal_par(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_BURST:
                    result = action_load_burst_par(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_SBM1:
                    result = action_load_sbm1_par(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_SBM2:
                    result = action_load_sbm2_par(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_DUMP:
                    result = action_dump_par(&TC, queue_snd_id);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_ENTER:
                    result = action_enter_mode(&TC, queue_snd_id);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_UPDT_INFO:
                    result = action_update_info(&TC);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_EN_CAL:
                    result = action_enable_calibration();
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_DIS_CAL:
                    result = action_disable_calibration();
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_K:
                    result = action_load_kcoefficients(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_DUMP_K:
                    result = action_dump_kcoefficients(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_FBINS:
                    result = action_load_fbins_mask(&TC, queue_snd_id, time);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_LOAD_FILTER_PAR:
                    result = action_load_filter_par(&TC, queue_snd_id);
                    close_action(&TC, result, queue_snd_id);
                    break;
                case TC_SUBTYPE_UPDT_TIME:
                    result = action_update_time(&TC);
                    close_action(&TC, result, queue_snd_id);
                    break;
                default:
                    break;
            }
        }
    }
}

//***********
// TC ACTIONS

int action_reset()
{
    /** This function executes specific actions when a TC_LFR_RESET TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    LFR_PRINTF("this is the end!!!\n");
#ifdef GCOV_ENABLED
    #ifndef GCOV_USE_EXIT
    extern void gcov_exit(void);
    gcov_exit();
    #endif
#endif
    exit(0);

#ifdef ENABLE_DEAD_CODE
    send_tm_lfr_tc_exe_not_implemented(TC, queue_id, time);
#endif

    return LFR_DEFAULT;
}

int action_enter_mode(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_ENTER_MODE TeleCommand has been
     * received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    rtems_status_code status;
    unsigned char requestedMode;
    unsigned int transitionCoarseTime;
    const unsigned char* const bytePosPtr = (const unsigned char*)&TC->packetID;
    requestedMode = bytePosPtr[BYTE_POS_CP_MODE_LFR_SET];
    copyInt32ByChar(
        (unsigned char*)&transitionCoarseTime, &bytePosPtr[BYTE_POS_CP_LFR_ENTER_MODE_TIME]);
    transitionCoarseTime = transitionCoarseTime & COARSE_TIME_MASK;
    status = check_mode_value(requestedMode);
    DEBUG_CHECK_STATUS(status);

    if (status != LFR_SUCCESSFUL) // the mode value is inconsistent
    {
        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(
                               TC, queue_id, BYTE_POS_CP_MODE_LFR_SET, requestedMode));
    }

    else // the mode value is valid, check the transition
    {
        status = check_mode_transition(requestedMode);
        DEBUG_CHECK_STATUS(status);
        if (status != LFR_SUCCESSFUL)
        {
            LFR_PRINTF("ERR *** in action_enter_mode *** check_mode_transition\n");
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_not_executable(TC, queue_id));
        }
    }

    if (status == LFR_SUCCESSFUL) // the transition is valid, check the date
    {
        status = check_transition_date(transitionCoarseTime);
        DEBUG_CHECK_STATUS(status);
        if (status != LFR_SUCCESSFUL)
        {
            LFR_PRINTF("ERR *** in action_enter_mode *** check_transition_date\n");
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_not_executable(TC, queue_id));
        }
    }

    if (status == LFR_SUCCESSFUL) // the date is valid, enter the mode
    {
        LFR_PRINTF("OK  *** in action_enter_mode *** enter mode %d\n", requestedMode);

        switch (requestedMode)
        {
            case LFR_MODE_STANDBY:
                status = enter_mode_standby();
                break;
            case LFR_MODE_NORMAL:
                status = enter_mode_normal(transitionCoarseTime);
                break;
            case LFR_MODE_BURST:
                status = enter_mode_burst(transitionCoarseTime);
                break;
            case LFR_MODE_SBM1:
                status = enter_mode_sbm1(transitionCoarseTime);
                break;
            case LFR_MODE_SBM2:
                status = enter_mode_sbm2(transitionCoarseTime);
                break;
            default:
                break;
        }
        DEBUG_CHECK_STATUS(status);
        if (status != RTEMS_SUCCESSFUL)
        {
            status = LFR_EXE_ERROR;
        }
    }

    return status;
}

int action_update_info(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_INFO TeleCommand has been
     * received.
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
    unsigned int status = LFR_DEFAULT;
    unsigned char mode;
    const unsigned char* const bytePosPtr = (const unsigned char*)&TC->packetID;
    int pos = 0;
    float value = 0.;

    // check LFR mode
    mode = (bytePosPtr[BYTE_POS_UPDATE_INFO_PARAMETERS_SET5] & BITS_LFR_MODE) >> SHIFT_LFR_MODE;
    status = check_update_info_hk_lfr_mode(mode);
    DEBUG_CHECK_STATUS(status);
    if (status == LFR_SUCCESSFUL) // check TDS mode
    {
        mode = (bytePosPtr[BYTE_POS_UPDATE_INFO_PARAMETERS_SET6] & BITS_TDS_MODE) >> SHIFT_TDS_MODE;
        status = check_update_info_hk_tds_mode(mode);
        DEBUG_CHECK_STATUS(status);
    }
    if (status == LFR_SUCCESSFUL) // check THR mode
    {
        mode = (bytePosPtr[BYTE_POS_UPDATE_INFO_PARAMETERS_SET6] & BITS_THR_MODE);
        status = check_update_info_hk_thr_mode(mode);
        DEBUG_CHECK_STATUS(status);
    }
    if (status == LFR_SUCCESSFUL) // check reaction wheels frequencies
    {
        status = check_all_sy_lfr_rw_f(TC, &pos, &value);
        DEBUG_CHECK_STATUS(status);
    }

    // if the parameters checking succeeds, udpate all parameters
    if (status == LFR_SUCCESSFUL)
    {
        // pa_bia_status_info
        // => pa_bia_mode_mux_set       3 bits
        // => pa_bia_mode_hv_enabled    1 bit
        // => pa_bia_mode_bias1_enabled 1 bit
        // => pa_bia_mode_bias2_enabled 1 bit
        // => pa_bia_mode_bias3_enabled 1 bit
        // => pa_bia_on_off (cp_dpu_bias_on_off)
        pa_bia_status_info
            = bytePosPtr[BYTE_POS_UPDATE_INFO_PARAMETERS_SET2] & BITS_BIA; // [1111 1110]
        pa_bia_status_info
            = pa_bia_status_info | (bytePosPtr[BYTE_POS_UPDATE_INFO_PARAMETERS_SET1] & 1);

        // REACTION_WHEELS_FREQUENCY, copy the incoming parameters in the local variable (to be
        // copied in HK packets)
        getReactionWheelsFrequencies(TC);
        set_hk_lfr_sc_rw_f_flags();
        build_sy_lfr_rw_masks();

        // once the masks are built, they have to be merged with the fbins_mask
        merge_fbins_masks();

        // increase the TC_LFR_UPDATE_INFO counter
        if (status == LFR_SUCCESSFUL) // if the parameter check is successful
        {
            val = (housekeeping_packet.hk_lfr_update_info_tc_cnt[0] * CONST_256)
                + housekeeping_packet.hk_lfr_update_info_tc_cnt[1];
            val++;
            housekeeping_packet.hk_lfr_update_info_tc_cnt[0] = (unsigned char)(val >> SHIFT_1_BYTE);
            housekeeping_packet.hk_lfr_update_info_tc_cnt[1] = (unsigned char)(val);
        }
    }

    return status;
}

int action_enable_calibration()
{
    /** This function executes specific actions when a TC_LFR_ENABLE_CALIBRATION TeleCommand has
     * been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */
    setCalibration(true);
    return LFR_SUCCESSFUL;
}

int action_disable_calibration()
{
    /** This function executes specific actions when a TC_LFR_DISABLE_CALIBRATION TeleCommand has
     * been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    setCalibration(false);
    return LFR_SUCCESSFUL;
}

int action_update_time(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_TIME TeleCommand has been
     * received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     * @return LFR_SUCCESSFUL
     *
     */

    unsigned int val;

    time_management_regs->coarse_time_load = (TC->dataAndCRC[BYTE_0] << SHIFT_3_BYTES)
        + (TC->dataAndCRC[BYTE_1] << SHIFT_2_BYTES) + (TC->dataAndCRC[BYTE_2] << SHIFT_1_BYTE)
        + TC->dataAndCRC[BYTE_3];

    val = (housekeeping_packet.hk_lfr_update_time_tc_cnt[0] * CONST_256)
        + housekeeping_packet.hk_lfr_update_time_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_update_time_tc_cnt[0] = (unsigned char)(val >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_update_time_tc_cnt[1] = (unsigned char)(val);

    oneTcLfrUpdateTimeReceived = 1;

    return LFR_SUCCESSFUL;
}

//*******************
// ENTERING THE MODES
int check_mode_value(unsigned char requestedMode)
{
    int status = LFR_DEFAULT;

    if ((requestedMode != LFR_MODE_STANDBY) && (requestedMode != LFR_MODE_NORMAL)
        && (requestedMode != LFR_MODE_BURST) && (requestedMode != LFR_MODE_SBM1)
        && (requestedMode != LFR_MODE_SBM2))
    {
        status = LFR_DEFAULT;
    }
    else
    {
        status = LFR_SUCCESSFUL;
    }

    return status;
}

int check_mode_transition(unsigned char requestedMode)
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
            if (lfrCurrentMode == LFR_MODE_STANDBY)
            {
                status = LFR_DEFAULT;
            }
            else
            {
                status = LFR_SUCCESSFUL;
            }
            break;
        case LFR_MODE_NORMAL:
            if (lfrCurrentMode == LFR_MODE_NORMAL)
            {
                status = LFR_DEFAULT;
            }
            else
            {
                status = LFR_SUCCESSFUL;
            }
            break;
        case LFR_MODE_BURST:
            if (lfrCurrentMode == LFR_MODE_BURST)
            {
                status = LFR_DEFAULT;
            }
            else
            {
                status = LFR_SUCCESSFUL;
            }
            break;
        case LFR_MODE_SBM1:
            if (lfrCurrentMode == LFR_MODE_SBM1)
            {
                status = LFR_DEFAULT;
            }
            else
            {
                status = LFR_SUCCESSFUL;
            }
            break;
        case LFR_MODE_SBM2:
            if (lfrCurrentMode == LFR_MODE_SBM2)
            {
                status = LFR_DEFAULT;
            }
            else
            {
                status = LFR_SUCCESSFUL;
            }
            break;
        default:
            status = LFR_DEFAULT;
            break;
    }

    return status;
}

void update_last_valid_transition_date(unsigned int transitionCoarseTime)
{
    if (transitionCoarseTime == 0)
    {
        lastValidEnterModeTime = time_management_regs->coarse_time + 1;
        LFR_PRINTF("lastValidEnterModeTime = 0x%x (transitionCoarseTime = 0 => coarse_time+1)\n",
            lastValidEnterModeTime);
    }
    else
    {
        lastValidEnterModeTime = transitionCoarseTime;
        LFR_PRINTF("lastValidEnterModeTime = 0x%x\n", transitionCoarseTime);
    }
}

int check_transition_date(unsigned int transitionCoarseTime)
{
    int status;
    unsigned int localCoarseTime;
    unsigned int deltaCoarseTime;

    status = LFR_SUCCESSFUL;

    if (transitionCoarseTime == 0) // transition time = 0 means an instant transition
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        localCoarseTime = time_management_regs->coarse_time & COARSE_TIME_MASK;

        LFR_PRINTF("localTime = %x, transitionTime = %x\n", localCoarseTime, transitionCoarseTime);

        if (transitionCoarseTime <= localCoarseTime) // SSS-CP-EQS-322
        {
            status = LFR_DEFAULT;
            LFR_PRINTF(
                "ERR *** in check_transition_date *** transitionCoarseTime <= localCoarseTime\n");
        }

        if (status == LFR_SUCCESSFUL)
        {
            deltaCoarseTime = transitionCoarseTime - localCoarseTime;
            if (deltaCoarseTime > MAX_DELTA_COARSE_TIME) // SSS-CP-EQS-323
            {
                status = LFR_DEFAULT;
                LFR_PRINTF(
                    "ERR *** in check_transition_date *** deltaCoarseTime = %x\n", deltaCoarseTime);
            }
        }
    }

    return status;
}

int restart_asm_activities(unsigned char lfrRequestedMode)
{
    rtems_status_code status;

    status = stop_spectral_matrices();
    DEBUG_CHECK_STATUS(status);

    thisIsAnASMRestart = 1;

    status = restart_asm_tasks(lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    launch_spectral_matrix();

    return status;
}

int stop_spectral_matrices(void)
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
    LEON_Mask_interrupt(IRQ_SPECTRAL_MATRIX); // mask spectral matrix interrupt

    // (2) reset spectral matrices registers
    set_sm_irq_onNewMatrix(0); // stop the spectral matrices
    reset_sm_status();

    // (3) clear interruptions
    LEON_Clear_interrupt(IRQ_SPECTRAL_MATRIX); // clear spectral matrix interrupt

    // suspend several tasks
    if (lfrCurrentMode != LFR_MODE_STANDBY)
    {
        status = suspend_asm_tasks();
        DEBUG_CHECK_STATUS(status);
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in stop_current_mode *** in suspend_science_tasks *** ERR code: %d\n", status);
    }

    return status;
}

int stop_current_mode(void)
{
    /** This function stops the current mode by masking interrupt lines and suspending science
     * tasks.
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
    LEON_Mask_interrupt(IRQ_WAVEFORM_PICKER); // mask waveform picker interrupt
    LEON_Mask_interrupt(IRQ_SPECTRAL_MATRIX); // clear spectral matrix interrupt

    // (2) reset waveform picker registers
    reset_wfp_burst_enable(); // reset burst and enable bits
    reset_wfp_status(); // reset all the status bits

    // (3) reset spectral matrices registers
    set_sm_irq_onNewMatrix(0); // stop the spectral matrices
    reset_sm_status();

    // reset lfr VHDL module
    reset_lfr();

    reset_extractSWF(); // reset the extractSWF flag to false

    // (4) clear interruptions
    LEON_Clear_interrupt(IRQ_WAVEFORM_PICKER); // clear waveform picker interrupt
    LEON_Clear_interrupt(IRQ_SPECTRAL_MATRIX); // clear spectral matrix interrupt

    // suspend several tasks
    if (lfrCurrentMode != LFR_MODE_STANDBY)
    {
        status = suspend_science_tasks();
        DEBUG_CHECK_STATUS(status);
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in stop_current_mode *** in suspend_science_tasks *** ERR code: %d\n", status);
    }

    return status;
}

int enter_mode_standby(void)
{
    /** This function is used to put LFR in the STANDBY mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The STANDBY mode does not depends on a specific transition date, the effect of the
     * TC_LFR_ENTER_MODE is immediate.
     *
     */

    int status;

    status = stop_current_mode(); // STOP THE CURRENT MODE
    DEBUG_CHECK_STATUS(status);

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_report();
#endif

#ifdef PRINT_STACK_REPORT
    LFR_PRINTF("stack report selected\n");
    rtems_stack_checker_report_usage();
#endif

    return status;
}

int enter_mode_normal(unsigned int transitionCoarseTime)
{
    /** This function is used to start the NORMAL mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the NORMAL mode is started depends on the LFR current mode. If LFR is in SBM1 or
     * SBM2, the snapshots are not restarted, only ASM, BP and CWF data generation are affected.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch (lfrCurrentMode)
    {
        case LFR_MODE_STANDBY:
            status = restart_science_tasks(LFR_MODE_NORMAL); // restart science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_NORMAL, transitionCoarseTime);
            }
            break;
        case LFR_MODE_BURST:
            status = stop_current_mode(); // stop the current mode
            DEBUG_CHECK_STATUS(status);
            status = restart_science_tasks(LFR_MODE_NORMAL); // restart the science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_NORMAL, transitionCoarseTime);
            }
            break;
        case LFR_MODE_SBM1:
        case LFR_MODE_SBM2:
            status = restart_asm_activities(
                LFR_MODE_NORMAL); // this is necessary to restart ASM tasks to update the parameters
            DEBUG_CHECK_STATUS(status);
            status = LFR_SUCCESSFUL; // lfrCurrentMode will be updated after the execution of
                                     // close_action
            update_last_valid_transition_date(transitionCoarseTime);
            break;
        default:
            break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in enter_mode_normal *** status = %d\n", status);
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_burst(unsigned int transitionCoarseTime)
{
    /** This function is used to start the BURST mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
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

    status = stop_current_mode(); // stop the current mode
    DEBUG_CHECK_STATUS(status);
    status = restart_science_tasks(LFR_MODE_BURST); // restart the science tasks
    DEBUG_CHECK_STATUS(status);
    if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
    {
        launch_spectral_matrix();
        launch_waveform_picker(LFR_MODE_BURST, transitionCoarseTime);
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in enter_mode_burst *** status = %d\n", status);
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_sbm1(unsigned int transitionCoarseTime)
{
    /** This function is used to start the SBM1 mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the SBM1 mode is started depends on the LFR current mode. If LFR is in NORMAL or
     * SBM2, the snapshots are not restarted, only ASM, BP and CWF data generation are affected. In
     * other cases, the acquisition is completely restarted.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch (lfrCurrentMode)
    {
        case LFR_MODE_STANDBY:
            status = restart_science_tasks(LFR_MODE_SBM1); // restart science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_SBM1, transitionCoarseTime);
            }
            break;
        case LFR_MODE_NORMAL: // lfrCurrentMode will be updated after the execution of close_action
        case LFR_MODE_SBM2:
            status = restart_asm_activities(LFR_MODE_SBM1);
            DEBUG_CHECK_STATUS(status);
            status = LFR_SUCCESSFUL;
            update_last_valid_transition_date(transitionCoarseTime);
            break;
        case LFR_MODE_BURST:
            status = stop_current_mode(); // stop the current mode
            DEBUG_CHECK_STATUS(status);
            status = restart_science_tasks(LFR_MODE_SBM1); // restart the science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_SBM1, transitionCoarseTime);
            }
            break;
        default:
            break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in enter_mode_sbm1 *** status = %d\n", status);
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_mode_sbm2(unsigned int transitionCoarseTime)
{
    /** This function is used to start the SBM2 mode.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_INCORRECT_STATE - task never started
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot restart remote task
     *
     * The way the SBM2 mode is started depends on the LFR current mode. If LFR is in NORMAL or
     * SBM1, the snapshots are not restarted, only ASM, BP and CWF data generation are affected. In
     * other cases, the acquisition is completely restarted.
     *
     */

    int status;

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif

    status = RTEMS_UNSATISFIED;

    switch (lfrCurrentMode)
    {
        case LFR_MODE_STANDBY:
            status = restart_science_tasks(LFR_MODE_SBM2); // restart science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_SBM2, transitionCoarseTime);
            }
            break;
        case LFR_MODE_NORMAL:
        case LFR_MODE_SBM1:
            status = restart_asm_activities(LFR_MODE_SBM2);
            DEBUG_CHECK_STATUS(status);
            status = LFR_SUCCESSFUL; // lfrCurrentMode will be updated after the execution of
                                     // close_action
            update_last_valid_transition_date(transitionCoarseTime);
            break;
        case LFR_MODE_BURST:
            status = stop_current_mode(); // stop the current mode
            DEBUG_CHECK_STATUS(status);
            status = restart_science_tasks(LFR_MODE_SBM2); // restart the science tasks
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL) // relaunch spectral_matrix and waveform_picker modules
            {
                launch_spectral_matrix();
                launch_waveform_picker(LFR_MODE_SBM2, transitionCoarseTime);
            }
            break;
        default:
            break;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in enter_mode_sbm2 *** status = %d\n", status);
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int restart_science_tasks(unsigned char lfrRequestedMode)
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

    rtems_status_code status = RTEMS_SUCCESSFUL;
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status |= rtems_task_restart(Task_id[TASKID_AVF0], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC0], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_WFRM], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_CWF3], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_CWF2], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_CWF1], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_AVF1], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC1], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_AVF2], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC2], 1);
    DEBUG_CHECK_STATUS(status);

    if (status != RTEMS_SUCCESSFUL)
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

int restart_asm_tasks(unsigned char lfrRequestedMode)
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

    rtems_status_code status= RTEMS_SUCCESSFUL;

    status |= rtems_task_restart(Task_id[TASKID_AVF0], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC0], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_AVF1], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC1], lfrRequestedMode);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_AVF2], 1);
    DEBUG_CHECK_STATUS(status);

    status |= rtems_task_restart(Task_id[TASKID_PRC2], 1);
    DEBUG_CHECK_STATUS(status);

    if (status != RTEMS_SUCCESSFUL)
    {
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int suspend_science_tasks(void)
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

    LFR_PRINTF("in suspend_science_tasks\n");

    status = rtems_task_suspend(Task_id[TASKID_AVF0]); // suspend AVF0
    if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
    {
        LFR_PRINTF("in suspend_science_task *** AVF0 ERR %d\n", status);
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }
    if (status == RTEMS_SUCCESSFUL) // suspend PRC0
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC0]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC0 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend AVF1
    {
        status = rtems_task_suspend(Task_id[TASKID_AVF1]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** AVF1 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend PRC1
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC1]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC1 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend AVF2
    {
        status = rtems_task_suspend(Task_id[TASKID_AVF2]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** AVF2 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend PRC2
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC2]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC2 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend WFRM
    {
        status = rtems_task_suspend(Task_id[TASKID_WFRM]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** WFRM ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend CWF3
    {
        status = rtems_task_suspend(Task_id[TASKID_CWF3]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** CWF3 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend CWF2
    {
        status = rtems_task_suspend(Task_id[TASKID_CWF2]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** CWF2 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }
    if (status == RTEMS_SUCCESSFUL) // suspend CWF1
    {
        status = rtems_task_suspend(Task_id[TASKID_CWF1]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** CWF1 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    return status;
}

int suspend_asm_tasks(void)
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

    LFR_PRINTF("in suspend_science_tasks\n");

    status = rtems_task_suspend(Task_id[TASKID_AVF0]); // suspend AVF0
    if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
    {
        LFR_PRINTF("in suspend_science_task *** AVF0 ERR %d\n", status);
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }

    if (status == RTEMS_SUCCESSFUL) // suspend PRC0
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC0]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC0 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL) // suspend AVF1
    {
        status = rtems_task_suspend(Task_id[TASKID_AVF1]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** AVF1 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL) // suspend PRC1
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC1]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC1 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL) // suspend AVF2
    {
        status = rtems_task_suspend(Task_id[TASKID_AVF2]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** AVF2 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    if (status == RTEMS_SUCCESSFUL) // suspend PRC2
    {
        status = rtems_task_suspend(Task_id[TASKID_PRC2]);
        if ((status != RTEMS_SUCCESSFUL) && (status != RTEMS_ALREADY_SUSPENDED))
        {
            LFR_PRINTF("in suspend_science_task *** PRC2 ERR %d\n", status);
        }
        else
        {
            status = RTEMS_SUCCESSFUL;
        }
    }

    return status;
}

void launch_waveform_picker(unsigned char mode, unsigned int transitionCoarseTime)
{

    WFP_reset_current_ring_nodes();

    reset_waveform_picker_regs();

    set_wfp_burst_enable_register(mode);

    LEON_Clear_interrupt(IRQ_WAVEFORM_PICKER);
    LEON_Unmask_interrupt(IRQ_WAVEFORM_PICKER);

    if (transitionCoarseTime == 0)
    {
        // instant transition means transition on the next valid date
        // this is mandatory to have a good snapshot period and a good correction of the snapshot
        // period
        waveform_picker_regs->start_date = time_management_regs->coarse_time + 1;
    }
    else
    {
        waveform_picker_regs->start_date = transitionCoarseTime;
    }

    update_last_valid_transition_date(waveform_picker_regs->start_date);
}

void launch_spectral_matrix(void)
{
    SM_reset_current_ring_nodes();

    reset_spectral_matrix_regs();

    reset_nb_sm();

    set_sm_irq_onNewMatrix(1);

    LEON_Clear_interrupt(IRQ_SPECTRAL_MATRIX);
    LEON_Unmask_interrupt(IRQ_SPECTRAL_MATRIX);
}

void set_sm_irq_onNewMatrix(unsigned char value)
{
    if (value == 1)
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config | BIT_IRQ_ON_NEW_MATRIX;
    }
    else
    {
        spectral_matrix_regs->config
            = spectral_matrix_regs->config & MASK_IRQ_ON_NEW_MATRIX; // 1110
    }
}

void set_sm_irq_onError(unsigned char value)
{
    if (value == 1)
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config | BIT_IRQ_ON_ERROR;
    }
    else
    {
        spectral_matrix_regs->config = spectral_matrix_regs->config & MASK_IRQ_ON_ERROR; // 1101
    }
}

//*****************************
// CONFIGURE CALIBRATION SIGNAL
void setCalibrationPrescaler(unsigned int prescaler)
{
    // prescaling of the master clock (25 MHz)
    // master clock is divided by 2^prescaler
    time_management_regs->calPrescaler = prescaler;
}

void setCalibrationDivisor(unsigned int divisionFactor)
{
    // division of the prescaled clock by the division factor
    time_management_regs->calDivisor = divisionFactor;
}

void setCalibrationData(void)
{
    /** This function is used to store the values used to drive the DAC in order to generate the SCM
     * calibration signal
     *
     * @param void
     *
     * @return void
     *
     */

    unsigned short data;
    float val;
    float Ts = (float)(1. / CAL_FS);

    time_management_regs->calDataPtr = 0;


    // build the signal for the SCM calibration
    for (unsigned int k = 0; k < CAL_NB_PTS; k++)
    {
        val = (float)(CAL_A0 * sin(CAL_W0 * k * Ts) + CAL_A1 * sin(CAL_W1 * k * Ts));
        data = (unsigned short)((val * CAL_SCALE_FACTOR) + CONST_2048);
        time_management_regs->calData = data & CAL_DATA_MASK;
    }
}

void setCalibrationReload(bool state)
{
    if (state == true)
    {
        time_management_regs->calDACCtrl
            = time_management_regs->calDACCtrl | BIT_CAL_RELOAD; // [0001 0000]
    }
    else
    {
        time_management_regs->calDACCtrl
            = time_management_regs->calDACCtrl & MASK_CAL_RELOAD; // [1110 1111]
    }
}

void setCalibrationEnable(bool state)
{
    // this bit drives the multiplexer
    if (state == true)
    {
        time_management_regs->calDACCtrl
            = time_management_regs->calDACCtrl | BIT_CAL_ENABLE; // [0100 0000]
    }
    else
    {
        time_management_regs->calDACCtrl
            = time_management_regs->calDACCtrl & MASK_CAL_ENABLE; // [1011 1111]
    }
}


void setCalibration(bool state)
{
    if (state == true)
    {
        setCalibrationEnable(true);
        setCalibrationReload(false);
        set_hk_lfr_calib_enable(true);
    }
    else
    {
        setCalibrationEnable(false);
        setCalibrationReload(true);
        set_hk_lfr_calib_enable(false);
    }
}

void configureCalibration()
{
    setCalibration(false);
    setCalibrationPrescaler(0); // 25 MHz   => 25 000 000
    setCalibrationDivisor(CAL_F_DIVISOR); //          => 160 256 (39 - 1)
    setCalibrationData();
}

//****************
// CLOSING ACTIONS
void update_last_TC_exe(const ccsdsTelecommandPacket_t* const TC, const unsigned char* const time)
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
    housekeeping_packet.hk_lfr_last_exe_tc_type[0] = INIT_CHAR;
    housekeeping_packet.hk_lfr_last_exe_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[0] = INIT_CHAR;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_0] = time[BYTE_0];
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_1] = time[BYTE_1];
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_2] = time[BYTE_2];
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_3] = time[BYTE_3];
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_4] = time[BYTE_4];
    housekeeping_packet.hk_lfr_last_exe_tc_time[BYTE_5] = time[BYTE_5];

    val = (housekeeping_packet.hk_lfr_exe_tc_cnt[0] * CONST_256)
        + housekeeping_packet.hk_lfr_exe_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_exe_tc_cnt[0] = (unsigned char)(val >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_exe_tc_cnt[1] = (unsigned char)(val);
}

void update_last_TC_rej(const ccsdsTelecommandPacket_t* const TC, const unsigned char* const time)
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
    housekeeping_packet.hk_lfr_last_rej_tc_type[0] = INIT_CHAR;
    housekeeping_packet.hk_lfr_last_rej_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[0] = INIT_CHAR;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_0] = time[BYTE_0];
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_1] = time[BYTE_1];
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_2] = time[BYTE_2];
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_3] = time[BYTE_3];
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_4] = time[BYTE_4];
    housekeeping_packet.hk_lfr_last_rej_tc_time[BYTE_5] = time[BYTE_5];

    val = (housekeeping_packet.hk_lfr_rej_tc_cnt[0] * CONST_256)
        + housekeeping_packet.hk_lfr_rej_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_rej_tc_cnt[0] = (unsigned char)(val >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_rej_tc_cnt[1] = (unsigned char)(val);
}

void close_action(ccsdsTelecommandPacket_t* TC, int result, rtems_id queue_id)
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
        if (!((TC->serviceType == TC_TYPE_TIME) & (TC->serviceSubType == TC_SUBTYPE_UPDT_TIME))
            & !((TC->serviceType == TC_TYPE_GEN) & (TC->serviceSubType == TC_SUBTYPE_UPDT_INFO)))
        {
            send_tm_lfr_tc_exe_success(TC, queue_id);
        }
        if ((TC->serviceType == TC_TYPE_GEN) & (TC->serviceSubType == TC_SUBTYPE_ENTER))
        {
            //**********************************
            // UPDATE THE LFRMODE LOCAL VARIABLE
            requestedMode = TC->dataAndCRC[1];
            updateLFRCurrentMode(requestedMode);
        }
    }
    else if (result == LFR_EXE_ERROR)
    {
        send_tm_lfr_tc_exe_error(TC, queue_id);
    }
}

//****************
// OTHER FUNCTIONS
void updateLFRCurrentMode(unsigned char requestedMode)
{
    /** This function updates the value of the global variable lfrCurrentMode.
     *
     * lfrCurrentMode is a parameter used by several functions to know in which mode LFR is running.
     *
     */

    // update the local value of lfrCurrentMode with the value contained in the housekeeping_packet
    // structure
    housekeeping_packet.lfr_status_word[0]
        = (housekeeping_packet.lfr_status_word[0] & STATUS_WORD_LFR_MODE_MASK)
        + (unsigned char)(requestedMode << STATUS_WORD_LFR_MODE_SHIFT);
    lfrCurrentMode = requestedMode;
}

void set_lfr_soft_reset(unsigned char value)
{
    if (value == 1)
    {
        time_management_regs->ctrl = time_management_regs->ctrl | BIT_SOFT_RESET; // [0100]
    }
    else
    {
        time_management_regs->ctrl = time_management_regs->ctrl & MASK_SOFT_RESET; // [1011]
    }
}

void reset_lfr(void)
{
    set_lfr_soft_reset(1);

    set_lfr_soft_reset(0);

    set_hk_lfr_sc_potential_flag(true);
}

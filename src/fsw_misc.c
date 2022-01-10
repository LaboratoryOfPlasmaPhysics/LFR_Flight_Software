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

/** General usage functions and RTEMS tasks.
 *
 * @file
 * @author P. LEROY
 *
 */

#include "fsw_misc.h"
#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_globals.h"
#include "fsw_housekeeping.h"
#include "fsw_watchdog.h"
#include "processing/iir_filter.h"
#include "tc_tm/tc_handler.h"
#include <stdlib.h>

int16_t hk_lfr_sc_v_f3 = 0;
int16_t hk_lfr_sc_e1_f3 = 0;
int16_t hk_lfr_sc_e2_f3 = 0;

/**
 * @brief load_task starts and keeps the watchdog alive.
 * @param argument
 * @return
 */

rtems_task load_task(rtems_task_argument argument)
{
    IGNORE_UNUSED_PARAMETER(argument);

    BOOT_PRINTF("in LOAD *** \n");

    rtems_status_code status;
    unsigned int i;
    unsigned int j;
    rtems_name name_watchdog_rate_monotonic; // name of the watchdog rate monotonic
    rtems_id watchdog_period_id; // id of the watchdog rate monotonic period

    watchdog_period_id = RTEMS_ID_NONE;

    name_watchdog_rate_monotonic = rtems_build_name('L', 'O', 'A', 'D');

    status = rtems_rate_monotonic_create(name_watchdog_rate_monotonic, &watchdog_period_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in LOAD *** rtems_rate_monotonic_create failed with status of %d\n", status);
    }

    i = 0;
    j = 0;

    watchdog_configure();

    watchdog_start();

    set_sy_lfr_watchdog_enabled(true);

    while (1)
    {
        status = rtems_rate_monotonic_period(watchdog_period_id, WATCHDOG_PERIOD);
        watchdog_reload();
        i = i + 1;
        if (i == WATCHDOG_LOOP_PRINTF)
        {
            i = 0;
            j = j + 1;
            LFR_PRINTF("%d\n", j);
        }
#ifdef DEBUG_WATCHDOG
        if (j == WATCHDOG_LOOP_DEBUG)
        {
            status = rtems_task_delete(RTEMS_SELF);
        }
#endif
    }
}

/**
 * @brief hous_task produces and sends HK each seconds
 * @param argument
 * @return
 */
rtems_task hous_task(rtems_task_argument argument)
{
    IGNORE_UNUSED_PARAMETER(argument);

    rtems_status_code status;
    rtems_status_code spare_status;
    rtems_id queue_id;
    rtems_rate_monotonic_period_status period_status;
    bool isSynchronized;

    queue_id = RTEMS_ID_NONE;
    memset(&period_status, 0, sizeof(rtems_rate_monotonic_period_status));
    isSynchronized = false;

    status = get_message_queue_id_send(&queue_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in HOUS *** ERR get_message_queue_id_send %d\n", status);
    }

    BOOT_PRINTF("in HOUS ***\n");

    if (rtems_rate_monotonic_ident(name_hk_rate_monotonic, &HK_id) != RTEMS_SUCCESSFUL)
    {
        status = rtems_rate_monotonic_create(name_hk_rate_monotonic, &HK_id);
        if (status != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("rtems_rate_monotonic_create failed with status of %d\n", status);
        }
    }

    status = rtems_rate_monotonic_cancel(HK_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status);
    }
    else
    {
        DEBUG_PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n");
    }

    // startup phase
    status = rtems_rate_monotonic_period(HK_id, SY_LFR_TIME_SYN_TIMEOUT_in_ticks);
    DEBUG_CHECK_STATUS(status);
    status = rtems_rate_monotonic_get_status(HK_id, &period_status);
    DEBUG_CHECK_STATUS(status);
    DEBUG_PRINTF("startup HK, HK_id status = %d\n", period_status.state);
    while ((period_status.state != RATE_MONOTONIC_EXPIRED)
        && (isSynchronized == false)) // after SY_LFR_TIME_SYN_TIMEOUT ms, starts HK anyway
    {
        if ((time_management_regs->coarse_time & VAL_LFR_SYNCHRONIZED)
            == INT32_ALL_0) // check time synchronization
        {
            isSynchronized = true;
        }
        else
        {
            status = rtems_rate_monotonic_get_status(HK_id, &period_status);
            DEBUG_CHECK_STATUS(status);
            status = rtems_task_wake_after(HK_SYNC_WAIT); // wait HK_SYNCH_WAIT 100 ms = 10 * 10 ms
            DEBUG_CHECK_STATUS(status);
        }
    }
    status = rtems_rate_monotonic_cancel(HK_id);
    DEBUG_CHECK_STATUS(status);
    DEBUG_PRINTF("startup HK, HK_id status = %d\n", period_status.state);

    set_hk_lfr_reset_cause(POWER_ON);

    while (1)
    { // launch the rate monotonic task
        status = rtems_rate_monotonic_period(HK_id, HK_PERIOD);
        if (status != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("in HOUS *** ERR period: %d\n", status);
            spare_status = send_event_dumb_task(RTEMS_EVENT_6);
        }
        else
        {
            housekeeping_packet.packetSequenceControl[BYTE_0]
                = (unsigned char)(sequenceCounterHK >> SHIFT_1_BYTE);
            housekeeping_packet.packetSequenceControl[BYTE_1] = (unsigned char)(sequenceCounterHK);
            increment_seq_counter(&sequenceCounterHK);

            housekeeping_packet.time[BYTE_0]
                = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
            housekeeping_packet.time[BYTE_1]
                = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
            housekeeping_packet.time[BYTE_2]
                = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
            housekeeping_packet.time[BYTE_3] = (unsigned char)(time_management_regs->coarse_time);
            housekeeping_packet.time[BYTE_4]
                = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
            housekeeping_packet.time[BYTE_5] = (unsigned char)(time_management_regs->fine_time);

            spacewire_update_hk_lfr_link_state(&housekeeping_packet.lfr_status_word[0]);

            spacewire_read_statistics();

            update_hk_with_grspw_stats();

            set_hk_lfr_time_not_synchro();

            housekeeping_packet.hk_lfr_q_sd_fifo_size_max = hk_lfr_q_sd_fifo_size_max;
            housekeeping_packet.hk_lfr_q_rv_fifo_size_max = hk_lfr_q_rv_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p0_fifo_size_max = hk_lfr_q_p0_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p1_fifo_size_max = hk_lfr_q_p1_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p2_fifo_size_max = hk_lfr_q_p2_fifo_size_max;

            housekeeping_packet.sy_lfr_common_parameters_spare
                = parameter_dump_packet.sy_lfr_common_parameters_spare;
            housekeeping_packet.sy_lfr_common_parameters
                = parameter_dump_packet.sy_lfr_common_parameters;
            encode_temperatures(&housekeeping_packet);
            encode_f3_E_field(
                hk_lfr_sc_v_f3, hk_lfr_sc_e2_f3, hk_lfr_sc_e2_f3, &housekeeping_packet);
            encode_cpu_load(&housekeeping_packet);

            hk_lfr_le_me_he_update();

            // SEND PACKET
            status = rtems_message_queue_send(queue_id, &housekeeping_packet,
                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
            if (status != RTEMS_SUCCESSFUL)
            {
                LFR_PRINTF("in HOUS *** ERR send: %d\n", status);
            }
        }
    }

    LFR_PRINTF("in HOUS *** deleting task\n");

    status = rtems_task_delete(RTEMS_SELF); // should not return

    return;
}


/**
 * @brief avgv_task pruduces HK rate elctrical field from F3 data
 * @param argument
 * @return
 */
rtems_task avgv_task(rtems_task_argument argument)
{
    IGNORE_UNUSED_PARAMETER(argument);

    rtems_status_code status;
    static int old_v = 0;
    static int old_e1 = 0;
    static int old_e2 = 0;
    int32_t current_v = 0;
    int32_t current_e1 = 0;
    int32_t current_e2 = 0;
    int32_t average_v = 0;
    int32_t average_e1 = 0;
    int32_t average_e2 = 0;

    static filter_ctx ctx_v = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };
    static filter_ctx ctx_e1 = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };
    static filter_ctx ctx_e2 = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };

    BOOT_PRINTF("in AVGV ***\n");

    if (rtems_rate_monotonic_ident(name_avgv_rate_monotonic, &AVGV_id) != RTEMS_SUCCESSFUL)
    {
        status = rtems_rate_monotonic_create(name_avgv_rate_monotonic, &AVGV_id);
        DEBUG_CHECK_STATUS(status);
    }

    status = rtems_rate_monotonic_cancel(AVGV_id);
    DEBUG_CHECK_STATUS(status);

    while (1)
    { // launch the rate monotonic task
        status = rtems_rate_monotonic_period(AVGV_id, AVGV_PERIOD);
        DEBUG_CHECK_STATUS(status);
        current_v = waveform_picker_regs->v;
        current_e1 = waveform_picker_regs->e1;
        current_e2 = waveform_picker_regs->e2;
        // this tests is weak but the only way to detect a new sample
        // it assumes that there is enough noise to flip at least one bit on v, E1 or E2
        if ((current_v != old_v) || (current_e1 != old_e1) || (current_e2 != old_e2))
        {
            average_v = filter(current_v, &ctx_v);
            average_e1 = filter(current_e1, &ctx_e1);
            average_e2 = filter(current_e2, &ctx_e2);

            // update int16 values
            hk_lfr_sc_v_f3 = (int16_t)average_v;
            hk_lfr_sc_e1_f3 = (int16_t)average_e1;
            hk_lfr_sc_e2_f3 = (int16_t)average_e2;
        }
        old_v = current_v;
        old_e1 = current_e1;
        old_e2 = current_e2;
    }

    LFR_PRINTF("in AVGV *** deleting task\n");

    status = rtems_task_delete(RTEMS_SELF); // should not return

    return;
}

rtems_task dumb_task(rtems_task_argument unused)
{
    /** This RTEMS taks is used to print messages without affecting the general behaviour of the
     * software.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The DUMB taks waits for RTEMS events and print messages depending on the incoming events.
     *
     */

    IGNORE_UNUSED_PARAMETER(unused);

    unsigned int intEventOut;
    unsigned int coarse_time = 0;
    unsigned int fine_time = 0;
    rtems_event_set event_out;

    event_out = EVENT_SETS_NONE_PENDING;

    BOOT_PRINTF("in DUMB *** \n");

    while (1)
    {
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3
                | RTEMS_EVENT_4 | RTEMS_EVENT_5 | RTEMS_EVENT_6 | RTEMS_EVENT_7 | RTEMS_EVENT_8
                | RTEMS_EVENT_9 | RTEMS_EVENT_12 | RTEMS_EVENT_13 | RTEMS_EVENT_14,
            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut = (unsigned int)event_out;
        for (unsigned int i = 0; i < NB_RTEMS_EVENTS; i++)
        {
            if (((intEventOut >> i) & 1) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                if (i == EVENT_12)
                {
                    LFR_PRINTF("%s\n", DUMB_MESSAGE_12);
                }
                if (i == EVENT_13)
                {
                    LFR_PRINTF("%s\n", DUMB_MESSAGE_13);
                }
                if (i == EVENT_14)
                {
                    LFR_PRINTF("%s\n", DUMB_MESSAGE_1);
                }
            }
        }
    }
}

rtems_task scrubbing_task(rtems_task_argument unused)
{
    /** This RTEMS taks is used to avoid entering IDLE task and also scrub memory to increase
     * scubbing frequency.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The scrubbing reads continuously memory when no other tasks are ready.
     *
     */
    IGNORE_UNUSED_PARAMETER(unused);

    BOOT_PRINTF("in SCRUBBING *** \n");
    volatile int i = 0;
    volatile float valuef = 1.;
    volatile uint32_t* RAM = (uint32_t*)0x40000000;

#ifdef ENABLE_SCRUBBING_COUNTER
    housekeeping_packet.lfr_fpga_version[BYTE_0] = 0;
#endif
    while (1)
    {
        i = (i + 1) % (1024 * 1024);
        valuef += 10.f * (float)RAM[i];
#ifdef ENABLE_SCRUBBING_COUNTER
        if (i == 0)
        {
            housekeeping_packet.lfr_fpga_version[BYTE_0] += 1;
        }
#endif
    }
}

rtems_task calibration_sweep_task(rtems_task_argument unused)
{
    /** This RTEMS taks is used to change calibration signal smapling frequency between snapshots.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * If calibration is enabled, this task will divide by two the calibration signal smapling
     * frequency between snapshots. When minimum sampling frequency is reach it will jump to maximum
     * sampling frequency to loop indefinitely.
     *
     */

    IGNORE_UNUSED_PARAMETER(unused);

    rtems_event_set event_out;
    BOOT_PRINTF("in calibration sweep *** \n");
    rtems_interval ticks_per_seconds = rtems_clock_get_ticks_per_second();
    while (1)
    {
        // Waiting for next F0 snapshot
        rtems_event_receive(RTEMS_EVENT_CAL_SWEEP_WAKE, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out);
        if (time_management_regs->calDACCtrl & BIT_CAL_ENABLE)
        {
            unsigned int delta_snapshot;
            delta_snapshot = (parameter_dump_packet.sy_lfr_n_swf_p[0] * CONST_256)
                + parameter_dump_packet.sy_lfr_n_swf_p[1];
            // We are woken almost in the center of a snapshot -> let's wait for sy_lfr_n_swf_p / 2
            rtems_task_wake_after(ticks_per_seconds * delta_snapshot / 2);
            if (time_management_regs->calDivisor >= CAL_F_DIVISOR_MAX)
            {
                time_management_regs->calDivisor = CAL_F_DIVISOR_MIN;
            }
            else
            {
                time_management_regs->calDivisor *= 2;
            }
        }
    }
}


//*****************************
// init housekeeping parameters

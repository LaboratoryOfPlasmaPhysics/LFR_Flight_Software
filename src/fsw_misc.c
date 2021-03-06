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

int16_t hk_lfr_sc_v_f3_as_int16 = 0;
int16_t hk_lfr_sc_e1_f3_as_int16 = 0;
int16_t hk_lfr_sc_e2_f3_as_int16 = 0;

void timer_configure(unsigned char timer, unsigned int clock_divider, unsigned char interrupt_level,
    rtems_isr (*timer_isr)())
{
    /** This function configures a GPTIMER timer instantiated in the VHDL design.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     * @param interrupt_level is the interrupt level that the timer drives.
     * @param timer_isr is the interrupt subroutine that will be attached to the IRQ driven by the
     * timer.
     *
     * Interrupt levels are described in the SPARC documentation sparcv8.pdf p.76
     *
     */

    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    old_isr_handler = NULL;

    gptimer_regs->timer[timer].ctrl = INIT_CHAR; // reset the control register

    status = rtems_interrupt_catch(
        timer_isr, interrupt_level, &old_isr_handler); // see sparcv8.pdf p.76 for interrupt levels
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF("in configure_timer *** ERR rtems_interrupt_catch\n")
    }

    timer_set_clock_divider(timer, clock_divider);
}

#ifdef ENABLE_DEAD_CODE
void timer_start(unsigned char timer)
{
    /** This function starts a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_CLEAR_IRQ;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_LD;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_EN;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_RS;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_IE;
}
#endif

void timer_stop(unsigned char timer)
{
    /** This function stops a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & GPTIMER_EN_MASK;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & GPTIMER_IE_MASK;
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | GPTIMER_CLEAR_IRQ;
}

void timer_set_clock_divider(unsigned char timer, unsigned int clock_divider)
{
    /** This function sets the clock divider of a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     *
     */

    gptimer_regs->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
}

// WATCHDOG, this ISR should never be triggered.

rtems_isr watchdog_isr(rtems_vector_number vector)
{
    rtems_status_code status_code;

    status_code = rtems_event_send(Task_id[TASKID_DUMB], RTEMS_EVENT_12);

    PRINTF("watchdog_isr *** this is the end, exit(0)\n");

    exit(0);
}

void watchdog_configure(void)
{
    /** This function configure the watchdog.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     * The watchdog is a timer provided by the GPTIMER IP core of the GRLIB.
     *
     */

    LEON_Mask_interrupt(
        IRQ_GPTIMER_WATCHDOG); // mask gptimer/watchdog interrupt during configuration

    timer_configure(TIMER_WATCHDOG, CLKDIV_WATCHDOG, IRQ_SPARC_GPTIMER_WATCHDOG, watchdog_isr);

    LEON_Clear_interrupt(IRQ_GPTIMER_WATCHDOG); // clear gptimer/watchdog interrupt
}

void watchdog_stop(void)
{
    LEON_Mask_interrupt(IRQ_GPTIMER_WATCHDOG); // mask gptimer/watchdog interrupt line
    timer_stop(TIMER_WATCHDOG);
    LEON_Clear_interrupt(IRQ_GPTIMER_WATCHDOG); // clear gptimer/watchdog interrupt
}

void watchdog_reload(void)
{
    /** This function reloads the watchdog timer counter with the timer reload value.
     *
     * @param void
     *
     * @return void
     *
     */

    gptimer_regs->timer[TIMER_WATCHDOG].ctrl
        = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | GPTIMER_LD;
}

void watchdog_start(void)
{
    /** This function starts the watchdog timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    LEON_Clear_interrupt(IRQ_GPTIMER_WATCHDOG);

    gptimer_regs->timer[TIMER_WATCHDOG].ctrl
        = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | GPTIMER_CLEAR_IRQ;
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl
        = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | GPTIMER_LD;
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl
        = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | GPTIMER_EN;
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl
        = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | GPTIMER_IE;

    LEON_Unmask_interrupt(IRQ_GPTIMER_WATCHDOG);
}

int enable_apbuart_transmitter(
    void) // set the bit 1, TE Transmitter Enable to 1 in the APBUART control register
{
    struct apbuart_regs_str* apbuart_regs = (struct apbuart_regs_str*)REGS_ADDR_APBUART;

    apbuart_regs->ctrl = APBUART_CTRL_REG_MASK_TE;

    return 0;
}

void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value)
{
    /** This function sets the scaler reload register of the apbuart module
     *
     * @param regs is the address of the apbuart registers in memory
     * @param value is the value that will be stored in the scaler register
     *
     * The value shall be set by the software to get data on the serial interface.
     *
     */

    struct apbuart_regs_str* apbuart_regs = (struct apbuart_regs_str*)regs;

    apbuart_regs->scaler = value;

    BOOT_PRINTF1("OK  *** apbuart port scaler reload register set to 0x%x\n", value)
}

/**
 * @brief load_task starts and keeps the watchdog alive.
 * @param argument
 * @return
 */

rtems_task load_task(rtems_task_argument argument)
{
    BOOT_PRINTF("in LOAD *** \n")

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
        PRINTF1("in LOAD *** rtems_rate_monotonic_create failed with status of %d\n", status)
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
            PRINTF1("%d\n", j)
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
        PRINTF1("in HOUS *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in HOUS ***\n");

    if (rtems_rate_monotonic_ident(name_hk_rate_monotonic, &HK_id) != RTEMS_SUCCESSFUL)
    {
        status = rtems_rate_monotonic_create(name_hk_rate_monotonic, &HK_id);
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("rtems_rate_monotonic_create failed with status of %d\n", status);
        }
    }

    status = rtems_rate_monotonic_cancel(HK_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status);
    }
    else
    {
        DEBUG_PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n");
    }

    // startup phase
    status = rtems_rate_monotonic_period(HK_id, SY_LFR_TIME_SYN_TIMEOUT_in_ticks);
    status = rtems_rate_monotonic_get_status(HK_id, &period_status);
    DEBUG_PRINTF1("startup HK, HK_id status = %d\n", period_status.state)
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

            status = rtems_task_wake_after(HK_SYNC_WAIT); // wait HK_SYNCH_WAIT 100 ms = 10 * 10 ms
        }
    }
    status = rtems_rate_monotonic_cancel(HK_id);
    DEBUG_PRINTF1("startup HK, HK_id status = %d\n", period_status.state)

    set_hk_lfr_reset_cause(POWER_ON);

    while (1)
    { // launch the rate monotonic task
        status = rtems_rate_monotonic_period(HK_id, HK_PERIOD);
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in HOUS *** ERR period: %d\n", status);
            spare_status = rtems_event_send(Task_id[TASKID_DUMB], RTEMS_EVENT_6);
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
            get_temperatures(housekeeping_packet.hk_lfr_temp_scm);
            get_v_e1_e2_f3(housekeeping_packet.hk_lfr_sc_v_f3);
            get_cpu_load((unsigned char*)&housekeeping_packet.hk_lfr_cpu_load);

            hk_lfr_le_me_he_update();

            // SEND PACKET
            status = rtems_message_queue_send(queue_id, &housekeeping_packet,
                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
            if (status != RTEMS_SUCCESSFUL)
            {
                PRINTF1("in HOUS *** ERR send: %d\n", status)
            }
        }
    }

    PRINTF("in HOUS *** deleting task\n")

    status = rtems_task_delete(RTEMS_SELF); // should not return

    return;
}

/**
 * @brief filter is a Direct-Form-II filter implementation, mostly used to filter electric field for
 * HK
 * @param x, new sample
 * @param ctx, filter context, used to store previous input and output samples
 * @return a new filtered sample
 */
int filter(int x, filter_ctx* ctx)
{
    static const int b[NB_COEFFS][NB_COEFFS]
        = { { B00, B01, B02 }, { B10, B11, B12 }, { B20, B21, B22 } };
    static const int a[NB_COEFFS][NB_COEFFS]
        = { { A00, A01, A02 }, { A10, A11, A12 }, { A20, A21, A22 } };
    static const int b_gain[NB_COEFFS] = { GAIN_B0, GAIN_B1, GAIN_B2 };
    static const int a_gain[NB_COEFFS] = { GAIN_A0, GAIN_A1, GAIN_A2 };

    int_fast32_t W;
    int i;

    W = INIT_INT;
    i = INIT_INT;

    // Direct-Form-II
    for (i = 0; i < NB_COEFFS; i++)
    {
        x = x << a_gain[i];
        W = (x - (a[i][COEFF1] * ctx->W[i][COEFF0]) - (a[i][COEFF2] * ctx->W[i][COEFF1]))
            >> a_gain[i];
        x = (b[i][COEFF0] * W) + (b[i][COEFF1] * ctx->W[i][COEFF0])
            + (b[i][COEFF2] * ctx->W[i][COEFF1]);
        x = x >> b_gain[i];
        ctx->W[i][1] = ctx->W[i][0];
        ctx->W[i][0] = W;
    }
    return x;
}

/**
 * @brief avgv_task pruduces HK rate elctrical field from F3 data
 * @param argument
 * @return
 */
rtems_task avgv_task(rtems_task_argument argument)
{
#define MOVING_AVERAGE 16
    rtems_status_code status;
    static int32_t v[MOVING_AVERAGE] = { 0 };
    static int32_t e1[MOVING_AVERAGE] = { 0 };
    static int32_t e2[MOVING_AVERAGE] = { 0 };
    static int old_v = 0;
    static int old_e1 = 0;
    static int old_e2 = 0;
    int32_t current_v;
    int32_t current_e1;
    int32_t current_e2;
    int32_t average_v;
    int32_t average_e1;
    int32_t average_e2;
    int32_t newValue_v;
    int32_t newValue_e1;
    int32_t newValue_e2;
    unsigned char k;
    unsigned char indexOfOldValue;

    static filter_ctx ctx_v = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };
    static filter_ctx ctx_e1 = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };
    static filter_ctx ctx_e2 = { { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } } };

    BOOT_PRINTF("in AVGV ***\n");

    if (rtems_rate_monotonic_ident(name_avgv_rate_monotonic, &AVGV_id) != RTEMS_SUCCESSFUL)
    {
        status = rtems_rate_monotonic_create(name_avgv_rate_monotonic, &AVGV_id);
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("rtems_rate_monotonic_create failed with status of %d\n", status);
        }
    }

    status = rtems_rate_monotonic_cancel(AVGV_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("ERR *** in AVGV *** rtems_rate_monotonic_cancel(AVGV_id) ***code: %d\n", status);
    }
    else
    {
        DEBUG_PRINTF("OK  *** in AVGV *** rtems_rate_monotonic_cancel(AVGV_id)\n");
    }

    // initialize values
    indexOfOldValue = MOVING_AVERAGE - 1;
    current_v = 0;
    current_e1 = 0;
    current_e2 = 0;
    average_v = 0;
    average_e1 = 0;
    average_e2 = 0;
    newValue_v = 0;
    newValue_e1 = 0;
    newValue_e2 = 0;

    k = INIT_CHAR;

    while (1)
    { // launch the rate monotonic task
        status = rtems_rate_monotonic_period(AVGV_id, AVGV_PERIOD);
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in AVGV *** ERR period: %d\n", status);
        }
        else
        {
            current_v = waveform_picker_regs->v;
            current_e1 = waveform_picker_regs->e1;
            current_e2 = waveform_picker_regs->e2;
            if ((current_v != old_v) || (current_e1 != old_e1) || (current_e2 != old_e2))
            {
                average_v = filter(current_v, &ctx_v);
                average_e1 = filter(current_e1, &ctx_e1);
                average_e2 = filter(current_e2, &ctx_e2);

                // update int16 values
                hk_lfr_sc_v_f3_as_int16 = (int16_t)average_v;
                hk_lfr_sc_e1_f3_as_int16 = (int16_t)average_e1;
                hk_lfr_sc_e2_f3_as_int16 = (int16_t)average_e2;
            }
            old_v = current_v;
            old_e1 = current_e1;
            old_e2 = current_e2;
        }
    }

    PRINTF("in AVGV *** deleting task\n");

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

    unsigned int i;
    unsigned int intEventOut;
    unsigned int coarse_time = 0;
    unsigned int fine_time = 0;
    rtems_event_set event_out;

    event_out = EVENT_SETS_NONE_PENDING;

    BOOT_PRINTF("in DUMB *** \n")

    while (1)
    {
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3
                | RTEMS_EVENT_4 | RTEMS_EVENT_5 | RTEMS_EVENT_6 | RTEMS_EVENT_7 | RTEMS_EVENT_8
                | RTEMS_EVENT_9 | RTEMS_EVENT_12 | RTEMS_EVENT_13 | RTEMS_EVENT_14,
            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut = (unsigned int)event_out;
        for (i = 0; i < NB_RTEMS_EVENTS; i++)
        {
            if (((intEventOut >> i) & 1) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                if (i == EVENT_12)
                {
                    PRINTF1("%s\n", DUMB_MESSAGE_12)
                }
                if (i == EVENT_13)
                {
                    PRINTF1("%s\n", DUMB_MESSAGE_13)
                }
                if (i == EVENT_14)
                {
                    PRINTF1("%s\n", DUMB_MESSAGE_1)
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

    BOOT_PRINTF("in SCRUBBING *** \n");
    volatile int i = 0;
    volatile float valuef = 1.;
    volatile uint32_t* RAM = (uint32_t*)0x40000000;
    volatile uint32_t value;
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

void init_housekeeping_parameters(void)
{
    /** This function initialize the housekeeping_packet global variable with default values.
     *
     */

    unsigned int i = 0;
    unsigned char* parameters;
    unsigned char sizeOfHK;

    sizeOfHK = sizeof(Packet_TM_LFR_HK_t);

    parameters = (unsigned char*)&housekeeping_packet;

    for (i = 0; i < sizeOfHK; i++)
    {
        parameters[i] = INIT_CHAR;
    }

    housekeeping_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    housekeeping_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    housekeeping_packet.reserved = DEFAULT_RESERVED;
    housekeeping_packet.userApplication = CCSDS_USER_APP;
    housekeeping_packet.packetID[0] = (unsigned char)(APID_TM_HK >> SHIFT_1_BYTE);
    housekeeping_packet.packetID[1] = (unsigned char)(APID_TM_HK);
    housekeeping_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    housekeeping_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    housekeeping_packet.packetLength[0] = (unsigned char)(PACKET_LENGTH_HK >> SHIFT_1_BYTE);
    housekeeping_packet.packetLength[1] = (unsigned char)(PACKET_LENGTH_HK);
    housekeeping_packet.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    housekeeping_packet.serviceType = TM_TYPE_HK;
    housekeeping_packet.serviceSubType = TM_SUBTYPE_HK;
    housekeeping_packet.destinationID = TM_DESTINATION_ID_GROUND;
    housekeeping_packet.sid = SID_HK;

    // init status word
    housekeeping_packet.lfr_status_word[0] = DEFAULT_STATUS_WORD_BYTE0;
    housekeeping_packet.lfr_status_word[1] = DEFAULT_STATUS_WORD_BYTE1;
    // init software version
    housekeeping_packet.lfr_sw_version[0] = SW_VERSION_N1;
    housekeeping_packet.lfr_sw_version[1] = SW_VERSION_N2;
    housekeeping_packet.lfr_sw_version[BYTE_2] = SW_VERSION_N3;
    housekeeping_packet.lfr_sw_version[BYTE_3] = SW_VERSION_N4;
    // init fpga version
    parameters = (unsigned char*)(REGS_ADDR_VHDL_VERSION);
    housekeeping_packet.lfr_fpga_version[BYTE_0] = parameters[BYTE_1]; // n1
    housekeeping_packet.lfr_fpga_version[BYTE_1] = parameters[BYTE_2]; // n2
    housekeeping_packet.lfr_fpga_version[BYTE_2] = parameters[BYTE_3]; // n3

    housekeeping_packet.hk_lfr_q_sd_fifo_size = MSG_QUEUE_COUNT_SEND;
    housekeeping_packet.hk_lfr_q_rv_fifo_size = MSG_QUEUE_COUNT_RECV;
    housekeeping_packet.hk_lfr_q_p0_fifo_size = MSG_QUEUE_COUNT_PRC0;
    housekeeping_packet.hk_lfr_q_p1_fifo_size = MSG_QUEUE_COUNT_PRC1;
    housekeeping_packet.hk_lfr_q_p2_fifo_size = MSG_QUEUE_COUNT_PRC2;
}

void increment_seq_counter(unsigned short* packetSequenceControl)
{
    /** This function increment the sequence counter passes in argument.
     *
     * The increment does not affect the grouping flag. In case of an overflow, the counter is reset
     * to 0.
     *
     */

    unsigned short segmentation_grouping_flag;
    unsigned short sequence_cnt;

    segmentation_grouping_flag = TM_PACKET_SEQ_CTRL_STANDALONE
        << SHIFT_1_BYTE; // keep bits 7 downto 6
    sequence_cnt = (*packetSequenceControl) & SEQ_CNT_MASK; // [0011 1111 1111 1111]

    if (sequence_cnt < SEQ_CNT_MAX)
    {
        sequence_cnt = sequence_cnt + 1;
    }
    else
    {
        sequence_cnt = 0;
    }

    *packetSequenceControl = segmentation_grouping_flag | sequence_cnt;
}

void getTime(unsigned char* time)
{
    /** This function write the current local time in the time buffer passed in argument.
     *
     */

    time[0] = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
    time[1] = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
    time[2] = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
    time[3] = (unsigned char)(time_management_regs->coarse_time);
    time[4] = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
    time[5] = (unsigned char)(time_management_regs->fine_time);
}

unsigned long long int getTimeAsUnsignedLongLongInt()
{
    /** This function write the current local time in the time buffer passed in argument.
     *
     */
    unsigned long long int time;

    time = ((unsigned long long int)(time_management_regs->coarse_time & COARSE_TIME_MASK)
               << SHIFT_2_BYTES)
        + time_management_regs->fine_time;

    return time;
}

void get_temperatures(unsigned char* temperatures)
{
    unsigned char* temp_scm_ptr;
    unsigned char* temp_pcb_ptr;
    unsigned char* temp_fpga_ptr;

    // SEL1 SEL0
    // 0    0       => PCB
    // 0    1       => FPGA
    // 1    0       => SCM

    temp_scm_ptr = (unsigned char*)&time_management_regs->temp_scm;
    temp_pcb_ptr = (unsigned char*)&time_management_regs->temp_pcb;
    temp_fpga_ptr = (unsigned char*)&time_management_regs->temp_fpga;

    temperatures[BYTE_0] = temp_scm_ptr[BYTE_2];
    temperatures[BYTE_1] = temp_scm_ptr[BYTE_3];
    temperatures[BYTE_2] = temp_pcb_ptr[BYTE_2];
    temperatures[BYTE_3] = temp_pcb_ptr[BYTE_3];
    temperatures[BYTE_4] = temp_fpga_ptr[BYTE_2];
    temperatures[BYTE_5] = temp_fpga_ptr[BYTE_3];
}

void get_v_e1_e2_f3(unsigned char* spacecraft_potential)
{
    unsigned char* v_ptr;
    unsigned char* e1_ptr;
    unsigned char* e2_ptr;

    v_ptr = (unsigned char*)&hk_lfr_sc_v_f3_as_int16;
    e1_ptr = (unsigned char*)&hk_lfr_sc_e1_f3_as_int16;
    e2_ptr = (unsigned char*)&hk_lfr_sc_e2_f3_as_int16;

    spacecraft_potential[BYTE_0] = v_ptr[0];
    spacecraft_potential[BYTE_1] = v_ptr[1];
    spacecraft_potential[BYTE_2] = e1_ptr[0];
    spacecraft_potential[BYTE_3] = e1_ptr[1];
    spacecraft_potential[BYTE_4] = e2_ptr[0];
    spacecraft_potential[BYTE_5] = e2_ptr[1];
}

/**
 * @brief get_cpu_load, computes CPU load, CPU load average and CPU load max
 * @param resource_statistics stores:
 *          - CPU load at index 0
 *          - CPU load max at index 1
 *          - CPU load average at index 2
 *
 * The CPU load average is computed on the last 60 values with a simple moving average.
 */
void get_cpu_load(unsigned char* resource_statistics)
{
#define LOAD_AVG_SIZE 60
    static unsigned char cpu_load_hist[LOAD_AVG_SIZE] = { 0 };
    static char old_avg_pos = 0;
    static unsigned int cpu_load_avg;
    unsigned char cpu_load;

    cpu_load = lfr_rtems_cpu_usage_report();

    // HK_LFR_CPU_LOAD
    resource_statistics[BYTE_0] = cpu_load;

    // HK_LFR_CPU_LOAD_MAX
    if (cpu_load > resource_statistics[BYTE_1])
    {
        resource_statistics[BYTE_1] = cpu_load;
    }

    cpu_load_avg
        = cpu_load_avg - (unsigned int)cpu_load_hist[(int)old_avg_pos] + (unsigned int)cpu_load;
    cpu_load_hist[(int)old_avg_pos] = cpu_load;
    old_avg_pos += 1;
    old_avg_pos %= LOAD_AVG_SIZE;
    // CPU_LOAD_AVE
    resource_statistics[BYTE_2] = (unsigned char)(cpu_load_avg / LOAD_AVG_SIZE);
// this will change the way LFR compute usage
#ifndef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif
}

void set_hk_lfr_sc_potential_flag(bool state)
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
            | STATUS_WORD_SC_POTENTIAL_FLAG_BIT; // [0100 0000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
            & STATUS_WORD_SC_POTENTIAL_FLAG_MASK; // [1011 1111]
    }
}

void set_sy_lfr_pas_filter_enabled(bool state)
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
            | STATUS_WORD_PAS_FILTER_ENABLED_BIT; // [0010 0000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
            & STATUS_WORD_PAS_FILTER_ENABLED_MASK; // [1101 1111]
    }
}

void set_sy_lfr_watchdog_enabled(bool state)
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1]
            = housekeeping_packet.lfr_status_word[1] | STATUS_WORD_WATCHDOG_BIT; // [0001 0000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1]
            = housekeeping_packet.lfr_status_word[1] & STATUS_WORD_WATCHDOG_MASK; // [1110 1111]
    }
}

void set_hk_lfr_calib_enable(bool state)
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1]
            = housekeeping_packet.lfr_status_word[1] | STATUS_WORD_CALIB_BIT; // [0000 1000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1]
            = housekeeping_packet.lfr_status_word[1] & STATUS_WORD_CALIB_MASK; // [1111 0111]
    }
}

void set_hk_lfr_reset_cause(enum lfr_reset_cause_t lfr_reset_cause)
{
    housekeeping_packet.lfr_status_word[1]
        = housekeeping_packet.lfr_status_word[1] & STATUS_WORD_RESET_CAUSE_MASK; // [1111 1000]

    housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
        | (lfr_reset_cause & STATUS_WORD_RESET_CAUSE_BITS); // [0000 0111]
}

void increment_hk_counter(unsigned char newValue, unsigned char oldValue, unsigned int* counter)
{
    int delta;

    delta = 0;

    if (newValue >= oldValue)
    {
        delta = newValue - oldValue;
    }
    else
    {
        delta = (CONST_256 - oldValue) + newValue;
    }

    *counter = *counter + delta;
}

// Low severity error counters update
void hk_lfr_le_update(void)
{
    static hk_lfr_le_t old_hk_lfr_le = { 0 };
    hk_lfr_le_t new_hk_lfr_le;
    unsigned int counter;

    counter = (((unsigned int)housekeeping_packet.hk_lfr_le_cnt[0]) * CONST_256)
        + housekeeping_packet.hk_lfr_le_cnt[1];

    // DPU
    new_hk_lfr_le.dpu_spw_parity = housekeeping_packet.hk_lfr_dpu_spw_parity;
    new_hk_lfr_le.dpu_spw_disconnect = housekeeping_packet.hk_lfr_dpu_spw_disconnect;
    new_hk_lfr_le.dpu_spw_escape = housekeeping_packet.hk_lfr_dpu_spw_escape;
    new_hk_lfr_le.dpu_spw_credit = housekeeping_packet.hk_lfr_dpu_spw_credit;
    new_hk_lfr_le.dpu_spw_write_sync = housekeeping_packet.hk_lfr_dpu_spw_write_sync;
    // TIMECODE
    new_hk_lfr_le.timecode_erroneous = housekeeping_packet.hk_lfr_timecode_erroneous;
    new_hk_lfr_le.timecode_missing = housekeeping_packet.hk_lfr_timecode_missing;
    new_hk_lfr_le.timecode_invalid = housekeeping_packet.hk_lfr_timecode_invalid;
    // TIME
    new_hk_lfr_le.time_timecode_it = housekeeping_packet.hk_lfr_time_timecode_it;
    new_hk_lfr_le.time_not_synchro = housekeeping_packet.hk_lfr_time_not_synchro;
    new_hk_lfr_le.time_timecode_ctr = housekeeping_packet.hk_lfr_time_timecode_ctr;
    // AHB
    new_hk_lfr_le.ahb_correctable = housekeeping_packet.hk_lfr_ahb_correctable;
    // housekeeping_packet.hk_lfr_dpu_spw_rx_ahb => not handled by the grspw driver
    // housekeeping_packet.hk_lfr_dpu_spw_tx_ahb => not handled by the grspw driver

    // update the le counter
    // DPU
    increment_hk_counter(new_hk_lfr_le.dpu_spw_parity, old_hk_lfr_le.dpu_spw_parity, &counter);
    increment_hk_counter(
        new_hk_lfr_le.dpu_spw_disconnect, old_hk_lfr_le.dpu_spw_disconnect, &counter);
    increment_hk_counter(new_hk_lfr_le.dpu_spw_escape, old_hk_lfr_le.dpu_spw_escape, &counter);
    increment_hk_counter(new_hk_lfr_le.dpu_spw_credit, old_hk_lfr_le.dpu_spw_credit, &counter);
    increment_hk_counter(
        new_hk_lfr_le.dpu_spw_write_sync, old_hk_lfr_le.dpu_spw_write_sync, &counter);
    // TIMECODE
    increment_hk_counter(
        new_hk_lfr_le.timecode_erroneous, old_hk_lfr_le.timecode_erroneous, &counter);
    increment_hk_counter(new_hk_lfr_le.timecode_missing, old_hk_lfr_le.timecode_missing, &counter);
    increment_hk_counter(new_hk_lfr_le.timecode_invalid, old_hk_lfr_le.timecode_invalid, &counter);
    // TIME
    increment_hk_counter(new_hk_lfr_le.time_timecode_it, old_hk_lfr_le.time_timecode_it, &counter);
    increment_hk_counter(new_hk_lfr_le.time_not_synchro, old_hk_lfr_le.time_not_synchro, &counter);
    increment_hk_counter(
        new_hk_lfr_le.time_timecode_ctr, old_hk_lfr_le.time_timecode_ctr, &counter);
    // AHB
    increment_hk_counter(new_hk_lfr_le.ahb_correctable, old_hk_lfr_le.ahb_correctable, &counter);

    // DPU
    old_hk_lfr_le.dpu_spw_parity = new_hk_lfr_le.dpu_spw_parity;
    old_hk_lfr_le.dpu_spw_disconnect = new_hk_lfr_le.dpu_spw_disconnect;
    old_hk_lfr_le.dpu_spw_escape = new_hk_lfr_le.dpu_spw_escape;
    old_hk_lfr_le.dpu_spw_credit = new_hk_lfr_le.dpu_spw_credit;
    old_hk_lfr_le.dpu_spw_write_sync = new_hk_lfr_le.dpu_spw_write_sync;
    // TIMECODE
    old_hk_lfr_le.timecode_erroneous = new_hk_lfr_le.timecode_erroneous;
    old_hk_lfr_le.timecode_missing = new_hk_lfr_le.timecode_missing;
    old_hk_lfr_le.timecode_invalid = new_hk_lfr_le.timecode_invalid;
    // TIME
    old_hk_lfr_le.time_timecode_it = new_hk_lfr_le.time_timecode_it;
    old_hk_lfr_le.time_not_synchro = new_hk_lfr_le.time_not_synchro;
    old_hk_lfr_le.time_timecode_ctr = new_hk_lfr_le.time_timecode_ctr;
    // AHB
    old_hk_lfr_le.ahb_correctable = new_hk_lfr_le.ahb_correctable;
    // housekeeping_packet.hk_lfr_dpu_spw_rx_ahb => not handled by the grspw driver
    // housekeeping_packet.hk_lfr_dpu_spw_tx_ahb => not handled by the grspw driver

    // update housekeeping packet counters, convert unsigned int numbers in 2 bytes numbers
    // LE
    housekeeping_packet.hk_lfr_le_cnt[0] = (unsigned char)((counter & BYTE0_MASK) >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_le_cnt[1] = (unsigned char)(counter & BYTE1_MASK);
}

// Medium severity error counters update
void hk_lfr_me_update(void)
{
    static hk_lfr_me_t old_hk_lfr_me = { 0 };
    hk_lfr_me_t new_hk_lfr_me;
    unsigned int counter;

    counter = (((unsigned int)housekeeping_packet.hk_lfr_me_cnt[0]) * CONST_256)
        + housekeeping_packet.hk_lfr_me_cnt[1];

    // get the current values
    new_hk_lfr_me.dpu_spw_early_eop = housekeeping_packet.hk_lfr_dpu_spw_early_eop;
    new_hk_lfr_me.dpu_spw_invalid_addr = housekeeping_packet.hk_lfr_dpu_spw_invalid_addr;
    new_hk_lfr_me.dpu_spw_eep = housekeeping_packet.hk_lfr_dpu_spw_eep;
    new_hk_lfr_me.dpu_spw_rx_too_big = housekeeping_packet.hk_lfr_dpu_spw_rx_too_big;

    // update the me counter
    increment_hk_counter(
        new_hk_lfr_me.dpu_spw_early_eop, old_hk_lfr_me.dpu_spw_early_eop, &counter);
    increment_hk_counter(
        new_hk_lfr_me.dpu_spw_invalid_addr, old_hk_lfr_me.dpu_spw_invalid_addr, &counter);
    increment_hk_counter(new_hk_lfr_me.dpu_spw_eep, old_hk_lfr_me.dpu_spw_eep, &counter);
    increment_hk_counter(
        new_hk_lfr_me.dpu_spw_rx_too_big, old_hk_lfr_me.dpu_spw_rx_too_big, &counter);

    // store the counters for the next time
    old_hk_lfr_me.dpu_spw_early_eop = new_hk_lfr_me.dpu_spw_early_eop;
    old_hk_lfr_me.dpu_spw_invalid_addr = new_hk_lfr_me.dpu_spw_invalid_addr;
    old_hk_lfr_me.dpu_spw_eep = new_hk_lfr_me.dpu_spw_eep;
    old_hk_lfr_me.dpu_spw_rx_too_big = new_hk_lfr_me.dpu_spw_rx_too_big;

    // update housekeeping packet counters, convert unsigned int numbers in 2 bytes numbers
    // ME
    housekeeping_packet.hk_lfr_me_cnt[0] = (unsigned char)((counter & BYTE0_MASK) >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_me_cnt[1] = (unsigned char)(counter & BYTE1_MASK);
}

// High severity error counters update
void hk_lfr_le_me_he_update()
{

    unsigned int hk_lfr_he_cnt;

    hk_lfr_he_cnt = (((unsigned int)housekeeping_packet.hk_lfr_he_cnt[0]) * 256)
        + housekeeping_packet.hk_lfr_he_cnt[1];

    // update the low severity error counter
    hk_lfr_le_update();

    // update the medium severity error counter
    hk_lfr_me_update();

    // update the high severity error counter
    hk_lfr_he_cnt = 0;

    // update housekeeping packet counters, convert unsigned int numbers in 2 bytes numbers
    // HE
    housekeeping_packet.hk_lfr_he_cnt[0]
        = (unsigned char)((hk_lfr_he_cnt & BYTE0_MASK) >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_he_cnt[1] = (unsigned char)(hk_lfr_he_cnt & BYTE1_MASK);
}

void set_hk_lfr_time_not_synchro()
{
    static unsigned char synchroLost = 1;
    int synchronizationBit;

    // get the synchronization bit
    synchronizationBit = (time_management_regs->coarse_time & VAL_LFR_SYNCHRONIZED)
        >> BIT_SYNCHRONIZATION; // 1000 0000 0000 0000

    switch (synchronizationBit)
    {
        case 0:
            if (synchroLost == 1)
            {
                synchroLost = 0;
            }
            break;
        case 1:
            if (synchroLost == 0)
            {
                synchroLost = 1;
                increase_unsigned_char_counter(&housekeeping_packet.hk_lfr_time_not_synchro);
                update_hk_lfr_last_er_fields(RID_LE_LFR_TIME, CODE_NOT_SYNCHRO);
            }
            break;
        default:
            PRINTF1("in hk_lfr_time_not_synchro *** unexpected value for synchronizationBit = %d\n",
                synchronizationBit);
            break;
    }
}

void set_hk_lfr_ahb_correctable() // CRITICITY L
{
    /** This function builds the error counter hk_lfr_ahb_correctable using the statistics provided
     * by the Cache Control Register (ASI 2, offset 0) and in the Register Protection Control
     * Register (ASR16) on the detected errors in the cache, in the integer unit and in the floating
     * point unit.
     *
     * @param void
     *
     * @return void
     *
     * All errors are summed to set the value of the hk_lfr_ahb_correctable counter.
     *
     */

    unsigned int ahb_correctable;
    unsigned int instructionErrorCounter;
    unsigned int dataErrorCounter;
    unsigned int fprfErrorCounter;
    unsigned int iurfErrorCounter;

    instructionErrorCounter = 0;
    dataErrorCounter = 0;
    fprfErrorCounter = 0;
    iurfErrorCounter = 0;

    CCR_getInstructionAndDataErrorCounters(&instructionErrorCounter, &dataErrorCounter);
    ASR16_get_FPRF_IURF_ErrorCounters(&fprfErrorCounter, &iurfErrorCounter);

    ahb_correctable = instructionErrorCounter + dataErrorCounter + fprfErrorCounter
        + iurfErrorCounter + housekeeping_packet.hk_lfr_ahb_correctable;

    housekeeping_packet.hk_lfr_ahb_correctable
        = (unsigned char)(ahb_correctable & INT8_ALL_F); // [1111 1111]
}

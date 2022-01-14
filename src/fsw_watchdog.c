/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2021, Plasma Physics Laboratory - CNRS
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
/*--                  Author : Alexis Jeandet
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/
#include <stdint.h>

#include <leon.h>

#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_globals.h"
#include "fsw_misc.h"
#include "fsw_processing.h"
#include "fsw_watchdog.h"
#include "hw/lfr_regs.h"
#include "hw/timer.h"

// WATCHDOG, this ISR should never be triggered.

rtems_isr watchdog_isr(rtems_vector_number vector)
{
    IGNORE_UNUSED_PARAMETER(vector);

    DEBUG_CHECK_STATUS(send_event_dumb_task(RTEMS_EVENT_12));

    LFR_PRINTF("watchdog_isr *** this is the end, exit(0)\n");

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

    gptimer0->timer[TIMER_WATCHDOG].ctrl = gptimer0->timer[TIMER_WATCHDOG].ctrl | GPTIMER_LD;
}

void watchdog_start(void)
{
    /** This function starts the watchdog timer.
     *
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    LEON_Clear_interrupt(IRQ_GPTIMER_WATCHDOG);

    gptimer0->timer[TIMER_WATCHDOG].ctrl = gptimer0->timer[TIMER_WATCHDOG].ctrl | GPTIMER_CLEAR_IRQ;
    gptimer0->timer[TIMER_WATCHDOG].ctrl = gptimer0->timer[TIMER_WATCHDOG].ctrl | GPTIMER_LD;
    gptimer0->timer[TIMER_WATCHDOG].ctrl = gptimer0->timer[TIMER_WATCHDOG].ctrl | GPTIMER_EN;
    gptimer0->timer[TIMER_WATCHDOG].ctrl = gptimer0->timer[TIMER_WATCHDOG].ctrl | GPTIMER_IE;

    LEON_Unmask_interrupt(IRQ_GPTIMER_WATCHDOG);
}

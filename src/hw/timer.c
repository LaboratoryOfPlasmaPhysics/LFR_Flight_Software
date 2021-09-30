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
#include "hw/timer.h"
#include "fsw_globals.h"
#include "fsw_debug.h"
#include "hw/lfr_regs.h"

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

    gptimer0->timer[timer].ctrl = INIT_CHAR; // reset the control register

    status = rtems_interrupt_catch(
        timer_isr, interrupt_level, &old_isr_handler); // see sparcv8.pdf p.76 for interrupt levels
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in configure_timer *** ERR rtems_interrupt_catch\n");
    }

    timer_set_clock_divider(timer, clock_divider);
}


void timer_stop(unsigned char timer)
{
    /** This function stops a GPTIMER timer.
     *
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    gptimer0->timer[timer].ctrl = gptimer0->timer[timer].ctrl & GPTIMER_EN_MASK;
    gptimer0->timer[timer].ctrl = gptimer0->timer[timer].ctrl & GPTIMER_IE_MASK;
    gptimer0->timer[timer].ctrl = gptimer0->timer[timer].ctrl | GPTIMER_CLEAR_IRQ;
}

void timer_set_clock_divider(unsigned char timer, unsigned int clock_divider)
{
    /** This function sets the clock divider of a GPTIMER timer.
     *
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     *
     */

    gptimer0->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
}

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
#include "hw/uart.h"
#include "fsw_debug.h"
#include "fsw_globals.h"
#include "hw/lfr_regs.h"


int enable_apbuart_transmitter(
    void) // set the bit 1, TE Transmitter Enable to 1 in the APBUART control register
{
    apbuart0->ctrl = APBUART_CTRL_REG_MASK_TE;
    return 0;
}

void set_apbuart_scaler_reload_register(unsigned int value)
{
    /** This function sets the scaler reload register of the apbuart module
     *
     * @param regs is the address of the apbuart registers in memory
     * @param value is the value that will be stored in the scaler register
     *
     * The value shall be set by the software to get data on the serial interface.
     *
     */
    apbuart0->scaler = value;
    BOOT_PRINTF("OK  *** apbuart port scaler reload register set to 0x%x\n", value);
}

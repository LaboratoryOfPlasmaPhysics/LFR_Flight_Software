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
#include "mitigations/reaction_wheel_filtering.h"
#include "fsw_globals.h"

int getFBinMask(int index, unsigned char channel)
{
    unsigned int indexInChar;
    unsigned int indexInTheChar;
    int fbin;
    unsigned char* sy_lfr_fbins_fx_word1;

    sy_lfr_fbins_fx_word1 = parameter_dump_packet.sy_lfr_fbins_f0_word1;

    switch (channel)
    {
        case CHANNELF0:
            sy_lfr_fbins_fx_word1 = fbins_masks.merged_fbins_mask_f0;
            break;
        case CHANNELF1:
            sy_lfr_fbins_fx_word1 = fbins_masks.merged_fbins_mask_f1;
            break;
        case CHANNELF2:
            sy_lfr_fbins_fx_word1 = fbins_masks.merged_fbins_mask_f2;
            break;
        default:
            PRINTF("ERR *** in getFBinMask, wrong frequency channel")
    }

    indexInChar = index >> SHIFT_3_BITS;
    indexInTheChar = index - (indexInChar * BITS_PER_BYTE);

    fbin = (int)((sy_lfr_fbins_fx_word1[BYTES_PER_MASK - 1 - indexInChar] >> indexInTheChar) & 1);

    return fbin;
}

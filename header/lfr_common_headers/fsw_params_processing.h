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

#ifndef FSW_PARAMS_PROCESSING_H
#define FSW_PARAMS_PROCESSING_H

#define CHANNELF0 0
#define CHANNELF1 1
#define CHANNELF2 2
#define CHANNELF3 3

#define NB_SM_PER_S_F0     96
#define NB_SM_PER_S_F1     16
#define NB_SM_PER_S1_BP_P0 24

#define ASM_COMP_B1B2      1
#define ASM_COMP_B1B2_imag 2
#define ASM_COMP_B1B3      3
#define ASM_COMP_B1B3_imag 4
#define ASM_COMP_B1E1      5
#define ASM_COMP_B1E1_imag 6
#define ASM_COMP_B1E2      7
#define ASM_COMP_B1E2_imag 8
#define ASM_COMP_B2B3      10
#define ASM_COMP_B2B3_imag 11
#define ASM_COMP_B2E1      12
#define ASM_COMP_B2E1_imag 13
#define ASM_COMP_B2E2      14
#define ASM_COMP_B2E2_imag 15
#define ASM_COMP_B3E1      17
#define ASM_COMP_B3E1_imag 18
#define ASM_COMP_B3E2      19
#define ASM_COMP_B3E2_imag 20
#define ASM_COMP_E1E2      22
#define ASM_COMP_E1E2_imag 23
#define ASM_COMP_B1B1      0
#define ASM_COMP_B2B2      9
#define ASM_COMP_B3B3      16
#define ASM_COMP_E1E1      21
#define ASM_COMP_E2E2      24

#define FLOATS_PER_COMPLEX          2
#define NB_MAG_COMPONENT_PER_SM     3
#define NB_ELEC_COMPONENT_PER_SM    2
#define NB_ELEMENTS_MAG_CAL_MATRIX  (NB_MAG_COMPONENT_PER_SM * NB_MAG_COMPONENT_PER_SM)
#define NB_FLOATS_MAG_CAL_MATRIX    (NB_ELEMENTS_MAG_CAL_MATRIX * FLOATS_PER_COMPLEX)
#define NB_BYTES_MAG_CAL_MATRIX     (NB_FLOATS_MAG_CAL_MATRIX * NB_BYTES_PER_FLOAT)
#define NB_ELEMENTS_ELEC_CAL_MATRIX (NB_ELEC_COMPONENT_PER_SM * NB_ELEC_COMPONENT_PER_SM)
#define NB_FLOATS_ELEC_CAL_MATRIX   (NB_ELEMENTS_ELEC_CAL_MATRIX * FLOATS_PER_COMPLEX)
#define NB_BYTES_ELEC_CAL_MATRIX    (NB_FLOATS_ELEC_CAL_MATRIX * NB_BYTES_PER_FLOAT)
#define NB_BINS_PER_SM              128
#define NB_FLOATS_PER_SM            25
#define TOTAL_SIZE_SM               (NB_BINS_PER_SM * NB_FLOATS_PER_SM) // 25 * 128 = 0xC80
// F0
#define NB_RING_NODES_SM_F0            20 // AT LEAST 8 due to the way the averaging is done
#define NB_RING_NODES_ASM_BURST_SBM_F0 10 // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F0      10 // AT LEAST 3
#define NB_RING_NODES_ASM_F0           3 // AT LEAST 3
// F1
#define NB_RING_NODES_SM_F1            12 // AT LEAST 8 due to the way the averaging is done
#define NB_RING_NODES_ASM_BURST_SBM_F1 5 // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F1      5 // AT LEAST 3
#define NB_RING_NODES_ASM_F1           3 // AT LEAST 3
// F2
#define NB_RING_NODES_SM_F2       5 // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F2 3 // AT LEAST 3
#define NB_RING_NODES_ASM_F2      3 // AT LEAST 3
//
#define NB_BINS_PER_PKT_ASM_F0_1 32
#define NB_BINS_PER_PKT_ASM_F0_2 24
#define DLEN_ASM_F0_PKT_1        3200 // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define DLEN_ASM_F0_PKT_2        2400 // 24 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F0_INDICE_START      16 // 17  - 1, (-1) due to the VHDL behaviour
#define ASM_F0_KEEP_BINS         88
//
#define NB_BINS_PER_PKT_ASM_F1_1 36
#define NB_BINS_PER_PKT_ASM_F1_2 32
#define DLEN_ASM_F1_PKT_1        3600 // 36 * 25 * 4, 25 components per matrix, 4 bytes per float
#define DLEN_ASM_F1_PKT_2        3200 // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F1_INDICE_START      5 //   6 - 1, (-1) due to the VHDL behaviour
#define ASM_F1_KEEP_BINS         104
//
#define NB_BINS_PER_PKT_ASM_F2 32
#define DLEN_ASM_F2_PKT        3200 // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F2_INDICE_START    6 //   7 - 1, (-1) due to the VHDL behaviour
#define ASM_F2_KEEP_BINS       96
//
#define NB_BINS_COMPRESSED_SM_F0     11
#define NB_BINS_COMPRESSED_SM_F1     13
#define NB_BINS_COMPRESSED_SM_F2     12
#define NB_BINS_COMPRESSED_SM        36 // 11 + 12 + 13
#define NB_BINS_COMPRESSED_SM_SBM_F0 22
#define NB_BINS_COMPRESSED_SM_SBM_F1 26

//
#define NB_BINS_TO_AVERAGE_ASM_F0     8
#define NB_BINS_TO_AVERAGE_ASM_F1     8
#define NB_BINS_TO_AVERAGE_ASM_F2     8
#define NB_BINS_TO_AVERAGE_ASM_SBM_F0 4
#define NB_BINS_TO_AVERAGE_ASM_SBM_F1 4
//
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F0 275 // 11 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F1 325 // 13 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F2 300 // 12 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_SBM_F0  550 // 22 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_SBM_F1  650 // 26 * 25 WORDS
// GENERAL
#define NB_SM_BEFORE_AVF0_F1 8 // must be 8 due to the SM_average() function
#define NB_SM_BEFORE_AVF2    1 // must be 1 due to the SM_average_f2() function

#endif // FSW_PARAMS_PROCESSING_H

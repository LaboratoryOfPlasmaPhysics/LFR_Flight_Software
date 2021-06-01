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

#ifndef FSW_PARAMS_NB_BYTES_H
#define FSW_PARAMS_NB_BYTES_H

#define PACKET_POS_SEQUENCE_CNT   6 // 4 + 2
#define PACKET_POS_PA_LFR_SID_PKT 20 // 4 + 16
#define PACKET_POS_SERVICE_TYPE   11 // 4 + 7

#define DATAFIELD_OFFSET 10

// TC_LFR_LOAD_COMMON_PAR

// TC_LFR_LOAD_NORMAL_PAR
#define DATAFIELD_POS_SY_LFR_N_SWF_L       0
#define DATAFIELD_POS_SY_LFR_N_SWF_P       2
#define DATAFIELD_POS_SY_LFR_N_ASM_P       4
#define DATAFIELD_POS_SY_LFR_N_BP_P0       6
#define DATAFIELD_POS_SY_LFR_N_BP_P1       7
#define DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3 8

// TC_LFR_LOAD_BURST_PAR
#define DATAFIELD_POS_SY_LFR_B_BP_P0 0
#define DATAFIELD_POS_SY_LFR_B_BP_P1 1

// TC_LFR_LOAD_SBM1_PAR
#define DATAFIELD_POS_SY_LFR_S1_BP_P0 0
#define DATAFIELD_POS_SY_LFR_S1_BP_P1 1

// TC_LFR_LOAD_SBM2_PAR
#define DATAFIELD_POS_SY_LFR_S2_BP_P0 0
#define DATAFIELD_POS_SY_LFR_S2_BP_P1 1

// TC_LFR_UPDATE_INFO
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET1 10
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET2 11
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET5 34
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET6 35
// RW1
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F1 44
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F2 48
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F3 52
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F4 56
// RW2
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F1 60
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F2 64
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F3 68
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F4 72
// RW3
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F1 76
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F2 80
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F3 84
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F4 88
// RW4
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F1 92
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F2 96
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F3 100
#define BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F4 104

#define BITS_LFR_MODE  0x1e
#define SHIFT_LFR_MODE 1
#define BITS_TDS_MODE  0xf0
#define SHIFT_TDS_MODE 4
#define BITS_THR_MODE  0x0f
#define BITS_BIA       0xfe

// TC_LFR_ENTER_MODE
#define BYTE_POS_CP_MODE_LFR_SET        11
#define BYTE_POS_CP_LFR_ENTER_MODE_TIME 12

// TC_LFR_LOAD_FBINS_MASK

// TC_LFR_LOAD_FILTER_PAR
#define NB_RW_K_COEFFS                          16
#define NB_BYTES_PER_RW_K_COEFF                 4
#define DATAFIELD_POS_SY_LFR_PAS_FILTER_ENABLED 1 // 8  bits
#define DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS 2 // 8  bits
#define DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD    3 // 32 bits
#define DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET  7 // 8  bits
#define DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT   8 // 32 bits
#define DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F      12 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW1_K1             16 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW1_K2             20 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW1_K3             24 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW1_K4             28 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW2_K1             32 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW2_K2             36 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW2_K3             40 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW2_K4             44 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW3_K1             48 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW3_K2             52 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW3_K3             56 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW3_K4             60 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW4_K1             64 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW4_K2             68 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW4_K3             72 // 32  bits
#define DATAFIELD_POS_SY_LFR_RW4_K4             76 // 32  bits

// TC_LFR_LOAD_KCOEFFICIENTS
#define NB_BYTES_PER_FLOAT                    4
#define DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY 0 // 10 - 10
#define DATAFIELD_POS_SY_LFR_KCOEFF_1         2 // 12 - 10

// TM_LFR_KCOEFFICIENTS_DUMP
#define BYTE_POS_KCOEFFICIENTS_PARAMETES 20

#endif // FSW_PARAMS_NB_BYTES_H

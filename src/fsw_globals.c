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
/** Global variables of the LFR flight software.
 *
 * @file
 * @author P. LEROY
 *
 * Among global variables, there are:
 * - RTEMS names and id.
 * - APB configuration registers.
 * - waveforms global buffers, used by the waveform picker hardware module to store data.
 * - spectral  matrices buffesr, used by the hardware module to store data.
 * - variable related to LFR modes parameters.
 * - the global HK packet buffer.
 * - the global dump parameter buffer.
 *
 */

#include <rtems.h>

#include <grspw.h>

#include "ccsds_types.h"
#include "fsw_params.h"
#include "fsw_params_wf_handler.h"
#include "fsw_compile_warnings.h"

#define NB_OF_MISC_NAMES 5

// RTEMS GLOBAL VARIABLES
rtems_name misc_name[NB_OF_MISC_NAMES] = { 0 };
rtems_name Task_name[CONFIGURE_MAXIMUM_TASKS - 1] = { 0 }; /* array of task names */
rtems_id Task_id[CONFIGURE_MAXIMUM_TASKS - 1] = { 0 }; /* array of task ids */
rtems_name timecode_timer_name = 0;
rtems_id timecode_timer_id = RTEMS_ID_NONE;
rtems_name name_hk_rate_monotonic = 0; // name of the HK rate monotonic
rtems_id HK_id = RTEMS_ID_NONE; // id of the HK rate monotonic period
rtems_name name_avgv_rate_monotonic = 0; // name of the AVGV rate monotonic
rtems_id AVGV_id = RTEMS_ID_NONE; // id of the AVGV rate monotonic period
volatile int fdSPW = 0;
volatile int fdUART = 0;
volatile unsigned char lfrCurrentMode = 0;
volatile unsigned char pa_bia_status_info = 0;
volatile unsigned char thisIsAnASMRestart = 0;
volatile unsigned char oneTcLfrUpdateTimeReceived = 0;

// WAVEFORMS GLOBAL VARIABLES   // 2048 * 3 * 4 + 2 * 4 = 24576 + 8 bytes = 24584
//  97 * 256 = 24832 => delta = 248 bytes = 62 words
// WAVEFORMS GLOBAL VARIABLES   // 2688 * 3 * 4 + 2 * 4 = 32256 + 8 bytes = 32264
// 127 * 256 = 32512 => delta = 248 bytes = 62 words
// F0 F1 F2 F3
volatile int wf_buffer_f0[NB_RING_NODES_F0 * WFRM_BUFFER] __attribute__((aligned(0x100))) = { 0 };
volatile int wf_buffer_f1[NB_RING_NODES_F1 * WFRM_BUFFER] __attribute__((aligned(0x100))) = { 0 };
volatile int wf_buffer_f2[NB_RING_NODES_F2 * WFRM_BUFFER] __attribute__((aligned(0x100))) = { 0 };
volatile int wf_buffer_f3[NB_RING_NODES_F3 * WFRM_BUFFER] __attribute__((aligned(0x100))) = { 0 };

//***********************************
// SPECTRAL MATRICES GLOBAL VARIABLES

// alignment constraints for the spectral matrices buffers => the first data after the time (8
// bytes) shall be aligned on 0x00
volatile int sm_f0[NB_RING_NODES_SM_F0 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };
volatile int sm_f1[NB_RING_NODES_SM_F1 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };
volatile int sm_f2[NB_RING_NODES_SM_F2 * TOTAL_SIZE_SM] __attribute__((aligned(0x100))) = { 0 };

// MODE PARAMETERS

DISABLE_MISSING_FIELD_INITIALIZER_WARNING
Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet = { 0 };
struct param_local_str param_local = { 0 };
unsigned int lastValidEnterModeTime = { 0 };

// HK PACKETS
Packet_TM_LFR_HK_t housekeeping_packet = { 0 };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

// message queues occupancy
unsigned char hk_lfr_q_sd_fifo_size_max = 0;
unsigned char hk_lfr_q_rv_fifo_size_max = 0;
unsigned char hk_lfr_q_p0_fifo_size_max = 0;
unsigned char hk_lfr_q_p1_fifo_size_max = 0;
unsigned char hk_lfr_q_p2_fifo_size_max = 0;
// sequence counters are incremented by APID (PID + CAT) and destination ID
unsigned short sequenceCounters_SCIENCE_NORMAL_BURST __attribute__((aligned(0x4))) = 0;
unsigned short sequenceCounters_SCIENCE_SBM1_SBM2 __attribute__((aligned(0x4))) = 0;
unsigned short sequenceCounters_TC_EXE[SEQ_CNT_NB_DEST_ID] __attribute__((aligned(0x4))) = { 0 };
unsigned short sequenceCounters_TM_DUMP[SEQ_CNT_NB_DEST_ID] __attribute__((aligned(0x4))) = { 0 };
unsigned short sequenceCounterHK __attribute__((aligned(0x4))) = { 0 };

DISABLE_MISSING_FIELD_INITIALIZER_WARNING
spw_stats grspw_stats __attribute__((aligned(0x4))) = { 0 };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

// TC_LFR_UPDATE_INFO
rw_f_t rw_f;

DISABLE_MISSING_FIELD_INITIALIZER_WARNING
// TC_LFR_LOAD_FILTER_PAR
filterPar_t filterPar = { 0 };

fbins_masks_t fbins_masks = { {0} };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

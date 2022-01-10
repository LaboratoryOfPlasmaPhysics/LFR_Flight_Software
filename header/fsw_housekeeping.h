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
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "ccsds_types.h"
#include "hw/lfr_regs.h"

enum lfr_reset_cause_t
{
    UNKNOWN_CAUSE,
    POWER_ON,
    TC_RESET,
    WATCHDOG,
    ERROR_RESET,
    UNEXP_RESET
};

typedef struct
{
    unsigned char dpu_spw_parity;
    unsigned char dpu_spw_disconnect;
    unsigned char dpu_spw_escape;
    unsigned char dpu_spw_credit;
    unsigned char dpu_spw_write_sync;
    unsigned char timecode_erroneous;
    unsigned char timecode_missing;
    unsigned char timecode_invalid;
    unsigned char time_timecode_it;
    unsigned char time_not_synchro;
    unsigned char time_timecode_ctr;
    unsigned char ahb_correctable;
} hk_lfr_le_t;

typedef struct
{
    unsigned char dpu_spw_early_eop;
    unsigned char dpu_spw_invalid_addr;
    unsigned char dpu_spw_eep;
    unsigned char dpu_spw_rx_too_big;
} hk_lfr_me_t;

void init_housekeeping_parameters(void);
void increment_seq_counter(unsigned short* packetSequenceControl);
void getTime(unsigned char* time);
unsigned long long int getTimeAsUnsignedLongLongInt();

static inline void encode_temperatures(Packet_TM_LFR_HK_t* hk_packet)
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

    hk_packet->hk_lfr_temp_scm[0] = temp_scm_ptr[2];
    hk_packet->hk_lfr_temp_scm[1] = temp_scm_ptr[3];
    hk_packet->hk_lfr_temp_pcb[0] = temp_pcb_ptr[2];
    hk_packet->hk_lfr_temp_pcb[1] = temp_pcb_ptr[3];
    hk_packet->hk_lfr_temp_fpga[0] = temp_fpga_ptr[2];
    hk_packet->hk_lfr_temp_fpga[1] = temp_fpga_ptr[3];
}

static inline void encode_f3_E_field(int16_t V, int16_t E1, int16_t E2, Packet_TM_LFR_HK_t* hk_packet)
{
    unsigned char* _v = (unsigned char*)&V;
    unsigned char* _e1 = (unsigned char*)&E1;
    unsigned char* _e2 = (unsigned char*)&E2;
    hk_packet->hk_lfr_sc_v_f3[0]=_v[0];
    hk_packet->hk_lfr_sc_v_f3[1]=_v[1];
    hk_packet->hk_lfr_sc_e1_f3[0]=_e1[0];
    hk_packet->hk_lfr_sc_e1_f3[1]=_e1[1];
    hk_packet->hk_lfr_sc_e2_f3[0]=_e2[0];
    hk_packet->hk_lfr_sc_e2_f3[1]=_e2[1];
}

void encode_cpu_load(Packet_TM_LFR_HK_t* hk_packet);
void set_hk_lfr_sc_potential_flag(bool state);
void set_sy_lfr_pas_filter_enabled(bool state);
void set_sy_lfr_watchdog_enabled(bool state);
void set_hk_lfr_calib_enable(bool state);
void set_hk_lfr_reset_cause(enum lfr_reset_cause_t lfr_reset_cause);
void hk_lfr_le_me_he_update();
void set_hk_lfr_time_not_synchro();
void update_hk_lfr_last_er_fields(unsigned int rid, unsigned char code);

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
#include "fsw_housekeeping.h"
#include "fsw_debug.h"
#include "fsw_globals.h"
#include "fsw_misc.h"
#include "lfr_cpu_usage_report.h"


void init_housekeeping_parameters(void)
{
    /** This function initialize the housekeeping_packet global variable with default values.
     *
     */

    unsigned char* parameters;
    unsigned char sizeOfHK;

    sizeOfHK = sizeof(Packet_TM_LFR_HK_t);

    parameters = (unsigned char*)&housekeeping_packet;

    for (int i = 0; i < sizeOfHK; i++)
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
    int delta = 0;

    if (newValue >= oldValue)
    {
        delta = newValue - oldValue;
    }
    else
    {
        delta = (MAX_OF(unsigned char) + 1 - oldValue) + newValue;
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


    old_hk_lfr_le = new_hk_lfr_le;
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

    // update the low severity error counter
    hk_lfr_le_update();

    // update the medium severity error counter
    hk_lfr_me_update();

    // update the high severity error counter
    // LFR has no high severity errors
    housekeeping_packet.hk_lfr_he_cnt[0] = 0;
    housekeeping_packet.hk_lfr_he_cnt[1] = 0;
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
                housekeeping_packet.hk_lfr_time_not_synchro
                    = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_time_not_synchro);
                update_hk_lfr_last_er_fields(RID_LE_LFR_TIME, CODE_NOT_SYNCHRO);
            }
            break;
        default:
            LFR_PRINTF(
                "in hk_lfr_time_not_synchro *** unexpected value for synchronizationBit = %d\n",
                synchronizationBit);
            break;
    }
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

void update_hk_lfr_last_er_fields(unsigned int rid, unsigned char code)
{
    const volatile unsigned char* const coarseTimePtr
        = (volatile unsigned char*)&time_management_regs->coarse_time;
    const volatile unsigned char* const fineTimePtr
        = (volatile unsigned char*)&time_management_regs->fine_time;

    housekeeping_packet.hk_lfr_last_er_rid[0] = (unsigned char)((rid & BYTE0_MASK) >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_last_er_rid[1] = (unsigned char)(rid & BYTE1_MASK);
    housekeeping_packet.hk_lfr_last_er_code = code;
    housekeeping_packet.hk_lfr_last_er_time[0] = coarseTimePtr[0];
    housekeeping_packet.hk_lfr_last_er_time[1] = coarseTimePtr[1];
    housekeeping_packet.hk_lfr_last_er_time[BYTE_2] = coarseTimePtr[BYTE_2];
    housekeeping_packet.hk_lfr_last_er_time[BYTE_3] = coarseTimePtr[BYTE_3];
    housekeeping_packet.hk_lfr_last_er_time[BYTE_4] = fineTimePtr[BYTE_2];
    housekeeping_packet.hk_lfr_last_er_time[BYTE_5] = fineTimePtr[BYTE_3];
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


/**
 * @brief get_cpu_load, computes CPU load, CPU load average and CPU load max
 * @param resource_statistics stores:
 *          - CPU load at index 0
 *          - CPU load max at index 1
 *          - CPU load average at index 2
 *
 * The CPU load average is computed on the last 60 values with a simple moving average.
 */
void encode_cpu_load(Packet_TM_LFR_HK_t* hk_packet)
{
#define LOAD_AVG_SIZE 60
    static unsigned char cpu_load_hist[LOAD_AVG_SIZE] = { 0 };
    static char old_avg_pos = 0;
    static unsigned int cpu_load_avg;
    unsigned char cpu_load;

    cpu_load = lfr_rtems_cpu_usage_report();

    // HK_LFR_CPU_LOAD
    hk_packet->hk_lfr_cpu_load = cpu_load;

    // HK_LFR_CPU_LOAD_MAX
    if (cpu_load > hk_packet->hk_lfr_cpu_load_max)
    {
        hk_packet->hk_lfr_cpu_load_max = cpu_load;
    }

    cpu_load_avg
        = cpu_load_avg - (unsigned int)cpu_load_hist[(int)old_avg_pos] + (unsigned int)cpu_load;
    cpu_load_hist[(int)old_avg_pos] = cpu_load;
    old_avg_pos += 1;
    old_avg_pos %= LOAD_AVG_SIZE;
    // CPU_LOAD_AVE
    hk_packet->hk_lfr_cpu_load_aver = (unsigned char)(cpu_load_avg / LOAD_AVG_SIZE);
// this will change the way LFR compute usage
#ifndef PRINT_TASK_STATISTICS
    rtems_cpu_usage_reset();
#endif
}

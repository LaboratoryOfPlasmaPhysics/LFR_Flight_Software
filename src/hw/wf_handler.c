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

/** Functions and tasks related to waveform packet generation.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle waveforms, in snapshot or continuous format.\n
 *
 */

#include "hw/wf_handler.h"
#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_misc.h"
#include "hw/lfr_regs.h"
#include "lfr_common_headers/fsw_params.h"

//***************
// waveform rings
// F0
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node waveform_ring_f0[NB_RING_NODES_F0] = { { 0 } };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node* current_ring_node_f0 = NULL;
ring_node* ring_node_to_send_swf_f0 = NULL;
// F1
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node waveform_ring_f1[NB_RING_NODES_F1] = { { 0 } };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node* current_ring_node_f1 = NULL;
ring_node* ring_node_to_send_swf_f1 = NULL;
ring_node* ring_node_to_send_cwf_f1 = NULL;
// F2
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node waveform_ring_f2[NB_RING_NODES_F2] = { { 0 } };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node* current_ring_node_f2 = NULL;
ring_node* ring_node_to_send_swf_f2 = NULL;
ring_node* ring_node_to_send_cwf_f2 = NULL;
// F3
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node waveform_ring_f3[NB_RING_NODES_F3] = { { 0 } };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node* current_ring_node_f3 = NULL;
ring_node* ring_node_to_send_cwf_f3 = NULL;
char wf_cont_f3_light[NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_CWF3_LIGHT_BLK] = { 0 };

bool extractSWF1 = false;
bool extractSWF2 = false;
bool swf0_ready_flag_f1 = false;
bool swf0_ready_flag_f2 = false;
bool swf1_ready = false;
bool swf2_ready = false;

int swf1_extracted[(NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK)] = { 0 };
int swf2_extracted[(NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK)] = { 0 };

DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node ring_node_swf1_extracted = { 0 };
ring_node ring_node_swf2_extracted = { 0 };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

typedef enum resynchro_state_t
{
    MEASURE,
    CORRECTION
} resynchro_state;

//*********************
// Interrupt SubRoutine

ring_node* getRingNodeToSendCWF(unsigned char frequencyChannel)
{
    ring_node* node;

    node = NULL;
    switch (frequencyChannel)
    {
        case CHANNELF1:
            node = ring_node_to_send_cwf_f1;
            break;
        case CHANNELF2:
            node = ring_node_to_send_cwf_f2;
            break;
        case CHANNELF3:
            node = ring_node_to_send_cwf_f3;
            break;
        default:
            break;
    }

    return node;
}

ring_node* getRingNodeToSendSWF(unsigned char frequencyChannel)
{
    ring_node* node;

    node = NULL;
    switch (frequencyChannel)
    {
        case CHANNELF0:
            node = ring_node_to_send_swf_f0;
            break;
        case CHANNELF1:
            node = ring_node_to_send_swf_f1;
            break;
        case CHANNELF2:
            node = ring_node_to_send_swf_f2;
            break;
        default:
            break;
    }

    return node;
}

void reset_extractSWF(void)
{
    extractSWF1 = false;
    extractSWF2 = false;
    swf0_ready_flag_f1 = false;
    swf0_ready_flag_f2 = false;
    swf1_ready = false;
    swf2_ready = false;
}

void waveforms_isr_f3(void)
{
    if ((lfrCurrentMode == LFR_MODE_NORMAL)
        || (lfrCurrentMode
            == LFR_MODE_BURST) // in BURST the data are used to place v, e1 and e2 in the HK packet
        || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2))
    { // in modes other than STANDBY and BURST, send the CWF_F3 data
        //***
        // F3
        if ((waveform_picker_regs->status & BITS_WFP_STATUS_F3) != INIT_CHAR)
        { // [1100 0000] check the f3 full bits
            ring_node_to_send_cwf_f3 = current_ring_node_f3->previous;
            current_ring_node_f3 = current_ring_node_f3->next;
            if ((waveform_picker_regs->status & BIT_WFP_BUF_F3_0) == BIT_WFP_BUF_F3_0)
            { // [0100 0000] f3 buffer 0 is full
                ring_node_to_send_cwf_f3->coarseTime = waveform_picker_regs->f3_0_coarse_time;
                ring_node_to_send_cwf_f3->fineTime = waveform_picker_regs->f3_0_fine_time;
                waveform_picker_regs->addr_data_f3_0 = current_ring_node_f3->buffer_address;
                waveform_picker_regs->status
                    = waveform_picker_regs->status & RST_WFP_F3_0; // [1000 1000 0100 0000]
            }
            else if ((waveform_picker_regs->status & BIT_WFP_BUF_F3_1) == BIT_WFP_BUF_F3_1)
            { // [1000 0000] f3 buffer 1 is full
                ring_node_to_send_cwf_f3->coarseTime = waveform_picker_regs->f3_1_coarse_time;
                ring_node_to_send_cwf_f3->fineTime = waveform_picker_regs->f3_1_fine_time;
                waveform_picker_regs->addr_data_f3_1 = current_ring_node_f3->buffer_address;
                waveform_picker_regs->status
                    = waveform_picker_regs->status & RST_WFP_F3_1; // [1000 1000 1000 0000]
            }
            if (rtems_event_send(Task_id[TASKID_CWF3], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
            {
                DEBUG_CHECK_STATUS(send_event_dumb_task(RTEMS_EVENT_0));
            }
        }
    }
}

void waveforms_isr_burst(void)
{
    unsigned char status;

    status = (waveform_picker_regs->status & BITS_WFP_STATUS_F2)
        >> SHIFT_WFP_STATUS_F2; // [0011 0000] get the status bits for f2

    switch (status)
    {
        case BIT_WFP_BUFFER_0:
            ring_node_to_send_cwf_f2 = current_ring_node_f2->previous;
            ring_node_to_send_cwf_f2->packet_id = SID_BURST_CWF_F2;
            ring_node_to_send_cwf_f2->coarseTime = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_cwf_f2->fineTime = waveform_picker_regs->f2_0_fine_time;
            current_ring_node_f2 = current_ring_node_f2->next;
            waveform_picker_regs->addr_data_f2_0 = current_ring_node_f2->buffer_address;
            if (rtems_event_send(Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST) != RTEMS_SUCCESSFUL)
            {
                DEBUG_CHECK_STATUS(send_event_dumb_task(RTEMS_EVENT_0));
            }
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F2_0; // [0100 0100 0001 0000]
            break;
        case BIT_WFP_BUFFER_1:
            ring_node_to_send_cwf_f2 = current_ring_node_f2->previous;
            ring_node_to_send_cwf_f2->packet_id = SID_BURST_CWF_F2;
            ring_node_to_send_cwf_f2->coarseTime = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_cwf_f2->fineTime = waveform_picker_regs->f2_1_fine_time;
            current_ring_node_f2 = current_ring_node_f2->next;
            waveform_picker_regs->addr_data_f2_1 = current_ring_node_f2->buffer_address;
            if (rtems_event_send(Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST) != RTEMS_SUCCESSFUL)
            {
                DEBUG_CHECK_STATUS(send_event_dumb_task(RTEMS_EVENT_0));
            }
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F2_1; // [0100 0100 0010 0000]
            break;
        default:
            break;
    }
}

void waveform_isr_normal_sbm1_sbm2(void)
{
    //***
    // F0
    if ((waveform_picker_regs->status & BITS_WFP_STATUS_F0)
        != INIT_CHAR) // [0000 0011] check the f0 full bits
    {
        swf0_ready_flag_f1 = true;
        swf0_ready_flag_f2 = true;
        ring_node_to_send_swf_f0 = current_ring_node_f0->previous;
        current_ring_node_f0 = current_ring_node_f0->next;
        if ((waveform_picker_regs->status & BIT_WFP_BUFFER_0) == BIT_WFP_BUFFER_0)
        {

            ring_node_to_send_swf_f0->coarseTime = waveform_picker_regs->f0_0_coarse_time;
            ring_node_to_send_swf_f0->fineTime = waveform_picker_regs->f0_0_fine_time;
            waveform_picker_regs->addr_data_f0_0 = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F0_0; // [0001 0001 0000 0001]
        }
        else if ((waveform_picker_regs->status & BIT_WFP_BUFFER_1) == BIT_WFP_BUFFER_1)
        {
            ring_node_to_send_swf_f0->coarseTime = waveform_picker_regs->f0_1_coarse_time;
            ring_node_to_send_swf_f0->fineTime = waveform_picker_regs->f0_1_fine_time;
            waveform_picker_regs->addr_data_f0_1 = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F0_1; // [0001 0001 0000 0010]
        }
        // send an event to the WFRM task for resynchro activities
        DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_WFRM], RTEMS_EVENT_SWF_RESYNCH));
        DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_CALI], RTEMS_EVENT_CAL_SWEEP_WAKE));
    }

    //***
    // F1
    if ((waveform_picker_regs->status & BITS_WFP_STATUS_F1) != INIT_CHAR)
    { // [0000 1100] check the f1 full bits
        // (1) change the receiving buffer for the waveform picker
        ring_node_to_send_cwf_f1 = current_ring_node_f1->previous;
        current_ring_node_f1 = current_ring_node_f1->next;
        if ((waveform_picker_regs->status & BIT_WFP_BUF_F1_0) == BIT_WFP_BUF_F1_0)
        {
            ring_node_to_send_cwf_f1->coarseTime = waveform_picker_regs->f1_0_coarse_time;
            ring_node_to_send_cwf_f1->fineTime = waveform_picker_regs->f1_0_fine_time;
            waveform_picker_regs->addr_data_f1_0 = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F1_0; // [0010 0010 0000 0100] f1 bits = 0
        }
        else if ((waveform_picker_regs->status & BIT_WFP_BUF_F1_1) == BIT_WFP_BUF_F1_1)
        {
            ring_node_to_send_cwf_f1->coarseTime = waveform_picker_regs->f1_1_coarse_time;
            ring_node_to_send_cwf_f1->fineTime = waveform_picker_regs->f1_1_fine_time;
            waveform_picker_regs->addr_data_f1_1 = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F1_1; // [0010 0010 0000 1000] f1 bits = 0
        }
        // (2) send an event for the the CWF1 task for transmission (and snapshot extraction if
        // needed)
        DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_CWF1], RTEMS_EVENT_MODE_NORM_S1_S2));
    }

    //***
    // F2
    if ((waveform_picker_regs->status & BITS_WFP_STATUS_F2) != INIT_CHAR)
    { // [0011 0000] check the f2 full bit
        // (1) change the receiving buffer for the waveform picker
        ring_node_to_send_cwf_f2 = current_ring_node_f2->previous;
        ring_node_to_send_cwf_f2->packet_id = SID_SBM2_CWF_F2;
        current_ring_node_f2 = current_ring_node_f2->next;
        if ((waveform_picker_regs->status & BIT_WFP_BUF_F2_0) == BIT_WFP_BUF_F2_0)
        {
            ring_node_to_send_cwf_f2->coarseTime = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_cwf_f2->fineTime = waveform_picker_regs->f2_0_fine_time;
            waveform_picker_regs->addr_data_f2_0 = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F2_0; // [0100 0100 0001 0000]
        }
        else if ((waveform_picker_regs->status & BIT_WFP_BUF_F2_1) == BIT_WFP_BUF_F2_1)
        {
            ring_node_to_send_cwf_f2->coarseTime = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_cwf_f2->fineTime = waveform_picker_regs->f2_1_fine_time;
            waveform_picker_regs->addr_data_f2_1 = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status
                = waveform_picker_regs->status & RST_WFP_F2_1; // [0100 0100 0010 0000]
        }
        // (2) send an event for the waveforms transmission
        DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_NORM_S1_S2));
    }
}

rtems_isr waveforms_isr(rtems_vector_number vector)
{
    /** This is the interrupt sub routine called by the waveform picker core.
     *
     * This ISR launch different actions depending mainly on two pieces of information:
     * 1. the values read in the registers of the waveform picker.
     * 2. the current LFR mode.
     *
     */

    // STATUS
    // new error        error buffer full
    // 15 14 13 12      11 10 9  8
    // f3 f2 f1 f0      f3 f2 f1 f0
    //
    // ready buffer
    // 7    6    5    4    3    2    1    0
    // f3_1 f3_0 f2_1 f2_0 f1_1 f1_0 f0_1 f0_0
    IGNORE_UNUSED_PARAMETER(vector);

    waveforms_isr_f3();

    //*************************************************
    // copy the status bits in the housekeeping packets
    housekeeping_packet.hk_lfr_vhdl_iir_cal
        = (unsigned char)((waveform_picker_regs->status & BYTE0_MASK) >> SHIFT_1_BYTE);

    if ((waveform_picker_regs->status & BYTE0_MASK)
        != INIT_CHAR) // [1111 1111 0000 0000] check the error bits
    {
        DEBUG_CHECK_STATUS(send_event_dumb_task(RTEMS_EVENT_10));
    }

    switch (lfrCurrentMode)
    {
        //********
        // STANDBY
        case LFR_MODE_STANDBY:
            break;
            //**************************
            // LFR NORMAL, SBM1 and SBM2
        case LFR_MODE_NORMAL:
        case LFR_MODE_SBM1:
        case LFR_MODE_SBM2:
            waveform_isr_normal_sbm1_sbm2();
            break;
            //******
            // BURST
        case LFR_MODE_BURST:
            waveforms_isr_burst();
            break;
            //********
            // DEFAULT
        default:
            break;
    }
}

//************
// RTEMS TASKS

LFR_NO_RETURN rtems_task wfrm_task(
    rtems_task_argument argument) // used with the waveform picker VHDL IP
{
    /** This RTEMS task is dedicated to the transmission of snapshots of the NORMAL mode.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packets are sent by this task:
     * - TM_LFR_SCIENCE_NORMAL_SWF_F0
     * - TM_LFR_SCIENCE_NORMAL_SWF_F1
     * - TM_LFR_SCIENCE_NORMAL_SWF_F2
     *
     */

    IGNORE_UNUSED_PARAMETER(argument);

    rtems_event_set event_out = EVENT_SETS_NONE_PENDING;
    rtems_id queue_id = RTEMS_ID_NONE;
    ring_node* ring_node_swf1_extracted_ptr;
    ring_node* ring_node_swf2_extracted_ptr;

    ring_node_swf1_extracted_ptr = &ring_node_swf1_extracted;
    ring_node_swf2_extracted_ptr = &ring_node_swf2_extracted;

    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_id));

    BOOT_PRINTF("in WFRM ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_MODE_NORMAL | RTEMS_EVENT_SWF_RESYNCH,
            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);

        if (event_out == RTEMS_EVENT_MODE_NORMAL)
        {
            DEBUG_PRINTF("WFRM received RTEMS_EVENT_MODE_SBM2\n");
            ring_node_to_send_swf_f0->packet_id = SID_NORM_SWF_F0;
            ring_node_swf1_extracted_ptr->packet_id = SID_NORM_SWF_F1;
            ring_node_swf2_extracted_ptr->packet_id = SID_NORM_SWF_F2;
            DEBUG_CHECK_STATUS(
                rtems_message_queue_send(queue_id, &ring_node_to_send_swf_f0, sizeof(ring_node*)));
            DEBUG_CHECK_STATUS(rtems_message_queue_send(
                queue_id, &ring_node_swf1_extracted_ptr, sizeof(ring_node*)));
            DEBUG_CHECK_STATUS(rtems_message_queue_send(
                queue_id, &ring_node_swf2_extracted_ptr, sizeof(ring_node*)));
        }
        if (event_out == RTEMS_EVENT_SWF_RESYNCH)
        {
            snapshot_resynchronization((unsigned char*)&ring_node_to_send_swf_f0->coarseTime);
        }
    }
}

LFR_NO_RETURN rtems_task cwf3_task(
    rtems_task_argument argument) // used with the waveform picker VHDL IP
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f3.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this task:
     * - TM_LFR_SCIENCE_NORMAL_CWF_F3
     *
     */

    IGNORE_UNUSED_PARAMETER(argument);

    rtems_event_set event_out = EVENT_SETS_NONE_PENDING;
    rtems_id queue_id = RTEMS_ID_NONE;
    ring_node ring_node_cwf3_light;
    ring_node* ring_node_to_send_cwf;


    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_id));

    ring_node_to_send_cwf_f3->packet_id = SID_NORM_CWF_LONG_F3;

    // init the ring_node_cwf3_light structure
    ring_node_cwf3_light.buffer_address = wf_cont_f3_light;
    ring_node_cwf3_light.coarseTime = INIT_CHAR;
    ring_node_cwf3_light.fineTime = INIT_CHAR;
    ring_node_cwf3_light.next = NULL;
    ring_node_cwf3_light.previous = NULL;
    ring_node_cwf3_light.packet_id = SID_NORM_CWF_F3;
    ring_node_cwf3_light.status = INIT_CHAR;

    BOOT_PRINTF("in CWF3 ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive(
            RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if ((lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
            || (lfrCurrentMode == LFR_MODE_SBM2))
        {
            ring_node_to_send_cwf = getRingNodeToSendCWF(CHANNELF3);
            if ((parameter_dump_packet.sy_lfr_n_cwf_long_f3 & BIT_CWF_LONG_F3) == BIT_CWF_LONG_F3)
            {
                LFR_PRINTF("send CWF_LONG_F3\n");
                ring_node_to_send_cwf_f3->packet_id = SID_NORM_CWF_LONG_F3;
                DEBUG_CHECK_STATUS(
                    rtems_message_queue_send(queue_id, &ring_node_to_send_cwf, sizeof(ring_node*)));
            }
            else
            {
                LFR_PRINTF("send CWF_F3 (light)\n");
                send_waveform_CWF3_light(ring_node_to_send_cwf, &ring_node_cwf3_light, queue_id);
            }
        }
        else
        {
            LFR_PRINTF("in CWF3 *** lfrCurrentMode is %d, no data will be sent\n", lfrCurrentMode);
        }
    }
}

LFR_NO_RETURN rtems_task cwf2_task(rtems_task_argument argument) // ONLY USED IN BURST AND SBM2
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f2.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this function:
     * - TM_LFR_SCIENCE_BURST_CWF_F2
     * - TM_LFR_SCIENCE_SBM2_CWF_F2
     *
     */

    IGNORE_UNUSED_PARAMETER(argument);

    rtems_event_set event_out = EVENT_SETS_NONE_PENDING;
    rtems_id queue_id = RTEMS_ID_NONE;
    ring_node* ring_node_to_send;
    unsigned long long int acquisitionTimeF0_asLong;

    acquisitionTimeF0_asLong = INIT_CHAR;

    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_id));

    BOOT_PRINTF("in CWF2 ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT// send the snapshot when built
        DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2));
        DEBUG_CHECK_STATUS(rtems_event_receive(RTEMS_EVENT_MODE_NORM_S1_S2 | RTEMS_EVENT_MODE_BURST,
            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out));
        ring_node_to_send = getRingNodeToSendCWF(CHANNELF2);
        if (event_out == RTEMS_EVENT_MODE_BURST)
        { // data are sent whatever the transition time
            DEBUG_CHECK_STATUS(
                rtems_message_queue_send(queue_id, &ring_node_to_send, sizeof(ring_node*)));
        }
        else if (event_out == RTEMS_EVENT_MODE_NORM_S1_S2)
        {
            if (lfrCurrentMode == LFR_MODE_SBM2
                && time_management_regs->coarse_time >= lastValidEnterModeTime)
            {
                DEBUG_CHECK_STATUS(
                    rtems_message_queue_send(queue_id, &ring_node_to_send, sizeof(ring_node*)));
            }
            // launch snapshot extraction if needed
            if (extractSWF2 == true)
            {
                ring_node_to_send_swf_f2 = ring_node_to_send_cwf_f2;
                // extract the snapshot
                build_snapshot_from_ring(ring_node_to_send_swf_f2, CHANNELF2,
                    acquisitionTimeF0_asLong, &ring_node_swf2_extracted, swf2_extracted);
                extractSWF2 = false;
                swf2_ready = true; // once the snapshot at f2 is ready the CWF1 task will send an
                                   // event to WFRM
            }
            if (swf0_ready_flag_f2 == true)
            {
                extractSWF2 = true;
                // record the acquition time of the f0 snapshot to use to build the snapshot at f2
                acquisitionTimeF0_asLong
                    = get_acquisition_time((unsigned char*)&ring_node_to_send_swf_f0->coarseTime);
                swf0_ready_flag_f2 = false;
            }
        }
    }
}

LFR_NO_RETURN rtems_task cwf1_task(rtems_task_argument argument) // ONLY USED IN SBM1
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f1.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this function:
     * - TM_LFR_SCIENCE_SBM1_CWF_F1
     *
     */

    IGNORE_UNUSED_PARAMETER(argument);

    rtems_event_set event_out = EVENT_SETS_NONE_PENDING;
    rtems_id queue_id = RTEMS_ID_NONE;

    ring_node* ring_node_to_send_cwf;

    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_id));

    BOOT_PRINTF("in CWF1 ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_MODE_NORM_S1_S2, RTEMS_WAIT | RTEMS_EVENT_ANY,
            RTEMS_NO_TIMEOUT, &event_out);
        ring_node_to_send_cwf = getRingNodeToSendCWF(1);
        ring_node_to_send_cwf_f1->packet_id = SID_SBM1_CWF_F1;
        if (lfrCurrentMode == LFR_MODE_SBM1
            && time_management_regs->coarse_time >= lastValidEnterModeTime)
        {
            DEBUG_CHECK_STATUS(
                rtems_message_queue_send(queue_id, &ring_node_to_send_cwf, sizeof(ring_node*)));
        }
        // launch snapshot extraction if needed
        if (extractSWF1 == true)
        {
            ring_node_to_send_swf_f1 = ring_node_to_send_cwf;
            // launch the snapshot extraction
            DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_SWBD], RTEMS_EVENT_MODE_NORM_S1_S2));
            extractSWF1 = false;
        }
        if (swf0_ready_flag_f1 == true)
        {
            extractSWF1 = true;
            swf0_ready_flag_f1 = false; // this step shall be executed only one time
        }
        if ((swf1_ready == true) && (swf2_ready == true)) // swf_f1 is ready after the extraction
        {
            DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL));
            swf1_ready = false;
            swf2_ready = false;
        }
    }
}

LFR_NO_RETURN rtems_task swbd_task(rtems_task_argument argument)
{
    /** This RTEMS task is dedicated to the building of snapshots from different continuous
     * waveforms buffers.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     */

    IGNORE_UNUSED_PARAMETER(argument);

    rtems_event_set event_out = EVENT_SETS_NONE_PENDING;
    unsigned long long int acquisitionTimeF0_asLong = 0;

    BOOT_PRINTF("in SWBD ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_MODE_NORM_S1_S2, RTEMS_WAIT | RTEMS_EVENT_ANY,
            RTEMS_NO_TIMEOUT, &event_out);
        if (event_out == RTEMS_EVENT_MODE_NORM_S1_S2)
        {
            acquisitionTimeF0_asLong
                = get_acquisition_time((unsigned char*)&ring_node_to_send_swf_f0->coarseTime);
            build_snapshot_from_ring(ring_node_to_send_swf_f1, CHANNELF1, acquisitionTimeF0_asLong,
                &ring_node_swf1_extracted, swf1_extracted);
            swf1_ready = true; // the snapshot has been extracted and is ready to be sent
        }
        else
        {
            LFR_PRINTF("in SWBD *** unexpected rtems event received %x\n", (int)event_out);
        }
    }
}

//******************
// general functions

void WFP_init_rings(void)
{
    // F0 RING
    init_ring(waveform_ring_f0, NB_RING_NODES_F0, wf_buffer_f0, WFRM_BUFFER);
    // F1 RING
    init_ring(waveform_ring_f1, NB_RING_NODES_F1, wf_buffer_f1, WFRM_BUFFER);
    // F2 RING
    init_ring(waveform_ring_f2, NB_RING_NODES_F2, wf_buffer_f2, WFRM_BUFFER);
    // F3 RING
    init_ring(waveform_ring_f3, NB_RING_NODES_F3, wf_buffer_f3, WFRM_BUFFER);

    ring_node_swf1_extracted.buffer_address = swf1_extracted;
    ring_node_swf2_extracted.buffer_address = swf2_extracted;

    DEBUG_PRINTF("waveform_ring_f0 @%x\n", (unsigned int)waveform_ring_f0);
    DEBUG_PRINTF("waveform_ring_f1 @%x\n", (unsigned int)waveform_ring_f1);
    DEBUG_PRINTF("waveform_ring_f2 @%x\n", (unsigned int)waveform_ring_f2);
    DEBUG_PRINTF("waveform_ring_f3 @%x\n", (unsigned int)waveform_ring_f3);
    DEBUG_PRINTF("wf_buffer_f0 @%x\n", (unsigned int)wf_buffer_f0);
    DEBUG_PRINTF("wf_buffer_f1 @%x\n", (unsigned int)wf_buffer_f1);
    DEBUG_PRINTF("wf_buffer_f2 @%x\n", (unsigned int)wf_buffer_f2);
    DEBUG_PRINTF("wf_buffer_f3 @%x\n", (unsigned int)wf_buffer_f3);
}

void WFP_reset_current_ring_nodes(void)
{
    current_ring_node_f0 = waveform_ring_f0[0].next;
    current_ring_node_f1 = waveform_ring_f1[0].next;
    current_ring_node_f2 = waveform_ring_f2[0].next;
    current_ring_node_f3 = waveform_ring_f3[0].next;

    ring_node_to_send_swf_f0 = waveform_ring_f0;
    ring_node_to_send_swf_f1 = waveform_ring_f1;
    ring_node_to_send_swf_f2 = waveform_ring_f2;

    ring_node_to_send_cwf_f1 = waveform_ring_f1;
    ring_node_to_send_cwf_f2 = waveform_ring_f2;
    ring_node_to_send_cwf_f3 = waveform_ring_f3;
}

int send_waveform_CWF3_light(
    ring_node* ring_node_to_send, ring_node* ring_node_cwf3_light, rtems_id queue_id)
{
    /** This function sends CWF_F3 CCSDS packets without the b1, b2 and b3 data.
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param headerCWF points to a table of headers that have been prepared for the data
     * transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be
     * send. The structures contain information to setup the transmission of the data packets.
     *
     * By default, CWF_F3 packet are send without the b1, b2 and b3 data. This function rebuilds a
     * data buffer from the incoming data and sends it in 7 packets, 6 containing 340 blocks and 1
     * one containing 8 blocks.
     *
     */

    int ret = LFR_SUCCESSFUL;
    rtems_status_code status;

    const char* sample;
    const int* const dataPtr = (int*)ring_node_to_send->buffer_address;

    ring_node_cwf3_light->coarseTime = ring_node_to_send->coarseTime;
    ring_node_cwf3_light->fineTime = ring_node_to_send->fineTime;

    //**********************
    // BUILD CWF3_light DATA
    for (unsigned int i = 0; i < NB_SAMPLES_PER_SNAPSHOT; i++)
    {
        sample = (const char*)&dataPtr[i * NB_WORDS_SWF_BLK];
        for (unsigned int j = 0; j < CWF_BLK_SIZE; j++)
        {
            wf_cont_f3_light[(i * NB_BYTES_CWF3_LIGHT_BLK) + j] = sample[j];
        }
    }

    // SEND PACKET
    status = rtems_message_queue_send(queue_id, &ring_node_cwf3_light, sizeof(ring_node*));
    if (status != RTEMS_SUCCESSFUL)
    {
        ret = LFR_DEFAULT;
    }

    return ret;
}

void compute_acquisition_time(unsigned int coarseTime, unsigned int fineTime, unsigned int sid,
    unsigned char pa_lfr_pkt_nr, unsigned char* acquisitionTime)
{
    unsigned long long int acquisitionTimeAsLong;
    unsigned char localAcquisitionTime[BYTES_PER_TIME];
    double deltaT = 0.;

    localAcquisitionTime[BYTE_0] = (unsigned char)(coarseTime >> SHIFT_3_BYTES);
    localAcquisitionTime[BYTE_1] = (unsigned char)(coarseTime >> SHIFT_2_BYTES);
    localAcquisitionTime[BYTE_2] = (unsigned char)(coarseTime >> SHIFT_1_BYTE);
    localAcquisitionTime[BYTE_3] = (unsigned char)(coarseTime);
    localAcquisitionTime[BYTE_4] = (unsigned char)(fineTime >> SHIFT_1_BYTE);
    localAcquisitionTime[BYTE_5] = (unsigned char)(fineTime);

    acquisitionTimeAsLong = ((unsigned long long int)localAcquisitionTime[BYTE_0] << SHIFT_5_BYTES)
        + ((unsigned long long int)localAcquisitionTime[BYTE_1] << SHIFT_4_BYTES)
        + ((unsigned long long int)localAcquisitionTime[BYTE_2] << SHIFT_3_BYTES)
        + ((unsigned long long int)localAcquisitionTime[BYTE_3] << SHIFT_2_BYTES)
        + ((unsigned long long int)localAcquisitionTime[BYTE_4] << SHIFT_1_BYTE)
        + ((unsigned long long int)localAcquisitionTime[BYTE_5]);

    switch (sid)
    {
        case SID_NORM_SWF_F0:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_304 * T0_IN_FINETIME;
            break;

        case SID_NORM_SWF_F1:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_304 * T1_IN_FINETIME;
            break;

        case SID_NORM_SWF_F2:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_304 * T2_IN_FINETIME;
            break;

        case SID_SBM1_CWF_F1:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_CWF * T1_IN_FINETIME;
            break;

        case SID_SBM2_CWF_F2:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_CWF * T2_IN_FINETIME;
            break;

        case SID_BURST_CWF_F2:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_CWF * T2_IN_FINETIME;
            break;

        case SID_NORM_CWF_F3:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_CWF_SHORT_F3 * T3_IN_FINETIME;
            break;

        case SID_NORM_CWF_LONG_F3:
            deltaT = ((double)(pa_lfr_pkt_nr)) * BLK_NR_CWF * T3_IN_FINETIME;
            break;

        default:
            LFR_PRINTF("in compute_acquisition_time *** ERR unexpected sid %d\n", sid);
            deltaT = 0.;
            break;
    }

    acquisitionTimeAsLong = acquisitionTimeAsLong + (unsigned long long int)deltaT;
    //
    acquisitionTime[BYTE_0] = (unsigned char)(acquisitionTimeAsLong >> SHIFT_5_BYTES);
    acquisitionTime[BYTE_1] = (unsigned char)(acquisitionTimeAsLong >> SHIFT_4_BYTES);
    acquisitionTime[BYTE_2] = (unsigned char)(acquisitionTimeAsLong >> SHIFT_3_BYTES);
    acquisitionTime[BYTE_3] = (unsigned char)(acquisitionTimeAsLong >> SHIFT_2_BYTES);
    acquisitionTime[BYTE_4] = (unsigned char)(acquisitionTimeAsLong >> SHIFT_1_BYTE);
    acquisitionTime[BYTE_5] = (unsigned char)(acquisitionTimeAsLong);
}

void build_snapshot_from_ring(ring_node* ring_node_to_send, unsigned char frequencyChannel,
    unsigned long long int acquisitionTimeF0_asLong, ring_node* ring_node_swf_extracted,
    int* swf_extracted)
{
    unsigned int i;
    unsigned int node;
    unsigned long long int centerTime_asLong;
    unsigned long long int acquisitionTime_asLong;
    unsigned long long int bufferAcquisitionTime_asLong;
    const unsigned char* ptr1;
    unsigned char* ptr2;
    const unsigned char* timeCharPtr;
    unsigned char nb_ring_nodes;
    unsigned long long int frequency_asLong;
    // set to default value (Don_Initialisation_P2)
    unsigned long long int nbTicksPerSample_asLong = TICKS_PER_T2;
    long long int nbSamplesPart1_asLong;
    unsigned long long int sampleOffset_asLong = 0;

    unsigned int deltaT_F0 = DELTAT_F0;
    unsigned int deltaT_F1 = DELTAT_F1;
    unsigned long long int deltaT_F2 = DELTAT_F2;

    // (1) get the f0 acquisition time => the value is passed in argument

    // (2) compute the central reference time
    centerTime_asLong = acquisitionTimeF0_asLong + deltaT_F0;
    acquisitionTime_asLong = centerTime_asLong; // set to default value (Don_Initialisation_P2)
    bufferAcquisitionTime_asLong = centerTime_asLong; // set to default value
                                                      // (Don_Initialisation_P2)

    // (3) compute the acquisition time of the current snapshot
    switch (frequencyChannel)
    {
        case CHANNELF1: // 1 is for F1 = 4096 Hz
            acquisitionTime_asLong = centerTime_asLong - deltaT_F1;
            nb_ring_nodes = NB_RING_NODES_F1;
            frequency_asLong = FREQ_F1;
            nbTicksPerSample_asLong = TICKS_PER_T1; // 65536 / 4096;
            break;
        case CHANNELF2: // 2 is for F2 = 256 Hz
            acquisitionTime_asLong = centerTime_asLong - deltaT_F2;
            nb_ring_nodes = NB_RING_NODES_F2;
            frequency_asLong = FREQ_F2;
            nbTicksPerSample_asLong = TICKS_PER_T2; // 65536 / 256;
            break;
        default:
            acquisitionTime_asLong = centerTime_asLong;
            nb_ring_nodes = 0;
            frequency_asLong = FREQ_F2;
            nbTicksPerSample_asLong = TICKS_PER_T2;
            break;
    }

    //*****************************************************************************
    // (4) search the ring_node with the acquisition time <= acquisitionTime_asLong
    node = 0;
    while (node < nb_ring_nodes)
    {
        bufferAcquisitionTime_asLong
            = get_acquisition_time((unsigned char*)&ring_node_to_send->coarseTime);
        if (bufferAcquisitionTime_asLong <= acquisitionTime_asLong)
        {
            node = nb_ring_nodes;
        }
        else
        {
            node = node + 1;
            ring_node_to_send = ring_node_to_send->previous;
        }
    }

    // (5) compute the number of samples to take in the current buffer
    sampleOffset_asLong
        = ((acquisitionTime_asLong - bufferAcquisitionTime_asLong) * frequency_asLong)
        >> SHIFT_2_BYTES;
    nbSamplesPart1_asLong = NB_SAMPLES_PER_SNAPSHOT - sampleOffset_asLong;

    // (6) compute the final acquisition time
    acquisitionTime_asLong
        = bufferAcquisitionTime_asLong + (sampleOffset_asLong * nbTicksPerSample_asLong);

    // (7) copy the acquisition time at the beginning of the extrated snapshot
    ptr1 = (unsigned char*)&acquisitionTime_asLong;
    // fine time
    ptr2 = (unsigned char*)&ring_node_swf_extracted->fineTime;
    ptr2[BYTE_2] = ptr1[BYTE_4 + OFFSET_2_BYTES];
    ptr2[BYTE_3] = ptr1[BYTE_5 + OFFSET_2_BYTES];
    // coarse time
    ptr2 = (unsigned char*)&ring_node_swf_extracted->coarseTime;
    ptr2[BYTE_0] = ptr1[BYTE_0 + OFFSET_2_BYTES];
    ptr2[BYTE_1] = ptr1[BYTE_1 + OFFSET_2_BYTES];
    ptr2[BYTE_2] = ptr1[BYTE_2 + OFFSET_2_BYTES];
    ptr2[BYTE_3] = ptr1[BYTE_3 + OFFSET_2_BYTES];

    // re set the synchronization bit
    timeCharPtr = (unsigned char*)&ring_node_to_send->coarseTime;
    ptr2[0] = ptr2[0] | (timeCharPtr[0] & SYNC_BIT); // [1000 0000]

    if ((nbSamplesPart1_asLong > NB_SAMPLES_PER_SNAPSHOT) | (nbSamplesPart1_asLong < 0))
    {
        nbSamplesPart1_asLong = 0;
    }
    // copy the part 1 of the snapshot in the extracted buffer
    for (i = 0; i < (nbSamplesPart1_asLong * NB_WORDS_SWF_BLK); i++)
    {
        swf_extracted[i] = ((
            int*)ring_node_to_send->buffer_address)[i + (sampleOffset_asLong * NB_WORDS_SWF_BLK)];
    }
    // copy the part 2 of the snapshot in the extracted buffer
    ring_node_to_send = ring_node_to_send->next;
    for (i = (nbSamplesPart1_asLong * NB_WORDS_SWF_BLK);
         i < (NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK); i++)
    {
        swf_extracted[i] = ((
            int*)ring_node_to_send->buffer_address)[i - (nbSamplesPart1_asLong * NB_WORDS_SWF_BLK)];
    }
}

double computeCorrection(unsigned char* timePtr)
{
    unsigned long long int acquisitionTime;
    unsigned long long int centerTime;
    unsigned long long int previousTick;
    unsigned long long int nextTick;
    unsigned long long int deltaPreviousTick;
    unsigned long long int deltaNextTick;
    double deltaPrevious_ms;
    double deltaNext_ms;
    double correctionInF2 = 0.; // set to default value (Don_Initialisation_P2)


    // get acquisition time in fine time ticks
    acquisitionTime = get_acquisition_time(timePtr);

    // compute center time
    centerTime = acquisitionTime + DELTAT_F0; // (2048. / 24576. / 2.) * 65536. = 2730.667;
    previousTick = centerTime - (centerTime & INT16_ALL_F);
    nextTick = previousTick + (unsigned long long)TICKS_PER_S;

    deltaPreviousTick = centerTime - previousTick;
    deltaNextTick = nextTick - centerTime;

    deltaPrevious_ms = (((double)deltaPreviousTick) / TICKS_PER_S) * MS_PER_S;
    deltaNext_ms = (((double)deltaNextTick) / TICKS_PER_S) * MS_PER_S;

    LFR_PRINTF(
        "    delta previous = %.3f ms, delta next = %.2f ms\n", deltaPrevious_ms, deltaNext_ms);

    // which tick is the closest?
    if (deltaPreviousTick > deltaNextTick)
    {
        // the snapshot center is just before the second => increase delta_snapshot
        correctionInF2 = +(deltaNext_ms * FREQ_F2 / MS_PER_S);
    }
    else
    {
        // the snapshot center is just after the second => decrease delta_snapshot
        correctionInF2 = -(deltaPrevious_ms * FREQ_F2 / MS_PER_S);
    }

    LFR_PRINTF("    correctionInF2 = %.2f\n", correctionInF2);

    return correctionInF2;
}

void applyCorrection(double correction)
{
    int correctionInt = 0;

    if (correction >= 0.)
    {
        if ((ONE_TICK_CORR_INTERVAL_0_MIN < correction)
            && (correction < ONE_TICK_CORR_INTERVAL_0_MAX))
        {
            correctionInt = ONE_TICK_CORR;
        }
        else
        {
            correctionInt = (int)((double)CORR_MULT * floor(correction));
        }
    }
    else
    {
        if ((ONE_TICK_CORR_INTERVAL_1_MIN < correction)
            && (correction < ONE_TICK_CORR_INTERVAL_1_MAX))
        {
            correctionInt = -ONE_TICK_CORR;
        }
        else
        {
            correctionInt = (int)((double)CORR_MULT * ceil(correction));
        }
    }
    waveform_picker_regs->delta_snapshot = waveform_picker_regs->delta_snapshot + correctionInt;
}

void snapshot_resynchronization(unsigned char* timePtr)
{
    /** This function compute a correction to apply on delta_snapshot.
     *
     *
     * @param timePtr is a pointer to the acquisition time of the snapshot being considered.
     *
     * @return void
     *
     */

    static double correction = 0.;
    static resynchro_state state = MEASURE;
    static unsigned int nbSnapshots = 0;

    switch (state)
    {

        case MEASURE:
            // ********
            LFR_PRINTF("MEASURE === %d\n", nbSnapshots);
            state = CORRECTION;
            correction = computeCorrection(timePtr);
            LFR_PRINTF("MEASURE === correction = %.2f\n", correction);
            applyCorrection(correction);
            LFR_PRINTF("MEASURE === delta_snapshot = %u\n",
                (unsigned int)waveform_picker_regs->delta_snapshot);
            //****
            break;

        case CORRECTION:
            //************
            LFR_PRINTF("CORRECTION === %u\n", nbSnapshots);
            state = MEASURE;
            computeCorrection(timePtr);
            set_wfp_delta_snapshot();
            LFR_PRINTF("CORRECTION === delta_snapshot = %u\n",
                (unsigned int)waveform_picker_regs->delta_snapshot);
            //****
            break;

        default:
            break;
    }

    nbSnapshots++;
}

//**************
// wfp registers
void reset_wfp_burst_enable(void)
{
    /** This function resets the waveform picker burst_enable register.
     *
     * The burst bits [f2 f1 f0] and the enable bits [f3 f2 f1 f0] are set to 0.
     *
     */

    // [1000 000] burst f2, f1, f0     enable f3, f2, f1, f0
    waveform_picker_regs->run_burst_enable
        = waveform_picker_regs->run_burst_enable & RST_BITS_RUN_BURST_EN;
}

void reset_wfp_status(void)
{
    /** This function resets the waveform picker status register.
     *
     * All status bits are set to 0 [new_err full_err full].
     *
     */

    waveform_picker_regs->status = INT16_ALL_F;
}

void reset_wfp_buffer_addresses(void)
{
    // F0
    waveform_picker_regs->addr_data_f0_0 = current_ring_node_f0->previous->buffer_address; // 0x08
    waveform_picker_regs->addr_data_f0_1 = current_ring_node_f0->buffer_address; // 0x0c
    // F1
    waveform_picker_regs->addr_data_f1_0 = current_ring_node_f1->previous->buffer_address; // 0x10
    waveform_picker_regs->addr_data_f1_1 = current_ring_node_f1->buffer_address; // 0x14
    // F2
    waveform_picker_regs->addr_data_f2_0 = current_ring_node_f2->previous->buffer_address; // 0x18
    waveform_picker_regs->addr_data_f2_1 = current_ring_node_f2->buffer_address; // 0x1c
    // F3
    waveform_picker_regs->addr_data_f3_0 = current_ring_node_f3->previous->buffer_address; // 0x20
    waveform_picker_regs->addr_data_f3_1 = current_ring_node_f3->buffer_address; // 0x24
}

void reset_waveform_picker_regs(void)
{
    /** This function resets the waveform picker module registers.
     *
     * The registers affected by this function are located at the following offset addresses:
     * - 0x00 data_shaping
     * - 0x04 run_burst_enable
     * - 0x08 addr_data_f0
     * - 0x0C addr_data_f1
     * - 0x10 addr_data_f2
     * - 0x14 addr_data_f3
     * - 0x18 status
     * - 0x1C delta_snapshot
     * - 0x20 delta_f0
     * - 0x24 delta_f0_2
     * - 0x28 delta_f1 (obsolet parameter)
     * - 0x2c delta_f2
     * - 0x30 nb_data_by_buffer
     * - 0x34 nb_snapshot_param
     * - 0x38 start_date
     * - 0x3c nb_word_in_buffer
     *
     */

    set_wfp_data_shaping(); // 0x00 *** R1 R0 SP1 SP0 BW

    reset_wfp_burst_enable(); // 0x04 *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]

    reset_wfp_buffer_addresses();

    reset_wfp_status(); // 0x18

    set_wfp_delta_snapshot(); // 0x1c *** 300 s => 0x12bff

    set_wfp_delta_f0_f0_2(); // 0x20, 0x24

    // the parameter delta_f1 [0x28] is not used anymore

    set_wfp_delta_f2(); // 0x2c

    DEBUG_PRINTF("delta_snapshot %x\n", waveform_picker_regs->delta_snapshot);
    DEBUG_PRINTF("delta_f0 %x\n", waveform_picker_regs->delta_f0);
    DEBUG_PRINTF("delta_f0_2 %x\n", waveform_picker_regs->delta_f0_2);
    DEBUG_PRINTF("delta_f1 %x\n", waveform_picker_regs->delta_f1);
    DEBUG_PRINTF("delta_f2 %x\n", waveform_picker_regs->delta_f2);
    // 2688 = 8 * 336
    waveform_picker_regs->nb_data_by_buffer
        = DFLT_WFP_NB_DATA_BY_BUFFER; // 0x30 *** 2688 - 1 => nb samples -1
    waveform_picker_regs->snapshot_param = DFLT_WFP_SNAPSHOT_PARAM; // 0x34 *** 2688 => nb samples
    waveform_picker_regs->start_date = COARSE_TIME_MASK;
    //
    // coarse time and fine time registers are not initialized, they are volatile
    //
    waveform_picker_regs->buffer_length
        = DFLT_WFP_BUFFER_LENGTH; // buffer length in burst = 3 * 2688 / 16 = 504 = 0x1f8
}

void set_wfp_data_shaping(void)
{
    /** This function sets the data_shaping register of the waveform picker module.
     *
     * The value is read from one field of the parameter_dump_packet structure:\n
     * bw_sp0_sp1_r0_r1
     *
     */

    unsigned char data_shaping;

    // get the parameters for the data shaping [BW SP0 SP1 R0 R1] in sy_lfr_common1 and configure
    // the register waveform picker : [R1 R0 SP1 SP0 BW]

    data_shaping = parameter_dump_packet.sy_lfr_common_parameters;

    waveform_picker_regs->data_shaping = ((data_shaping & BIT_5) >> SHIFT_5_BITS) // BW
        + ((data_shaping & BIT_4) >> SHIFT_3_BITS) // SP0
        + ((data_shaping & BIT_3) >> 1) // SP1
        + ((data_shaping & BIT_2) << 1) // R0
        + ((data_shaping & BIT_1) << SHIFT_3_BITS) // R1
        + ((data_shaping & BIT_0) << SHIFT_5_BITS); // R2
}

void set_wfp_burst_enable_register(unsigned char mode)
{
    /** This function sets the waveform picker burst_enable register depending on the mode.
     *
     * @param mode is the LFR mode to launch.
     *
     * The burst bits shall be before the enable bits.
     *
     */

    // [0000 0000] burst f2, f1, f0 enable f3 f2 f1 f0
    // the burst bits shall be set first, before the enable bits
    switch (mode)
    {
        case LFR_MODE_NORMAL:
        case LFR_MODE_SBM1:
        case LFR_MODE_SBM2:
            waveform_picker_regs->run_burst_enable
                = RUN_BURST_ENABLE_SBM2; // [0110 0000] enable f2 and f1 burst
            waveform_picker_regs->run_burst_enable = waveform_picker_regs->run_burst_enable
                | BITS_WFP_ENABLE_ALL; // [1111] enable f3 f2 f1 f0
            break;
        case LFR_MODE_BURST:
            waveform_picker_regs->run_burst_enable
                = RUN_BURST_ENABLE_BURST; // [0100 0000] f2 burst enabled
            waveform_picker_regs->run_burst_enable = waveform_picker_regs->run_burst_enable
                | BITS_WFP_ENABLE_BURST; // [1100] enable f3 and f2
            break;
        default:
            waveform_picker_regs->run_burst_enable
                = INIT_CHAR; // [0000 0000] no burst enabled, no waveform enabled
            break;
    }
}

void set_wfp_delta_snapshot(void)
{
    /** This function sets the delta_snapshot register of the waveform picker module.
     *
     * The value is read from two (unsigned char) of the parameter_dump_packet structure:
     * - sy_lfr_n_swf_p[0]
     * - sy_lfr_n_swf_p[1]
     *
     */

    unsigned int delta_snapshot;
    unsigned int delta_snapshot_in_T2;

    delta_snapshot = (parameter_dump_packet.sy_lfr_n_swf_p[0] * CONST_256)
        + parameter_dump_packet.sy_lfr_n_swf_p[1];

    delta_snapshot_in_T2 = delta_snapshot * (unsigned int)FREQ_F2;
    waveform_picker_regs->delta_snapshot = delta_snapshot_in_T2 - 1; // max 4 bytes
}

void set_wfp_delta_f0_f0_2(void)
{
    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;
    float delta_f0_in_float;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = (parameter_dump_packet.sy_lfr_n_swf_l[0] * CONST_256)
        + parameter_dump_packet.sy_lfr_n_swf_l[1];
    delta_f0_in_float = (float)(((double)nb_samples_per_snapshot / 2.)
        * ((1. / FREQ_F2) - (1. / FREQ_F0)) * FREQ_F2);

    waveform_picker_regs->delta_f0 = delta_snapshot - (int)floor(delta_f0_in_float);
    waveform_picker_regs->delta_f0_2 = DFLT_WFP_DELTA_F0_2;
}

void set_wfp_delta_f1(void)
{
    /** Sets the value of the delta_f1 parameter
     *
     * @param void
     *
     * @return void
     *
     * delta_f1 is not used, the snapshots are extracted from CWF_F1 waveforms.
     *
     */

    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;
    float delta_f1_in_float;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = (parameter_dump_packet.sy_lfr_n_swf_l[0] * CONST_256)
        + parameter_dump_packet.sy_lfr_n_swf_l[1];
    delta_f1_in_float = (float)(((double)nb_samples_per_snapshot / 2.)
        * ((1. / FREQ_F2) - (1. / FREQ_F1)) * FREQ_F2);

    waveform_picker_regs->delta_f1 = delta_snapshot - (int)floor(delta_f1_in_float);
}

void set_wfp_delta_f2(void) // parameter not used, only delta_f0 and delta_f0_2 are used
{
    /** Sets the value of the delta_f2 parameter
     *
     * @param void
     *
     * @return void
     *
     * delta_f2 is used only for the first snapshot generation, even when the snapshots are
     * extracted from CWF_F2 waveforms (see lpp_waveform_snapshot_controler.vhd for details).
     *
     */

    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = (parameter_dump_packet.sy_lfr_n_swf_l[0] * CONST_256)
        + parameter_dump_packet.sy_lfr_n_swf_l[1];

    waveform_picker_regs->delta_f2 = delta_snapshot - (nb_samples_per_snapshot / 2) - 1;
}

//*****************
// local parameters

void increment_seq_counter_source_id(unsigned char* packet_sequence_control, unsigned char sid)
{
    /** This function increments the parameter "sequence_cnt" depending on the sid passed in
     * argument.
     *
     * @param packet_sequence_control is a pointer toward the parameter sequence_cnt to update.
     * @param sid is the source identifier of the packet being updated.
     *
     * REQ-LFR-SRS-5240 / SSS-CP-FS-590
     * The sequence counters shall wrap around from 2^14 to zero.
     * The sequence counter shall start at zero at startup.
     *
     * REQ-LFR-SRS-5239 / SSS-CP-FS-580
     * All TM_LFR_SCIENCE_ packets are sent to ground, i.e. destination id = 0
     *
     */

    unsigned short* sequence_cnt = NULL;
    unsigned short segmentation_grouping_flag;
    unsigned short new_packet_sequence_control;
    rtems_mode initial_mode_set = RTEMS_DEFAULT_MODES;
    rtems_mode current_mode_set = RTEMS_DEFAULT_MODES;

    //******************************************
    // CHANGE THE MODE OF THE CALLING RTEMS TASK
    DEBUG_CHECK_STATUS(rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &initial_mode_set));

    if ((sid == SID_NORM_SWF_F0) || (sid == SID_NORM_SWF_F1) || (sid == SID_NORM_SWF_F2)
        || (sid == SID_NORM_CWF_F3) || (sid == SID_NORM_CWF_LONG_F3) || (sid == SID_BURST_CWF_F2)
        || (sid == SID_NORM_ASM_F0) || (sid == SID_NORM_ASM_F1) || (sid == SID_NORM_ASM_F2)
        || (sid == SID_NORM_BP1_F0) || (sid == SID_NORM_BP1_F1) || (sid == SID_NORM_BP1_F2)
        || (sid == SID_NORM_BP2_F0) || (sid == SID_NORM_BP2_F1) || (sid == SID_NORM_BP2_F2)
        || (sid == SID_BURST_BP1_F0) || (sid == SID_BURST_BP2_F0) || (sid == SID_BURST_BP1_F1)
        || (sid == SID_BURST_BP2_F1))
    {
        sequence_cnt = &sequenceCounters_SCIENCE_NORMAL_BURST;
    }
    else if ((sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2) || (sid == SID_SBM1_BP1_F0)
        || (sid == SID_SBM1_BP2_F0) || (sid == SID_SBM2_BP1_F0) || (sid == SID_SBM2_BP2_F0)
        || (sid == SID_SBM2_BP1_F1) || (sid == SID_SBM2_BP2_F1))
    {
        sequence_cnt = &sequenceCounters_SCIENCE_SBM1_SBM2;
    }
    else
    {
        sequence_cnt = (unsigned short*)NULL;
        LFR_PRINTF(
            "in increment_seq_counter_source_id *** ERR apid_destid %d not known\n", (int)sid);
    }

    if (sequence_cnt != NULL)
    {
        segmentation_grouping_flag = TM_PACKET_SEQ_CTRL_STANDALONE << SHIFT_1_BYTE;
        *sequence_cnt = (*sequence_cnt) & SEQ_CNT_MASK;

        new_packet_sequence_control = segmentation_grouping_flag | (*sequence_cnt);

        packet_sequence_control[0] = (unsigned char)(new_packet_sequence_control >> SHIFT_1_BYTE);
        packet_sequence_control[1] = (unsigned char)(new_packet_sequence_control);

        // increment the sequence counter
        if (*sequence_cnt < SEQ_CNT_MAX)
        {
            *sequence_cnt = *sequence_cnt + 1;
        }
        else
        {
            *sequence_cnt = 0;
        }
    }

    //*************************************
    // RESTORE THE MODE OF THE CALLING TASK
    DEBUG_CHECK_STATUS(rtems_task_mode(initial_mode_set, RTEMS_PREEMPT_MASK, &current_mode_set));
}

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

/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic
 * parameters computation.
 *
 */

#include "processing/fsw_processing.h"
#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_init.h"
#include "fsw_processing_globals.c"
#include "hw/lfr_regs.h"

unsigned int nb_sm_f0 = 0;
unsigned int nb_sm_f0_aux_f1 = 0;
unsigned int nb_sm_f1 = 0;
unsigned int nb_sm_f0_aux_f2 = 0;

typedef enum restartState_t
{
    WAIT_FOR_F2,
    WAIT_FOR_F1,
    WAIT_FOR_F0
} restartState;

//************************
// spectral matrices rings
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node sm_ring_f0[NB_RING_NODES_SM_F0] = { { 0 } };
ring_node sm_ring_f1[NB_RING_NODES_SM_F1] = { { 0 } };
ring_node sm_ring_f2[NB_RING_NODES_SM_F2] = { { 0 } };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING
ring_node* current_ring_node_sm_f0 = NULL;
ring_node* current_ring_node_sm_f1 = NULL;
ring_node* current_ring_node_sm_f2 = NULL;
ring_node* ring_node_for_averaging_sm_f0 = NULL;
ring_node* ring_node_for_averaging_sm_f1 = NULL;
ring_node* ring_node_for_averaging_sm_f2 = NULL;

//
ring_node* getRingNodeForAveraging(unsigned char frequencyChannel)
{
    ring_node* node;

    node = NULL;
    switch (frequencyChannel)
    {
        case CHANNELF0:
            node = ring_node_for_averaging_sm_f0;
            break;
        case CHANNELF1:
            node = ring_node_for_averaging_sm_f1;
            break;
        case CHANNELF2:
            node = ring_node_for_averaging_sm_f2;
            break;
        default:
            break;
    }

    return node;
}

//***********************************************************
// Interrupt Service Routine for spectral matrices processing

void spectral_matrices_isr_f0(int statusReg)
{
    unsigned char status;
    rtems_status_code status_code;
    ring_node* full_ring_node;

    status = (unsigned char)(statusReg
        & BITS_STATUS_F0); // [0011] get the status_ready_matrix_f0_x bits

    switch (status)
    {
        case 0:
            break;
        case BIT_READY_0_1:
            // UNEXPECTED VALUE
            spectral_matrix_regs->status = BIT_READY_0_1; // [0011]
            status_code = send_event_dumb_task(RTEMS_EVENT_11);
            break;
        case BIT_READY_0:
            full_ring_node = current_ring_node_sm_f0->previous;
            full_ring_node->coarseTime = spectral_matrix_regs->f0_0_coarse_time;
            full_ring_node->fineTime = spectral_matrix_regs->f0_0_fine_time;
            current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
            spectral_matrix_regs->f0_0_address = current_ring_node_sm_f0->buffer_address;
            // if there are enough ring nodes ready, wake up an AVFx task
            nb_sm_f0 = nb_sm_f0 + 1;
            if (nb_sm_f0 == NB_SM_BEFORE_AVF0_F1)
            {
                ring_node_for_averaging_sm_f0 = full_ring_node;
                if (rtems_event_send(Task_id[TASKID_AVF0], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
                {
                    status_code = send_event_dumb_task(RTEMS_EVENT_3);
                }
                nb_sm_f0 = 0;
            }
            spectral_matrix_regs->status = BIT_READY_0; // [0000 0001]
            break;
        case BIT_READY_1:
            full_ring_node = current_ring_node_sm_f0->previous;
            full_ring_node->coarseTime = spectral_matrix_regs->f0_1_coarse_time;
            full_ring_node->fineTime = spectral_matrix_regs->f0_1_fine_time;
            current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
            spectral_matrix_regs->f0_1_address = current_ring_node_sm_f0->buffer_address;
            // if there are enough ring nodes ready, wake up an AVFx task
            nb_sm_f0 = nb_sm_f0 + 1;
            if (nb_sm_f0 == NB_SM_BEFORE_AVF0_F1)
            {
                ring_node_for_averaging_sm_f0 = full_ring_node;
                if (rtems_event_send(Task_id[TASKID_AVF0], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
                {
                    status_code = send_event_dumb_task(RTEMS_EVENT_3);
                }
                nb_sm_f0 = 0;
            }
            spectral_matrix_regs->status = BIT_READY_1; // [0000 0010]
            break;
        default:
            break;
    }
}

void spectral_matrices_isr_f1(int statusReg)
{
    rtems_status_code status_code;
    unsigned char status;
    ring_node* full_ring_node;

    status = (unsigned char)((statusReg & BITS_STATUS_F1)
        >> SHIFT_2_BITS); // [1100] get the status_ready_matrix_f1_x bits

    switch (status)
    {
        case 0:
            break;
        case BIT_READY_0_1:
            // UNEXPECTED VALUE
            spectral_matrix_regs->status = BITS_STATUS_F1; // [1100]
            status_code = send_event_dumb_task(RTEMS_EVENT_11);
            break;
        case BIT_READY_0:
            full_ring_node = current_ring_node_sm_f1->previous;
            full_ring_node->coarseTime = spectral_matrix_regs->f1_0_coarse_time;
            full_ring_node->fineTime = spectral_matrix_regs->f1_0_fine_time;
            current_ring_node_sm_f1 = current_ring_node_sm_f1->next;
            spectral_matrix_regs->f1_0_address = current_ring_node_sm_f1->buffer_address;
            // if there are enough ring nodes ready, wake up an AVFx task
            nb_sm_f1 = nb_sm_f1 + 1;
            if (nb_sm_f1 == NB_SM_BEFORE_AVF0_F1)
            {
                ring_node_for_averaging_sm_f1 = full_ring_node;
                if (rtems_event_send(Task_id[TASKID_AVF1], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
                {
                    status_code = send_event_dumb_task(RTEMS_EVENT_3);
                }
                nb_sm_f1 = 0;
            }
            spectral_matrix_regs->status = BIT_STATUS_F1_0; // [0000 0100]
            break;
        case BIT_READY_1:
            full_ring_node = current_ring_node_sm_f1->previous;
            full_ring_node->coarseTime = spectral_matrix_regs->f1_1_coarse_time;
            full_ring_node->fineTime = spectral_matrix_regs->f1_1_fine_time;
            current_ring_node_sm_f1 = current_ring_node_sm_f1->next;
            spectral_matrix_regs->f1_1_address = current_ring_node_sm_f1->buffer_address;
            // if there are enough ring nodes ready, wake up an AVFx task
            nb_sm_f1 = nb_sm_f1 + 1;
            if (nb_sm_f1 == NB_SM_BEFORE_AVF0_F1)
            {
                ring_node_for_averaging_sm_f1 = full_ring_node;
                if (rtems_event_send(Task_id[TASKID_AVF1], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
                {
                    status_code = send_event_dumb_task(RTEMS_EVENT_3);
                }
                nb_sm_f1 = 0;
            }
            spectral_matrix_regs->status = BIT_STATUS_F1_1; // [1000 0000]
            break;
        default:
            break;
    }
}

void spectral_matrices_isr_f2(int statusReg)
{
    unsigned char status;
    rtems_status_code status_code;

    status = (unsigned char)((statusReg & BITS_STATUS_F2)
        >> SHIFT_4_BITS); // [0011 0000] get the status_ready_matrix_f2_x bits

    switch (status)
    {
        case 0:
            break;
        case BIT_READY_0_1:
            // UNEXPECTED VALUE
            spectral_matrix_regs->status = BITS_STATUS_F2; // [0011 0000]
            status_code = send_event_dumb_task(RTEMS_EVENT_11);
            break;
        case BIT_READY_0:
            ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2->previous;
            current_ring_node_sm_f2 = current_ring_node_sm_f2->next;
            ring_node_for_averaging_sm_f2->coarseTime = spectral_matrix_regs->f2_0_coarse_time;
            ring_node_for_averaging_sm_f2->fineTime = spectral_matrix_regs->f2_0_fine_time;
            spectral_matrix_regs->f2_0_address = current_ring_node_sm_f2->buffer_address;
            spectral_matrix_regs->status = BIT_STATUS_F2_0; // [0001 0000]
            if (rtems_event_send(Task_id[TASKID_AVF2], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
            {
                status_code = send_event_dumb_task(RTEMS_EVENT_3);
            }
            break;
        case BIT_READY_1:
            ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2->previous;
            current_ring_node_sm_f2 = current_ring_node_sm_f2->next;
            ring_node_for_averaging_sm_f2->coarseTime = spectral_matrix_regs->f2_1_coarse_time;
            ring_node_for_averaging_sm_f2->fineTime = spectral_matrix_regs->f2_1_fine_time;
            spectral_matrix_regs->f2_1_address = current_ring_node_sm_f2->buffer_address;
            spectral_matrix_regs->status = BIT_STATUS_F2_1; // [0010 0000]
            if (rtems_event_send(Task_id[TASKID_AVF2], RTEMS_EVENT_0) != RTEMS_SUCCESSFUL)
            {
                status_code = send_event_dumb_task(RTEMS_EVENT_3);
            }
            break;
        default:
            break;
    }
}

void spectral_matrix_isr_error_handler(int statusReg)
{
    // STATUS REGISTER
    // input_fifo_write(2) *** input_fifo_write(1) *** input_fifo_write(0)
    //           10                    9                       8
    // buffer_full ** [bad_component_err] ** f2_1 ** f2_0 ** f1_1 ** f1_0 ** f0_1 ** f0_0
    //      7                  6             5       4       3       2       1       0
    // [bad_component_err] not defined in the last version of the VHDL code

    rtems_status_code status_code;

    //***************************************************
    // the ASM status register is copied in the HK packet
    housekeeping_packet.hk_lfr_vhdl_aa_sm
        = (unsigned char)((statusReg & BITS_HK_AA_SM) >> SHIFT_7_BITS); // [0111 1000 0000]

    if (statusReg & BITS_SM_ERR) // [0111 1100 0000]
    {
        status_code = send_event_dumb_task(RTEMS_EVENT_8);
    }

    spectral_matrix_regs->status = spectral_matrix_regs->status & BITS_SM_ERR;
}

rtems_isr spectral_matrices_isr(rtems_vector_number vector)
{
    // STATUS REGISTER
    // input_fifo_write(2) *** input_fifo_write(1) *** input_fifo_write(0)
    //           10                    9                       8
    // buffer_full ** bad_component_err ** f2_1 ** f2_0 ** f1_1 ** f1_0 ** f0_1 ** f0_0
    //      7                  6             5       4       3       2       1       0

    IGNORE_UNUSED_PARAMETER(vector);

    int statusReg;

    static restartState state = WAIT_FOR_F2;

    statusReg = spectral_matrix_regs->status;

    if (thisIsAnASMRestart == 0)
    { // this is not a restart sequence, process incoming matrices normally
        spectral_matrices_isr_f0(statusReg);

        spectral_matrices_isr_f1(statusReg);

        spectral_matrices_isr_f2(statusReg);
    }
    else
    { // a restart sequence has to be launched
        switch (state)
        {
            case WAIT_FOR_F2:
                if ((statusReg & BITS_STATUS_F2)
                    != INIT_CHAR) // [0011 0000] check the status_ready_matrix_f2_x bits
                {
                    state = WAIT_FOR_F1;
                }
                break;
            case WAIT_FOR_F1:
                if ((statusReg & BITS_STATUS_F1)
                    != INIT_CHAR) // [0000 1100] check the status_ready_matrix_f1_x bits
                {
                    state = WAIT_FOR_F0;
                }
                break;
            case WAIT_FOR_F0:
                if ((statusReg & BITS_STATUS_F0)
                    != INIT_CHAR) // [0000 0011] check the status_ready_matrix_f0_x bits
                {
                    state = WAIT_FOR_F2;
                    thisIsAnASMRestart = 0;
                }
                break;
            default:
                break;
        }
        reset_sm_status();
    }

    spectral_matrix_isr_error_handler(statusReg);
}

//******************
// Spectral Matrices

void reset_nb_sm(void)
{
    nb_sm_f0 = 0;
    nb_sm_f0_aux_f1 = 0;
    nb_sm_f0_aux_f2 = 0;

    nb_sm_f1 = 0;
}

void SM_init_rings(void)
{
    init_ring(sm_ring_f0, NB_RING_NODES_SM_F0, sm_f0, TOTAL_SIZE_SM);
    init_ring(sm_ring_f1, NB_RING_NODES_SM_F1, sm_f1, TOTAL_SIZE_SM);
    init_ring(sm_ring_f2, NB_RING_NODES_SM_F2, sm_f2, TOTAL_SIZE_SM);

    DEBUG_PRINTF("sm_ring_f0 @%x\n", (unsigned int)sm_ring_f0);
    DEBUG_PRINTF("sm_ring_f1 @%x\n", (unsigned int)sm_ring_f1);
    DEBUG_PRINTF("sm_ring_f2 @%x\n", (unsigned int)sm_ring_f2);
    DEBUG_PRINTF("sm_f0 @%x\n", (unsigned int)sm_f0);
    DEBUG_PRINTF("sm_f1 @%x\n", (unsigned int)sm_f1);
    DEBUG_PRINTF("sm_f2 @%x\n", (unsigned int)sm_f2);
}

void ASM_generic_init_ring(ring_node_asm* ring, unsigned char nbNodes)
{
    DEBUG_CHECK_PTR(ring);

    unsigned char i;

    ring[nbNodes - 1].next = (ring_node_asm*)&ring[0];

    for (i = 0; i < nbNodes - 1; i++)
    {
        ring[i].next = (ring_node_asm*)&ring[i + 1];
    }
}

void SM_reset_current_ring_nodes(void)
{
    current_ring_node_sm_f0 = sm_ring_f0[0].next;
    current_ring_node_sm_f1 = sm_ring_f1[0].next;
    current_ring_node_sm_f2 = sm_ring_f2[0].next;

    ring_node_for_averaging_sm_f0 = NULL;
    ring_node_for_averaging_sm_f1 = NULL;
    ring_node_for_averaging_sm_f2 = NULL;
}

//*****************
// Basic Parameters

void BP_init_header(bp_packet* packet, unsigned int apid, unsigned char sid,
    unsigned int packetLength, unsigned char blkNr)
{
    DEBUG_CHECK_PTR(packet);
    packet->targetLogicalAddress = CCSDS_DESTINATION_ID;
    packet->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    packet->reserved = INIT_CHAR;
    packet->userApplication = CCSDS_USER_APP;
    packet->packetID[0] = (unsigned char)(apid >> SHIFT_1_BYTE);
    packet->packetID[1] = (unsigned char)(apid);
    packet->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    packet->packetSequenceControl[1] = INIT_CHAR;
    packet->packetLength[0] = (unsigned char)(packetLength >> SHIFT_1_BYTE);
    packet->packetLength[1] = (unsigned char)(packetLength);
    // DATA FIELD HEADER
    packet->spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    packet->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    packet->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_3; // service subtype
    packet->destinationID = TM_DESTINATION_ID_GROUND;
    packet->time[BYTE_0] = INIT_CHAR;
    packet->time[BYTE_1] = INIT_CHAR;
    packet->time[BYTE_2] = INIT_CHAR;
    packet->time[BYTE_3] = INIT_CHAR;
    packet->time[BYTE_4] = INIT_CHAR;
    packet->time[BYTE_5] = INIT_CHAR;
    // AUXILIARY DATA HEADER
    packet->sid = sid;
    packet->pa_bia_status_info = INIT_CHAR;
    packet->sy_lfr_common_parameters_spare = INIT_CHAR;
    packet->sy_lfr_common_parameters = INIT_CHAR;
    packet->acquisitionTime[BYTE_0] = INIT_CHAR;
    packet->acquisitionTime[BYTE_1] = INIT_CHAR;
    packet->acquisitionTime[BYTE_2] = INIT_CHAR;
    packet->acquisitionTime[BYTE_3] = INIT_CHAR;
    packet->acquisitionTime[BYTE_4] = INIT_CHAR;
    packet->acquisitionTime[BYTE_5] = INIT_CHAR;
    packet->pa_lfr_bp_blk_nr[0] = INIT_CHAR; // BLK_NR MSB
    packet->pa_lfr_bp_blk_nr[1] = blkNr; // BLK_NR LSB
}

void BP_init_header_with_spare(bp_packet_with_spare* packet, unsigned int apid, unsigned char sid,
    unsigned int packetLength, unsigned char blkNr)
{
    packet->targetLogicalAddress = CCSDS_DESTINATION_ID;
    packet->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    packet->reserved = INIT_CHAR;
    packet->userApplication = CCSDS_USER_APP;
    packet->packetID[0] = (unsigned char)(apid >> SHIFT_1_BYTE);
    packet->packetID[1] = (unsigned char)(apid);
    packet->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    packet->packetSequenceControl[1] = INIT_CHAR;
    packet->packetLength[0] = (unsigned char)(packetLength >> SHIFT_1_BYTE);
    packet->packetLength[1] = (unsigned char)(packetLength);
    // DATA FIELD HEADER
    packet->spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    packet->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    packet->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_3; // service subtype
    packet->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    packet->sid = sid;
    packet->pa_bia_status_info = INIT_CHAR;
    packet->sy_lfr_common_parameters_spare = INIT_CHAR;
    packet->sy_lfr_common_parameters = INIT_CHAR;
    packet->time[BYTE_0] = INIT_CHAR;
    packet->time[BYTE_1] = INIT_CHAR;
    packet->time[BYTE_2] = INIT_CHAR;
    packet->time[BYTE_3] = INIT_CHAR;
    packet->time[BYTE_4] = INIT_CHAR;
    packet->time[BYTE_5] = INIT_CHAR;
    packet->source_data_spare = INIT_CHAR;
    packet->pa_lfr_bp_blk_nr[0] = INIT_CHAR; // BLK_NR MSB
    packet->pa_lfr_bp_blk_nr[1] = blkNr; // BLK_NR LSB
}

void BP_send(char* data, rtems_id queue_id, unsigned int nbBytesToSend)
{
    rtems_status_code status;

    // SEND PACKET
    status = rtems_message_queue_send(queue_id, data, nbBytesToSend);
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("ERR *** in BP_send *** ERR %d\n", (int)status);
    }
}

void BP_send_s1_s2(char* data, rtems_id queue_id, unsigned int nbBytesToSend)
{
    /** This function is used to send the BP paquets when needed.
     *
     * @param transitionCoarseTime is the requested transition time contained in the
     * TC_LFR_ENTER_MODE
     *
     * @return void
     *
     * SBM1 and SBM2 paquets are sent depending on the type of the LFR mode transition.
     * BURST paquets are sent everytime.
     *
     */
    DEBUG_CHECK_PTR(data);
    rtems_status_code status;

    // SEND PACKET
    // before lastValidTransitionDate, the data are drops even if they are ready
    // this guarantees that no SBM packets will be received before the requested enter mode time
    if (time_management_regs->coarse_time >= lastValidEnterModeTime)
    {
        status = rtems_message_queue_send(queue_id, data, nbBytesToSend);
        DEBUG_CHECK_STATUS(status);
        if (status != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("ERR *** in BP_send *** ERR %d\n", (int)status);
        }
    }
}

//******************
// general functions

void reset_sm_status(void)
{
    // error
    // 10 --------------- 9 ---------------- 8 ---------------- 7 ---------
    // input_fif0_write_2 input_fifo_write_1 input_fifo_write_0 buffer_full
    // ---------- 5 -- 4 -- 3 -- 2 -- 1 -- 0 --
    // ready bits f2_1 f2_0 f1_1 f1_1 f0_1 f0_0

    spectral_matrix_regs->status = BITS_STATUS_REG; // [0111 1111 1111]
}

void reset_spectral_matrix_regs(void)
{
    /** This function resets the spectral matrices module registers.
     *
     * The registers affected by this function are located at the following offset addresses:
     *
     * - 0x00 config
     * - 0x04 status
     * - 0x08 matrixF0_Address0
     * - 0x10 matrixFO_Address1
     * - 0x14 matrixF1_Address
     * - 0x18 matrixF2_Address
     *
     */

    set_sm_irq_onError(0);

    set_sm_irq_onNewMatrix(0);

    reset_sm_status();

    // F1
    spectral_matrix_regs->f0_0_address = current_ring_node_sm_f0->previous->buffer_address;
    spectral_matrix_regs->f0_1_address = current_ring_node_sm_f0->buffer_address;
    // F2
    spectral_matrix_regs->f1_0_address = current_ring_node_sm_f1->previous->buffer_address;
    spectral_matrix_regs->f1_1_address = current_ring_node_sm_f1->buffer_address;
    // F3
    spectral_matrix_regs->f2_0_address = current_ring_node_sm_f2->previous->buffer_address;
    spectral_matrix_regs->f2_1_address = current_ring_node_sm_f2->buffer_address;

    spectral_matrix_regs->matrix_length = DEFAULT_MATRIX_LENGTH; // 25 * 128 / 16 = 200 = 0xc8
}

void set_time(unsigned char* time, unsigned char* timeInBuffer)
{
    time[BYTE_0] = timeInBuffer[BYTE_0];
    time[BYTE_1] = timeInBuffer[BYTE_1];
    time[BYTE_2] = timeInBuffer[BYTE_2];
    time[BYTE_3] = timeInBuffer[BYTE_3];
    time[BYTE_4] = timeInBuffer[BYTE_6];
    time[BYTE_5] = timeInBuffer[BYTE_7];
}

unsigned long long int get_acquisition_time(unsigned char* timePtr)
{
    unsigned long long int acquisitionTimeAslong;
    acquisitionTimeAslong = INIT_CHAR;
    acquisitionTimeAslong = ((unsigned long long int)(timePtr[BYTE_0] & SYNC_BIT_MASK)
                                << SHIFT_5_BYTES) // [0111 1111] mask the synchronization bit
        + ((unsigned long long int)timePtr[BYTE_1] << SHIFT_4_BYTES)
        + ((unsigned long long int)timePtr[BYTE_2] << SHIFT_3_BYTES)
        + ((unsigned long long int)timePtr[BYTE_3] << SHIFT_2_BYTES)
        + ((unsigned long long int)timePtr[BYTE_6] << SHIFT_1_BYTE)
        + ((unsigned long long int)timePtr[BYTE_7]);
    return acquisitionTimeAslong;
}

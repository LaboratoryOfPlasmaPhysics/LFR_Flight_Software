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

#include "avf0_prc0.h"
#include <string.h>

nb_sm_before_bp_asm_f0 nb_sm_before_f0 = { 0 };

//***
// F0
ring_node_asm asm_ring_norm_f0[NB_RING_NODES_ASM_NORM_F0] = { 0 };
ring_node_asm asm_ring_burst_sbm_f0[NB_RING_NODES_ASM_BURST_SBM_F0] = { 0 };

ring_node ring_to_send_asm_f0[NB_RING_NODES_ASM_F0] = { 0 };
int buffer_asm_f0[NB_RING_NODES_ASM_F0 * TOTAL_SIZE_SM] = { 0 };

float asm_f0_patched_norm[TOTAL_SIZE_SM] = { 0 };
float asm_f0_patched_burst_sbm[TOTAL_SIZE_SM] = { 0 };
float asm_f0_reorganized[TOTAL_SIZE_SM] = { 0 };

float compressed_sm_norm_f0[TOTAL_SIZE_COMPRESSED_ASM_NORM_F0] = { 0 };
float compressed_sm_sbm_f0[TOTAL_SIZE_COMPRESSED_ASM_SBM_F0] = { 0 };

float k_coeff_intercalib_f0_norm[NB_BINS_COMPRESSED_SM_F0 * NB_K_COEFF_PER_BIN]
    = { 0 }; // 11 * 32 = 352
float k_coeff_intercalib_f0_sbm[NB_BINS_COMPRESSED_SM_SBM_F0 * NB_K_COEFF_PER_BIN]
    = { 0 }; // 22 * 32 = 704

#define UNITY_3x3_MATRIX                                                                           \
    1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f

#define UNITY_2x2_MATRIX 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f

// 128 * 3x3 float complex matrices
float mag_calibration_matrices_f0[NB_BINS_PER_SM * 3 * 3 * 2]
    = { UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX,
          UNITY_3x3_MATRIX, UNITY_3x3_MATRIX, UNITY_3x3_MATRIX };

// 128 * 2x2 float complex matrices
float elec_calibration_matrices_f0[NB_BINS_PER_SM * 2 * 2 * 2]
    = { UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX,
          UNITY_2x2_MATRIX, UNITY_2x2_MATRIX, UNITY_2x2_MATRIX };

//************
// RTEMS TASKS

rtems_task avf0_task(rtems_task_argument lfrRequestedMode)
{

    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc0;
    asm_msg msgForPRC;
    ring_node* nodeForAveraging;
    ring_node* ring_node_tab[NB_SM_BEFORE_AVF0_F1];
    ring_node_asm* current_ring_node_asm_burst_sbm_f0;
    ring_node_asm* current_ring_node_asm_norm_f0;

    unsigned int nb_norm_bp1;
    unsigned int nb_norm_bp2;
    unsigned int nb_norm_asm;
    unsigned int nb_sbm_bp1;
    unsigned int nb_sbm_bp2;

    nb_norm_bp1 = 0;
    nb_norm_bp2 = 0;
    nb_norm_asm = 0;
    nb_sbm_bp1 = 0;
    nb_sbm_bp2 = 0;
    event_out = EVENT_SETS_NONE_PENDING;
    queue_id_prc0 = RTEMS_ID_NONE;

    reset_nb_sm_f0(lfrRequestedMode); // reset the sm counters that drive the BP and ASM
                                      // computations / transmissions
    ASM_generic_init_ring(asm_ring_norm_f0, NB_RING_NODES_ASM_NORM_F0);
    ASM_generic_init_ring(asm_ring_burst_sbm_f0, NB_RING_NODES_ASM_BURST_SBM_F0);
    current_ring_node_asm_norm_f0 = asm_ring_norm_f0;
    current_ring_node_asm_burst_sbm_f0 = asm_ring_burst_sbm_f0;

    BOOT_PRINTF1("in AVFO *** lfrRequestedMode = %d\n", (int)lfrRequestedMode);

    status = get_message_queue_id_prc0(&queue_id_prc0);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in MATR *** ERR get_message_queue_id_prc0 %d\n", status)
    }

    while (1)
    {
        rtems_event_receive(
            RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0

        //****************************************
        // initialize the mesage for the MATR task
        msgForPRC.norm = current_ring_node_asm_norm_f0;
        msgForPRC.burst_sbm = current_ring_node_asm_burst_sbm_f0;
        msgForPRC.event
            = EVENT_SETS_NONE_PENDING; // this composite event will be sent to the PRC0 task
        //
        //****************************************

        nodeForAveraging = getRingNodeForAveraging(0);
        for (int i = NB_SM_BEFORE_AVF0_F1 - 1; i >= 0; i--)
        {
            ring_node_tab[i] = nodeForAveraging;
            nodeForAveraging = nodeForAveraging->previous;
        }

        // compute the average and store it in the averaged_sm_f1 buffer
        SM_average(current_ring_node_asm_norm_f0->matrix,
            current_ring_node_asm_burst_sbm_f0->matrix, ring_node_tab, nb_norm_bp1, nb_sbm_bp1,
            &msgForPRC, 0); // 0 => frequency channel 0

        // update nb_average
        nb_norm_bp1 = nb_norm_bp1 + NB_SM_BEFORE_AVF0_F1;
        nb_norm_bp2 = nb_norm_bp2 + NB_SM_BEFORE_AVF0_F1;
        nb_norm_asm = nb_norm_asm + NB_SM_BEFORE_AVF0_F1;
        nb_sbm_bp1 = nb_sbm_bp1 + NB_SM_BEFORE_AVF0_F1;
        nb_sbm_bp2 = nb_sbm_bp2 + NB_SM_BEFORE_AVF0_F1;

        if (nb_sbm_bp1 == nb_sm_before_f0.burst_sbm_bp1)
        {
            nb_sbm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_burst_sbm_f0 = current_ring_node_asm_burst_sbm_f0->next;
            if (lfrCurrentMode == LFR_MODE_BURST)
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_BURST_BP1_F0;
            }
            else if ((lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2))
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_SBM_BP1_F0;
            }
        }

        if (nb_sbm_bp2 == nb_sm_before_f0.burst_sbm_bp2)
        {
            nb_sbm_bp2 = 0;
            if (lfrCurrentMode == LFR_MODE_BURST)
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_BURST_BP2_F0;
            }
            else if ((lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2))
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_SBM_BP2_F0;
            }
        }

        if (nb_norm_bp1 == nb_sm_before_f0.norm_bp1)
        {
            nb_norm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f0 = current_ring_node_asm_norm_f0->next;
            if ((lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                || (lfrCurrentMode == LFR_MODE_SBM2))
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_NORM_BP1_F0;
            }
        }

        if (nb_norm_bp2 == nb_sm_before_f0.norm_bp2)
        {
            nb_norm_bp2 = 0;
            if ((lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                || (lfrCurrentMode == LFR_MODE_SBM2))
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_NORM_BP2_F0;
            }
        }

        if (nb_norm_asm == nb_sm_before_f0.norm_asm)
        {
            nb_norm_asm = 0;
            if ((lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                || (lfrCurrentMode == LFR_MODE_SBM2))
            {
                msgForPRC.event = msgForPRC.event | RTEMS_EVENT_NORM_ASM_F0;
            }
        }

        //*************************
        // send the message to PRC
        if (msgForPRC.event != EVENT_SETS_NONE_PENDING)
        {
            status
                = rtems_message_queue_send(queue_id_prc0, (char*)&msgForPRC, MSG_QUEUE_SIZE_PRC0);
        }

        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in AVF0 *** Error sending message to PRC, code %d\n", status)
        }
    }
}

rtems_task prc0_task(rtems_task_argument lfrRequestedMode)
{
    char incomingData[MSG_QUEUE_SIZE_SEND]; // incoming data buffer
    size_t size; // size of the incoming TC packet
    asm_msg* incomingMsg;
    //
    unsigned char sid;
    rtems_status_code status;
    rtems_id queue_id;
    rtems_id queue_id_q_p0;
    bp_packet_with_spare __attribute__((aligned(4))) packet_norm_bp1;
    bp_packet __attribute__((aligned(4))) packet_norm_bp2;
    bp_packet __attribute__((aligned(4))) packet_sbm_bp1;
    bp_packet __attribute__((aligned(4))) packet_sbm_bp2;
    ring_node* current_ring_node_to_send_asm_f0;
    float nbSMInASMNORM;
    float nbSMInASMSBM;

    size = 0;
    queue_id = RTEMS_ID_NONE;
    queue_id_q_p0 = RTEMS_ID_NONE;
    memset(&packet_norm_bp1, 0, sizeof(bp_packet_with_spare));
    memset(&packet_norm_bp2, 0, sizeof(bp_packet));
    memset(&packet_sbm_bp1, 0, sizeof(bp_packet));
    memset(&packet_sbm_bp2, 0, sizeof(bp_packet));

    // init the ring of the averaged spectral matrices which will be transmitted to the DPU
    init_ring(
        ring_to_send_asm_f0, NB_RING_NODES_ASM_F0, (volatile int*)buffer_asm_f0, TOTAL_SIZE_SM);
    current_ring_node_to_send_asm_f0 = ring_to_send_asm_f0;

    //*************
    // NORM headers
    BP_init_header_with_spare(&packet_norm_bp1, APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP1_F0,
        PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0, NB_BINS_COMPRESSED_SM_F0);
    BP_init_header(&packet_norm_bp2, APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP2_F0,
        PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0, NB_BINS_COMPRESSED_SM_F0);

    //****************************
    // BURST SBM1 and SBM2 headers
    if (lfrRequestedMode == LFR_MODE_BURST)
    {
        BP_init_header(&packet_sbm_bp1, APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP1_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header(&packet_sbm_bp2, APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP2_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else if (lfrRequestedMode == LFR_MODE_SBM1)
    {
        BP_init_header(&packet_sbm_bp1, APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP1_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header(&packet_sbm_bp2, APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP2_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else if (lfrRequestedMode == LFR_MODE_SBM2)
    {
        BP_init_header(&packet_sbm_bp1, APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP1_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header(&packet_sbm_bp2, APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP2_F0,
            PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else
    {
        PRINTF1("in PRC0 *** lfrRequestedMode is %d, several headers not initialized\n",
            (unsigned int)lfrRequestedMode)
    }

    status = get_message_queue_id_send(&queue_id);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC0 *** ERR get_message_queue_id_send %d\n", status)
    }
    status = get_message_queue_id_prc0(&queue_id_q_p0);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC0 *** ERR get_message_queue_id_prc0 %d\n", status)
    }

    BOOT_PRINTF1("in PRC0 *** lfrRequestedMode = %d\n", (int)lfrRequestedMode)

    while (1)
    {
        status = rtems_message_queue_receive(queue_id_q_p0, incomingData,
            &size, //************************************
            RTEMS_WAIT, RTEMS_NO_TIMEOUT); // wait for a message coming from AVF0

        incomingMsg = (asm_msg*)incomingData;

        //****************
        //****************
        // BURST SBM1 SBM2
        //****************
        //****************
        if ((incomingMsg->event & RTEMS_EVENT_BURST_BP1_F0)
            || (incomingMsg->event & RTEMS_EVENT_SBM_BP1_F0))
        {
            // ASM_patch( incomingMsg->burst_sbm->matrix, asm_f0_patched_burst_sbm );
            SM_calibrate_and_reorder(incomingMsg->burst_sbm->matrix, mag_calibration_matrices_f0,
                elec_calibration_matrices_f0, asm_f0_patched_burst_sbm);
            nbSMInASMSBM = incomingMsg->numberOfSMInASMSBM;

            sid = getSID(incomingMsg->event);
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_divide_and_mask(asm_f0_patched_burst_sbm, compressed_sm_sbm_f0,
                nbSMInASMSBM, NB_BINS_COMPRESSED_SM_SBM_F0, NB_BINS_TO_AVERAGE_ASM_SBM_F0,
                ASM_F0_INDICE_START, CHANNELF0);
            // 2) compute the BP1 set
            // BP1_set( compressed_sm_sbm_f0, k_coeff_intercalib_f0_sbm,
            // NB_BINS_COMPRESSED_SM_SBM_F0, packet_sbm_bp1.data );
            compute_BP1(compressed_sm_norm_f0, NB_BINS_COMPRESSED_SM_SBM_F0, packet_sbm_bp1.data);
            // 3) send the BP1 set
            set_time(packet_sbm_bp1.time, (unsigned char*)&incomingMsg->coarseTimeSBM);
            set_time(packet_sbm_bp1.acquisitionTime, (unsigned char*)&incomingMsg->coarseTimeSBM);
            packet_sbm_bp1.pa_bia_status_info = pa_bia_status_info;
            packet_sbm_bp1.sy_lfr_common_parameters
                = parameter_dump_packet.sy_lfr_common_parameters;
            BP_send_s1_s2((char*)&packet_sbm_bp1, queue_id,
                PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0 + PACKET_LENGTH_DELTA, sid);
            // 4) compute the BP2 set if needed
            if ((incomingMsg->event & RTEMS_EVENT_BURST_BP2_F0)
                || (incomingMsg->event & RTEMS_EVENT_SBM_BP2_F0))
            {
                // 1) compute the BP2 set
                BP2_set(compressed_sm_sbm_f0, NB_BINS_COMPRESSED_SM_SBM_F0, packet_sbm_bp2.data);
                // 2) send the BP2 set
                set_time(packet_sbm_bp2.time, (unsigned char*)&incomingMsg->coarseTimeSBM);
                set_time(
                    packet_sbm_bp2.acquisitionTime, (unsigned char*)&incomingMsg->coarseTimeSBM);
                packet_sbm_bp2.pa_bia_status_info = pa_bia_status_info;
                packet_sbm_bp2.sy_lfr_common_parameters
                    = parameter_dump_packet.sy_lfr_common_parameters;
                BP_send_s1_s2((char*)&packet_sbm_bp2, queue_id,
                    PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0 + PACKET_LENGTH_DELTA, sid);
            }
        }

        //*****
        //*****
        // NORM
        //*****
        //*****
        if ((incomingMsg->event & RTEMS_EVENT_NORM_BP1_F0)
            || (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F0))
        {
            // ASM_patch( incomingMsg->norm->matrix,      asm_f0_patched_norm      );
            SM_calibrate_and_reorder(incomingMsg->norm->matrix, mag_calibration_matrices_f0,
                elec_calibration_matrices_f0, asm_f0_patched_norm);
            nbSMInASMNORM = incomingMsg->numberOfSMInASMNORM;
        }

        if (incomingMsg->event & RTEMS_EVENT_NORM_BP1_F0)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_divide_and_mask(asm_f0_patched_norm, compressed_sm_norm_f0, nbSMInASMNORM,
                NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0, ASM_F0_INDICE_START,
                CHANNELF0);
            // 2) compute the BP1 set
            // BP1_set( compressed_sm_norm_f0, k_coeff_intercalib_f0_norm, NB_BINS_COMPRESSED_SM_F0,
            // packet_norm_bp1.data );
            compute_BP1(compressed_sm_norm_f0, NB_BINS_COMPRESSED_SM_F0, packet_norm_bp1.data);

            // 3) send the BP1 set
            set_time(packet_norm_bp1.time, (unsigned char*)&incomingMsg->coarseTimeNORM);
            set_time(packet_norm_bp1.acquisitionTime, (unsigned char*)&incomingMsg->coarseTimeNORM);
            packet_norm_bp1.pa_bia_status_info = pa_bia_status_info;
            packet_norm_bp1.sy_lfr_common_parameters
                = parameter_dump_packet.sy_lfr_common_parameters;
            BP_send((char*)&packet_norm_bp1, queue_id,
                PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0 + PACKET_LENGTH_DELTA, SID_NORM_BP1_F0);
            if (incomingMsg->event & RTEMS_EVENT_NORM_BP2_F0)
            {
                // 1) compute the BP2 set using the same ASM as the one used for BP1
                BP2_set(compressed_sm_norm_f0, NB_BINS_COMPRESSED_SM_F0, packet_norm_bp2.data);
                // 2) send the BP2 set
                set_time(packet_norm_bp2.time, (unsigned char*)&incomingMsg->coarseTimeNORM);
                set_time(
                    packet_norm_bp2.acquisitionTime, (unsigned char*)&incomingMsg->coarseTimeNORM);
                packet_norm_bp2.pa_bia_status_info = pa_bia_status_info;
                packet_norm_bp2.sy_lfr_common_parameters
                    = parameter_dump_packet.sy_lfr_common_parameters;
                BP_send((char*)&packet_norm_bp2, queue_id,
                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0 + PACKET_LENGTH_DELTA,
                    SID_NORM_BP2_F0);
            }
        }

        if (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F0)
        {
            // 1) reorganize the ASM and divide
            ASM_divide(asm_f0_patched_norm,
                (float*)current_ring_node_to_send_asm_f0->buffer_address, nbSMInASMNORM);
            current_ring_node_to_send_asm_f0->coarseTime = incomingMsg->coarseTimeNORM;
            current_ring_node_to_send_asm_f0->fineTime = incomingMsg->fineTimeNORM;
            current_ring_node_to_send_asm_f0->sid = SID_NORM_ASM_F0;

            // 3) send the spectral matrix packets
            status = rtems_message_queue_send(
                queue_id, &current_ring_node_to_send_asm_f0, sizeof(ring_node*));

            // change asm ring node
            current_ring_node_to_send_asm_f0 = current_ring_node_to_send_asm_f0->next;
        }

        update_queue_max_count(queue_id_q_p0, &hk_lfr_q_p0_fifo_size_max);
    }
}

//**********
// FUNCTIONS

void reset_nb_sm_f0(unsigned char lfrMode)
{
    nb_sm_before_f0.norm_bp1 = parameter_dump_packet.sy_lfr_n_bp_p0 * NB_SM_PER_S_F0;
    nb_sm_before_f0.norm_bp2 = parameter_dump_packet.sy_lfr_n_bp_p1 * NB_SM_PER_S_F0;
    nb_sm_before_f0.norm_asm = ((parameter_dump_packet.sy_lfr_n_asm_p[0] * 256)
                                   + parameter_dump_packet.sy_lfr_n_asm_p[1])
        * NB_SM_PER_S_F0;
    nb_sm_before_f0.sbm1_bp1
        = parameter_dump_packet.sy_lfr_s1_bp_p0 * NB_SM_PER_S1_BP_P0; // 0.25 s per digit
    nb_sm_before_f0.sbm1_bp2 = parameter_dump_packet.sy_lfr_s1_bp_p1 * NB_SM_PER_S_F0;
    nb_sm_before_f0.sbm2_bp1 = parameter_dump_packet.sy_lfr_s2_bp_p0 * NB_SM_PER_S_F0;
    nb_sm_before_f0.sbm2_bp2 = parameter_dump_packet.sy_lfr_s2_bp_p1 * NB_SM_PER_S_F0;
    nb_sm_before_f0.burst_bp1 = parameter_dump_packet.sy_lfr_b_bp_p0 * NB_SM_PER_S_F0;
    nb_sm_before_f0.burst_bp2 = parameter_dump_packet.sy_lfr_b_bp_p1 * NB_SM_PER_S_F0;

    if (lfrMode == LFR_MODE_SBM1)
    {
        nb_sm_before_f0.burst_sbm_bp1 = nb_sm_before_f0.sbm1_bp1;
        nb_sm_before_f0.burst_sbm_bp2 = nb_sm_before_f0.sbm1_bp2;
    }
    else if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f0.burst_sbm_bp1 = nb_sm_before_f0.sbm2_bp1;
        nb_sm_before_f0.burst_sbm_bp2 = nb_sm_before_f0.sbm2_bp2;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f0.burst_sbm_bp1 = nb_sm_before_f0.burst_bp1;
        nb_sm_before_f0.burst_sbm_bp2 = nb_sm_before_f0.burst_bp2;
    }
    else
    {
        nb_sm_before_f0.burst_sbm_bp1 = nb_sm_before_f0.burst_bp1;
        nb_sm_before_f0.burst_sbm_bp2 = nb_sm_before_f0.burst_bp2;
    }
}

void init_k_coefficients_prc0(void)
{
    init_k_coefficients(k_coeff_intercalib_f0_norm, NB_BINS_COMPRESSED_SM_F0);

    init_kcoeff_sbm_from_kcoeff_norm(
        k_coeff_intercalib_f0_norm, k_coeff_intercalib_f0_sbm, NB_BINS_COMPRESSED_SM_F0);
}

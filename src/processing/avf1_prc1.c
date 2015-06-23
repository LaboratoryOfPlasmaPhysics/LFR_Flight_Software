/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic parameters computation.
 *
 */

#include "avf1_prc1.h"

nb_sm_before_bp_asm_f1 nb_sm_before_f1;

extern ring_node sm_ring_f1[ ];

//***
// F1
ring_node_asm asm_ring_norm_f1      [ NB_RING_NODES_ASM_NORM_F1      ];
ring_node_asm asm_ring_burst_sbm_f1 [ NB_RING_NODES_ASM_BURST_SBM_F1 ];

ring_node ring_to_send_asm_f1       [ NB_RING_NODES_ASM_F1 ];
int buffer_asm_f1                   [ NB_RING_NODES_ASM_F1 * TOTAL_SIZE_SM ];

float asm_f1_patched_norm       [ TOTAL_SIZE_SM ];
float asm_f1_patched_burst_sbm  [ TOTAL_SIZE_SM ];
float asm_f1_reorganized        [ TOTAL_SIZE_SM ];

char  asm_f1_char          [ TOTAL_SIZE_SM * 2 ];
float compressed_sm_norm_f1[ TOTAL_SIZE_COMPRESSED_ASM_NORM_F1];
float compressed_sm_sbm_f1 [ TOTAL_SIZE_COMPRESSED_ASM_SBM_F1 ];

float k_coeff_intercalib_f1_norm[ NB_BINS_COMPRESSED_SM_F1     * NB_K_COEFF_PER_BIN ];   // 13 * 32 = 416
float k_coeff_intercalib_f1_sbm[  NB_BINS_COMPRESSED_SM_SBM_F1 * NB_K_COEFF_PER_BIN ];   // 26 * 32 = 832

//************
// RTEMS TASKS

rtems_task avf1_task( rtems_task_argument lfrRequestedMode )
{
    int i;

    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc1;
    asm_msg msgForMATR;
    ring_node *nodeForAveraging;
    ring_node *ring_node_tab[NB_SM_BEFORE_AVF0];
    ring_node_asm *current_ring_node_asm_burst_sbm_f1;
    ring_node_asm *current_ring_node_asm_norm_f1;

    unsigned int nb_norm_bp1;
    unsigned int nb_norm_bp2;
    unsigned int nb_norm_asm;
    unsigned int nb_sbm_bp1;
    unsigned int nb_sbm_bp2;

    nb_norm_bp1 = 0;
    nb_norm_bp2 = 0;
    nb_norm_asm = 0;
    nb_sbm_bp1  = 0;
    nb_sbm_bp2  = 0;

    reset_nb_sm_f1( lfrRequestedMode );   // reset the sm counters that drive the BP and ASM computations / transmissions
    ASM_generic_init_ring( asm_ring_norm_f1, NB_RING_NODES_ASM_NORM_F1 );
    ASM_generic_init_ring( asm_ring_burst_sbm_f1, NB_RING_NODES_ASM_BURST_SBM_F1 );
    current_ring_node_asm_norm_f1      = asm_ring_norm_f1;
    current_ring_node_asm_burst_sbm_f1 = asm_ring_burst_sbm_f1;

    BOOT_PRINTF1("in AVF1 *** lfrRequestedMode = %d\n", (int) lfrRequestedMode)

    status = get_message_queue_id_prc1( &queue_id_prc1 );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in AVF1 *** ERR get_message_queue_id_prc1 %d\n", status)
    }

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0

        //****************************************
        // initialize the mesage for the MATR task
        msgForMATR.norm       = current_ring_node_asm_norm_f1;
        msgForMATR.burst_sbm  = current_ring_node_asm_burst_sbm_f1;
        msgForMATR.event      = 0x00;  // this composite event will be sent to the PRC1 task
        //
        //****************************************

        nodeForAveraging = getRingNodeForAveraging( 1 );

        ring_node_tab[NB_SM_BEFORE_AVF1-1] = nodeForAveraging;
        for ( i = 2; i < (NB_SM_BEFORE_AVF1+1); i++ )
        {
            nodeForAveraging = nodeForAveraging->previous;
            ring_node_tab[NB_SM_BEFORE_AVF1-i] = nodeForAveraging;
        }

        // compute the average and store it in the averaged_sm_f1 buffer
        SM_average( current_ring_node_asm_norm_f1->matrix,
                    current_ring_node_asm_burst_sbm_f1->matrix,
                    ring_node_tab,
                    nb_norm_bp1, nb_sbm_bp1,
                    &msgForMATR );

        // update nb_average
        nb_norm_bp1 = nb_norm_bp1 + NB_SM_BEFORE_AVF1;
        nb_norm_bp2 = nb_norm_bp2 + NB_SM_BEFORE_AVF1;
        nb_norm_asm = nb_norm_asm + NB_SM_BEFORE_AVF1;
        nb_sbm_bp1  = nb_sbm_bp1  + NB_SM_BEFORE_AVF1;
        nb_sbm_bp2  = nb_sbm_bp2  + NB_SM_BEFORE_AVF1;

        if (nb_sbm_bp1 == nb_sm_before_f1.burst_sbm_bp1)
        {
            nb_sbm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_burst_sbm_f1 = current_ring_node_asm_burst_sbm_f1->next;
            if ( lfrCurrentMode == LFR_MODE_BURST )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_BP1_F1;
            }
            else if ( lfrCurrentMode == LFR_MODE_SBM2 )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_SBM_BP1_F1;
            }
        }

        if (nb_sbm_bp2 == nb_sm_before_f1.burst_sbm_bp2)
        {
            nb_sbm_bp2 = 0;
            if ( lfrCurrentMode == LFR_MODE_BURST )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_BP2_F1;
            }
            else if ( lfrCurrentMode == LFR_MODE_SBM2 )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_SBM_BP2_F1;
            }
        }

        if (nb_norm_bp1 == nb_sm_before_f1.norm_bp1)
        {
            nb_norm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f1 = current_ring_node_asm_norm_f1->next;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP1_F1;
            }
        }

        if (nb_norm_bp2 == nb_sm_before_f1.norm_bp2)
        {
            nb_norm_bp2 = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP2_F1;
            }
        }

        if (nb_norm_asm == nb_sm_before_f1.norm_asm)
        {
            nb_norm_asm = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_ASM_F1;
            }
        }

        //*************************
        // send the message to MATR
        if (msgForMATR.event != 0x00)
        {
            status =  rtems_message_queue_send( queue_id_prc1, (char *) &msgForMATR, MSG_QUEUE_SIZE_PRC1);
        }

        if (status != RTEMS_SUCCESSFUL) {
            printf("in AVF1 *** Error sending message to PRC1, code %d\n", status);
        }
    }
}

rtems_task prc1_task( rtems_task_argument lfrRequestedMode )
{
    char incomingData[MSG_QUEUE_SIZE_SEND];  // incoming data buffer
    size_t size;                             // size of the incoming TC packet
    asm_msg *incomingMsg;
    //
    unsigned char sid;
    rtems_status_code status;
    rtems_id queue_id_send;
    rtems_id queue_id_q_p1;
    bp_packet_with_spare    packet_norm_bp1;
    bp_packet               packet_norm_bp2;
    bp_packet               packet_sbm_bp1;
    bp_packet               packet_sbm_bp2;
    ring_node               *current_ring_node_to_send_asm_f1;

    unsigned long long int localTime;

    // init the ring of the averaged spectral matrices which will be transmitted to the DPU
    init_ring( ring_to_send_asm_f1, NB_RING_NODES_ASM_F1, (volatile int*) buffer_asm_f1, TOTAL_SIZE_SM );
    current_ring_node_to_send_asm_f1 = ring_to_send_asm_f1;

    //*************
    // NORM headers
    BP_init_header_with_spare( &packet_norm_bp1,
                               APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP1_F1,
                               PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F1, NB_BINS_COMPRESSED_SM_F1 );
    BP_init_header( &packet_norm_bp2,
                    APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP2_F1,
                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F1, NB_BINS_COMPRESSED_SM_F1);

    //***********************
    // BURST and SBM2 headers
    if ( lfrRequestedMode == LFR_MODE_BURST )
    {
        BP_init_header( &packet_sbm_bp1,
                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP1_F1,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F1, NB_BINS_COMPRESSED_SM_SBM_F1);
        BP_init_header( &packet_sbm_bp2,
                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP2_F1,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F1, NB_BINS_COMPRESSED_SM_SBM_F1);
    }
    else if ( lfrRequestedMode == LFR_MODE_SBM2 )
    {
        BP_init_header( &packet_sbm_bp1,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP1_F1,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F1, NB_BINS_COMPRESSED_SM_SBM_F1);
        BP_init_header( &packet_sbm_bp2,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP2_F1,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F1, NB_BINS_COMPRESSED_SM_SBM_F1);
    }
    else
    {
        PRINTF1("in PRC1 *** lfrRequestedMode is %d, several headers not initialized\n", (unsigned int) lfrRequestedMode)
    }

    status = get_message_queue_id_send( &queue_id_send );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC1 *** ERR get_message_queue_id_send %d\n", status)
    }
    status = get_message_queue_id_prc1( &queue_id_q_p1);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC1 *** ERR get_message_queue_id_prc1 %d\n", status)
    }

    BOOT_PRINTF1("in PRC1 *** lfrRequestedMode = %d\n", (int) lfrRequestedMode)

    while(1){
        status = rtems_message_queue_receive( queue_id_q_p1, incomingData, &size, //************************************
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT );      // wait for a message coming from AVF0

        incomingMsg = (asm_msg*) incomingData;

        ASM_patch( incomingMsg->norm->matrix,      asm_f1_patched_norm      );
        ASM_patch( incomingMsg->burst_sbm->matrix, asm_f1_patched_burst_sbm );

        localTime = getTimeAsUnsignedLongLongInt( );
        //***********
        //***********
        // BURST SBM2
        //***********
        //***********
        if ( (incomingMsg->event & RTEMS_EVENT_BURST_BP1_F1) || (incomingMsg->event & RTEMS_EVENT_SBM_BP1_F1) )
        {
            sid = getSID( incomingMsg->event );
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( asm_f1_patched_burst_sbm, compressed_sm_sbm_f1,
                                         nb_sm_before_f1.burst_sbm_bp1,
                                         NB_BINS_COMPRESSED_SM_SBM_F1, NB_BINS_TO_AVERAGE_ASM_SBM_F1,
                                         ASM_F1_INDICE_START);
            // 2) compute the BP1 set
            BP1_set( compressed_sm_sbm_f1, k_coeff_intercalib_f1_sbm, NB_BINS_COMPRESSED_SM_SBM_F1, packet_sbm_bp1.data );
            // 3) send the BP1 set
            set_time( packet_sbm_bp1.time,            (unsigned char *) &incomingMsg->coarseTimeSBM );
            set_time( packet_sbm_bp1.acquisitionTime, (unsigned char *) &incomingMsg->coarseTimeSBM );
            packet_sbm_bp1.sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
            BP_send( (char *) &packet_sbm_bp1, queue_id_send,
                     PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F1 + PACKET_LENGTH_DELTA,
                     sid );
            // 4) compute the BP2 set if needed
            if ( (incomingMsg->event & RTEMS_EVENT_BURST_BP2_F1) || (incomingMsg->event & RTEMS_EVENT_SBM_BP2_F1) )
            {
                // 1) compute the BP2 set
                BP2_set( compressed_sm_sbm_f1, NB_BINS_COMPRESSED_SM_SBM_F1, packet_sbm_bp2.data );
                // 2) send the BP2 set
                set_time( packet_sbm_bp2.time,            (unsigned char *) &incomingMsg->coarseTimeSBM );
                set_time( packet_sbm_bp2.acquisitionTime, (unsigned char *) &incomingMsg->coarseTimeSBM );
                packet_sbm_bp2.sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
                BP_send( (char *) &packet_sbm_bp2, queue_id_send,
                         PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F1 + PACKET_LENGTH_DELTA,
                         sid );
            }
        }

        //*****
        //*****
        // NORM
        //*****
        //*****
        if (incomingMsg->event & RTEMS_EVENT_NORM_BP1_F1)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( asm_f1_patched_norm, compressed_sm_norm_f1,
                                         nb_sm_before_f1.norm_bp1,
                                         NB_BINS_COMPRESSED_SM_F1, NB_BINS_TO_AVERAGE_ASM_F1,
                                         ASM_F1_INDICE_START );
            // 2) compute the BP1 set
            BP1_set( compressed_sm_norm_f1, k_coeff_intercalib_f1_norm, NB_BINS_COMPRESSED_SM_F1, packet_norm_bp1.data );
            // 3) send the BP1 set
            set_time( packet_norm_bp1.time,            (unsigned char *) &incomingMsg->coarseTimeNORM );
            set_time( packet_norm_bp1.acquisitionTime, (unsigned char *) &incomingMsg->coarseTimeNORM );
            packet_norm_bp1.sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
            BP_send( (char *) &packet_norm_bp1, queue_id_send,
                     PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F1 + PACKET_LENGTH_DELTA,
                     SID_NORM_BP1_F1 );
            if (incomingMsg->event & RTEMS_EVENT_NORM_BP2_F1)
            {
                // 1) compute the BP2 set
                BP2_set( compressed_sm_norm_f1, NB_BINS_COMPRESSED_SM_F1, packet_norm_bp2.data );
                // 2) send the BP2 set
                set_time( packet_norm_bp2.time,            (unsigned char *) &incomingMsg->coarseTimeNORM );
                set_time( packet_norm_bp2.acquisitionTime, (unsigned char *) &incomingMsg->coarseTimeNORM );
                packet_norm_bp2.sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
                BP_send( (char *) &packet_norm_bp2, queue_id_send,
                         PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F1 + PACKET_LENGTH_DELTA,
                         SID_NORM_BP2_F1 );
            }
        }

        if (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F1)
        {
            // 1) reorganize the ASM and divide
            ASM_reorganize_and_divide( asm_f1_patched_norm,
                                       (float*) current_ring_node_to_send_asm_f1->buffer_address,
                                       nb_sm_before_f1.norm_bp1 );
            current_ring_node_to_send_asm_f1->coarseTime    = incomingMsg->coarseTimeNORM;
            current_ring_node_to_send_asm_f1->fineTime      = incomingMsg->fineTimeNORM;
            current_ring_node_to_send_asm_f1->sid           = SID_NORM_ASM_F1;
            // 3) send the spectral matrix packets
            status =  rtems_message_queue_send( queue_id_send, &current_ring_node_to_send_asm_f1, sizeof( ring_node* ) );
            // change asm ring node
            current_ring_node_to_send_asm_f1 = current_ring_node_to_send_asm_f1->next;
        }

        update_queue_max_count( queue_id_q_p1, &hk_lfr_q_p1_fifo_size_max );

    }
}

//**********
// FUNCTIONS

void reset_nb_sm_f1( unsigned char lfrMode )
{
    nb_sm_before_f1.norm_bp1  = parameter_dump_packet.sy_lfr_n_bp_p0 * 16;
    nb_sm_before_f1.norm_bp2  = parameter_dump_packet.sy_lfr_n_bp_p1 * 16;
    nb_sm_before_f1.norm_asm  = (parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1]) * 16;
    nb_sm_before_f1.sbm2_bp1  =  parameter_dump_packet.sy_lfr_s2_bp_p0 * 16;
    nb_sm_before_f1.sbm2_bp2  =  parameter_dump_packet.sy_lfr_s2_bp_p1 * 16;
    nb_sm_before_f1.burst_bp1 =  parameter_dump_packet.sy_lfr_b_bp_p0 * 16;
    nb_sm_before_f1.burst_bp2 =  parameter_dump_packet.sy_lfr_b_bp_p1 * 16;

    if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f1.burst_sbm_bp1 =  nb_sm_before_f1.sbm2_bp1;
        nb_sm_before_f1.burst_sbm_bp2 =  nb_sm_before_f1.sbm2_bp2;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f1.burst_sbm_bp1 =  nb_sm_before_f1.burst_bp1;
        nb_sm_before_f1.burst_sbm_bp2 =  nb_sm_before_f1.burst_bp2;
    }
    else
    {
        nb_sm_before_f1.burst_sbm_bp1 =  nb_sm_before_f1.burst_bp1;
        nb_sm_before_f1.burst_sbm_bp2 =  nb_sm_before_f1.burst_bp2;
    }
}

void init_k_coefficients_prc1( void )
{
    init_k_coefficients( k_coeff_intercalib_f1_norm, NB_BINS_COMPRESSED_SM_F1 );

    init_kcoeff_sbm_from_kcoeff_norm( k_coeff_intercalib_f1_norm, k_coeff_intercalib_f1_sbm, NB_BINS_COMPRESSED_SM_F1);
}

/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic parameters computation.
 *
 */

#include "avf2_prc2.h"

nb_sm_before_bp_asm_f2 nb_sm_before_f2;

//***
// F2
ring_node_asm asm_ring_norm_f2     [ NB_RING_NODES_ASM_NORM_F2      ];
ring_node_asm asm_ring_burst_sbm_f2[ NB_RING_NODES_ASM_BURST_SBM_F2 ];

float asm_f2_reorganized   [ TOTAL_SIZE_SM ];
char  asm_f2_char          [ TIME_OFFSET_IN_BYTES + (TOTAL_SIZE_SM * 2) ];
float compressed_sm_norm_f2[ TOTAL_SIZE_COMPRESSED_ASM_NORM_F2];
float compressed_sm_sbm_f2 [ TOTAL_SIZE_COMPRESSED_ASM_SBM_F2 ];

//************
// RTEMS TASKS

//***
// F2
rtems_task avf2_task( rtems_task_argument argument )
{
    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc2;
    asm_msg msgForMATR;
    ring_node_asm *current_ring_node_asm_norm_f2;

    unsigned int nb_norm_bp1;
    unsigned int nb_norm_bp2;
    unsigned int nb_norm_asm;

    nb_norm_bp1 = 0;
    nb_norm_bp2 = 0;
    nb_norm_asm = 0;

    reset_nb_sm_f2( );   // reset the sm counters that drive the BP and ASM computations / transmissions
    ASM_generic_init_ring( asm_ring_norm_f2, NB_RING_NODES_ASM_NORM_F2 );
    current_ring_node_asm_norm_f2 = asm_ring_norm_f2;

    BOOT_PRINTF("in AVF2 ***\n")

    status = get_message_queue_id_prc2( &queue_id_prc2 );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in AVF2 *** ERR get_message_queue_id_prc2 %d\n", status)
    }

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0

        //****************************************
        // initialize the mesage for the MATR task
        msgForMATR.event      = 0x00;  // this composite event will be sent to the MATR task
        msgForMATR.burst_sbm  = NULL;
        msgForMATR.norm       = current_ring_node_asm_norm_f2;
        msgForMATR.coarseTime = ring_node_for_averaging_sm_f2->coarseTime;
        msgForMATR.fineTime   = ring_node_for_averaging_sm_f2->fineTime;
        //
        //****************************************

        // compute the average and store it in the averaged_sm_f2 buffer
        SM_average_f2( current_ring_node_asm_norm_f2->matrix,
                       ring_node_for_averaging_sm_f2,
                       nb_norm_bp1 );

        // update nb_average
        nb_norm_bp1 = nb_norm_bp1 + NB_SM_BEFORE_AVF2;
        nb_norm_bp2 = nb_norm_bp2 + NB_SM_BEFORE_AVF2;
        nb_norm_asm = nb_norm_asm + NB_SM_BEFORE_AVF2;

        if (nb_norm_bp1 == nb_sm_before_f2.norm_bp1)
        {
            nb_norm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f2 = current_ring_node_asm_norm_f2->next;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                 || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP1_F2;
            }
        }

        if (nb_norm_bp2 == nb_sm_before_f2.norm_bp2)
        {
            nb_norm_bp2 = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                 || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP2_F2;
            }
        }

        if (nb_norm_asm == nb_sm_before_f2.norm_asm)
        {
            nb_norm_asm = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
                 || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
//                PRINTF1("%lld\n", localTime)
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_ASM_F2;
            }
        }

        //*************************
        // send the message to MATR
        if (msgForMATR.event != 0x00)
        {
            status =  rtems_message_queue_send( queue_id_prc2, (char *) &msgForMATR, MSG_QUEUE_SIZE_PRC0);
        }

        if (status != RTEMS_SUCCESSFUL) {
            printf("in AVF2 *** Error sending message to MATR, code %d\n", status);
        }
    }
}

rtems_task prc2_task( rtems_task_argument argument )
{
    char incomingData[MSG_QUEUE_SIZE_SEND];  // incoming data buffer
    size_t size;                            // size of the incoming TC packet
    asm_msg *incomingMsg;
    //
    spw_ioctl_pkt_send spw_ioctl_send_ASM;
    rtems_status_code status;
    rtems_id queue_id;
    rtems_id queue_id_q_p2;
    Header_TM_LFR_SCIENCE_ASM_t headerASM;
    bp_packet packet_norm_bp1_f2;
    bp_packet packet_norm_bp2_f2;

    unsigned long long int localTime;

    incomingMsg = NULL;

    ASM_init_header( &headerASM );

    //*************
    // NORM headers
    BP_init_header( &packet_norm_bp1_f2.header,
                    APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP1_F2,
                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F2, NB_BINS_COMPRESSED_SM_F2 );
    BP_init_header( &packet_norm_bp2_f2.header,
                    APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP2_F2,
                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F2, NB_BINS_COMPRESSED_SM_F2 );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC2 *** ERR get_message_queue_id_send %d\n", status)
    }
    status = get_message_queue_id_prc2( &queue_id_q_p2);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC2 *** ERR get_message_queue_id_prc2 %d\n", status)
    }

    BOOT_PRINTF("in PRC2 ***\n")

    while(1){
        status = rtems_message_queue_receive( queue_id_q_p2, incomingData, &size, //************************************
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT );      // wait for a message coming from AVF0

        incomingMsg = (asm_msg*) incomingData;

        localTime = getTimeAsUnsignedLongLongInt( );

        //*****
        //*****
        // NORM
        //*****
        //*****
        if (incomingMsg->event & RTEMS_EVENT_NORM_BP1_F2)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( incomingMsg->norm->matrix, compressed_sm_norm_f2,
                                         nb_sm_before_f2.norm_bp1,
                                         NB_BINS_COMPRESSED_SM_F2, NB_BINS_TO_AVERAGE_ASM_F2,
                                         ASM_F2_INDICE_START );
            // 2) compute the BP1 set

            // 3) send the BP1 set
            set_time( packet_norm_bp1_f2.header.time,            (unsigned char *) &incomingMsg->coarseTime );
            set_time( packet_norm_bp1_f2.header.acquisitionTime, (unsigned char *) &incomingMsg->coarseTime );
            BP_send( (char *) &packet_norm_bp1_f2, queue_id,
                     PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F2 + PACKET_LENGTH_DELTA,
                     SID_NORM_BP1_F2 );
            if (incomingMsg->event & RTEMS_EVENT_NORM_BP2_F2)
            {
                // 1) compute the BP2 set using the same ASM as the one used for BP1

                // 2) send the BP2 set
                set_time( packet_norm_bp2_f2.header.time,            (unsigned char *) &incomingMsg->coarseTime );
                set_time( packet_norm_bp2_f2.header.acquisitionTime, (unsigned char *) &incomingMsg->coarseTime );
                BP_send( (char *) &packet_norm_bp2_f2, queue_id,
                         PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F2 + PACKET_LENGTH_DELTA,
                         SID_NORM_BP2_F2 );
            }
        }

        if (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F2)
        {
            // 1) reorganize the ASM and divide
            ASM_reorganize_and_divide( incomingMsg->norm->matrix,
                                       asm_f2_reorganized,
                                       nb_sm_before_f2.norm_bp1 );
            // 2) convert the float array in a char array
            ASM_convert( asm_f2_reorganized, asm_f2_char);
            // 3) send the spectral matrix packets
            set_time( headerASM.time           , (unsigned char *) &incomingMsg->coarseTime );
            set_time( headerASM.acquisitionTime, (unsigned char *) &incomingMsg->coarseTime );
            ASM_send( &headerASM, asm_f2_char, SID_NORM_ASM_F2, &spw_ioctl_send_ASM, queue_id);
        }

    }
}

//**********
// FUNCTIONS

void reset_nb_sm_f2( void )
{
    nb_sm_before_f2.norm_bp1  = parameter_dump_packet.sy_lfr_n_bp_p0;
    nb_sm_before_f2.norm_bp2  = parameter_dump_packet.sy_lfr_n_bp_p1;
    nb_sm_before_f2.norm_asm  = parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1];
}

void SM_average_f2( float *averaged_spec_mat_f2,
                  ring_node_sm *ring_node,
                  unsigned int nbAverageNormF2 )
{
    float sum;
    unsigned int i;

    for(i=0; i<TOTAL_SIZE_SM; i++)
    {
        sum = ( (int *) (ring_node->buffer_address) ) [ i ];
        if ( (nbAverageNormF2 == 0) )
        {
            averaged_spec_mat_f2[ i ] = sum;
        }
        else
        {
            averaged_spec_mat_f2[ i ] = ( averaged_spec_mat_f2[  i ] + sum );
        }
    }
}

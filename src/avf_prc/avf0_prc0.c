/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic parameters computation.
 *
 */

#include "avf0_prc0.h"

ring_node_asm asm_ring_norm_f0     [ NB_RING_NODES_ASM_NORM_F0      ];
ring_node_asm asm_ring_burst_sbm_f0[ NB_RING_NODES_ASM_BURST_SBM_F0 ];
ring_node_asm *current_ring_node_asm_burst_sbm_f0;
ring_node_asm *current_ring_node_asm_norm_f0;

float asm_f0_reorganized   [ TOTAL_SIZE_SM ];
char  asm_f0_char          [ TIME_OFFSET_IN_BYTES + (TOTAL_SIZE_SM * 2) ];
float compressed_sm_norm_f0[ TOTAL_SIZE_COMPRESSED_ASM_NORM_F0];
float compressed_sm_sbm_f0 [ TOTAL_SIZE_COMPRESSED_ASM_SBM_F0 ];

nb_sm_before_bp_asm_f0 nb_sm_before_f0;

void reset_nb_sm_f0( unsigned char lfrMode )
{
    nb_sm_before_f0.norm_bp1 = parameter_dump_packet.sy_lfr_n_bp_p0 * 96;
    nb_sm_before_f0.norm_bp2 = parameter_dump_packet.sy_lfr_n_bp_p1 * 96;
    nb_sm_before_f0.norm_asm = (parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1]) * 96;
    nb_sm_before_f0.sbm1_bp1 =  parameter_dump_packet.sy_lfr_s1_bp_p0 * 24;
    nb_sm_before_f0.sbm1_bp2 =  parameter_dump_packet.sy_lfr_s1_bp_p1 * 96;
    nb_sm_before_f0.sbm2_bp1 =  parameter_dump_packet.sy_lfr_s2_bp_p0 * 96;
    nb_sm_before_f0.sbm2_bp2 =  parameter_dump_packet.sy_lfr_s2_bp_p1 * 96;
    nb_sm_before_f0.burst_bp1 =  parameter_dump_packet.sy_lfr_b_bp_p0 * 96;
    nb_sm_before_f0.burst_bp2 =  parameter_dump_packet.sy_lfr_b_bp_p1 * 96;

    if (lfrMode == LFR_MODE_SBM1)
    {
        nb_sm_before_f0.burst_sbm_bp1 =  nb_sm_before_f0.sbm1_bp1;
        nb_sm_before_f0.burst_sbm_bp2 =  nb_sm_before_f0.sbm1_bp2;
    }
    else if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f0.burst_sbm_bp1 =  nb_sm_before_f0.sbm2_bp1;
        nb_sm_before_f0.burst_sbm_bp2 =  nb_sm_before_f0.sbm2_bp2;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f0.burst_sbm_bp1 =  nb_sm_before_f0.burst_bp1;
        nb_sm_before_f0.burst_sbm_bp2 =  nb_sm_before_f0.burst_bp2;
    }
    else
    {
        nb_sm_before_f0.burst_sbm_bp1 =  nb_sm_before_f0.burst_bp1;
        nb_sm_before_f0.burst_sbm_bp2 =  nb_sm_before_f0.burst_bp2;
    }
}

//************
// RTEMS TASKS

rtems_task avf0_task( rtems_task_argument lfrRequestedMode )
{
    int i;

    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc0;
    asm_msg msgForMATR;
    ring_node_sm *ring_node_tab[8];

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

    reset_nb_sm_f0( lfrRequestedMode );   // reset the sm counters that drive the BP and ASM computations / transmissions
    ASM_generic_init_ring( asm_ring_norm_f0, NB_RING_NODES_ASM_NORM_F0 );
    ASM_generic_init_ring( asm_ring_burst_sbm_f0, NB_RING_NODES_ASM_BURST_SBM_F0 );
    current_ring_node_asm_norm_f0      = asm_ring_norm_f0;
    current_ring_node_asm_burst_sbm_f0 = asm_ring_burst_sbm_f0;

    BOOT_PRINTF1("in AVFO *** lfrRequestedMode = %d\n", (int) lfrRequestedMode)

    status = get_message_queue_id_prc0( &queue_id_prc0 );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in MATR *** ERR get_message_queue_id_prc0 %d\n", status)
    }

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        ring_node_tab[NB_SM_BEFORE_AVF0-1] = ring_node_for_averaging_sm_f0;
        for ( i = 2; i < (NB_SM_BEFORE_AVF0+1); i++ )
        {
            ring_node_for_averaging_sm_f0 = ring_node_for_averaging_sm_f0->previous;
            ring_node_tab[NB_SM_BEFORE_AVF0-i] = ring_node_for_averaging_sm_f0;
        }

        // compute the average and store it in the averaged_sm_f1 buffer
        SM_average( current_ring_node_asm_norm_f0->matrix,
                    current_ring_node_asm_burst_sbm_f0->matrix,
                    ring_node_tab,
                    nb_norm_bp1, nb_sbm_bp1 );

        // update nb_average
        nb_norm_bp1 = nb_norm_bp1 + NB_SM_BEFORE_AVF0;
        nb_norm_bp2 = nb_norm_bp2 + NB_SM_BEFORE_AVF0;
        nb_norm_asm = nb_norm_asm + NB_SM_BEFORE_AVF0;
        nb_sbm_bp1 = nb_sbm_bp1   + NB_SM_BEFORE_AVF0;
        nb_sbm_bp2 = nb_sbm_bp2   + NB_SM_BEFORE_AVF0;

        //****************************************
        // initialize the mesage for the MATR task
        msgForMATR.event      = 0x00;  // this composite event will be sent to the MATR task
        msgForMATR.burst_sbm  = current_ring_node_asm_burst_sbm_f0;
        msgForMATR.norm       = current_ring_node_asm_norm_f0;
//        msgForMATR.coarseTime = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[0];
//        msgForMATR.fineTime   = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[1];
        msgForMATR.coarseTime = time_management_regs->coarse_time;
        msgForMATR.fineTime   = time_management_regs->fine_time;

        if (nb_sbm_bp1 == nb_sm_before_f0.burst_sbm_bp1)
        {
            nb_sbm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_burst_sbm_f0 = current_ring_node_asm_burst_sbm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_BURST)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP1_F0;
            }
        }

        if (nb_sbm_bp2 == nb_sm_before_f0.burst_sbm_bp2)
        {
            nb_sbm_bp2 = 0;
            if ( (lfrCurrentMode == LFR_MODE_BURST)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP2_F0;
            }
        }

        if (nb_norm_bp1 == nb_sm_before_f0.norm_bp1)
        {
            nb_norm_bp1 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f0 = current_ring_node_asm_norm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP1_F0;
            }
        }

        if (nb_norm_bp2 == nb_sm_before_f0.norm_bp2)
        {
            nb_norm_bp2 = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP2_F0;
            }
        }

        if (nb_norm_asm == nb_sm_before_f0.norm_asm)
        {
            nb_norm_asm = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
//                PRINTF1("%lld\n", localTime)
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_ASM_F0;
            }
        }

        //*************************
        // send the message to MATR
        if (msgForMATR.event != 0x00)
        {
            status =  rtems_message_queue_send( queue_id_prc0, (char *) &msgForMATR, MSG_QUEUE_SIZE_PRC0);
        }

        if (status != RTEMS_SUCCESSFUL) {
            printf("in AVF0 *** Error sending message to MATR, code %d\n", status);
        }
    }
}

rtems_task prc0_task( rtems_task_argument lfrRequestedMode )
{
    char incomingData[MSG_QUEUE_SIZE_SEND];  // incoming data buffer
    size_t size;                            // size of the incoming TC packet
    asm_msg *incomingMsg;
    //
    spw_ioctl_pkt_send spw_ioctl_send_ASM;
    rtems_status_code status;
    rtems_id queue_id;
    rtems_id queue_id_q_p0;
    Header_TM_LFR_SCIENCE_ASM_t headerASM;
    bp_packet_with_spare packet_norm_bp1_f0;
    bp_packet            packet_norm_bp2_f0;
    bp_packet            packet_sbm_bp1_f0;
    bp_packet            packet_sbm_bp2_f0;

    unsigned long long int localTime;

    ASM_init_header( &headerASM );

    //*************
    // NORM headers
    BP_init_header_with_spare( &packet_norm_bp1_f0.header,
                               APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP1_F0,
                               PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0, NB_BINS_COMPRESSED_SM_F0 );
    BP_init_header( &packet_norm_bp2_f0.header,
                    APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP2_F0,
                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0, NB_BINS_COMPRESSED_SM_F0);

    //****************************
    // BURST SBM1 and SBM2 headers
    if ( lfrRequestedMode == LFR_MODE_BURST )
    {
        BP_init_header( &packet_sbm_bp1_f0.header,
                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP1_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header( &packet_sbm_bp2_f0.header,
                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP2_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else if ( lfrRequestedMode == LFR_MODE_SBM1 )
    {
        BP_init_header( &packet_sbm_bp1_f0.header,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP1_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header( &packet_sbm_bp2_f0.header,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP2_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else if ( lfrRequestedMode == LFR_MODE_SBM2 )
    {
        BP_init_header( &packet_sbm_bp1_f0.header,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP1_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
        BP_init_header( &packet_sbm_bp2_f0.header,
                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP2_F0,
                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
    }
    else
    {
        PRINTF1("in PRC0 *** lfrRequestedMode is %d, several headers not initialized\n", (unsigned int) lfrRequestedMode)
    }

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC0 *** ERR get_message_queue_id_send %d\n", status)
    }
    status = get_message_queue_id_prc0( &queue_id_q_p0);
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in PRC0 *** ERR get_message_queue_id_prc0 %d\n", status)
    }

    BOOT_PRINTF1("in PRC0 *** lfrRequestedMode = %d\n", (int) lfrRequestedMode)

    while(1){
        status = rtems_message_queue_receive( queue_id_q_p0, incomingData, &size, //************************************
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT );      // wait for a message coming from AVF0

        incomingMsg = (asm_msg*) incomingData;

        localTime = getTimeAsUnsignedLongLongInt( );
        //****************
        //****************
        // BURST SBM1 SBM2
        //****************
        //****************
        if (incomingMsg->event & RTEMS_EVENT_BURST_SBM_BP1_F0 )
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( incomingMsg->burst_sbm->matrix, compressed_sm_sbm_f0,
                                         nb_sm_before_f0.burst_sbm_bp1,
                                         NB_BINS_COMPRESSED_SM_SBM_F0, NB_BINS_TO_AVERAGE_ASM_SBM_F0,
                                         ASM_F0_INDICE_START);
            // 2) compute the BP1 set

            // 3) send the BP1 set
            set_time( packet_sbm_bp1_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
            set_time( packet_sbm_bp1_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
            BP_send( (char *) &packet_sbm_bp1_f0.header, queue_id,
                      PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0 + PACKET_LENGTH_DELTA);
            // 4) compute the BP2 set if needed
            if ( incomingMsg->event & RTEMS_EVENT_BURST_SBM_BP2_F0 )
            {
                // 1) compute the BP2 set

                // 2) send the BP2 set
                set_time( packet_sbm_bp2_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
                set_time( packet_sbm_bp2_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
                BP_send( (char *) &packet_sbm_bp2_f0.header, queue_id,
                          PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0 + PACKET_LENGTH_DELTA);
            }
        }

        //*****
        //*****
        // NORM
        //*****
        //*****
        if (incomingMsg->event & RTEMS_EVENT_NORM_BP1_F0)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( incomingMsg->norm->matrix, compressed_sm_norm_f0,
                                         nb_sm_before_f0.norm_bp1,
                                         NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0,
                                         ASM_F0_INDICE_START );
            // 2) compute the BP1 set

            // 3) send the BP1 set
            set_time( packet_norm_bp1_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
            set_time( packet_norm_bp1_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
            BP_send( (char *) &packet_norm_bp1_f0.header, queue_id,
                      PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0 + PACKET_LENGTH_DELTA);
            if (incomingMsg->event & RTEMS_EVENT_NORM_BP2_F0)
            {
                // 1) compute the BP2 set using the same ASM as the one used for BP1

                // 2) send the BP2 set
                set_time( packet_norm_bp2_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
                set_time( packet_norm_bp2_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
                BP_send( (char *) &packet_norm_bp2_f0.header, queue_id,
                          PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0 + PACKET_LENGTH_DELTA);
            }
        }

        if (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F0)
        {
            // 1) reorganize the ASM and divide
            ASM_reorganize_and_divide( incomingMsg->norm->matrix,
                                       asm_f0_reorganized,
                                       nb_sm_before_f0.norm_bp1 );
            // 2) convert the float array in a char array
            ASM_convert( asm_f0_reorganized, asm_f0_char);
            // 3) send the spectral matrix packets
            set_time( headerASM.time           , (unsigned char *) &incomingMsg->coarseTime );
            set_time( headerASM.acquisitionTime, (unsigned char *) &incomingMsg->coarseTime );
            ASM_send( &headerASM, asm_f0_char, SID_NORM_ASM_F0, &spw_ioctl_send_ASM, queue_id);
        }

    }
}

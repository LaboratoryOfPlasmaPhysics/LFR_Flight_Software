/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic parameters computation.
 *
 */

#include <fsw_processing.h>

#include "fsw_processing_globals.c"

//************************
// spectral matrices rings
ring_node_sm sm_ring_f0[ NB_RING_NODES_SM_F0 ];
ring_node_sm sm_ring_f1[ NB_RING_NODES_SM_F1 ];
ring_node_sm sm_ring_f2[ NB_RING_NODES_SM_F2 ];
ring_node_sm *current_ring_node_sm_f0;
ring_node_sm *current_ring_node_sm_f1;
ring_node_sm *current_ring_node_sm_f2;
ring_node_sm *ring_node_for_averaging_sm_f0;
ring_node_sm *ring_node_for_averaging_sm_f1;

ring_node_asm asm_ring_burst_sbm_f0[ NB_RING_NODES_ASM_BURST_SBM_F0 ];
ring_node_asm asm_ring_norm_f0     [ NB_RING_NODES_ASM_BURST_SBM_F0 ];
ring_node_asm *current_ring_node_asm_burst_sbm_f0;
ring_node_asm *current_ring_node_asm_burst_sbm_f1;
ring_node_asm *current_ring_node_asm_norm_f0;
ring_node_asm *current_ring_node_asm_norm_f1;

float asm_norm_f0          [ TOTAL_SIZE_SM ];
float asm_f0_reorganized   [ TOTAL_SIZE_SM ];
char  asm_f0_char          [ TIME_OFFSET_IN_BYTES + (TOTAL_SIZE_SM * 2) ];
float compressed_sm_norm_f0[ TOTAL_SIZE_COMPRESSED_ASM_F0   ];
float compressed_sm_sbm    [ TOTAL_SIZE_COMPRESSED_ASM_SBM1 ];

//***********************************************************
// Interrupt Service Routine for spectral matrices processing
void reset_nb_sm_f0( unsigned char lfrMode )
{
    nb_sm.f0 = 0;
    nb_sm.norm_bp1_f0 = 0;
    nb_sm.norm_bp2_f0 = 0;
    nb_sm.norm_asm_f0 = 0;
    nb_sm.sbm_bp1_f0 = 0;
    nb_sm.sbm_bp2_f0 = 0;

    nb_sm_before_f0.norm_bp1_f0 = parameter_dump_packet.sy_lfr_n_bp_p0 * 96;
    nb_sm_before_f0.norm_bp2_f0 = parameter_dump_packet.sy_lfr_n_bp_p1 * 96;
    nb_sm_before_f0.norm_asm_f0 = (parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1]) * 96;
    nb_sm_before_f0.sbm1_bp1_f0 =  parameter_dump_packet.sy_lfr_s1_bp_p0 * 24;
    nb_sm_before_f0.sbm1_bp2_f0 =  parameter_dump_packet.sy_lfr_s1_bp_p1 * 96;
    nb_sm_before_f0.sbm2_bp1_f0 =  parameter_dump_packet.sy_lfr_s2_bp_p0 * 96;
    nb_sm_before_f0.sbm2_bp2_f0 =  parameter_dump_packet.sy_lfr_s2_bp_p1 * 96;
    nb_sm_before_f0.burst_bp1_f0 =  parameter_dump_packet.sy_lfr_b_bp_p0 * 96;
    nb_sm_before_f0.burst_bp2_f0 =  parameter_dump_packet.sy_lfr_b_bp_p1 * 96;

    if (lfrMode == LFR_MODE_SBM1)
    {
        nb_sm_before_f0.burst_sbm_bp1_f0 =  nb_sm_before_f0.sbm1_bp1_f0;
        nb_sm_before_f0.burst_sbm_bp2_f0 =  nb_sm_before_f0.sbm1_bp2_f0;
    }
    else if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f0.burst_sbm_bp1_f0 =  nb_sm_before_f0.sbm2_bp1_f0;
        nb_sm_before_f0.burst_sbm_bp2_f0 =  nb_sm_before_f0.sbm2_bp2_f0;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f0.burst_sbm_bp1_f0 =  nb_sm_before_f0.burst_bp1_f0;
        nb_sm_before_f0.burst_sbm_bp2_f0 =  nb_sm_before_f0.burst_bp2_f0;
    }
    else
    {
        nb_sm_before_f0.burst_sbm_bp1_f0 =  nb_sm_before_f0.burst_bp1_f0;
        nb_sm_before_f0.burst_sbm_bp2_f0 =  nb_sm_before_f0.burst_bp2_f0;
    }
}

void reset_nb_sm_f1( unsigned char lfrMode )
{
    nb_sm.f1 = 0;
    nb_sm.norm_bp1_f1 = 0;
    nb_sm.norm_bp2_f1 = 0;
    nb_sm.norm_asm_f1 = 0;
    nb_sm.sbm_bp1_f1 = 0;
    nb_sm.sbm_bp2_f1 = 0;

    nb_sm_before_f1.norm_bp1_f1 = parameter_dump_packet.sy_lfr_n_bp_p0 * 16;
    nb_sm_before_f1.norm_bp2_f1 = parameter_dump_packet.sy_lfr_n_bp_p1 * 16;
    nb_sm_before_f1.norm_asm_f1 = (parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1]) * 16;
    nb_sm_before_f1.sbm2_bp1_f1 =  parameter_dump_packet.sy_lfr_s2_bp_p0 * 16;
    nb_sm_before_f1.sbm2_bp2_f1 =  parameter_dump_packet.sy_lfr_s2_bp_p1 * 16;
    nb_sm_before_f1.burst_bp1_f1 =  parameter_dump_packet.sy_lfr_b_bp_p0 * 16;
    nb_sm_before_f1.burst_bp2_f1 =  parameter_dump_packet.sy_lfr_b_bp_p1 * 16;

    if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f1.burst_sbm_bp1_f1 =  nb_sm_before_f1.sbm2_bp1_f1;
        nb_sm_before_f1.burst_sbm_bp2_f1 =  nb_sm_before_f1.sbm2_bp2_f1;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f1.burst_sbm_bp1_f1 =  nb_sm_before_f1.burst_bp1_f1;
        nb_sm_before_f1.burst_sbm_bp2_f1 =  nb_sm_before_f1.burst_bp2_f1;
    }
    else
    {
        nb_sm_before_f1.burst_sbm_bp1_f1 =  nb_sm_before_f1.burst_bp1_f1;
        nb_sm_before_f1.burst_sbm_bp2_f1 =  nb_sm_before_f1.burst_bp2_f1;
    }
}

void reset_nb_sm_f2( unsigned char lfrMode )
{
    nb_sm.f2 = 0;
    nb_sm.norm_bp1_f2 = 0;
    nb_sm.norm_bp2_f2 = 0;
    nb_sm.norm_asm_f2 = 0;
    nb_sm.sbm_bp1_f2 = 0;
    nb_sm.sbm_bp2_f2 = 0;

    nb_sm_before_f2.norm_bp1_f2  = parameter_dump_packet.sy_lfr_n_bp_p0;
    nb_sm_before_f2.norm_bp2_f2  = parameter_dump_packet.sy_lfr_n_bp_p1;
    nb_sm_before_f2.norm_asm_f2  = parameter_dump_packet.sy_lfr_n_asm_p[0] * 256 + parameter_dump_packet.sy_lfr_n_asm_p[1];
    nb_sm_before_f2.sbm2_bp1_f2  =  parameter_dump_packet.sy_lfr_s2_bp_p0;
    nb_sm_before_f2.sbm2_bp2_f2  =  parameter_dump_packet.sy_lfr_s2_bp_p1;
    nb_sm_before_f2.burst_bp1_f2 =  parameter_dump_packet.sy_lfr_b_bp_p0;
    nb_sm_before_f2.burst_bp2_f2 =  parameter_dump_packet.sy_lfr_b_bp_p1;

    if (lfrMode == LFR_MODE_SBM2)
    {
        nb_sm_before_f2.burst_sbm_bp1_f2 =  nb_sm_before_f2.sbm2_bp1_f2;
        nb_sm_before_f2.burst_sbm_bp2_f2 =  nb_sm_before_f2.sbm2_bp2_f2;
    }
    else if (lfrMode == LFR_MODE_BURST)
    {
        nb_sm_before_f2.burst_sbm_bp1_f2 =  nb_sm_before_f2.burst_bp1_f2;
        nb_sm_before_f2.burst_sbm_bp2_f2 =  nb_sm_before_f2.burst_bp2_f2;
    }
    else
    {
        nb_sm_before_f2.burst_sbm_bp1_f2 =  nb_sm_before_f2.burst_bp1_f2;
        nb_sm_before_f2.burst_sbm_bp2_f2 =  nb_sm_before_f2.burst_bp2_f2;
    }
}

rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{
    ring_node_sm *previous_ring_node_sm_f0;

//    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );

    previous_ring_node_sm_f0 = current_ring_node_sm_f0;

    if ( (spectral_matrix_regs->status & 0x2) == 0x02)  // check ready matrix bit f0_1
    {
        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
        spectral_matrix_regs->matrixF0_Address0 = current_ring_node_sm_f0->buffer_address;
        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffd;   // 1101
        nb_sm.f0 = nb_sm.f0 + 1;
    }

    //************************
    // reset status error bits
    if ( (spectral_matrix_regs->status & 0x30) != 0x00)
    {
        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );
        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xffffffcf;   // 1100 1111
    }

    //**************************************
    // reset ready matrix bits for f0_0, f1 and f2
    spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffff2;       // 0010

    if (nb_sm.f0 == NB_SM_BEFORE_AVF0)
    {
        ring_node_for_averaging_sm_f0 = previous_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm.f0 = 0;
    }

}

rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector )
{
    if (nb_sm.f0 == (NB_SM_BEFORE_AVF0-1) )
    {
        ring_node_for_averaging_sm_f0 = current_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm.f0 = 0;
    }
    else
    {
        nb_sm.f0 = nb_sm.f0 + 1;
    }
}

//************
// RTEMS TASKS

rtems_task smiq_task( rtems_task_argument argument ) // process the Spectral Matrices IRQ
{
    rtems_event_set event_out;

    BOOT_PRINTF("in SMIQ *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
    }
}

rtems_task avf0_task( rtems_task_argument lfrRequestedMode )
{
    int i;

    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc0;
    asm_msg msgForMATR;
    ring_node_sm *ring_node_tab[8];

    reset_nb_sm_f0( lfrRequestedMode );   // reset the sm counters that drive the BP and ASM computations / transmissions

    BOOT_PRINTF1("in AVFO *** lfrRequestedMode = %d\n", lfrRequestedMode)

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
                    nb_sm.norm_bp1_f0, nb_sm.sbm_bp1_f0 );

        // update nb_average
        nb_sm.norm_bp1_f0 = nb_sm.norm_bp1_f0 + NB_SM_BEFORE_AVF0;
        nb_sm.norm_bp2_f0 = nb_sm.norm_bp2_f0 + NB_SM_BEFORE_AVF0;
        nb_sm.norm_asm_f0 = nb_sm.norm_asm_f0 + NB_SM_BEFORE_AVF0;
        nb_sm.sbm_bp1_f0 = nb_sm.sbm_bp1_f0   + NB_SM_BEFORE_AVF0;
        nb_sm.sbm_bp2_f0 = nb_sm.sbm_bp2_f0   + NB_SM_BEFORE_AVF0;

        //****************************************
        // initialize the mesage for the MATR task
        msgForMATR.event      = 0x00;  // this composite event will be sent to the MATR task
        msgForMATR.burst_sbm  = current_ring_node_asm_burst_sbm_f0;
        msgForMATR.norm       = current_ring_node_asm_norm_f0;
        msgForMATR.coarseTime = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[0];
        msgForMATR.fineTime   = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[1];

        if (nb_sm.sbm_bp1_f0 == nb_sm_before_f0.burst_sbm_bp1_f0)
        {
            nb_sm.sbm_bp1_f0 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_burst_sbm_f0 = current_ring_node_asm_burst_sbm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_BURST)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP1_F0;
            }
        }

        if (nb_sm.sbm_bp2_f0 == nb_sm_before_f0.burst_sbm_bp2_f0)
        {
            nb_sm.sbm_bp2_f0 = 0;
            if ( (lfrCurrentMode == LFR_MODE_BURST)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP2_F0;
            }
        }

        if (nb_sm.norm_bp1_f0 == nb_sm_before_f0.norm_bp1_f0)
        {
            nb_sm.norm_bp1_f0 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f0 = current_ring_node_asm_norm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP1_F0;
            }
        }

        if (nb_sm.norm_bp2_f0 == nb_sm_before_f0.norm_bp2_f0)
        {
            nb_sm.norm_bp2_f0 = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP2_F0;
            }
        }

        if (nb_sm.norm_asm_f0 == nb_sm_before_f0.norm_asm_f0)
        {
            nb_sm.norm_asm_f0 = 0;
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
            status =  rtems_message_queue_send( queue_id_prc0, (char *) & msgForMATR, MSG_QUEUE_SIZE_PRC0);
        }

        if (status != RTEMS_SUCCESSFUL) {
            printf("in AVF0 *** Error sending message to MATR, code %d\n", status);
        }
    }
}

rtems_task prc0_task( rtems_task_argument lfrRequestedMode )
{
//    char incomingData[MSG_QUEUE_SIZE_SEND];  // incoming data buffer
//    size_t size;                            // size of the incoming TC packet
//    asm_msg *incomingMsg;
//    //
//    spw_ioctl_pkt_send spw_ioctl_send_ASM;
//    rtems_status_code status;
//    rtems_id queue_id;
//    rtems_id queue_id_matr;
//    Header_TM_LFR_SCIENCE_ASM_t headerASM;
//    bp_packet_with_spare current_node_norm_bp1_f0;
//    bp_packet            current_node_norm_bp2_f0;
//    bp_packet            current_node_sbm_bp1_f0;
//    bp_packet            current_node_sbm_bp2_f0;

//    unsigned long long int localTime;

//    ASM_init_header( &headerASM );

//    //*************
//    // NORM headers
//    BP_init_header_with_spare( &current_node_norm_bp1_f0.header,
//                               APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP1_F0,
//                               PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0, NB_BINS_COMPRESSED_SM_F0 );
//    BP_init_header( &current_node_norm_bp2_f0.header,
//                    APID_TM_SCIENCE_NORMAL_BURST, SID_NORM_BP2_F0,
//                    PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0, NB_BINS_COMPRESSED_SM_F0);

//    //****************************
//    // BURST SBM1 and SBM2 headers
//    if ( (lfrRequestedMode == LFR_MODE_BURST)
//         || (lfrRequestedMode == LFR_MODE_NORMAL) || (lfrRequestedMode == LFR_MODE_STANDBY) )
//    {
//        BP_init_header( &current_node_sbm_bp1_f0.header,
//                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP1_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//        BP_init_header( &current_node_sbm_bp2_f0.header,
//                        APID_TM_SCIENCE_NORMAL_BURST, SID_BURST_BP2_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//    }
//    else if ( lfrRequestedMode == LFR_MODE_SBM1 )
//    {
//        BP_init_header( &current_node_sbm_bp1_f0.header,
//                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP1_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//        BP_init_header( &current_node_sbm_bp2_f0.header,
//                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM1_BP2_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//    }
//    else if ( lfrRequestedMode == LFR_MODE_SBM2 )
//    {
//        BP_init_header( &current_node_sbm_bp1_f0.header,
//                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP1_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//        BP_init_header( &current_node_sbm_bp2_f0.header,
//                        APID_TM_SCIENCE_SBM1_SBM2, SID_SBM2_BP2_F0,
//                        PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0, NB_BINS_COMPRESSED_SM_SBM_F0);
//    }
//    else
//    {
//        PRINTF1("ERR *** in MATR *** unexpected lfrRequestedMode passed as argument = %d\n", (unsigned int) lfrRequestedMode)
//    }

//    status =  get_message_queue_id_send( &queue_id );
//    if (status != RTEMS_SUCCESSFUL)
//    {
//        PRINTF1("in MATR *** ERR get_message_queue_id_send %d\n", status)
//    }
//    status = get_message_queue_id_prc0( &queue_id_matr);
//    if (status != RTEMS_SUCCESSFUL)
//    {
//        PRINTF1("in MATR *** ERR get_message_queue_id_prc0 %d\n", status)
//    }

//    BOOT_PRINTF1("in PRC0 *** lfrRequestedMode = %d\n", lfrRequestedMode)

//    while(1){
//        status = rtems_message_queue_receive( queue_id_matr, incomingData, &size,   //************************************
//                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT );        // wait for a message coming from AVF0

//        incomingMsg = (asm_msg*) incomingData;

//        localTime = getTimeAsUnsignedLongLongInt( );
//        //****************
//        //****************
//        // BURST SBM1 SBM2
//        //****************
//        //****************
//        if (incomingMsg->event & RTEMS_EVENT_BURST_SBM_BP1_F0 )
//        {
//            // 1)  compress the matrix for Basic Parameters calculation
//            ASM_compress_reorganize_and_divide( incomingMsg->burst_sbm->matrix, compressed_sm_sbm,
//                                         nb_sm_before_f0.burst_sbm_bp1_f0,
//                                         NB_BINS_COMPRESSED_SM_SBM_F0, NB_BINS_TO_AVERAGE_ASM_SBM_F0,
//                                         ASM_F0_INDICE_START);
//            // 2) compute the BP1 set

//            // 3) send the BP1 set
//            set_time( current_node_sbm_bp1_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
//            set_time( current_node_sbm_bp1_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
//            BP_send( (char *) &current_node_sbm_bp1_f0.header, queue_id,
//                      PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0 + PACKET_LENGTH_DELTA);
//            // 4) compute the BP2 set if needed
//            if ( incomingMsg->event & RTEMS_EVENT_BURST_SBM_BP2_F0 )
//            {
//                // 1) compute the BP2 set

//                // 2) send the BP2 set
//                set_time( current_node_sbm_bp2_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
//                set_time( current_node_sbm_bp2_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
//                BP_send( (char *) &current_node_sbm_bp2_f0.header, queue_id,
//                          PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0 + PACKET_LENGTH_DELTA);
//            }
//        }

//        //*****
//        //*****
//        // NORM
//        //*****
//        //*****
//        if (incomingMsg->event & RTEMS_EVENT_NORM_BP1_F0)
//        {
//            // 1)  compress the matrix for Basic Parameters calculation
//            ASM_compress_reorganize_and_divide( incomingMsg->norm->matrix, compressed_sm_norm_f0,
//                                         nb_sm_before_f0.norm_bp1_f0,
//                                         NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0,
//                                         ASM_F0_INDICE_START );
//            // 2) compute the BP1 set

//            // 3) send the BP1 set
//            set_time( current_node_norm_bp1_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
//            set_time( current_node_norm_bp1_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
//            BP_send( (char *) &current_node_norm_bp1_f0.header, queue_id,
//                      PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0 + PACKET_LENGTH_DELTA);
//            if (incomingMsg->event & RTEMS_EVENT_NORM_BP2_F0)
//            {
//                // 1) compute the BP2 set

//                // 2) send the BP2 set
//                set_time( current_node_norm_bp2_f0.header.time,            (unsigned char *) &incomingMsg->coarseTime );
//                set_time( current_node_norm_bp2_f0.header.acquisitionTime, (unsigned char *) &incomingMsg->fineTime   );
//                BP_send( (char *) &current_node_norm_bp2_f0.header, queue_id,
//                          PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0 + PACKET_LENGTH_DELTA);
//            }
//        }

//        if (incomingMsg->event & RTEMS_EVENT_NORM_ASM_F0)
//        {
//            // 1) reorganize the ASM and divide
//            ASM_reorganize_and_divide( incomingMsg->norm->matrix, asm_f0_reorganized, NB_SM_BEFORE_NORM_BP1_F0 );
//            // 2) convert the float array in a char array
//            ASM_convert( asm_f0_reorganized, asm_f0_char);
//            // 3) send the spectral matrix packets
//            set_time( headerASM.time           , (unsigned char *) &incomingMsg->coarseTime );
//            set_time( headerASM.acquisitionTime, (unsigned char *) &incomingMsg->coarseTime );
//            ASM_send( &headerASM, asm_f0_char, SID_NORM_ASM_F0, &spw_ioctl_send_ASM, queue_id);
//        }

//    }
}

rtems_task avf1_task( rtems_task_argument lfrRequestedMode )
{
    int i;

    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id_prc1;
    asm_msg msgForMATR;
    ring_node_sm *ring_node_tab[8];

    reset_nb_sm_f1( lfrRequestedMode );   // reset the sm counters that drive the BP and ASM computations / transmissions

    BOOT_PRINTF1("in AVF1 *** lfrRequestedMode = %d\n", lfrRequestedMode)

    status = get_message_queue_id_prc1( &queue_id_prc1 );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in MATR *** ERR get_message_queue_id_prc0 %d\n", status)
    }

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        ring_node_tab[NB_SM_BEFORE_AVF1-1] = ring_node_for_averaging_sm_f1;
        for ( i = 2; i < (NB_SM_BEFORE_AVF1+1); i++ )
        {
            ring_node_for_averaging_sm_f0 = ring_node_for_averaging_sm_f0->previous;
            ring_node_tab[NB_SM_BEFORE_AVF1-i] = ring_node_for_averaging_sm_f0;
        }

        // compute the average and store it in the averaged_sm_f1 buffer
        SM_average( current_ring_node_asm_norm_f1->matrix,
                    current_ring_node_asm_burst_sbm_f1->matrix,
                    ring_node_tab,
                    nb_sm.norm_bp1_f1, nb_sm.sbm_bp1_f1 );

        // update nb_average
        nb_sm.norm_bp1_f1 = nb_sm.norm_bp1_f1 + NB_SM_BEFORE_AVF1;
        nb_sm.norm_bp2_f1 = nb_sm.norm_bp2_f1 + NB_SM_BEFORE_AVF1;
        nb_sm.norm_asm_f1 = nb_sm.norm_asm_f1 + NB_SM_BEFORE_AVF1;
        nb_sm.sbm_bp1_f1 = nb_sm.sbm_bp1_f1   + NB_SM_BEFORE_AVF1;
        nb_sm.sbm_bp2_f1 = nb_sm.sbm_bp2_f1   + NB_SM_BEFORE_AVF1;

        //****************************************
        // initialize the mesage for the MATR task
        msgForMATR.event      = 0x00;  // this composite event will be sent to the MATR task
        msgForMATR.burst_sbm  = current_ring_node_asm_burst_sbm_f1;
        msgForMATR.norm       = current_ring_node_asm_norm_f1;
        msgForMATR.coarseTime = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[0];
        msgForMATR.fineTime   = ( (unsigned int *) (ring_node_tab[0]->buffer_address) )[1];

        if (nb_sm.sbm_bp1_f1 == nb_sm_before_f1.burst_sbm_bp1_f1)
        {
            nb_sm.sbm_bp1_f0 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_burst_sbm_f0 = current_ring_node_asm_burst_sbm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_BURST) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP1_F1;
            }
        }

        if (nb_sm.sbm_bp2_f1 == nb_sm_before_f1.burst_sbm_bp2_f1)
        {
            nb_sm.sbm_bp2_f0 = 0;
            if ( (lfrCurrentMode == LFR_MODE_BURST) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_BURST_SBM_BP2_F1;
            }
        }

        if (nb_sm.norm_bp1_f1 == nb_sm_before_f1.norm_bp1_f1)
        {
            nb_sm.norm_bp1_f0 = 0;
            // set another ring for the ASM storage
            current_ring_node_asm_norm_f0 = current_ring_node_asm_norm_f0->next;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP1_F0;
            }
        }

        if (nb_sm.norm_bp2_f1 == nb_sm_before_f1.norm_bp2_f1)
        {
            nb_sm.norm_bp2_f0 = 0;
            if ( (lfrCurrentMode == LFR_MODE_NORMAL)
                 || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
            {
                msgForMATR.event = msgForMATR.event | RTEMS_EVENT_NORM_BP2_F1;
            }
        }

        if (nb_sm.norm_asm_f1 == nb_sm_before_f1.norm_asm_f1)
        {
            nb_sm.norm_asm_f0 = 0;
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
            status =  rtems_message_queue_send( queue_id_prc1, (char *) & msgForMATR, MSG_QUEUE_SIZE_PRC1);
        }

        if (status != RTEMS_SUCCESSFUL) {
            printf("in AVF1 *** Error sending message to PRC1, code %d\n", status);
        }
    }
}

//******************
// Spectral Matrices

void SM_init_rings( void )
{
    unsigned char i;

    // F0 RING
    sm_ring_f0[0].next            = (ring_node_sm*) &sm_ring_f0[1];
    sm_ring_f0[0].previous        = (ring_node_sm*) &sm_ring_f0[NB_RING_NODES_SM_F0-1];
    sm_ring_f0[0].buffer_address  =
            (int) &sm_f0[ 0 ];

    sm_ring_f0[NB_RING_NODES_SM_F0-1].next           = (ring_node_sm*) &sm_ring_f0[0];
    sm_ring_f0[NB_RING_NODES_SM_F0-1].previous       = (ring_node_sm*) &sm_ring_f0[NB_RING_NODES_SM_F0-2];
    sm_ring_f0[NB_RING_NODES_SM_F0-1].buffer_address =
            (int) &sm_f0[ (NB_RING_NODES_SM_F0-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_SM_F0-1; i++)
    {
        sm_ring_f0[i].next            = (ring_node_sm*) &sm_ring_f0[i+1];
        sm_ring_f0[i].previous        = (ring_node_sm*) &sm_ring_f0[i-1];
        sm_ring_f0[i].buffer_address  =
                (int) &sm_f0[ i * TOTAL_SIZE_SM ];
    }

    // F1 RING
    sm_ring_f1[0].next            = (ring_node_sm*) &sm_ring_f1[1];
    sm_ring_f1[0].previous        = (ring_node_sm*) &sm_ring_f1[NB_RING_NODES_SM_F1-1];
    sm_ring_f1[0].buffer_address  =
            (int) &sm_f1[ 0 ];

    sm_ring_f1[NB_RING_NODES_SM_F1-1].next           = (ring_node_sm*) &sm_ring_f1[0];
    sm_ring_f1[NB_RING_NODES_SM_F1-1].previous       = (ring_node_sm*) &sm_ring_f1[NB_RING_NODES_SM_F1-2];
    sm_ring_f1[NB_RING_NODES_SM_F1-1].buffer_address =
            (int) &sm_f1[ (NB_RING_NODES_SM_F1-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_SM_F1-1; i++)
    {
        sm_ring_f1[i].next            = (ring_node_sm*) &sm_ring_f1[i+1];
        sm_ring_f1[i].previous        = (ring_node_sm*) &sm_ring_f1[i-1];
        sm_ring_f1[i].buffer_address  =
                (int) &sm_f1[ i * TOTAL_SIZE_SM ];
    }

    // F2 RING
    sm_ring_f2[0].next            = (ring_node_sm*) &sm_ring_f2[1];
    sm_ring_f2[0].previous        = (ring_node_sm*) &sm_ring_f2[NB_RING_NODES_SM_F2-1];
    sm_ring_f2[0].buffer_address  =
            (int) &sm_f2[ 0 ];

    sm_ring_f2[NB_RING_NODES_SM_F2-1].next           = (ring_node_sm*) &sm_ring_f2[0];
    sm_ring_f2[NB_RING_NODES_SM_F2-1].previous       = (ring_node_sm*) &sm_ring_f2[NB_RING_NODES_SM_F2-2];
    sm_ring_f2[NB_RING_NODES_SM_F2-1].buffer_address =
            (int) &sm_f2[ (NB_RING_NODES_SM_F2-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_SM_F2-1; i++)
    {
        sm_ring_f2[i].next            = (ring_node_sm*) &sm_ring_f2[i+1];
        sm_ring_f2[i].previous        = (ring_node_sm*) &sm_ring_f2[i-1];
        sm_ring_f2[i].buffer_address  =
                (int) &sm_f2[ i * TOTAL_SIZE_SM ];
    }

    DEBUG_PRINTF1("asm_ring_f0 @%x\n", (unsigned int) sm_ring_f0)
    DEBUG_PRINTF1("asm_ring_f1 @%x\n", (unsigned int) sm_ring_f1)
    DEBUG_PRINTF1("asm_ring_f2 @%x\n", (unsigned int) sm_ring_f2)

    spectral_matrix_regs->matrixF0_Address0 = sm_ring_f0[0].buffer_address;
    DEBUG_PRINTF1("spectral_matrix_regs->matrixF0_Address0 @%x\n", spectral_matrix_regs->matrixF0_Address0)
}

void ASM_init_rings( void )
{
    unsigned char i;

    //*************
    // BURST_SBM_F0
    asm_ring_burst_sbm_f0[0].next            = (ring_node_asm*) &asm_ring_burst_sbm_f0[1];
    asm_ring_burst_sbm_f0[0].previous        = (ring_node_asm*) &asm_ring_burst_sbm_f0[NB_RING_NODES_ASM_BURST_SBM_F0-1];

    asm_ring_burst_sbm_f0[NB_RING_NODES_ASM_BURST_SBM_F0-1].next
            = (ring_node_asm*) &asm_ring_burst_sbm_f0[0];
    asm_ring_burst_sbm_f0[NB_RING_NODES_ASM_BURST_SBM_F0-1].previous
            = (ring_node_asm*) &asm_ring_burst_sbm_f0[NB_RING_NODES_ASM_BURST_SBM_F0-2];

    for(i=1; i<NB_RING_NODES_ASM_BURST_SBM_F0-1; i++)
    {
        asm_ring_burst_sbm_f0[i].next            = (ring_node_asm*) &asm_ring_burst_sbm_f0[i+1];
        asm_ring_burst_sbm_f0[i].previous        = (ring_node_asm*) &asm_ring_burst_sbm_f0[i-1];
    }

    //*************
    // NORM_F0
    asm_ring_norm_f0[0].next            = (ring_node_asm*) &asm_ring_norm_f0[1];
    asm_ring_norm_f0[0].previous        = (ring_node_asm*) &asm_ring_norm_f0[NB_RING_NODES_ASM_BURST_SBM_F0-1];

    asm_ring_norm_f0[NB_RING_NODES_ASM_NORM_F0-1].next
            = (ring_node_asm*) &asm_ring_norm_f0[0];
    asm_ring_norm_f0[NB_RING_NODES_ASM_NORM_F0-1].previous
            = (ring_node_asm*) &asm_ring_norm_f0[NB_RING_NODES_ASM_NORM_F0-2];

    for(i=1; i<NB_RING_NODES_ASM_NORM_F0-1; i++)
    {
        asm_ring_norm_f0[i].next            = (ring_node_asm*) &asm_ring_norm_f0[i+1];
        asm_ring_norm_f0[i].previous        = (ring_node_asm*) &asm_ring_norm_f0[i-1];
    }
}

void SM_reset_current_ring_nodes( void )
{
    current_ring_node_sm_f0 = sm_ring_f0;
    current_ring_node_sm_f1 = sm_ring_f1;
    current_ring_node_sm_f2 = sm_ring_f2;

    ring_node_for_averaging_sm_f0   = sm_ring_f0;
}

void ASM_reset_current_ring_node( void )
{
    current_ring_node_asm_norm_f0      = asm_ring_norm_f0;
    current_ring_node_asm_burst_sbm_f0 = asm_ring_burst_sbm_f0;
}

void ASM_init_header( Header_TM_LFR_SCIENCE_ASM_t *header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
    header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
    header->packetSequenceControl[0] = 0xc0;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = 0x00;
    header->packetLength[1] = 0x00;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = 0x10;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    header->sid = 0x00;
    header->biaStatusInfo = 0x00;
    header->pa_lfr_pkt_cnt_asm = 0x00;
    header->pa_lfr_pkt_nr_asm = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->pa_lfr_asm_blk_nr[0] = 0x00;  // BLK_NR MSB
    header->pa_lfr_asm_blk_nr[1] = 0x00;  // BLK_NR LSB
}

void SM_average( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int nbAverageNormF0, unsigned int nbAverageSBM1F0 )
{
    float sum;
    unsigned int i;

    for(i=0; i<TOTAL_SIZE_SM; i++)
    {
        sum = ( (int *) (ring_node_tab[0]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[1]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[2]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[3]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[4]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[5]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[6]->buffer_address) ) [ i ]
                + ( (int *) (ring_node_tab[7]->buffer_address) ) [ i ];

        if ( (nbAverageNormF0 == 0) && (nbAverageSBM1F0 == 0) )
        {
            averaged_spec_mat_f0[ i ] = sum;
            averaged_spec_mat_f1[ i ] = sum;
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 != 0) )
        {
            averaged_spec_mat_f0[ i ] = ( averaged_spec_mat_f0[  i ] + sum );
            averaged_spec_mat_f1[ i ] = ( averaged_spec_mat_f1[  i ] + sum );
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 == 0) )
        {
            averaged_spec_mat_f0[ i ] = ( averaged_spec_mat_f0[ i ] + sum );
            averaged_spec_mat_f1[ i ] = sum;
        }
        else
        {
            PRINTF2("ERR *** in SM_average *** unexpected parameters %d %d\n", nbAverageNormF0, nbAverageSBM1F0)
        }
    }
}

void ASM_reorganize_and_divide( float *averaged_spec_mat, float *averaged_spec_mat_reorganized, float divider )
{
    int frequencyBin;
    int asmComponent;
    unsigned int offsetAveragedSpecMatReorganized;
    unsigned int offsetAveragedSpecMat;

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++ )
        {
            offsetAveragedSpecMatReorganized =
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetAveragedSpecMat            =
                    asmComponent * NB_BINS_PER_SM
                    + frequencyBin;
            averaged_spec_mat_reorganized[offsetAveragedSpecMatReorganized  ] =
                    averaged_spec_mat[ offsetAveragedSpecMat ] / divider;
        }
    }
}

void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat , float divider,
                                 unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage, unsigned char ASMIndexStart )
{
    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int k;

    // build data
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix; frequencyBin++ )
        {
            offsetCompressed =  // NO TIME OFFSET
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetASM =         // NO TIME OFFSET
                    asmComponent * NB_BINS_PER_SM
                    + ASMIndexStart
                    + frequencyBin * nbBinsToAverage;
            compressed_spec_mat[ offsetCompressed ] = 0;
            for ( k = 0; k < nbBinsToAverage; k++ )
            {
                compressed_spec_mat[offsetCompressed ] =
                        ( compressed_spec_mat[ offsetCompressed ]
                        + averaged_spec_mat[ offsetASM + k ] ) / (divider * nbBinsToAverage);
            }
        }
    }
}

void ASM_convert( volatile float *input_matrix, char *output_matrix)
{
    unsigned int frequencyBin;
    unsigned int asmComponent;
    char * pt_char_input;
    char * pt_char_output;
    unsigned int offsetInput;
    unsigned int offsetOutput;

    pt_char_input = (char*) &input_matrix;
    pt_char_output = (char*) &output_matrix;

    // convert all other data
    for( frequencyBin=0; frequencyBin<NB_BINS_PER_SM; frequencyBin++)
    {
        for ( asmComponent=0; asmComponent<NB_VALUES_PER_SM; asmComponent++)
        {
            offsetInput  =       (frequencyBin*NB_VALUES_PER_SM) + asmComponent   ;
            offsetOutput = 2 * ( (frequencyBin*NB_VALUES_PER_SM) + asmComponent ) ;
            pt_char_input =  (char*) &input_matrix [ offsetInput  ];
            pt_char_output = (char*) &output_matrix[ offsetOutput ];
            pt_char_output[0] = pt_char_input[0];   // bits 31 downto 24 of the float
            pt_char_output[1] = pt_char_input[1];   // bits 23 downto 16 of the float
        }
    }
}

void ASM_send(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id)
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;

    for (i=0; i<2; i++)
    {
        // (1) BUILD THE DATA
        switch(sid)
        {
        case SID_NORM_ASM_F0:
            spw_ioctl_send->dlen = TOTAL_SIZE_ASM_F0_IN_BYTES / 2;
            spw_ioctl_send->data = &spectral_matrix[
                    ( (ASM_F0_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F0) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F0) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F0);        // BLK_NR LSB
            break;
        case SID_NORM_ASM_F1:
            break;
        case SID_NORM_ASM_F2:
            break;
        default:
            PRINTF1("ERR *** in ASM_send *** unexpected sid %d\n", sid)
            break;
        }
        spw_ioctl_send->hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM + CCSDS_PROTOCOLE_EXTRA_BYTES;
        spw_ioctl_send->hdr = (char *) header;
        spw_ioctl_send->options = 0;

        // (2) BUILD THE HEADER
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        header->sid = (unsigned char) sid;   // SID
        header->pa_lfr_pkt_cnt_asm = 2;
        header->pa_lfr_pkt_nr_asm = (unsigned char) (i+1);

        // (3) SET PACKET TIME
        header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->time[3] = (unsigned char) (time_management_regs->coarse_time);
        header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->time[5] = (unsigned char) (time_management_regs->fine_time);
        //
        header->acquisitionTime[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->acquisitionTime[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->acquisitionTime[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->acquisitionTime[3] = (unsigned char) (time_management_regs->coarse_time);
        header->acquisitionTime[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->acquisitionTime[5] = (unsigned char) (time_management_regs->fine_time);

        // (4) SEND PACKET
        status =  rtems_message_queue_send( queue_id, spw_ioctl_send, ACTION_MSG_SPW_IOCTL_SEND_SIZE);
        if (status != RTEMS_SUCCESSFUL) {
            printf("in ASM_send *** ERR %d\n", (int) status);
        }
    }
}

//*****************
// Basic Parameters

void BP_init_header( Header_TM_LFR_SCIENCE_BP_t *header,
                     unsigned int apid, unsigned char sid,
                     unsigned int packetLength, unsigned char blkNr )
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (apid >> 8);
    header->packetID[1] = (unsigned char) (apid);
    header->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = (unsigned char) (packetLength >> 8);
    header->packetLength[1] = (unsigned char) (packetLength);
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = 0x10;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    header->sid = sid;
    header->biaStatusInfo = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->pa_lfr_bp_blk_nr[0] = 0x00;  // BLK_NR MSB
    header->pa_lfr_bp_blk_nr[1] = blkNr;  // BLK_NR LSB
}

void BP_init_header_with_spare(Header_TM_LFR_SCIENCE_BP_with_spare_t *header,
                                unsigned int apid, unsigned char sid,
                                unsigned int packetLength , unsigned char blkNr)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (apid >> 8);
    header->packetID[1] = (unsigned char) (apid);
    header->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = (unsigned char) (packetLength >> 8);
    header->packetLength[1] = (unsigned char) (packetLength);
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = 0x10;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    header->sid = sid;
    header->biaStatusInfo = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->pa_lfr_bp_blk_nr[0] = 0x00;  // BLK_NR MSB
    header->pa_lfr_bp_blk_nr[1] = blkNr;  // BLK_NR LSB
}

void BP_send(char *data, rtems_id queue_id, unsigned int nbBytesToSend )
{
    rtems_status_code status;

    // SEND PACKET
    status =  rtems_message_queue_send( queue_id, data, nbBytesToSend);
    if (status != RTEMS_SUCCESSFUL)
    {
        printf("ERR *** in BP_send *** ERR %d\n", (int) status);
    }
}

//******************
// general functions

void reset_spectral_matrix_regs( void )
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

    spectral_matrix_regs->config = 0x00;
    spectral_matrix_regs->status = 0x00;

    spectral_matrix_regs->matrixF0_Address0 = current_ring_node_sm_f0->buffer_address;
    spectral_matrix_regs->matrixFO_Address1 = current_ring_node_sm_f0->buffer_address;
    spectral_matrix_regs->matrixF1_Address = current_ring_node_sm_f1->buffer_address;
    spectral_matrix_regs->matrixF2_Address = current_ring_node_sm_f2->buffer_address;
}

void set_time( unsigned char *time, unsigned char * timeInBuffer )
{
//    time[0] = timeInBuffer[2];
//    time[1] = timeInBuffer[3];
//    time[2] = timeInBuffer[0];
//    time[3] = timeInBuffer[1];
//    time[4] = timeInBuffer[6];
//    time[5] = timeInBuffer[7];

    time[0] = timeInBuffer[0];
    time[1] = timeInBuffer[1];
    time[2] = timeInBuffer[2];
    time[3] = timeInBuffer[3];
    time[4] = timeInBuffer[6];
    time[5] = timeInBuffer[7];
}



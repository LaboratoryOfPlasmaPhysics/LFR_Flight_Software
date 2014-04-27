/** Functions related to data processing.
 *
 * @file
 * @author P. LEROY
 *
 * These function are related to data processing, i.e. spectral matrices averaging and basic parameters computation.
 *
 */

#include "fsw_processing.h"
#include "fsw_processing_globals.c"

unsigned int nb_sm_f0;
unsigned int nb_sm_f0_aux_f1;
unsigned int nb_sm_f1;
unsigned int nb_sm_f0_aux_f2;

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
ring_node_sm *ring_node_for_averaging_sm_f2;

//***********************************************************
// Interrupt Service Routine for spectral matrices processing

rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{
//    ring_node_sm *previous_ring_node_sm_f0;

////    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );

//    previous_ring_node_sm_f0 = current_ring_node_sm_f0;

//    if ( (spectral_matrix_regs->status & 0x2) == 0x02)  // check ready matrix bit f0_1
//    {
//        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
//        spectral_matrix_regs->matrixF0_Address0 = current_ring_node_sm_f0->buffer_address;
//        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffd;   // 1101
//        nb_sm_f0 = nb_sm_f0 + 1;
//    }

//    //************************
//    // reset status error bits
//    if ( (spectral_matrix_regs->status & 0x30) != 0x00)
//    {
//        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );
//        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xffffffcf;   // 1100 1111
//    }

//    //**************************************
//    // reset ready matrix bits for f0_0, f1 and f2
//    spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffff2;       // 0010

//    if (nb_sm_f0 == NB_SM_BEFORE_AVF0)
//    {
//        ring_node_for_averaging_sm_f0 = previous_ring_node_sm_f0;
//        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
//        {
//            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
//        }
//        nb_sm_f0 = 0;
//    }

}

rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector )
{
    //***
    // F0
    nb_sm_f0 = nb_sm_f0 + 1;
    if (nb_sm_f0 == NB_SM_BEFORE_AVF0 )
    {
        ring_node_for_averaging_sm_f0 = current_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm_f0 = 0;
    }

    //***
    // F1
    nb_sm_f0_aux_f1 = nb_sm_f0_aux_f1 + 1;
    if (nb_sm_f0_aux_f1 == 6)
    {
        nb_sm_f0_aux_f1 = 0;
        nb_sm_f1 = nb_sm_f1 + 1;
    }
    if (nb_sm_f1 == NB_SM_BEFORE_AVF1 )
    {
        ring_node_for_averaging_sm_f1 = current_ring_node_sm_f1;
        if (rtems_event_send( Task_id[TASKID_AVF1], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm_f1 = 0;
    }

    //***
    // F2
//    nb_sm_f0_aux_f2 = nb_sm_f0_aux_f2 + 1;
//    if (nb_sm_f0_aux_f2 == 96)
//    {
//        nb_sm_f0_aux_f2 = 0;
//        ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2;
//        if (rtems_event_send( Task_id[TASKID_AVF2], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
//        {
//            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
//        }
//    }
}

//******************
// Spectral Matrices

void reset_nb_sm( void )
{
    nb_sm_f0 = 0;
    nb_sm_f0_aux_f1 = 0;
    nb_sm_f0_aux_f2 = 0;

    nb_sm_f1 = 0;
}

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

void ASM_generic_init_ring( ring_node_asm *ring, unsigned char nbNodes )
{
    unsigned char i;

    ring[ nbNodes - 1 ].next
            = (ring_node_asm*) &ring[ 0 ];

    for(i=0; i<nbNodes-1; i++)
    {
        ring[ i ].next            = (ring_node_asm*) &ring[ i + 1 ];
    }
}

void SM_reset_current_ring_nodes( void )
{
    current_ring_node_sm_f0 = sm_ring_f0;
    current_ring_node_sm_f1 = sm_ring_f1;
    current_ring_node_sm_f2 = sm_ring_f2;

    ring_node_for_averaging_sm_f0 = sm_ring_f0;
    ring_node_for_averaging_sm_f1 = sm_ring_f1;
    ring_node_for_averaging_sm_f2 = sm_ring_f2;
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
            spw_ioctl_send->dlen = TOTAL_SIZE_ASM_F0_IN_BYTES / 2;  // 2 packets will be sent
            spw_ioctl_send->data = &spectral_matrix[
                    ( (ASM_F0_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F0) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F0) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F0);        // BLK_NR LSB
            break;
        case SID_NORM_ASM_F1:
            spw_ioctl_send->dlen = TOTAL_SIZE_ASM_F1_IN_BYTES / 2;  // 2 packets will be sent
            spw_ioctl_send->data = &spectral_matrix[
                    ( (ASM_F1_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F1) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F1) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F1);        // BLK_NR LSB
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



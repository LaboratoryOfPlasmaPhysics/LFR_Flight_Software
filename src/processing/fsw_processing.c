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
ring_node sm_ring_f0[ NB_RING_NODES_SM_F0 ];
ring_node sm_ring_f1[ NB_RING_NODES_SM_F1 ];
ring_node sm_ring_f2[ NB_RING_NODES_SM_F2 ];
ring_node *current_ring_node_sm_f0;
ring_node *current_ring_node_sm_f1;
ring_node *current_ring_node_sm_f2;
ring_node *ring_node_for_averaging_sm_f0;
ring_node *ring_node_for_averaging_sm_f1;
ring_node *ring_node_for_averaging_sm_f2;

//
ring_node * getRingNodeForAveraging( unsigned char frequencyChannel)
{
    ring_node *node;

    node = NULL;
    switch ( frequencyChannel ) {
    case 0:
        node = ring_node_for_averaging_sm_f0;
        break;
    case 1:
        node = ring_node_for_averaging_sm_f1;
        break;
    case 2:
        node = ring_node_for_averaging_sm_f2;
        break;
    default:
        break;
    }

    return node;
}

//***********************************************************
// Interrupt Service Routine for spectral matrices processing

void spectral_matrices_isr_f0( void )
{
    unsigned char status;
    rtems_status_code status_code;
    ring_node *full_ring_node;

    status = spectral_matrix_regs->status & 0x03;   // [0011] get the status_ready_matrix_f0_x bits

    switch(status)
    {
    case 0:
        break;
    case 3:
        // UNEXPECTED VALUE
        spectral_matrix_regs->status = 0x03;   // [0011]
        status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_11 );
        break;
    case 1:
        full_ring_node = current_ring_node_sm_f0->previous;
        full_ring_node->coarseTime = spectral_matrix_regs->f0_0_coarse_time;
        full_ring_node->fineTime = spectral_matrix_regs->f0_0_fine_time;
        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
        spectral_matrix_regs->f0_0_address = current_ring_node_sm_f0->buffer_address;
        // if there are enough ring nodes ready, wake up an AVFx task
        nb_sm_f0 = nb_sm_f0 + 1;
        if (nb_sm_f0 == NB_SM_BEFORE_AVF0)
        {
            ring_node_for_averaging_sm_f0 = full_ring_node;
            if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
            {
                status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
            }
            nb_sm_f0 = 0;
        }
        spectral_matrix_regs->status = 0x01;   // [0000 0001]
        break;
    case 2:
        full_ring_node = current_ring_node_sm_f0->previous;
        full_ring_node->coarseTime = spectral_matrix_regs->f0_1_coarse_time;
        full_ring_node->fineTime = spectral_matrix_regs->f0_1_fine_time;
        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
        spectral_matrix_regs->f0_1_address = current_ring_node_sm_f0->buffer_address;
        // if there are enough ring nodes ready, wake up an AVFx task
        nb_sm_f0 = nb_sm_f0 + 1;
        if (nb_sm_f0 == NB_SM_BEFORE_AVF0)
        {
            ring_node_for_averaging_sm_f0 = full_ring_node;
            if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
            {
                status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
            }
            nb_sm_f0 = 0;
        }
        spectral_matrix_regs->status = 0x02;   // [0000 0010]
        break;
    }
}

void spectral_matrices_isr_f1( void )
{
    rtems_status_code status_code;
    unsigned char status;
    ring_node *full_ring_node;

    status = (spectral_matrix_regs->status & 0x0c) >> 2;   // [1100] get the status_ready_matrix_f0_x bits

    switch(status)
    {
    case 0:
        break;
    case 3:
        // UNEXPECTED VALUE
        spectral_matrix_regs->status = 0xc0;   // [1100]
        status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_11 );
        break;
    case 1:
        full_ring_node = current_ring_node_sm_f1->previous;
        full_ring_node->coarseTime = spectral_matrix_regs->f1_0_coarse_time;
        full_ring_node->fineTime = spectral_matrix_regs->f1_0_fine_time;
        current_ring_node_sm_f1 = current_ring_node_sm_f1->next;
        spectral_matrix_regs->f1_0_address = current_ring_node_sm_f1->buffer_address;
        // if there are enough ring nodes ready, wake up an AVFx task
        nb_sm_f1 = nb_sm_f1 + 1;
        if (nb_sm_f1 == NB_SM_BEFORE_AVF1)
        {
            ring_node_for_averaging_sm_f1 = full_ring_node;
            if (rtems_event_send( Task_id[TASKID_AVF1], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
            {
                status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
            }
            nb_sm_f1 = 0;
        }
        spectral_matrix_regs->status = 0x04;   // [0000 0100]
        break;
    case 2:
        full_ring_node = current_ring_node_sm_f1->previous;
        full_ring_node->coarseTime = spectral_matrix_regs->f1_1_coarse_time;
        full_ring_node->fineTime = spectral_matrix_regs->f1_1_fine_time;
        current_ring_node_sm_f1 = current_ring_node_sm_f1->next;
        spectral_matrix_regs->f1_1_address = current_ring_node_sm_f1->buffer_address;
        // if there are enough ring nodes ready, wake up an AVFx task
        nb_sm_f1 = nb_sm_f1 + 1;
        if (nb_sm_f1 == NB_SM_BEFORE_AVF1)
        {
            ring_node_for_averaging_sm_f1 = full_ring_node;
            if (rtems_event_send( Task_id[TASKID_AVF1], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
            {
                status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
            }
            nb_sm_f1 = 0;
        }
        spectral_matrix_regs->status = 0x08;   // [1000 0000]
        break;
    }
}

void spectral_matrices_isr_f2( void )
{
    unsigned char status;
    rtems_status_code status_code;

    status = (spectral_matrix_regs->status & 0x30) >> 4;   // [0011 0000] get the status_ready_matrix_f0_x bits

    switch(status)
    {
    case 0:
        break;
    case 3:
        // UNEXPECTED VALUE
        spectral_matrix_regs->status = 0x30;   // [0011 0000]
        status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_11 );
        break;
    case 1:
        ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2->previous;
        current_ring_node_sm_f2 = current_ring_node_sm_f2->next;
        ring_node_for_averaging_sm_f2->coarseTime = spectral_matrix_regs->f2_0_coarse_time;
        ring_node_for_averaging_sm_f2->fineTime = spectral_matrix_regs->f2_0_fine_time;
        spectral_matrix_regs->f2_0_address = current_ring_node_sm_f2->buffer_address;
        spectral_matrix_regs->status = 0x10;   // [0001 0000]
        if (rtems_event_send( Task_id[TASKID_AVF2], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        break;
    case 2:
        ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2->previous;
        current_ring_node_sm_f2 = current_ring_node_sm_f2->next;
        ring_node_for_averaging_sm_f2->coarseTime = spectral_matrix_regs->f2_1_coarse_time;
        ring_node_for_averaging_sm_f2->fineTime = spectral_matrix_regs->f2_1_fine_time;
        spectral_matrix_regs->f2_1_address = current_ring_node_sm_f2->buffer_address;
        spectral_matrix_regs->status = 0x20;   // [0010 0000]
        if (rtems_event_send( Task_id[TASKID_AVF2], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        break;
    }
}

void spectral_matrix_isr_error_handler( void )
{
    rtems_status_code status_code;

    if (spectral_matrix_regs->status & 0x7c0)    // [0111 1100 0000]
    {
        status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );
    }

    spectral_matrix_regs->status = spectral_matrix_regs->status & 0x7c0;
}

rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{    
    // STATUS REGISTER
    // input_fifo_write(2) *** input_fifo_write(1) *** input_fifo_write(0)
    //           10                    9                       8
    // buffer_full ** bad_component_err ** f2_1 ** f2_0 ** f1_1 ** f1_0 ** f0_1 ** f0_0
    //      7                  6             5       4       3       2       1       0

    spectral_matrices_isr_f0();

    spectral_matrices_isr_f1();

    spectral_matrices_isr_f2();

    spectral_matrix_isr_error_handler();
}

rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector )
{
    rtems_status_code status_code;

    //***
    // F0
    nb_sm_f0 = nb_sm_f0 + 1;
    if (nb_sm_f0 == NB_SM_BEFORE_AVF0 )
    {
        ring_node_for_averaging_sm_f0 = current_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
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
            status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm_f1 = 0;
    }

    //***
    // F2
    nb_sm_f0_aux_f2 = nb_sm_f0_aux_f2 + 1;
    if (nb_sm_f0_aux_f2 == 96)
    {
        nb_sm_f0_aux_f2 = 0;
        ring_node_for_averaging_sm_f2 = current_ring_node_sm_f2;
        if (rtems_event_send( Task_id[TASKID_AVF2], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
    }
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
    init_ring( sm_ring_f0, NB_RING_NODES_SM_F0, sm_f0, TOTAL_SIZE_SM );
    init_ring( sm_ring_f1, NB_RING_NODES_SM_F1, sm_f1, TOTAL_SIZE_SM );
    init_ring( sm_ring_f2, NB_RING_NODES_SM_F2, sm_f2, TOTAL_SIZE_SM );

    DEBUG_PRINTF1("sm_ring_f0 @%x\n", (unsigned int) sm_ring_f0)
    DEBUG_PRINTF1("sm_ring_f1 @%x\n", (unsigned int) sm_ring_f1)
    DEBUG_PRINTF1("sm_ring_f2 @%x\n", (unsigned int) sm_ring_f2)
    DEBUG_PRINTF1("sm_f0 @%x\n", (unsigned int) sm_f0)
    DEBUG_PRINTF1("sm_f1 @%x\n", (unsigned int) sm_f1)
    DEBUG_PRINTF1("sm_f2 @%x\n", (unsigned int) sm_f2)
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
    current_ring_node_sm_f0 = sm_ring_f0[0].next;
    current_ring_node_sm_f1 = sm_ring_f1[0].next;
    current_ring_node_sm_f2 = sm_ring_f2[0].next;

    ring_node_for_averaging_sm_f0 = NULL;
    ring_node_for_averaging_sm_f1 = NULL;
    ring_node_for_averaging_sm_f2 = NULL;
}

//*****************
// Basic Parameters

void BP_init_header( bp_packet *packet,
                     unsigned int apid, unsigned char sid,
                     unsigned int packetLength, unsigned char blkNr )
{
    packet->targetLogicalAddress = CCSDS_DESTINATION_ID;
    packet->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    packet->reserved = 0x00;
    packet->userApplication = CCSDS_USER_APP;
    packet->packetID[0] = (unsigned char) (apid >> 8);
    packet->packetID[1] = (unsigned char) (apid);
    packet->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    packet->packetSequenceControl[1] = 0x00;
    packet->packetLength[0] = (unsigned char) (packetLength >> 8);
    packet->packetLength[1] = (unsigned char) (packetLength);
    // DATA FIELD HEADER
    packet->spare1_pusVersion_spare2 = 0x10;
    packet->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    packet->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    packet->destinationID = TM_DESTINATION_ID_GROUND;
    packet->time[0] = 0x00;
    packet->time[1] = 0x00;
    packet->time[2] = 0x00;
    packet->time[3] = 0x00;
    packet->time[4] = 0x00;
    packet->time[5] = 0x00;
    // AUXILIARY DATA HEADER
    packet->sid = sid;
    packet->biaStatusInfo = 0x00;
    packet->acquisitionTime[0] = 0x00;
    packet->acquisitionTime[1] = 0x00;
    packet->acquisitionTime[2] = 0x00;
    packet->acquisitionTime[3] = 0x00;
    packet->acquisitionTime[4] = 0x00;
    packet->acquisitionTime[5] = 0x00;
    packet->pa_lfr_bp_blk_nr[0] = 0x00;  // BLK_NR MSB
    packet->pa_lfr_bp_blk_nr[1] = blkNr;  // BLK_NR LSB
}

void BP_init_header_with_spare( bp_packet_with_spare *packet,
                                unsigned int apid, unsigned char sid,
                                unsigned int packetLength , unsigned char blkNr)
{
    packet->targetLogicalAddress = CCSDS_DESTINATION_ID;
    packet->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    packet->reserved = 0x00;
    packet->userApplication = CCSDS_USER_APP;
    packet->packetID[0] = (unsigned char) (apid >> 8);
    packet->packetID[1] = (unsigned char) (apid);
    packet->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    packet->packetSequenceControl[1] = 0x00;
    packet->packetLength[0] = (unsigned char) (packetLength >> 8);
    packet->packetLength[1] = (unsigned char) (packetLength);
    // DATA FIELD HEADER
    packet->spare1_pusVersion_spare2 = 0x10;
    packet->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    packet->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    packet->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    packet->sid = sid;
    packet->biaStatusInfo = 0x00;
    packet->time[0] = 0x00;
    packet->time[0] = 0x00;
    packet->time[0] = 0x00;
    packet->time[0] = 0x00;
    packet->time[0] = 0x00;
    packet->time[0] = 0x00;
    packet->source_data_spare = 0x00;
    packet->pa_lfr_bp_blk_nr[0] = 0x00;  // BLK_NR MSB
    packet->pa_lfr_bp_blk_nr[1] = blkNr;  // BLK_NR LSB
}

void BP_send(char *data, rtems_id queue_id, unsigned int nbBytesToSend, unsigned int sid )
{
    rtems_status_code status;

    // SET THE SEQUENCE_CNT PARAMETER
    increment_seq_counter_source_id( (unsigned char*) &data[ PACKET_POS_SEQUENCE_CNT ], sid );
    // SEND PACKET
    status =  rtems_message_queue_send( queue_id, data, nbBytesToSend);
    if (status != RTEMS_SUCCESSFUL)
    {
        printf("ERR *** in BP_send *** ERR %d\n", (int) status);
    }
}

//******************
// general functions

void reset_sm_status( void )
{
    // error
    // 10 --------------- 9 ---------------- 8 ---------------- 7 ---------
    // input_fif0_write_2 input_fifo_write_1 input_fifo_write_0 buffer_full
    // ---------- 5 -- 4 -- 3 -- 2 -- 1 -- 0 --
    // ready bits f2_1 f2_0 f1_1 f1_1 f0_1 f0_0

    spectral_matrix_regs->status = 0x7ff;   // [0111 1111 1111]
}

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

    set_sm_irq_onError( 0 );

    set_sm_irq_onNewMatrix( 0 );

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

    spectral_matrix_regs->matrix_length = 0xc8; // 25 * 128 / 16 = 200 = 0xc8
}

void set_time( unsigned char *time, unsigned char * timeInBuffer )
{
    time[0] = timeInBuffer[0];
    time[1] = timeInBuffer[1];
    time[2] = timeInBuffer[2];
    time[3] = timeInBuffer[3];
    time[4] = timeInBuffer[6];
    time[5] = timeInBuffer[7];
}

unsigned long long int get_acquisition_time( unsigned char *timePtr )
{
    unsigned long long int acquisitionTimeAslong;
    acquisitionTimeAslong = 0x00;
    acquisitionTimeAslong = ( (unsigned long long int) (timePtr[0] & 0x7f) << 40 ) // [0111 1111] mask the synchronization bit
            + ( (unsigned long long int) timePtr[1] << 32 )
            + ( (unsigned long long int) timePtr[2] << 24 )
            + ( (unsigned long long int) timePtr[3] << 16 )
            + ( (unsigned long long int) timePtr[6] << 8  )
            + ( (unsigned long long int) timePtr[7]       );
    return acquisitionTimeAslong;
}

unsigned char getSID( rtems_event_set event )
{
    unsigned char sid;

    rtems_event_set eventSetBURST;
    rtems_event_set eventSetSBM;

    //******
    // BURST
    eventSetBURST = RTEMS_EVENT_BURST_BP1_F0
            | RTEMS_EVENT_BURST_BP1_F1
            | RTEMS_EVENT_BURST_BP2_F0
            | RTEMS_EVENT_BURST_BP2_F1;

    //****
    // SBM
    eventSetSBM = RTEMS_EVENT_SBM_BP1_F0
            | RTEMS_EVENT_SBM_BP1_F1
            | RTEMS_EVENT_SBM_BP2_F0
            | RTEMS_EVENT_SBM_BP2_F1;

    if (event & eventSetBURST)
    {
        sid = SID_BURST_BP1_F0;
    }
    else if (event & eventSetSBM)
    {
        sid = SID_SBM1_BP1_F0;
    }
    else
    {
        sid = 0;
    }

    return sid;
}


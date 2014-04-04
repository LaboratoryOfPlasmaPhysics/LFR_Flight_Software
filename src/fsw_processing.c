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
ring_node_sm sm_ring_f0[ NB_RING_NODES_ASM_F0 ];
ring_node_sm sm_ring_f1[ NB_RING_NODES_ASM_F1 ];
ring_node_sm sm_ring_f2[ NB_RING_NODES_ASM_F2 ];
ring_node_sm *current_ring_node_sm_f0;
ring_node_sm *ring_node_for_averaging_sm_f0;
ring_node_sm *current_ring_node_sm_f1;
ring_node_sm *current_ring_node_sm_f2;

//**********************
// basic parameter rings
ring_node_bp *current_node_sbm1_bp1_f0;
ring_node_bp bp_ring_sbm1[ NB_RING_NODES_BP1_SBM1 ];

//*****
// NORM
// F0
float asm_f0            [ TIME_OFFSET + TOTAL_SIZE_SM ];
float asm_f0_reorganized[ TIME_OFFSET + TOTAL_SIZE_SM ];
char asm_f0_char        [ TIME_OFFSET_IN_BYTES + (TOTAL_SIZE_SM * 2) ];
float compressed_sm_f0          [ TIME_OFFSET + TOTAL_SIZE_COMPRESSED_ASM_F0 ];

//*****
// SBM1
float averaged_sm_sbm1          [ TIME_OFFSET + TOTAL_SIZE_SM    ];
float compressed_sm_sbm1        [ TIME_OFFSET + TOTAL_SIZE_COMPRESSED_ASM_SBM1 ];

unsigned char LFR_BP1_F0[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_NORM_BP1_F0 * 2 ];
unsigned char LFR_BP1_F1[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_NORM_BP1_F1 ];
unsigned char LFR_BP1_F2[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_NORM_BP1_F2 ];

unsigned int nb_sm_f0;

void init_sm_rings( void )
{
    unsigned char i;

    // F0 RING
    sm_ring_f0[0].next            = (ring_node_sm*) &sm_ring_f0[1];
    sm_ring_f0[0].previous        = (ring_node_sm*) &sm_ring_f0[NB_RING_NODES_ASM_F0-1];
    sm_ring_f0[0].buffer_address  =
            (int) &sm_f0[ 0 ];

    sm_ring_f0[NB_RING_NODES_ASM_F0-1].next           = (ring_node_sm*) &sm_ring_f0[0];
    sm_ring_f0[NB_RING_NODES_ASM_F0-1].previous       = (ring_node_sm*) &sm_ring_f0[NB_RING_NODES_ASM_F0-2];
    sm_ring_f0[NB_RING_NODES_ASM_F0-1].buffer_address =
            (int) &sm_f0[ (NB_RING_NODES_ASM_F0-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_ASM_F0-1; i++)
    {
        sm_ring_f0[i].next            = (ring_node_sm*) &sm_ring_f0[i+1];
        sm_ring_f0[i].previous        = (ring_node_sm*) &sm_ring_f0[i-1];
        sm_ring_f0[i].buffer_address  =
                (int) &sm_f0[ i * TOTAL_SIZE_SM ];
    }

    // F1 RING
    sm_ring_f1[0].next            = (ring_node_sm*) &sm_ring_f1[1];
    sm_ring_f1[0].previous        = (ring_node_sm*) &sm_ring_f1[NB_RING_NODES_ASM_F1-1];
    sm_ring_f1[0].buffer_address  =
            (int) &sm_f1[ 0 ];

    sm_ring_f1[NB_RING_NODES_ASM_F1-1].next           = (ring_node_sm*) &sm_ring_f1[0];
    sm_ring_f1[NB_RING_NODES_ASM_F1-1].previous       = (ring_node_sm*) &sm_ring_f1[NB_RING_NODES_ASM_F1-2];
    sm_ring_f1[NB_RING_NODES_ASM_F1-1].buffer_address =
            (int) &sm_f1[ (NB_RING_NODES_ASM_F1-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_ASM_F1-1; i++)
    {
        sm_ring_f1[i].next            = (ring_node_sm*) &sm_ring_f1[i+1];
        sm_ring_f1[i].previous        = (ring_node_sm*) &sm_ring_f1[i-1];
        sm_ring_f1[i].buffer_address  =
                (int) &sm_f1[ i * TOTAL_SIZE_SM ];
    }

    // F2 RING
    sm_ring_f2[0].next            = (ring_node_sm*) &sm_ring_f2[1];
    sm_ring_f2[0].previous        = (ring_node_sm*) &sm_ring_f2[NB_RING_NODES_ASM_F2-1];
    sm_ring_f2[0].buffer_address  =
            (int) &sm_f2[ 0 ];

    sm_ring_f2[NB_RING_NODES_ASM_F2-1].next           = (ring_node_sm*) &sm_ring_f2[0];
    sm_ring_f2[NB_RING_NODES_ASM_F2-1].previous       = (ring_node_sm*) &sm_ring_f2[NB_RING_NODES_ASM_F2-2];
    sm_ring_f2[NB_RING_NODES_ASM_F2-1].buffer_address =
            (int) &sm_f2[ (NB_RING_NODES_ASM_F2-1) * TOTAL_SIZE_SM ];

    for(i=1; i<NB_RING_NODES_ASM_F2-1; i++)
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

void reset_current_sm_ring_nodes( void )
{
    current_ring_node_sm_f0 = sm_ring_f0;
    current_ring_node_sm_f1 = sm_ring_f1;
    current_ring_node_sm_f2 = sm_ring_f2;

    ring_node_for_averaging_sm_f0   = sm_ring_f0;
}

void reset_current_node_sbm1_bp1_f0( void )
{
    current_node_sbm1_bp1_f0 = bp_ring_sbm1;
}

//***********************************************************
// Interrupt Service Routine for spectral matrices processing
void reset_nb_sm_f0( void )
{
    nb_sm_f0 = 0;
}

rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{
    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );

    if ( (spectral_matrix_regs->status & 0x1) == 0x01)
    {
        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
        spectral_matrix_regs->matrixF0_Address0 = current_ring_node_sm_f0->buffer_address;
        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffe;   // 1110
        nb_sm_f0 = nb_sm_f0 + 1;
    }
    else if ( (spectral_matrix_regs->status & 0x2) == 0x02)
    {
        current_ring_node_sm_f0 = current_ring_node_sm_f0->next;
        spectral_matrix_regs->matrixFO_Address1 = current_ring_node_sm_f0->buffer_address;
        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffd;   // 1101
        nb_sm_f0 = nb_sm_f0 + 1;
    }

    if ( (spectral_matrix_regs->status & 0x30) != 0x00)
    {
        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_8 );
        spectral_matrix_regs->status = spectral_matrix_regs->status & 0xffffffcf;   // 1100 1111
    }

    spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffff3;   // 0011

    if (nb_sm_f0 == (NB_SM_TO_RECEIVE_BEFORE_AVF0-1) )
    {
        ring_node_for_averaging_sm_f0 = current_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm_f0 = 0;
    }
    else
    {
        nb_sm_f0 = nb_sm_f0 + 1;
    }
}

rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector )
{
    if (nb_sm_f0 == (NB_SM_TO_RECEIVE_BEFORE_AVF0-1) )
    {
        ring_node_for_averaging_sm_f0 = current_ring_node_sm_f0;
        if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
        }
        nb_sm_f0 = 0;
    }
    else
    {
        nb_sm_f0 = nb_sm_f0 + 1;
    }
}

//************
// RTEMS TASKS

rtems_task smiq_task(rtems_task_argument argument) // process the Spectral Matrices IRQ
{
    rtems_event_set event_out;

    BOOT_PRINTF("in SMIQ *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
    }
}

rtems_task avf0_task(rtems_task_argument argument)
{
    int i;
    static unsigned int nb_average_norm_f0;
    static unsigned int nb_average_sbm1_f0;
    rtems_event_set event_out;
    rtems_status_code status;
    ring_node_sm *ring_node_tab[8];

    nb_average_norm_f0 = 0;
    nb_average_sbm1_f0 = 0;

    BOOT_PRINTF("in AVFO *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        ring_node_tab[NB_SM_TO_RECEIVE_BEFORE_AVF0-1] = ring_node_for_averaging_sm_f0;
        for ( i = 2; i < (NB_SM_TO_RECEIVE_BEFORE_AVF0+1); i++ )
        {
            ring_node_for_averaging_sm_f0 = ring_node_for_averaging_sm_f0->previous;
            ring_node_tab[NB_SM_TO_RECEIVE_BEFORE_AVF0-i] = ring_node_for_averaging_sm_f0;
        }

        // copy time information in the asm_f0 buffer
        asm_f0[0] = ring_node_tab[7]->coarseTime;
        asm_f0[1] = ring_node_tab[7]->fineTime;
        averaged_sm_sbm1[0] = ring_node_tab[7]->coarseTime;
        averaged_sm_sbm1[1] = ring_node_tab[7]->fineTime;

        // compute the average and store it in the averaged_sm_f1 buffer
        ASM_average( asm_f0, averaged_sm_sbm1,
                     ring_node_tab,
                     nb_average_norm_f0, nb_average_sbm1_f0 );


        // update nb_average
        nb_average_norm_f0 = nb_average_norm_f0 + NB_SM_TO_RECEIVE_BEFORE_AVF0;
        nb_average_sbm1_f0 = nb_average_sbm1_f0 + NB_SM_TO_RECEIVE_BEFORE_AVF0;

        // launch actions depending on the current mode

        if (nb_average_sbm1_f0 == NB_AVERAGE_SBM1_F0)
        {
            nb_average_sbm1_f0 = 0;
            if (lfrCurrentMode == LFR_MODE_SBM1)
            {
                status = rtems_event_send( Task_id[TASKID_MATR], RTEMS_EVENT_MODE_SBM1 ); // sending an event to the task 7, BPF0
                if (status != RTEMS_SUCCESSFUL)
                {
                    printf("in AVF0 *** Error sending RTEMS_EVENT_MODE_SBM1, code %d\n", status);
                }
            }
        }

        if (nb_average_norm_f0 == NB_AVERAGE_NORMAL_F0) {
            nb_average_norm_f0 = 0;
            status = rtems_event_send( Task_id[TASKID_MATR], RTEMS_EVENT_MODE_NORMAL ); // sending an event to the task 7, BPF0
            if (status != RTEMS_SUCCESSFUL) {
                printf("in AVF0 *** Error sending RTEMS_EVENT_0, code %d\n", status);
            }
        }
    }
}

rtems_task matr_task(rtems_task_argument argument)
{
    spw_ioctl_pkt_send spw_ioctl_send_ASM;
    rtems_event_set event_out;
    rtems_status_code status;
    rtems_id queue_id;
    Header_TM_LFR_SCIENCE_ASM_t headerASM;
    ring_node_norm_bp current_node_norm_bp1_f0;

    init_header_asm( &headerASM );
//    init_header_bp( &current_node_norm_bp1_f0.header );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in MATR *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in MATR *** \n")

    while(1){
        rtems_event_receive( RTEMS_EVENT_MODE_NORMAL | RTEMS_EVENT_MODE_SBM1,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if (event_out==RTEMS_EVENT_MODE_NORMAL)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( asm_f0, compressed_sm_f0,
                                         NB_AVERAGE_NORMAL_F0,
                                         NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0,
                                         ASM_F0_INDICE_START );
            // 2) compute the BP1 set

            // 3) send the BP1 set
            BP1_send( &current_node_norm_bp1_f0, SID_NORM_BP1_F0, queue_id );
            // 4) reorganize the ASM and divide
            ASM_reorganize_and_divide( asm_f0, asm_f0_reorganized, NB_AVERAGE_NORMAL_F0 );
            // 5) convert the float array in a char array
            ASM_convert( asm_f0_reorganized, asm_f0_char);
            // 6) send the spectral matrix packets
            ASM_send( &headerASM, asm_f0_char, SID_NORM_ASM_F0, &spw_ioctl_send_ASM, queue_id);
        }
        else if (event_out==RTEMS_EVENT_MODE_SBM1)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress_reorganize_and_divide( averaged_sm_sbm1, compressed_sm_sbm1,
                                         NB_AVERAGE_SBM1_F0,
                                         NB_BINS_COMPRESSED_SM_SBM1_F0, NB_BINS_TO_AVERAGE_ASM_SBM1_F0,
                                         ASM_F0_INDICE_START);
            // 2) compute the BP1 set

            // 3) send the basic parameters set 1 packet
            BP1_send( current_node_sbm1_bp1_f0, SID_SBM1_BP1_F0, queue_id );
            // 4) update current_node_sbm1_bp1_f0
            current_node_sbm1_bp1_f0 = current_node_sbm1_bp1_f0->next;
        }
        else
        {
            PRINTF1("ERR *** in MATR *** unexect event = %x\n", (unsigned int) event_out)
        }
    }
}

//*****************************
// Spectral matrices processing

void ASM_average( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
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
            averaged_spec_mat_f0[ TIME_OFFSET + i ] = sum;
            averaged_spec_mat_f1[ TIME_OFFSET + i ] = sum;
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 != 0) )
        {
            averaged_spec_mat_f0[ TIME_OFFSET + i ] = ( averaged_spec_mat_f0[ TIME_OFFSET + i ] + sum );
            averaged_spec_mat_f1[ TIME_OFFSET + i ] = ( averaged_spec_mat_f1[ TIME_OFFSET + i ] + sum );
        }
        else if ( (nbAverageNormF0 != 0) && (nbAverageSBM1F0 == 0) )
        {
            averaged_spec_mat_f0[ TIME_OFFSET + i ] = ( averaged_spec_mat_f0[ TIME_OFFSET + i ] + sum );
            averaged_spec_mat_f1[ TIME_OFFSET + i ] = sum;
        }
        else
        {
            PRINTF2("ERR *** in ASM_average *** unexpected parameters %d %d\n", nbAverageNormF0, nbAverageSBM1F0)
        }
    }
}

void ASM_reorganize_and_divide( float *averaged_spec_mat, float *averaged_spec_mat_reorganized, float divider )
{
    int frequencyBin;
    int asmComponent;

    // copy the time information
    averaged_spec_mat_reorganized[ 0 ] = averaged_spec_mat[ 0 ];
    averaged_spec_mat_reorganized[ 1 ] = averaged_spec_mat[ 1 ];

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++ )
        {
            averaged_spec_mat_reorganized[ TIME_OFFSET + frequencyBin * NB_VALUES_PER_SM + asmComponent ] =
                    averaged_spec_mat[ TIME_OFFSET + asmComponent * NB_BINS_PER_SM + frequencyBin ] / divider;
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

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix; frequencyBin++ )
        {
            offsetCompressed = TIME_OFFSET
                    + frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetASM = TIME_OFFSET
                    + asmComponent * NB_BINS_PER_SM
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
    unsigned int i;
    unsigned int frequencyBin;
    unsigned int asmComponent;
    char * pt_char_input;
    char * pt_char_output;

    pt_char_input = (char*) &input_matrix;
    pt_char_output = (char*) &output_matrix;

    // copy the time information
    for (i=0; i<TIME_OFFSET_IN_BYTES; i++)
    {
        pt_char_output[ i ] = pt_char_output[ i ];
    }

    // convert all other data
    for( frequencyBin=0; frequencyBin<NB_BINS_PER_SM; frequencyBin++)
    {
        for ( asmComponent=0; asmComponent<NB_VALUES_PER_SM; asmComponent++)
        {
            pt_char_input =  (char*) &input_matrix [       (frequencyBin*NB_VALUES_PER_SM) + asmComponent   + TIME_OFFSET ];
            pt_char_output = (char*) &output_matrix[ 2 * ( (frequencyBin*NB_VALUES_PER_SM) + asmComponent ) + TIME_OFFSET_IN_BYTES ];
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
                    + TIME_OFFSET_IN_BYTES
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

void BP1_send( ring_node_bp *ring_node_to_send, unsigned int sid, rtems_id queue_id )
{
    unsigned int length = 0;
    rtems_status_code status;
    unsigned char nbBytesTosend;

    // (1) BUILD THE DATA
    switch(sid)
    {
    case SID_NORM_BP1_F0:
        length = PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0;
        ring_node_to_send->header.packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST >> 8);
        ring_node_to_send->header.packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST);
        ring_node_to_send->header.pa_lfr_bp_blk_nr[0] = (unsigned char)  ( (NB_BINS_COMPRESSED_SM_F0) >> 8 ); // BLK_NR MSB
        ring_node_to_send->header.pa_lfr_bp_blk_nr[1] = (unsigned char)    (NB_BINS_COMPRESSED_SM_F0);        // BLK_NR LSB
        nbBytesTosend = PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0
                + CCSDS_TC_TM_PACKET_OFFSET
                + CCSDS_PROTOCOLE_EXTRA_BYTES;
    case SID_SBM1_BP1_F0:
        length = PACKET_LENGTH_TM_LFR_SCIENCE_SBM1_BP1_F0;
        ring_node_to_send->header.packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2 >> 8);
        ring_node_to_send->header.packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2);
        ring_node_to_send->header.pa_lfr_bp_blk_nr[0] = (unsigned char)  ( (NB_BINS_COMPRESSED_SM_SBM1_F0) >> 8 ); // BLK_NR MSB
        ring_node_to_send->header.pa_lfr_bp_blk_nr[1] = (unsigned char)    (NB_BINS_COMPRESSED_SM_SBM1_F0);        // BLK_NR LSB
        nbBytesTosend = PACKET_LENGTH_TM_LFR_SCIENCE_SBM1_BP1_F0
                + CCSDS_TC_TM_PACKET_OFFSET
                + CCSDS_PROTOCOLE_EXTRA_BYTES;
        break;
    default:
        nbBytesTosend = 0;
        PRINTF1("ERR *** in BP1_send *** unexpected sid %d\n", sid)
        break;
    }

    // (2) BUILD THE HEADER
    ring_node_to_send->header.packetLength[0] = (unsigned char) (length>>8);
    ring_node_to_send->header.packetLength[1] = (unsigned char) (length);
    ring_node_to_send->header.sid = sid;

    // (3) SET PACKET TIME
    ring_node_to_send->header.time[0] = (unsigned char) (ring_node_to_send->coarseTime>>24);
    ring_node_to_send->header.time[1] = (unsigned char) (ring_node_to_send->coarseTime>>16);
    ring_node_to_send->header.time[2] = (unsigned char) (ring_node_to_send->coarseTime>>8);
    ring_node_to_send->header.time[3] = (unsigned char) (ring_node_to_send->coarseTime);
    ring_node_to_send->header.time[4] = (unsigned char) (ring_node_to_send->fineTime>>8);
    ring_node_to_send->header.time[5] = (unsigned char) (ring_node_to_send->fineTime);
    //
    ring_node_to_send->header.acquisitionTime[0] = (unsigned char) (ring_node_to_send->coarseTime>>24);
    ring_node_to_send->header.acquisitionTime[1] = (unsigned char) (ring_node_to_send->coarseTime>>16);
    ring_node_to_send->header.acquisitionTime[2] = (unsigned char) (ring_node_to_send->coarseTime>>8);
    ring_node_to_send->header.acquisitionTime[3] = (unsigned char) (ring_node_to_send->coarseTime);
    ring_node_to_send->header.acquisitionTime[4] = (unsigned char) (ring_node_to_send->fineTime>>8);
    ring_node_to_send->header.acquisitionTime[5] = (unsigned char) (ring_node_to_send->fineTime);

    // (4) SEND PACKET
    status =  rtems_message_queue_send( queue_id, &ring_node_to_send->header, nbBytesTosend);
    if (status != RTEMS_SUCCESSFUL)
    {
        printf("ERR *** in BP1_send *** ERR %d\n", (int) status);
    }
}

void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST);
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

void init_bp_ring_sbm1()
{
    unsigned int i;

    //********
    // F0 RING
    bp_ring_sbm1[0].next            = (ring_node_bp*) &bp_ring_sbm1[1];
    bp_ring_sbm1[0].previous        = (ring_node_bp*) &bp_ring_sbm1[NB_RING_NODES_BP1_SBM1-1];

    bp_ring_sbm1[NB_RING_NODES_BP1_SBM1-1].next           = (ring_node_bp*) &bp_ring_sbm1[0];
    bp_ring_sbm1[NB_RING_NODES_BP1_SBM1-1].previous       = (ring_node_bp*) &bp_ring_sbm1[NB_RING_NODES_ASM_F0-2];

    for(i=1; i<NB_RING_NODES_BP1_SBM1-1; i++)
    {
        bp_ring_sbm1[i].next            = (ring_node_bp*) &bp_ring_sbm1[i+1];
        bp_ring_sbm1[i].previous        = (ring_node_bp*) &bp_ring_sbm1[i-1];
    }
    //
    //********

    for (i=0; i<NB_RING_NODES_BP1_SBM1; i++)
    {
        init_header_bp( (Header_TM_LFR_SCIENCE_BP_SBM_t*) &bp_ring_sbm1[ i ] );
        bp_ring_sbm1[ i ].status = 0;
    }
}

void init_header_bp(Header_TM_LFR_SCIENCE_BP_SBM_t *header )
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2 >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2);
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
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->pa_lfr_bp_blk_nr[0] = 0x00;  // BLK_NR MSB
    header->pa_lfr_bp_blk_nr[1] = 0x00;  // BLK_NR LSB
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

    spectral_matrix_regs->config = 0x00;
    spectral_matrix_regs->status = 0x00;

    spectral_matrix_regs->matrixF0_Address0 = current_ring_node_sm_f0->buffer_address;
    spectral_matrix_regs->matrixFO_Address1 = current_ring_node_sm_f0->buffer_address;
    spectral_matrix_regs->matrixF1_Address = current_ring_node_sm_f1->buffer_address;
    spectral_matrix_regs->matrixF2_Address = current_ring_node_sm_f2->buffer_address;
}

//******************
// general functions





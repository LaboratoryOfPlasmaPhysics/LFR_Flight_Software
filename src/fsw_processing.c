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
ring_node_sm sm_ring_f0[NB_RING_NODES_ASM_F0];
ring_node_sm sm_ring_f1[NB_RING_NODES_ASM_F1];
ring_node_sm sm_ring_f2[NB_RING_NODES_ASM_F2];
ring_node_sm *current_ring_node_sm_f0;
ring_node_sm *ring_node_for_averaging_sm_f0;
ring_node_sm *current_ring_node_sm_f1;
ring_node_sm *current_ring_node_sm_f2;

BP1_t data_BP1[ NB_BINS_COMPRESSED_SM_F0 ];

//*****
// NORM
// F0
float averaged_sm_f0            [ TIME_OFFSET + TOTAL_SIZE_SM ];
float averaged_sm_f0_reorganized[ TIME_OFFSET + TOTAL_SIZE_SM ];
char averaged_sm_f0_char        [ TIME_OFFSET_IN_BYTES + (TOTAL_SIZE_SM * 2) ];
float compressed_sm_f0          [ TOTAL_SIZE_COMPRESSED_ASM_F0 ];

//*****
// SBM1
float averaged_sm_sbm1            [ TIME_OFFSET + TOTAL_SIZE_SM    ];
float compressed_sm_sbm1          [ TOTAL_SIZE_COMPRESSED_ASM_SBM1 ];

unsigned char LFR_BP1_F0[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_BP1_F0 * 2 ];
unsigned char LFR_BP1_F1[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_BP1_F1 ];
unsigned char LFR_BP1_F2[ TIME_OFFSET_IN_BYTES + TOTAL_SIZE_BP1_F2 ];

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
    static unsigned int nb_average_norm;
    static unsigned int nb_average_sbm1;
    rtems_event_set event_out;
    rtems_status_code status;
    ring_node_sm *ring_node_tab[8];

    nb_average_norm = 0;
    nb_average_sbm1 = 0;

    BOOT_PRINTF("in AVFO *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        ring_node_tab[NB_SM_TO_RECEIVE_BEFORE_AVF0-1] = ring_node_for_averaging_sm_f0;
        for ( i = 2; i < (NB_SM_TO_RECEIVE_BEFORE_AVF0+1); i++ )
        {
            ring_node_for_averaging_sm_f0 = ring_node_for_averaging_sm_f0->previous;
            ring_node_tab[NB_SM_TO_RECEIVE_BEFORE_AVF0-i] = ring_node_for_averaging_sm_f0;
        }

        // copy time information in the averaged_sm_f0 buffer
        averaged_sm_f0[0] = ring_node_tab[7]->coarseTime;
        averaged_sm_f0[1] = ring_node_tab[7]->fineTime;
        averaged_sm_f1[0] = ring_node_tab[7]->coarseTime;
        averaged_sm_f1[1] = ring_node_tab[7]->fineTime;

        // compute the average and store it in the averaged_sm_f1 buffer
        ASM_average( averaged_sm_f0, averaged_sm_f1,
                     ring_node_tab,
                     nb_average_norm, nb_average_sbm1 );


        // update nb_average
        nb_average_norm = nb_average_norm + NB_SM_TO_RECEIVE_BEFORE_AVF0;
        nb_average_sbm1 = nb_average_sbm1 + NB_SM_TO_RECEIVE_BEFORE_AVF0;

        // launch actions depending on the current mode
        if (lfrCurrentMode == LFR_MODE_SBM1)
        {
            if (nb_average_sbm1 == NB_AVERAGE_SBM1_f0) {
                nb_average_sbm1 = 0;
                status = rtems_event_send( Task_id[TASKID_MATR], RTEMS_EVENT_MODE_SBM1 ); // sending an event to the task 7, BPF0
                if (status != RTEMS_SUCCESSFUL) {
                    printf("in AVF0 *** Error sending RTEMS_EVENT_0, code %d\n", status);
                }
            }
        }
        if (lfrCurrentMode == LFR_MODE_NORMAL)
        {
            if (nb_average_norm == NB_AVERAGE_NORMAL_f0) {
                nb_average_norm = 0;
                status = rtems_event_send( Task_id[TASKID_MATR], RTEMS_EVENT_MODE_NORMAL ); // sending an event to the task 7, BPF0
                if (status != RTEMS_SUCCESSFUL) {
                    printf("in AVF0 *** Error sending RTEMS_EVENT_0, code %d\n", status);
                }
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

    init_header_asm( &headerASM );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in MATR *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in MATR *** \n")

    fill_averaged_spectral_matrix( );

    while(1){
        rtems_event_receive( RTEMS_EVENT_MODE_NORMAL | RTEMS_EVENT_MODE_SBM1,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if (event_out==RTEMS_EVENT_MODE_NORMAL)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress( averaged_sm_f0, 0, compressed_sm_f0 );
            // 2) compute the BP1 set

            // 3) convert the float array in a char array
            ASM_reorganize( averaged_sm_f0, averaged_sm_f0_reorganized );
            ASM_convert( averaged_sm_f0_reorganized, averaged_sm_f0_char);
            // 4) send the spectral matrix packets
            ASM_send( &headerASM, averaged_sm_f0_char, SID_NORM_ASM_F0, &spw_ioctl_send_ASM, queue_id);
        }
        else if (event_out==RTEMS_EVENT_MODE_SBM1)
        {
            // 1)  compress the matrix for Basic Parameters calculation
            ASM_compress( averaged_sm_f1, 0, compressed_sm_f1 );
            // 2) compute the BP1 set

            // 4) send the basic parameters set 1 packet
            BP1_send( );
        }
        else
        {
            PRINTF1("ERR *** in MATR *** unexect event = %x\n", (unsigned int) event_out)
        }
    }
}

//*****************************
// Spectral matrices processing

void matrix_reset(volatile float *averaged_spec_mat)
{
    int i;
    for(i=0; i<TOTAL_SIZE_SM; i++){
        averaged_spec_mat[i] = 0;
    }
}

void ASM_average( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int firstTimeF0, unsigned int firstTimeF1 )
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

        if ( (firstTimeF0 == 0) && (firstTimeF1 == 0) )
        {
            averaged_spec_mat_f0[ i ] = averaged_spec_mat_f0[ i ] + sum;
            averaged_spec_mat_f1[ i ] = averaged_spec_mat_f1[ i ] + sum;
        }
        else if ( (firstTimeF0 == 0) && (firstTimeF1 != 0) )
        {
            averaged_spec_mat_f0[ i ] = averaged_spec_mat_f0[ i ] + sum;
            averaged_spec_mat_f1[ i ] = sum;
        }
        else if ( (firstTimeF0 != 0) && (firstTimeF1 == 0) )
        {
            averaged_spec_mat_f0[ i ] = sum;
            averaged_spec_mat_f1[ i ] = averaged_spec_mat_f1[ i ] + sum;
        }
        else
        {
            averaged_spec_mat_f0[ i ] = sum;
            averaged_spec_mat_f1[ i ] = sum;
        }
    }
}

void ASM_reorganize( float *averaged_spec_mat, float *averaged_spec_mat_reorganized )
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
            averaged_spec_mat_reorganized[ frequencyBin * NB_VALUES_PER_SM + asmComponent + TIME_OFFSET ] =
                    averaged_spec_mat[ asmComponent * NB_BINS_PER_SM + frequencyBin + TIME_OFFSET];
        }
    }
}

void ASM_compress( float *averaged_spec_mat, unsigned char fChannel, float *compressed_spec_mat )
{
    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int k;

    switch (fChannel){
    case 0:
        for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
        {
            for( frequencyBin = 0; frequencyBin < NB_BINS_COMPRESSED_SM_F0; frequencyBin++ )
            {
                offsetCompressed = TIME_OFFSET
                        + frequencyBin * NB_VALUES_PER_SM
                        + asmComponent;
                offsetASM = TIME_OFFSET
                        + asmComponent * NB_BINS_PER_SM
                        + ASM_F0_INDICE_START
                        + frequencyBin * NB_BINS_TO_AVERAGE_ASM_F0;
                compressed_spec_mat[ offsetCompressed ] = 0;
                for ( k = 0; k < NB_BINS_TO_AVERAGE_ASM_F0; k++ )
                {
                    compressed_spec_mat[offsetCompressed ] =
                            compressed_spec_mat[ offsetCompressed ]
                            + averaged_spec_mat[ offsetASM + k ];
                }
            }
        }
        break;

    case 1:
            // case fChannel = f1 to be completed later
        break;

    case 2:
            // case fChannel = f1 to be completed later
        break;

    default:
        break;
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

void BP1_send()
{

}

void BP1_set_old(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1){
    int i;
    int j;
    unsigned char tmp_u_char;
    unsigned char * pt_char = NULL;
    float PSDB, PSDE;
    float NVEC_V0;
    float NVEC_V1;
    float NVEC_V2;
    //float significand;
    //int exponent;
    float aux;
    float tr_SB_SB;
    float tmp;
    float sx_re;
    float sx_im;
    float nebx_re = 0;
    float nebx_im = 0;
    float ny = 0;
    float nz = 0;
    float bx_bx_star = 0;
    for(i=0; i<nb_bins_compressed_spec_mat; i++){
        //==============================================
        // BP1 PSD == B PAR_LFR_SC_BP1_PE_FL0 == 16 bits
        PSDB = compressed_spec_mat[i*30]      // S11
            + compressed_spec_mat[(i*30) + 10]    // S22
            + compressed_spec_mat[(i*30) + 18];   // S33
        //significand = frexp(PSDB, &exponent);
        pt_char = (unsigned char*) &PSDB;
        LFR_BP1[(i*9) + 2] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1[(i*9) + 3] = pt_char[1];  // bits 23 downto 16 of the float
        //==============================================
        // BP1 PSD == E PAR_LFR_SC_BP1_PB_FL0 == 16 bits
        PSDE = compressed_spec_mat[(i*30) + 24] * K44_pe     // S44
            + compressed_spec_mat[(i*30) + 28] * K55_pe      // S55
            + compressed_spec_mat[(i*30) + 26] * K45_pe_re   // S45
            - compressed_spec_mat[(i*30) + 27] * K45_pe_im;  // S45
        pt_char = (unsigned char*) &PSDE;
        LFR_BP1[(i*9) + 0] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1[(i*9) + 1] = pt_char[1]; // bits 23 downto 16 of the float
        //==============================================================================
        // BP1 normal wave vector == PAR_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V2_F0 == 1 bits
        tmp = sqrt(
                    compressed_spec_mat[(i*30) + 3]*compressed_spec_mat[(i*30) + 3]     //Im S12
                    +compressed_spec_mat[(i*30) + 5]*compressed_spec_mat[(i*30) + 5]    //Im S13
                    +compressed_spec_mat[(i*30) + 13]*compressed_spec_mat[(i*30) + 13]   //Im S23
                    );
        NVEC_V0 = compressed_spec_mat[(i*30) + 13] / tmp;  // Im S23
        NVEC_V1 = -compressed_spec_mat[(i*30) + 5] / tmp;  // Im S13
        NVEC_V2 = compressed_spec_mat[(i*30) + 3] / tmp;   // Im S12
        LFR_BP1[(i*9) + 4] = (char) (NVEC_V0*127);
        LFR_BP1[(i*9) + 5] = (char) (NVEC_V1*127);
        pt_char = (unsigned char*) &NVEC_V2;
        LFR_BP1[(i*9) + 6] = pt_char[0] & 0x80;  // extract the sign of NVEC_V2
        //=======================================================
        // BP1 ellipticity == PAR_LFR_SC_BP1_ELLIP_F0   == 4 bits
        aux = 2*tmp / PSDB;                                             // compute the ellipticity
        tmp_u_char = (unsigned char) (aux*(16-1));                      // convert the ellipticity
        LFR_BP1[i*9+6] = LFR_BP1[i*9+6] | ((tmp_u_char&0x0f)<<3); // keeps 4 bits of the resulting unsigned char
        //==============================================================
        // BP1 degree of polarization == PAR_LFR_SC_BP1_DOP_F0 == 3 bits
        for(j = 0; j<NB_VALUES_PER_SM;j++){
            tr_SB_SB = compressed_spec_mat[i*30] * compressed_spec_mat[i*30]
                    + compressed_spec_mat[(i*30) + 10] * compressed_spec_mat[(i*30) + 10]
                    + compressed_spec_mat[(i*30) + 18] * compressed_spec_mat[(i*30) + 18]
                    + 2 * compressed_spec_mat[(i*30) + 2] * compressed_spec_mat[(i*30) + 2]
                    + 2 * compressed_spec_mat[(i*30) + 3] * compressed_spec_mat[(i*30) + 3]
                    + 2 * compressed_spec_mat[(i*30) + 4] * compressed_spec_mat[(i*30) + 4]
                    + 2 * compressed_spec_mat[(i*30) + 5] * compressed_spec_mat[(i*30) + 5]
                    + 2 * compressed_spec_mat[(i*30) + 12] * compressed_spec_mat[(i*30) + 12]
                    + 2 * compressed_spec_mat[(i*30) + 13] * compressed_spec_mat[(i*30) + 13];
        }
        aux = PSDB*PSDB;
        tmp = sqrt( abs( ( 3*tr_SB_SB - aux ) / ( 2 * aux ) ) );
        tmp_u_char = (unsigned char) (NVEC_V0*(8-1));
        LFR_BP1[(i*9) + 6] = LFR_BP1[(i*9) + 6] | (tmp_u_char & 0x07); // keeps 3 bits of the resulting unsigned char
        //=======================================================================================
        // BP1 x-component of the normalized Poynting flux == PAR_LFR_SC_BP1_SZ_F0 == 8 bits (7+1)
        sx_re = compressed_spec_mat[(i*30) + 20] * K34_sx_re
                        + compressed_spec_mat[(i*30) + 6] * K14_sx_re
                        + compressed_spec_mat[(i*30) + 8] * K15_sx_re
                        + compressed_spec_mat[(i*30) + 14] * K24_sx_re
                        + compressed_spec_mat[(i*30) + 16] * K25_sx_re
                        + compressed_spec_mat[(i*30) + 22] * K35_sx_re;
        sx_im = compressed_spec_mat[(i*30) + 21] * K34_sx_im
                        + compressed_spec_mat[(i*30) + 7] * K14_sx_im
                        + compressed_spec_mat[(i*30) + 9] * K15_sx_im
                        + compressed_spec_mat[(i*30) + 15] * K24_sx_im
                        + compressed_spec_mat[(i*30) + 17] * K25_sx_im
                        + compressed_spec_mat[(i*30) + 23] * K35_sx_im;
        LFR_BP1[(i*9) + 7] = ((unsigned char) (sx_re * 128)) & 0x7f; // cf DOC for the compression
        if ( abs(sx_re) > abs(sx_im) ) {
            LFR_BP1[(i*9) + 7] = LFR_BP1[(i*9) + 1] | (0x80);  // extract the sector of sx
        }
        else {
            LFR_BP1[(i*9) + 7] = LFR_BP1[(i*9) + 1] & (0x7f);  // extract the sector of sx
        }
        //======================================================================
        // BP1 phase velocity estimator == PAR_LFR_SC_BP1_VPHI_F0 == 8 bits (7+1)
        ny = sin(Alpha_M)*NVEC_V1 + cos(Alpha_M)*NVEC_V2;
        nz = NVEC_V0;
        bx_bx_star = cos(Alpha_M) * cos(Alpha_M) * compressed_spec_mat[i*30+10]            // re S22
                        + sin(Alpha_M) * sin(Alpha_M) * compressed_spec_mat[i*30+18]       // re S33
                        - 2 * sin(Alpha_M) * cos(Alpha_M) * compressed_spec_mat[i*30+12];  // re S23
        nebx_re = ny * (compressed_spec_mat[(i*30) + 14] * K24_ny_re
                                        +compressed_spec_mat[(i*30) + 16] * K25_ny_re
                                        +compressed_spec_mat[(i*30) + 20] * K34_ny_re
                                        +compressed_spec_mat[(i*30) + 22] * K35_ny_re)
                                + nz * (compressed_spec_mat[(i*30) + 14] * K24_nz_re
                                        +compressed_spec_mat[(i*30) + 16] * K25_nz_re
                                        +compressed_spec_mat[(i*30) + 20] * K34_nz_re
                                        +compressed_spec_mat[(i*30) + 22] * K35_nz_re);
        nebx_im = ny * (compressed_spec_mat[(i*30) + 15]*K24_ny_re
                                        +compressed_spec_mat[(i*30) + 17] * K25_ny_re
                                        +compressed_spec_mat[(i*30) + 21] * K34_ny_re
                                        +compressed_spec_mat[(i*30) + 23] * K35_ny_re)
                                + nz * (compressed_spec_mat[(i*30) + 15] * K24_nz_im
                                        +compressed_spec_mat[(i*30) + 17] * K25_nz_im
                                        +compressed_spec_mat[(i*30) + 21] * K34_nz_im
                                        +compressed_spec_mat[(i*30) + 23] * K35_nz_im);
        tmp = nebx_re / bx_bx_star;
        LFR_BP1[(i*9) + 8] = ((unsigned char) (tmp * 128)) & 0x7f; // cf DOC for the compression
        if ( abs(nebx_re) > abs(nebx_im) ) {
            LFR_BP1[(i*9) + 8] = LFR_BP1[(i*9) + 8] | (0x80);  // extract the sector of nebx
        }
        else {
            LFR_BP1[(i*9) + 8] = LFR_BP1[(i*9) + 8] & (0x7f);  // extract the sector of nebx
        }
    }

}

void BP2_set_old(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat){
    // BP2 autocorrelation
    int i;
    int aux = 0;

    for(i = 0; i<nb_bins_compressed_spec_mat; i++){
        // S12
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[(i*30) + 10]);
        compressed_spec_mat[(i*30) + 2] = compressed_spec_mat[(i*30) + 2] / aux;
        compressed_spec_mat[(i*30) + 3] = compressed_spec_mat[(i*30) + 3] / aux;
        // S13
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[(i*30) + 18]);
        compressed_spec_mat[(i*30) + 4] = compressed_spec_mat[(i*30) + 4] / aux;
        compressed_spec_mat[(i*30) + 5] = compressed_spec_mat[(i*30) + 5] / aux;
        // S23
        aux = sqrt(compressed_spec_mat[i*30+12]*compressed_spec_mat[(i*30) + 18]);
        compressed_spec_mat[(i*30) + 12] = compressed_spec_mat[(i*30) + 12] / aux;
        compressed_spec_mat[(i*30) + 13] = compressed_spec_mat[(i*30) + 13] / aux;
        // S45
        aux = sqrt(compressed_spec_mat[i*30+24]*compressed_spec_mat[(i*30) + 28]);
        compressed_spec_mat[(i*30) + 26] = compressed_spec_mat[(i*30) + 26] / aux;
        compressed_spec_mat[(i*30) + 27] = compressed_spec_mat[(i*30) + 27] / aux;
        // S14
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[(i*30)  +24]);
        compressed_spec_mat[(i*30) + 6] = compressed_spec_mat[(i*30) + 6] / aux;
        compressed_spec_mat[(i*30) + 7] = compressed_spec_mat[(i*30) + 7] / aux;
        // S15
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[(i*30) + 28]);
        compressed_spec_mat[(i*30) + 8] = compressed_spec_mat[(i*30) + 8] / aux;
        compressed_spec_mat[(i*30) + 9] = compressed_spec_mat[(i*30) + 9] / aux;
        // S24
        aux = sqrt(compressed_spec_mat[i*10]*compressed_spec_mat[(i*30) + 24]);
        compressed_spec_mat[(i*30) + 14] = compressed_spec_mat[(i*30) + 14] / aux;
        compressed_spec_mat[(i*30) + 15] = compressed_spec_mat[(i*30) + 15] / aux;
        // S25
        aux = sqrt(compressed_spec_mat[i*10]*compressed_spec_mat[(i*30) + 28]);
        compressed_spec_mat[(i*30) + 16] = compressed_spec_mat[(i*30) + 16] / aux;
        compressed_spec_mat[(i*30) + 17] = compressed_spec_mat[(i*30) + 17] / aux;
        // S34
        aux = sqrt(compressed_spec_mat[i*18]*compressed_spec_mat[(i*30) + 24]);
        compressed_spec_mat[(i*30) + 20] = compressed_spec_mat[(i*30) + 20] / aux;
        compressed_spec_mat[(i*30) + 21] = compressed_spec_mat[(i*30) + 21] / aux;
        // S35
        aux = sqrt(compressed_spec_mat[i*18]*compressed_spec_mat[(i*30) + 28]);
        compressed_spec_mat[(i*30) + 22] = compressed_spec_mat[(i*30) + 22] / aux;
        compressed_spec_mat[(i*30) + 23] = compressed_spec_mat[(i*30) + 23] / aux;
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

void init_header_bp( Header_TM_LFR_SCIENCE_BP_t *header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
//    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST >> 8);
//    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST);
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

void fill_averaged_spectral_matrix(void)
{
    /** This function fills spectral matrices related buffers with arbitrary data.
     *
     *  This function is for testing purpose only.
     *
     */

    float offset;
    float coeff;

    offset = 10.;
    coeff = 100000.;
    averaged_sm_f0[ 0 + 25 * 0  ] = 0. + offset;
    averaged_sm_f0[ 0 + 25 * 1  ] = 1. + offset;
    averaged_sm_f0[ 0 + 25 * 2  ] = 2. + offset;
    averaged_sm_f0[ 0 + 25 * 3  ] = 3. + offset;
    averaged_sm_f0[ 0 + 25 * 4  ] = 4. + offset;
    averaged_sm_f0[ 0 + 25 * 5  ] = 5. + offset;
    averaged_sm_f0[ 0 + 25 * 6  ] = 6. + offset;
    averaged_sm_f0[ 0 + 25 * 7  ] = 7. + offset;
    averaged_sm_f0[ 0 + 25 * 8  ] = 8. + offset;
    averaged_sm_f0[ 0 + 25 * 9  ] = 9. + offset;
    averaged_sm_f0[ 0 + 25 * 10 ] = 10. + offset;
    averaged_sm_f0[ 0 + 25 * 11 ] = 11. + offset;
    averaged_sm_f0[ 0 + 25 * 12 ] = 12. + offset;
    averaged_sm_f0[ 0 + 25 * 13 ] = 13. + offset;
    averaged_sm_f0[ 0 + 25 * 14 ] = 14. + offset;
    averaged_sm_f0[ 9 + 25 * 0  ] = -(0. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 1  ] = -(1. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 2  ] = -(2. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 3  ] = -(3. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 4  ] = -(4. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 5  ] = -(5. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 6  ] = -(6. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 7  ] = -(7. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 8  ] = -(8. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 9  ] = -(9. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 10 ] = -(10. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 11 ] = -(11. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 12 ] = -(12. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 13 ] = -(13. + offset)* coeff;
    averaged_sm_f0[ 9 + 25 * 14 ] = -(14. + offset)* coeff;

    offset = 10000000;
    averaged_sm_f0[ 16 + 25 * 0  ] = (0. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 1  ] = (1. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 2  ] = (2. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 3  ] = (3. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 4  ] = (4. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 5  ] = (5. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 6  ] = (6. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 7  ] = (7. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 8  ] = (8. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 9  ] = (9. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 10 ] = (10. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 11 ] = (11. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 12 ] = (12. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 13 ] = (13. + offset)* coeff;
    averaged_sm_f0[ 16 + 25 * 14 ] = (14. + offset)* coeff;

    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 0 ] = averaged_sm_f0[ 0 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 1 ] = averaged_sm_f0[ 1 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 2 ] = averaged_sm_f0[ 2 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 3 ] = averaged_sm_f0[ 3 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 4 ] = averaged_sm_f0[ 4 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 5 ] = averaged_sm_f0[ 5 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 6 ] = averaged_sm_f0[ 6 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 7 ] = averaged_sm_f0[ 7 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 8 ] = averaged_sm_f0[ 8 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 9 ] = averaged_sm_f0[ 9 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 10 ] = averaged_sm_f0[ 10 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 11 ] = averaged_sm_f0[ 11 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 12 ] = averaged_sm_f0[ 12 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 13 ] = averaged_sm_f0[ 13 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 14 ] = averaged_sm_f0[ 14 ];
    averaged_sm_f0[ (TOTAL_SIZE_SM/2) + 15 ] = averaged_sm_f0[ 15 ];
}

void reset_spectral_matrix_regs()
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





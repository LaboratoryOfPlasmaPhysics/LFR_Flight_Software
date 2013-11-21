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

BP1_t data_BP1[ NB_BINS_COMPRESSED_SM_F0 ];
float averaged_spec_mat_f0[ TOTAL_SIZE_SM ];
char averaged_spec_mat_f0_char[ TOTAL_SIZE_SM * 2 ];
float compressed_spec_mat_f0[ TOTAL_SIZE_COMPRESSED_MATRIX_f0 ];

//***********************************************************
// Interrupt Service Routine for spectral matrices processing
rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{
    unsigned char status;
    unsigned char i;

    status = spectral_matrix_regs->status; //[f2 f1 f0_1 f0_0]
    for (i=0; i<4; i++)
    {
        if ( ( (status >> i) & 0x01) == 1)  // (1) buffer rotation
        {
            switch(i)
            {
            case 0:
                if (spectral_matrix_regs->matrixF0_Address0 == (int) spec_mat_f0_0)
                {
                    spectral_matrix_regs->matrixF0_Address0 = (int) spec_mat_f0_0_bis;
                }
                else
                {
                    spectral_matrix_regs->matrixF0_Address0 = (int) spec_mat_f0_0;
                }
                spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffe;
                break;
            case 1:
                if (spectral_matrix_regs->matrixFO_Address1 == (int) spec_mat_f0_1)
                {
                    spectral_matrix_regs->matrixFO_Address1 = (int) spec_mat_f0_1_bis;
                }
                else
                {
                    spectral_matrix_regs->matrixFO_Address1 = (int) spec_mat_f0_1;
                }
                spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffd;
                break;
            case 2:
                if (spectral_matrix_regs->matrixF1_Address == (int) spec_mat_f1)
                {
                    spectral_matrix_regs->matrixF1_Address = (int) spec_mat_f1_bis;
                }
                else
                {
                    spectral_matrix_regs->matrixF1_Address = (int) spec_mat_f1;
                }
                spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffffb;
                break;
            case 3:
                if (spectral_matrix_regs->matrixF2_Address == (int) spec_mat_f2)
                {
                    spectral_matrix_regs->matrixF2_Address = (int) spec_mat_f2_bis;
                }
                else
                {
                    spectral_matrix_regs->matrixF2_Address = (int) spec_mat_f2;
                }
                spectral_matrix_regs->status = spectral_matrix_regs->status & 0xfffffff7;
                break;
            default:
                break;
            }
        }
    }

    // reset error codes to 0
    spectral_matrix_regs->status = spectral_matrix_regs->status & 0xffffffcf; // [1100 1111]

    if (rtems_event_send( Task_id[TASKID_SMIQ], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_4 );
    }
}

rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_SMIQ], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_4 );
    }
}

//************
// RTEMS TASKS

rtems_task smiq_task(rtems_task_argument argument) // process the Spectral Matrices IRQ
{
    rtems_event_set event_out;
    unsigned int nb_interrupt_f0 = 0;

    BOOT_PRINTF("in SMIQ *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        nb_interrupt_f0 = nb_interrupt_f0 + 1;
        if (nb_interrupt_f0 == NB_SM_TO_RECEIVE_BEFORE_AVF0 ){
            if (rtems_event_send( Task_id[TASKID_AVF0], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
            {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_3 );
            }
            nb_interrupt_f0 = 0;
        }
    }
}

rtems_task spw_bppr_task(rtems_task_argument argument)
{
    rtems_status_code status;
    rtems_event_set event_out;

    BOOT_PRINTF("in BPPR ***\n");

    while( true ){ // wait for an event to begin with the processing
        status = rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out);
    }
}

rtems_task avf0_task(rtems_task_argument argument)
{
    int i;
    static int nb_average;
    rtems_event_set event_out;
    rtems_status_code status;

    nb_average = 0;

    BOOT_PRINTF("in AVFO *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        for(i=0; i<TOTAL_SIZE_SM; i++){
            averaged_spec_mat_f0[i] = averaged_spec_mat_f0[i] + spec_mat_f0_a[i]
                                            + spec_mat_f0_b[i]
                                            + spec_mat_f0_c[i]
                                            + spec_mat_f0_d[i]
                                            + spec_mat_f0_e[i]
                                            + spec_mat_f0_f[i]
                                            + spec_mat_f0_g[i]
                                            + spec_mat_f0_h[i];
        }
        nb_average = nb_average + NB_SM_TO_RECEIVE_BEFORE_AVF0;
        if (nb_average == NB_AVERAGE_NORMAL_f0) {
            nb_average = 0;
            status = rtems_event_send( Task_id[TASKID_MATR], RTEMS_EVENT_0 ); // sending an event to the task 7, BPF0
            if (status != RTEMS_SUCCESSFUL) {
                printf("in AVF0 *** Error sending RTEMS_EVENT_0, code %d\n", status);
            }
        }
    }
}

rtems_task bpf0_task(rtems_task_argument argument)
{
    rtems_event_set event_out;
    static unsigned char LFR_BP1_F0[ NB_BINS_COMPRESSED_SM_F0 * 9 ];

    BOOT_PRINTF("in BPFO *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        matrix_compression(averaged_spec_mat_f0, 0, compressed_spec_mat_f0);
        BP1_set(compressed_spec_mat_f0, NB_BINS_COMPRESSED_SM_F0, LFR_BP1_F0);
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
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0

#ifdef GSA
#else
        fill_averaged_spectral_matrix( );
#endif
        convert_averaged_spectral_matrix( averaged_spec_mat_f0, averaged_spec_mat_f0_char);

        send_spectral_matrix( &headerASM, averaged_spec_mat_f0_char, SID_NORM_ASM_F0, &spw_ioctl_send_ASM, queue_id);
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

void matrix_compression(volatile float *averaged_spec_mat, unsigned char fChannel, float *compressed_spec_mat)
{
    int i;
    int j;
    switch (fChannel){
        case 0:
                for(i=0;i<NB_BINS_COMPRESSED_SM_F0;i++){
                    j = 17 + (i * 8);
                    compressed_spec_mat[i] = (averaged_spec_mat[j]
                                                    + averaged_spec_mat[j+1]
                                                    + averaged_spec_mat[j+2]
                                                    + averaged_spec_mat[j+3]
                                                    + averaged_spec_mat[j+4]
                                                    + averaged_spec_mat[j+5]
                                                    + averaged_spec_mat[j+6]
                                                    + averaged_spec_mat[j+7])/(8*NB_AVERAGE_NORMAL_f0);
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

void BP1_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1){
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

void BP2_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat){
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
    header->cntASM = 0x00;
    header->nrASM = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->blkNr[0] = 0x00;  // BLK_NR MSB
    header->blkNr[1] = 0x00;  // BLK_NR LSB
}

void send_spectral_matrix(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id)
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;

    header->sid = (unsigned char) sid;

    for (i=0; i<2; i++)
    {
        // BUILD THE DATA
        spw_ioctl_send->dlen = TOTAL_SIZE_SM;
        spw_ioctl_send->data = &spectral_matrix[ i * TOTAL_SIZE_SM];
        spw_ioctl_send->hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM + CCSDS_PROTOCOLE_EXTRA_BYTES;
        spw_ioctl_send->hdr = (char *) header;
        spw_ioctl_send->options = 0;

        // BUILD THE HEADER
        length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM;
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        header->sid = (unsigned char) sid;   // SID
        header->cntASM = 2;
        header->nrASM = (unsigned char) (i+1);
        header->blkNr[0] =(unsigned char)  ( (NB_BINS_PER_SM/2) >> 8 ); // BLK_NR MSB
        header->blkNr[1] = (unsigned char)   (NB_BINS_PER_SM/2);        // BLK_NR LSB
        // SET PACKET TIME
        header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->time[3] = (unsigned char) (time_management_regs->coarse_time);
        header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->time[5] = (unsigned char) (time_management_regs->fine_time);
        header->acquisitionTime[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->acquisitionTime[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->acquisitionTime[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->acquisitionTime[3] = (unsigned char) (time_management_regs->coarse_time);
        header->acquisitionTime[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->acquisitionTime[5] = (unsigned char) (time_management_regs->fine_time);
        // SEND PACKET
        status =  rtems_message_queue_send( queue_id, spw_ioctl_send, ACTION_MSG_SPW_IOCTL_SEND_SIZE);
        if (status != RTEMS_SUCCESSFUL) {
            printf("in send_spectral_matrix *** ERR %d\n", (int) status);
        }
    }
}

void convert_averaged_spectral_matrix( volatile float *input_matrix, char *output_matrix)
{
    unsigned int i;
    unsigned int j;
    char * pt_char_input;
    char * pt_char_output;

    pt_char_input = NULL;
    pt_char_output = NULL;

    for( i=0; i<NB_BINS_PER_SM; i++)
    {
        for ( j=0; j<NB_VALUES_PER_SM; j++)
        {
            pt_char_input =  (char*)  &input_matrix[       (i*NB_VALUES_PER_SM) + j   ];
            pt_char_output = (char*) &output_matrix[ 2 * ( (i*NB_VALUES_PER_SM) + j ) ];
            pt_char_output[0] = pt_char_input[0]; // bits 31 downto 24 of the float
            pt_char_output[1] = pt_char_input[1];  // bits 23 downto 16 of the float
        }
    }
}

void fill_averaged_spectral_matrix(void)
{
    /** This function fills spectral matrices related buffers with arbitrary data.
     *
     *  This function is for testing purpose only.
     *
     */

#ifdef GSA
    float offset = 10.;
    float coeff = 100000.;

    averaged_spec_mat_f0[ 0 + 25 * 0  ] = 0. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 1  ] = 1. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 2  ] = 2. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 3  ] = 3. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 4  ] = 4. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 5  ] = 5. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 6  ] = 6. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 7  ] = 7. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 8  ] = 8. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 9  ] = 9. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 10 ] = 10. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 11 ] = 11. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 12 ] = 12. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 13 ] = 13. + offset;
    averaged_spec_mat_f0[ 0 + 25 * 14 ] = 14. + offset;
    averaged_spec_mat_f0[ 9 + 25 * 0  ] = -(0. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 1  ] = -(1. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 2  ] = -(2. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 3  ] = -(3. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 4  ] = -(4. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 5  ] = -(5. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 6  ] = -(6. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 7  ] = -(7. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 8  ] = -(8. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 9  ] = -(9. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 10 ] = -(10. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 11 ] = -(11. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 12 ] = -(12. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 13 ] = -(13. + offset)* coeff;
    averaged_spec_mat_f0[ 9 + 25 * 14 ] = -(14. + offset)* coeff;
    offset = 10000000;
    averaged_spec_mat_f0[ 16 + 25 * 0  ] = (0. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 1  ] = (1. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 2  ] = (2. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 3  ] = (3. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 4  ] = (4. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 5  ] = (5. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 6  ] = (6. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 7  ] = (7. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 8  ] = (8. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 9  ] = (9. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 10 ] = (10. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 11 ] = (11. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 12 ] = (12. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 13 ] = (13. + offset)* coeff;
    averaged_spec_mat_f0[ 16 + 25 * 14 ] = (14. + offset)* coeff;

    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 0 ] = averaged_spec_mat_f0[ 0 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 1 ] = averaged_spec_mat_f0[ 1 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 2 ] = averaged_spec_mat_f0[ 2 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 3 ] = averaged_spec_mat_f0[ 3 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 4 ] = averaged_spec_mat_f0[ 4 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 5 ] = averaged_spec_mat_f0[ 5 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 6 ] = averaged_spec_mat_f0[ 6 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 7 ] = averaged_spec_mat_f0[ 7 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 8 ] = averaged_spec_mat_f0[ 8 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 9 ] = averaged_spec_mat_f0[ 9 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 10 ] = averaged_spec_mat_f0[ 10 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 11 ] = averaged_spec_mat_f0[ 11 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 12 ] = averaged_spec_mat_f0[ 12 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 13 ] = averaged_spec_mat_f0[ 13 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 14 ] = averaged_spec_mat_f0[ 14 ];
    averaged_spec_mat_f0[ (TOTAL_SIZE_SM/2) + 15 ] = averaged_spec_mat_f0[ 15 ];
#else
    unsigned int i;

    for(i=0; i<TOTAL_SIZE_SM; i++)
    {
        if (spectral_matrix_regs->matrixF0_Address0 == (int) spec_mat_f0_0)
            averaged_spec_mat_f0[i] = (float) spec_mat_f0_0_bis[ SM_HEADER + i ];
        else
            averaged_spec_mat_f0[i] = (float) spec_mat_f0_0[ SM_HEADER + i ];
    }
#endif
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

#ifdef GSA
#else
    spectral_matrix_regs->matrixF0_Address0 = (int) spec_mat_f0_0;
    spectral_matrix_regs->matrixFO_Address1 = (int) spec_mat_f0_1;
    spectral_matrix_regs->matrixF1_Address = (int) spec_mat_f1;
    spectral_matrix_regs->matrixF2_Address = (int) spec_mat_f2;
#endif
}

//******************
// general functions





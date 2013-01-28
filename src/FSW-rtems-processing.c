#include <FSW-rtems-processing.h>
#include<math.h>
#include <stdio.h>
#include <leon.h>

float k14_re = 1;
float k14_im = 1;
float k14_bis_re = 1;
float k14_bis_im = 1;
float k14_tris_re = 1;
float k14_tris_im = 1;
float k15_re = 1;
float k15_im = 1;
float k15_bis_re = 1;
float k15_bis_im = 1;
float k24_re = 1;
float k24_im = 1;
float k24_bis_re = 1;
float k24_bis_im = 1;
float k24_tris_re = 1;
float k24_tris_im = 1;
float k25_re = 1;
float k25_im = 1;
float k25_bis_re = 1;
float k25_bis_im = 1;
float k34_re = 1;
float k34_im = 1;
float k44 = 1;
float k55 = 1;
float k45_re = 1;
float k45_im = 1;
float alpha_M = M_PI/4;

volatile int spectral_matrix_f0_a[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_b[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_c[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_d[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_e[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_f[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_g[TOTAL_SIZE_SPECTRAL_MATRIX];
volatile int spectral_matrix_f0_h[TOTAL_SIZE_SPECTRAL_MATRIX];
float averaged_spectral_matrix_f0[TOTAL_SIZE_SPECTRAL_MATRIX];
float compressed_spectral_matrix_f0[TOTAL_SIZE_COMPRESSED_MATRIX_f0];
unsigned char LFR_BP1_F0[NB_BINS_COMPRESSED_MATRIX_f0*9];

BP1_t data_BP1[NB_BINS_COMPRESSED_MATRIX_f0];

extern rtems_id   Task_id[];         /* array of task ids */

spectral_matrices_regs_t *spectral_matrices_regs;

// Interrupt Service Routine for spectral matrices processing
rtems_isr spectral_matrices_isr( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[4], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        printf("In spectral_matrices_isr *** Error sending event to AVF0\n");
}

rtems_task spw_smiq_task(rtems_task_argument argument) // process the Spectral Matrices IRQ
{
    rtems_event_set event_out;
    gptimer_regs_t *gptimer_regs;
    gptimer_regs = (gptimer_regs_t *) REGS_ADDRESS_GPTIMER;
    unsigned char nb_interrupt_f0 = 0;

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        nb_interrupt_f0 = nb_interrupt_f0 + 1;
        if (nb_interrupt_f0 == (NB_SM_TO_RECEIVE_BEFORE_AVF0-1) ){
            if (rtems_event_send( Task_id[6], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
                printf("In spw_smiq_task *** Error sending event to AVF0\n");
            nb_interrupt_f0 = 0;
        }
        gptimer_regs->timer2_ctrl = gptimer_regs->timer2_ctrl | 0x00000010;
    }
}

rtems_task spw_bppr_task(rtems_task_argument argument)
{
    rtems_status_code status;
    rtems_event_set event_out;
    static int nb_average_f0 = 0;
    //static int nb_average_f1 = 0;
    //static int nb_average_f2 = 0;

    while(1)

    spectral_matrices_regs = (struct spectral_matrices_regs_str *) REGS_ADDRESS_SPECTRAL_MATRICES;
    spectral_matrices_regs->address0 = (volatile int) spectral_matrix_f0_a;
    spectral_matrices_regs->address1 = (volatile int) spectral_matrix_f0_b;

    printf("In BPPR ***\n");

    while(1){ // wait for an event to begin with the processing
        status = rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out);
        if (status == RTEMS_SUCCESSFUL){
            if ((spectral_matrices_regs->ctrl & 0x00000001)==1){
                matrix_average(spectral_matrix_f0_a, averaged_spectral_matrix_f0);
                spectral_matrices_regs->ctrl = spectral_matrices_regs->ctrl & 0xfffffffe;
                //printf("f0_a\n");
                nb_average_f0++;
            }
            if (((spectral_matrices_regs->ctrl>>1) & 0x00000001)==1){
                matrix_average(spectral_matrix_f0_b, compressed_spectral_matrix_f0);
                spectral_matrices_regs->ctrl = spectral_matrices_regs->ctrl & 0xfffffffd;
                //printf("f0_b\n");
                nb_average_f0++;
            }
            if (nb_average_f0 == NB_AVERAGE_NORMAL_f0){
                    matrix_compression(averaged_spectral_matrix_f0, 0, compressed_spectral_matrix_f0);
                    //printf("f0 compressed\n");
                    nb_average_f0 = 0;
                    matrix_reset(averaged_spectral_matrix_f0);
            }
        }
    }
}

void matrix_average(volatile int *spectral_matrix, float *averaged_spectral_matrix)
{
    int i;
    for(i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++){
        averaged_spectral_matrix[i] = averaged_spectral_matrix[i] + spectral_matrix_f0_a[i]
                                        + spectral_matrix_f0_b[i]
                                        + spectral_matrix_f0_c[i]
                                        + spectral_matrix_f0_d[i]
                                        + spectral_matrix_f0_e[i]
                                        + spectral_matrix_f0_f[i]
                                        + spectral_matrix_f0_g[i]
                                        + spectral_matrix_f0_h[i];
    }
}

void matrix_reset(float *averaged_spectral_matrix)
{
    int i;
    for(i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++){
        averaged_spectral_matrix_f0[i] = 0;
    }
}

void matrix_compression(float *averaged_spectral_matrix, unsigned char fChannel, float *compressed_spectral_matrix)
{
    int i, j;
    switch (fChannel){
        case 0:
                for(i=0;i<NB_BINS_COMPRESSED_MATRIX_f0;i++){
                    j = 17 + i * 8;
                    compressed_spectral_matrix[i] = (averaged_spectral_matrix[j]
                                                    + averaged_spectral_matrix[j+1]
                                                    + averaged_spectral_matrix[j+2]
                                                    + averaged_spectral_matrix[j+3]
                                                    + averaged_spectral_matrix[j+4]
                                                    + averaged_spectral_matrix[j+5]
                                                    + averaged_spectral_matrix[j+6]
                                                    + averaged_spectral_matrix[j+7])/(8*NB_AVERAGE_NORMAL_f0);
                }
            break;
        case 1:
            // case fChannel = f1 tp be completed later
            break;
        case 2:
            // case fChannel = f1 tp be completed later
            break;
        default:
            break;
    }
}

void BP1_set(float * compressed_spectral_matrix, unsigned char nb_bins_compressed_spectral_matrix, unsigned char * LFR_BP1){
    int i, j;
    unsigned char tmp_u_char;
    unsigned char * pt_char;
    float PSDB, PSDE;
    float NVEC_V0, NVEC_V1, NVEC_V2;
    float significand;
    int exponent;
    float aux, tr_SB_SB, tmp;
    float e_cross_b_re, e_cross_b_im;
    float n_cross_e_scal_b_re = 0, n_cross_e_scal_b_im = 0;
    float nx = 0, ny = 0;
    float bz_bz_star = 0;
    for(i=0; i<nb_bins_compressed_spectral_matrix; i++){
        //==============================================
        // BP1 PSD == B PAR_LFR_SC_BP1_PE_FL0 == 16 bits
        PSDB = compressed_spectral_matrix[i*30]      // S11
            + compressed_spectral_matrix[i*30+10]    // S22
            + compressed_spectral_matrix[i*30+18];   // S33
        significand = frexp(PSDB, &exponent);
        pt_char = (unsigned char*) &PSDB;
        LFR_BP1[i*9+8] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1[i*9+7] = pt_char[1];  // bits 23 downto 16 of the float
        //==============================================
        // BP1 PSD == E PAR_LFR_SC_BP1_PB_FL0 == 16 bits
        PSDE = compressed_spectral_matrix[i*30+24] * k44     // S44
            + compressed_spectral_matrix[i*30+28] * k55      // S55
            + compressed_spectral_matrix[i*30+26] * k45_re   // S45
            - compressed_spectral_matrix[i*30+27] * k45_im;  // S45
        pt_char = (unsigned char*) &PSDE;
        LFR_BP1[i*9+6] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1[i*9+5] = pt_char[1]; // bits 23 downto 16 of the float
        //==============================================================================
        // BP1 normal wave vector == PAR_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V2_F0 == 1 bits
        tmp = sqrt(
                    compressed_spectral_matrix[i*30+3]*compressed_spectral_matrix[i*30+3]     //Im S12
                    +compressed_spectral_matrix[i*30+5]*compressed_spectral_matrix[i*30+5]    //Im S13
                    +compressed_spectral_matrix[i*30+5]*compressed_spectral_matrix[i*30+13]   //Im S23
                    );
        NVEC_V0 = compressed_spectral_matrix[i*30+13] / tmp;  // Im S23
        NVEC_V1 = -compressed_spectral_matrix[i*30+5] / tmp;  // Im S13
        NVEC_V2 = compressed_spectral_matrix[i*30+1] / tmp;   // Im S12
        LFR_BP1[i*9+4] = (char) (NVEC_V0*256);
        LFR_BP1[i*9+3] = (char) (NVEC_V1*256);
        pt_char = (unsigned char*) &NVEC_V2;
        LFR_BP1[i*9+2] = pt_char[0] & 0x80;  // extract the sign of NVEC_V2
        //=======================================================
        // BP1 ellipticity == PAR_LFR_SC_BP1_ELLIP_F0   == 4 bits
        aux = 2*tmp / PSDB;                                             // compute the ellipticity
        tmp_u_char = (unsigned char) (aux*(16-1));                      // convert the ellipticity
        LFR_BP1[i*9+2] = LFR_BP1[i*9+2] | ((tmp_u_char&0x0f)<<3); // keeps 4 bits of the resulting unsigned char
        //==============================================================
        // BP1 degree of polarization == PAR_LFR_SC_BP1_DOP_F0 == 3 bits
        for(j = 0; j<NB_VALUES_PER_SPECTRAL_MATRIX;j++){
            tr_SB_SB = compressed_spectral_matrix[i*30]*compressed_spectral_matrix[i*30]
                    + compressed_spectral_matrix[i*30+10]*compressed_spectral_matrix[i*30+10]
                    + compressed_spectral_matrix[i*30+18]*compressed_spectral_matrix[i*30+18]
                    + 2 * compressed_spectral_matrix[i*30+2]*compressed_spectral_matrix[i*30+2]
                    + 2 * compressed_spectral_matrix[i*30+3]*compressed_spectral_matrix[i*30+3]
                    + 2 * compressed_spectral_matrix[i*30+4]*compressed_spectral_matrix[i*30+4]
                    + 2 * compressed_spectral_matrix[i*30+5]*compressed_spectral_matrix[i*30+5]
                    + 2 * compressed_spectral_matrix[i*30+12]*compressed_spectral_matrix[i*30+12]
                    + 2 * compressed_spectral_matrix[i*30+13]*compressed_spectral_matrix[i*30+13];
        }
        aux = PSDB*PSDB;
        tmp = ( 3*tr_SB_SB - aux ) / ( 2 * aux );
        tmp_u_char = (unsigned char) (NVEC_V0*(8-1));
        LFR_BP1[i*9+2] = LFR_BP1[i*9+2] | ((tmp_u_char&0x07)); // keeps 3 bits of the resulting unsigned char
        //=======================================================================================
        // BP1 z-component of the normalized Poynting flux == PAR_LFR_SC_BP1_SZ_F0 == 8 bits (7+1)
        e_cross_b_re = compressed_spectral_matrix[i*30+20]*k34_re
                        + compressed_spectral_matrix[i*30+6]*k14_re
                        + compressed_spectral_matrix[i*30+8]*k15_re
                        + compressed_spectral_matrix[i*30+14]*k24_re
                        + compressed_spectral_matrix[i*30+16]*k25_re;
        e_cross_b_im = compressed_spectral_matrix[i*30+21]*k34_im
                        + compressed_spectral_matrix[i*30+7]*k14_im
                        + compressed_spectral_matrix[i*30+9]*k15_im
                        + compressed_spectral_matrix[i*30+15]*k24_im
                        + compressed_spectral_matrix[i*30+17]*k25_im;
        tmp = e_cross_b_re / PSDE; // compute ReaSz
        LFR_BP1[i*9+1] = ((unsigned char) (tmp * 128)) & 0x7f; // is it always positive?
        tmp = e_cross_b_re * e_cross_b_im;
        pt_char = (unsigned char*) &tmp;
        LFR_BP1[i*9+1] = LFR_BP1[i*9+1] | (pt_char[0] & 0x80);  // extract the sign of ArgSz
        //======================================================================
        // BP1 phase velocity estimator == PAR_LFR_SC_BP1_VPHI_F0 == 8 bits (7+1)
        nx = -sin(alpha_M)*NVEC_V0 - cos(alpha_M)*NVEC_V1;
        ny = NVEC_V2;
        bz_bz_star = cos(alpha_M) * cos(alpha_M) * compressed_spectral_matrix[i*30]              // re S11
                        + sin(alpha_M) * sin(alpha_M) * compressed_spectral_matrix[i*30+10]      // re S22
                        - 2 * sin(alpha_M) * cos(alpha_M) * compressed_spectral_matrix[i*30+2];  // re S12
        n_cross_e_scal_b_re = nx * (compressed_spectral_matrix[i*30+8]*k15_bis_re
                                        +compressed_spectral_matrix[i*30+6]*k14_bis_re
                                        +compressed_spectral_matrix[i*30+16]*k25_bis_re
                                        +compressed_spectral_matrix[i*30+14]*k24_bis_re)
                                + ny * (compressed_spectral_matrix[i*30+6]*k14_tris_re
                                        +compressed_spectral_matrix[i*30+14]*k24_tris_re);
        n_cross_e_scal_b_im = nx * (compressed_spectral_matrix[i*30+8]*k15_bis_im
                                        +compressed_spectral_matrix[i*30+6]*k14_bis_im
                                        +compressed_spectral_matrix[i*30+16]*k25_bis_im
                                        +compressed_spectral_matrix[i*30+14]*k24_bis_im)
                                + ny * (compressed_spectral_matrix[i*30+6]*k14_tris_im
                                        +compressed_spectral_matrix[i*30+14]*k24_tris_im);
        tmp = n_cross_e_scal_b_re / bz_bz_star;
        LFR_BP1[i*9+0] = ((unsigned char) (tmp * 128)) & 0x7f; // is it always positive?
        tmp = n_cross_e_scal_b_re * n_cross_e_scal_b_im;
        pt_char = (unsigned char*) &tmp;
        LFR_BP1[i*9+1] = LFR_BP1[i*9+0] | (pt_char[0] & 0x80);  // extract the sign of ArgV
    }

}

void BP2_set(float * compressed_spectral_matrix, unsigned char nb_bins_compressed_spectral_matrix){
    // BP2 autocorrelation
    int i, aux = 0;
    for(i = 0; i<nb_bins_compressed_spectral_matrix; i++){
        // S12
        aux = sqrt(compressed_spectral_matrix[i*30]*compressed_spectral_matrix[i*30+10]);
        compressed_spectral_matrix[i*30+2] = compressed_spectral_matrix[i*30+2] / aux;
        compressed_spectral_matrix[i*30+3] = compressed_spectral_matrix[i*30+3] / aux;
        // S13
        aux = sqrt(compressed_spectral_matrix[i*30]*compressed_spectral_matrix[i*30+18]);
        compressed_spectral_matrix[i*30+4] = compressed_spectral_matrix[i*30+4] / aux;
        compressed_spectral_matrix[i*30+5] = compressed_spectral_matrix[i*30+5] / aux;
        // S23
        aux = sqrt(compressed_spectral_matrix[i*30+12]*compressed_spectral_matrix[i*30+18]);
        compressed_spectral_matrix[i*30+12] = compressed_spectral_matrix[i*30+12] / aux;
        compressed_spectral_matrix[i*30+13] = compressed_spectral_matrix[i*30+13] / aux;
        // S45
        aux = sqrt(compressed_spectral_matrix[i*30+24]*compressed_spectral_matrix[i*30+28]);
        compressed_spectral_matrix[i*30+26] = compressed_spectral_matrix[i*30+26] / aux;
        compressed_spectral_matrix[i*30+27] = compressed_spectral_matrix[i*30+27] / aux;
        // S14
        aux = sqrt(compressed_spectral_matrix[i*30]*compressed_spectral_matrix[i*30+24]);
        compressed_spectral_matrix[i*30+6] = compressed_spectral_matrix[i*30+6] / aux;
        compressed_spectral_matrix[i*30+7] = compressed_spectral_matrix[i*30+7] / aux;
        // S15
        aux = sqrt(compressed_spectral_matrix[i*30]*compressed_spectral_matrix[i*30+28]);
        compressed_spectral_matrix[i*30+8] = compressed_spectral_matrix[i*30+8] / aux;
        compressed_spectral_matrix[i*30+9] = compressed_spectral_matrix[i*30+9] / aux;
        // S24
        aux = sqrt(compressed_spectral_matrix[i*10]*compressed_spectral_matrix[i*30+24]);
        compressed_spectral_matrix[i*30+14] = compressed_spectral_matrix[i*30+14] / aux;
        compressed_spectral_matrix[i*30+15] = compressed_spectral_matrix[i*30+15] / aux;
        // S25
        aux = sqrt(compressed_spectral_matrix[i*10]*compressed_spectral_matrix[i*30+28]);
        compressed_spectral_matrix[i*30+16] = compressed_spectral_matrix[i*30+16] / aux;
        compressed_spectral_matrix[i*30+17] = compressed_spectral_matrix[i*30+17] / aux;
        // S34
        aux = sqrt(compressed_spectral_matrix[i*18]*compressed_spectral_matrix[i*30+24]);
        compressed_spectral_matrix[i*30+20] = compressed_spectral_matrix[i*30+20] / aux;
        compressed_spectral_matrix[i*30+21] = compressed_spectral_matrix[i*30+21] / aux;
        // S35
        aux = sqrt(compressed_spectral_matrix[i*18]*compressed_spectral_matrix[i*30+28]);
        compressed_spectral_matrix[i*30+22] = compressed_spectral_matrix[i*30+22] / aux;
        compressed_spectral_matrix[i*30+23] = compressed_spectral_matrix[i*30+23] / aux;
    }
}

rtems_task spw_avf0_task(rtems_task_argument argument){
    int i;
    static int nb_average;
    rtems_event_set event_out;
    rtems_status_code status;

    spectral_matrices_regs = (struct spectral_matrices_regs_str *) REGS_ADDRESS_SPECTRAL_MATRICES;
    spectral_matrices_regs->address0 = (volatile int) spectral_matrix_f0_a;
    spectral_matrices_regs->address1 = (volatile int) spectral_matrix_f0_b;

    nb_average = 0;

    PRINTF("In AVFO *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        for(i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++){
            averaged_spectral_matrix_f0[i] = averaged_spectral_matrix_f0[i] + spectral_matrix_f0_a[i]
                                            + spectral_matrix_f0_b[i]
                                            + spectral_matrix_f0_c[i]
                                            + spectral_matrix_f0_d[i]
                                            + spectral_matrix_f0_e[i]
                                            + spectral_matrix_f0_f[i]
                                            + spectral_matrix_f0_g[i]
                                            + spectral_matrix_f0_h[i];
        }
        spectral_matrices_regs->ctrl = spectral_matrices_regs->ctrl & 0xfffffffe; // reset the appropriate bit in the register
        nb_average = nb_average + NB_SM_TO_RECEIVE_BEFORE_AVF0;
        if (nb_average == NB_AVERAGE_NORMAL_f0) {
            nb_average = 0;
            status = rtems_event_send( Task_id[7], RTEMS_EVENT_0 ); // sending an event to the task 7, BPF0
            if (status != RTEMS_SUCCESSFUL) printf("IN TASK AVF0 *** Error sending RTEMS_EVENT_0, code %d\n", status);
        }
    }
}

rtems_task spw_bpf0_task(rtems_task_argument argument){
    rtems_event_set event_out;

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        matrix_compression(averaged_spectral_matrix_f0, 0, compressed_spectral_matrix_f0);
        BP1_set(compressed_spectral_matrix_f0, NB_BINS_COMPRESSED_MATRIX_f0, LFR_BP1_F0);
        //PRINTF("IN TASK BPF0 *** Matrix compressed, parameters calculated\n")
    }
}

//*******
// UNUSED
rtems_task spw_bppr_task_rate_monotonic(rtems_task_argument argument)
{/*
    rtems_status_code status;
    //static int nb_average_f1 = 0;
    //static int nb_average_f2 = 0;

    rtems_name  name;
    rtems_id    period;
    name = rtems_build_name( 'P', 'E', 'R', 'D' );
    status = rtems_rate_monotonic_create( name, &period );
    if( status != RTEMS_SUCCESSFUL ) {
        printf( "rtems_rate_monotonic_create failed with status of %d\n", status );
        //exit( 1 );
    }

    spectral_matrices_regs = (struct spectral_matrices_regs_str *) REGS_ADDRESS_SPECTRAL_MATRICES;
    spectral_matrices_regs->address0 = (volatile int) spectral_matrix_f0_a;
    spectral_matrices_regs->address1 = (volatile int) spectral_matrix_f0_b;

    printf("In BPPR BIS ***\n");

    while(1){ // launch the rate monotonic task
        if ( rtems_rate_monotonic_period( period, 8 ) == RTEMS_TIMEOUT ){
            printf("TIMEOUT\n");
            //break;
        }
        status = rtems_event_send( Task_id[6], RTEMS_EVENT_0 ); // sending an event to the task 6, AVF0
        if (status != RTEMS_SUCCESSFUL) printf("IN TASK BPPR BIS *** Error sending RTEMS_EVENT_0 to AVF0, code %d\n", status);
    }

    status = rtems_rate_monotonic_delete( period );
    if ( status != RTEMS_SUCCESSFUL ) {
        printf( "rtems_rate_monotonic_delete failed with status of %d.\n", status );
        //exit( 1 );
    }
    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
    //exit( 1 );*/
}





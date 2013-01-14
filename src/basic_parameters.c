#include <..\header\basic_parameters.h>
#include<math.h>

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

void BP1_set(){
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
    for(i=0; i<NB_BINS_COMPRESSED_MATRIX_f0; i++){
        //==============================================
        // BP1 PSD == B PAR_LFR_SC_BP1_PE_FL0 == 16 bits
        PSDB = compressed_spectral_matrix_f0[i*30]      // S11
            + compressed_spectral_matrix_f0[i*30+10]    // S22
            + compressed_spectral_matrix_f0[i*30+18];   // S33
        significand = frexp(PSDB, &exponent);
        pt_char = (unsigned char*) &PSDB;
        LFR_BP1_F0[i*9+8] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1_F0[i*9+7] = pt_char[1];  // bits 23 downto 16 of the float
        //==============================================
        // BP1 PSD == E PAR_LFR_SC_BP1_PB_FL0 == 16 bits
        PSDE = compressed_spectral_matrix_f0[i*30+24] * k44     // S44
            + compressed_spectral_matrix_f0[i*30+28] * k55      // S55
            + compressed_spectral_matrix_f0[i*30+26] * k45_re   // S45
            - compressed_spectral_matrix_f0[i*30+27] * k45_im;  // S45
        pt_char = (unsigned char*) &PSDE;
        LFR_BP1_F0[i*9+6] = pt_char[0]; // bits 31 downto 24 of the float
        LFR_BP1_F0[i*9+5] = pt_char[1]; // bits 23 downto 16 of the float
        //==============================================================================
        // BP1 normal wave vector == PAR_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
                               // == PAR_LFR_SC_BP1_NVEC_V2_F0 == 1 bits
        tmp = sqrt(
                    compressed_spectral_matrix_f0[i*30+3]*compressed_spectral_matrix_f0[i*30+3]     //Im S12
                    +compressed_spectral_matrix_f0[i*30+5]*compressed_spectral_matrix_f0[i*30+5]    //Im S13
                    +compressed_spectral_matrix_f0[i*30+5]*compressed_spectral_matrix_f0[i*30+13]   //Im S23
                    );
        NVEC_V0 = compressed_spectral_matrix_f0[i*30+13] / tmp;  // Im S23
        NVEC_V1 = -compressed_spectral_matrix_f0[i*30+5] / tmp;  // Im S13
        NVEC_V2 = compressed_spectral_matrix_f0[i*30+1] / tmp;   // Im S12
        LFR_BP1_F0[i*9+4] = (char) (NVEC_V0*256);
        LFR_BP1_F0[i*9+3] = (char) (NVEC_V1*256);
        pt_char = (unsigned char*) &NVEC_V2;
        LFR_BP1_F0[i*9+2] = pt_char[0] & 0x80;  // extract the sign of NVEC_V2
        //=======================================================
        // BP1 ellipticity == PAR_LFR_SC_BP1_ELLIP_F0   == 4 bits
        aux = 2*tmp / PSDB;                                             // compute the ellipticity
        tmp_u_char = (unsigned char) (aux*(16-1));                      // convert the ellipticity
        LFR_BP1_F0[i*9+2] = LFR_BP1_F0[i*9+2] | ((tmp_u_char&0x0f)<<3); // keeps 4 bits of the resulting unsigned char
        //==============================================================
        // BP1 degree of polarization == PAR_LFR_SC_BP1_DOP_F0 == 3 bits
        for(j = 0; j<NB_VALUES_PER_SPECTRAL_MATRIX;j++){
            tr_SB_SB = compressed_spectral_matrix_f0[i*30]*compressed_spectral_matrix_f0[i*30]
                    + compressed_spectral_matrix_f0[i*30+10]*compressed_spectral_matrix_f0[i*30+10]
                    + compressed_spectral_matrix_f0[i*30+18]*compressed_spectral_matrix_f0[i*30+18]
                    + 2 * compressed_spectral_matrix_f0[i*30+2]*compressed_spectral_matrix_f0[i*30+2]
                    + 2 * compressed_spectral_matrix_f0[i*30+3]*compressed_spectral_matrix_f0[i*30+3]
                    + 2 * compressed_spectral_matrix_f0[i*30+4]*compressed_spectral_matrix_f0[i*30+4]
                    + 2 * compressed_spectral_matrix_f0[i*30+5]*compressed_spectral_matrix_f0[i*30+5]
                    + 2 * compressed_spectral_matrix_f0[i*30+12]*compressed_spectral_matrix_f0[i*30+12]
                    + 2 * compressed_spectral_matrix_f0[i*30+13]*compressed_spectral_matrix_f0[i*30+13];
        }
        aux = PSDB*PSDB;
        tmp = ( 3*tr_SB_SB - aux ) / ( 2 * aux );
        tmp_u_char = (unsigned char) (NVEC_V0*(8-1));
        LFR_BP1_F0[i*9+2] = LFR_BP1_F0[i*9+2] | ((tmp_u_char&0x07)); // keeps 3 bits of the resulting unsigned char
        //=======================================================================================
        // BP1 z-component of the normalized Poynting flux == PAR_LFR_SC_BP1_SZ_F0 == 8 bits (7+1)
        e_cross_b_re = compressed_spectral_matrix_f0[i*30+20]*k34_re
                        + compressed_spectral_matrix_f0[i*30+6]*k14_re
                        + compressed_spectral_matrix_f0[i*30+8]*k15_re
                        + compressed_spectral_matrix_f0[i*30+14]*k24_re
                        + compressed_spectral_matrix_f0[i*30+16]*k25_re;
        e_cross_b_im = compressed_spectral_matrix_f0[i*30+21]*k34_im
                        + compressed_spectral_matrix_f0[i*30+7]*k14_im
                        + compressed_spectral_matrix_f0[i*30+9]*k15_im
                        + compressed_spectral_matrix_f0[i*30+15]*k24_im
                        + compressed_spectral_matrix_f0[i*30+17]*k25_im;
        tmp = e_cross_b_re / PSDE; // compute ReaSz
        LFR_BP1_F0[i*9+1] = ((unsigned char) (tmp * 128)) & 0x7f; // is it always positive?
        tmp = e_cross_b_re * e_cross_b_im;
        pt_char = (unsigned char*) &tmp;
        LFR_BP1_F0[i*9+1] = LFR_BP1_F0[i*9+1] | (pt_char[0] & 0x80);  // extract the sign of ArgSz
        //======================================================================
        // BP1 phase velocity estimator == PAR_LFR_SC_BP1_VPHI_F0 == 8 bits (7+1)
        nx = -sin(alpha_M)*NVEC_V0 - cos(alpha_M)*NVEC_V1;
        ny = NVEC_V2;
        bz_bz_star = cos(alpha_M) * cos(alpha_M) * compressed_spectral_matrix_f0[i*30]              // re S11
                        + sin(alpha_M) * sin(alpha_M) * compressed_spectral_matrix_f0[i*30+10]      // re S22
                        - 2 * sin(alpha_M) * cos(alpha_M) * compressed_spectral_matrix_f0[i*30+2];  // re S12
        n_cross_e_scal_b_re = nx * (compressed_spectral_matrix_f0[i*30+8]*k15_bis_re
                                        +compressed_spectral_matrix_f0[i*30+6]*k14_bis_re
                                        +compressed_spectral_matrix_f0[i*30+16]*k25_bis_re
                                        +compressed_spectral_matrix_f0[i*30+14]*k24_bis_re)
                                + ny * (compressed_spectral_matrix_f0[i*30+6]*k14_tris_re
                                        +compressed_spectral_matrix_f0[i*30+14]*k24_tris_re);
        n_cross_e_scal_b_im = nx * (compressed_spectral_matrix_f0[i*30+8]*k15_bis_im
                                        +compressed_spectral_matrix_f0[i*30+6]*k14_bis_im
                                        +compressed_spectral_matrix_f0[i*30+16]*k25_bis_im
                                        +compressed_spectral_matrix_f0[i*30+14]*k24_bis_im)
                                + ny * (compressed_spectral_matrix_f0[i*30+6]*k14_tris_im
                                        +compressed_spectral_matrix_f0[i*30+14]*k24_tris_im);
        tmp = n_cross_e_scal_b_re / bz_bz_star;
        LFR_BP1_F0[i*9+0] = ((unsigned char) (tmp * 128)) & 0x7f; // is it always positive?
        tmp = n_cross_e_scal_b_re * n_cross_e_scal_b_im;
        pt_char = (unsigned char*) &tmp;
        LFR_BP1_F0[i*9+1] = LFR_BP1_F0[i*9+0] | (pt_char[0] & 0x80);  // extract the sign of ArgV
    }

}

void BP2_set(){
    // BP2 autocorrelation
    int i, aux = 0;
    for(i = 0; i<NB_BINS_COMPRESSED_MATRIX_f0; i++){
        // S12
        aux = sqrt(compressed_spectral_matrix_f0[i*30]*compressed_spectral_matrix_f0[i*30+10]);
        compressed_spectral_matrix_f0[i*30+2] = compressed_spectral_matrix_f0[i*30+2] / aux;
        compressed_spectral_matrix_f0[i*30+3] = compressed_spectral_matrix_f0[i*30+3] / aux;
        // S13
        aux = sqrt(compressed_spectral_matrix_f0[i*30]*compressed_spectral_matrix_f0[i*30+18]);
        compressed_spectral_matrix_f0[i*30+4] = compressed_spectral_matrix_f0[i*30+4] / aux;
        compressed_spectral_matrix_f0[i*30+5] = compressed_spectral_matrix_f0[i*30+5] / aux;
        // S23
        aux = sqrt(compressed_spectral_matrix_f0[i*30+12]*compressed_spectral_matrix_f0[i*30+18]);
        compressed_spectral_matrix_f0[i*30+12] = compressed_spectral_matrix_f0[i*30+12] / aux;
        compressed_spectral_matrix_f0[i*30+13] = compressed_spectral_matrix_f0[i*30+13] / aux;
        // S45
        aux = sqrt(compressed_spectral_matrix_f0[i*30+24]*compressed_spectral_matrix_f0[i*30+28]);
        compressed_spectral_matrix_f0[i*30+26] = compressed_spectral_matrix_f0[i*30+26] / aux;
        compressed_spectral_matrix_f0[i*30+27] = compressed_spectral_matrix_f0[i*30+27] / aux;
        // S14
        aux = sqrt(compressed_spectral_matrix_f0[i*30]*compressed_spectral_matrix_f0[i*30+24]);
        compressed_spectral_matrix_f0[i*30+6] = compressed_spectral_matrix_f0[i*30+6] / aux;
        compressed_spectral_matrix_f0[i*30+7] = compressed_spectral_matrix_f0[i*30+7] / aux;
        // S15
        aux = sqrt(compressed_spectral_matrix_f0[i*30]*compressed_spectral_matrix_f0[i*30+28]);
        compressed_spectral_matrix_f0[i*30+8] = compressed_spectral_matrix_f0[i*30+8] / aux;
        compressed_spectral_matrix_f0[i*30+9] = compressed_spectral_matrix_f0[i*30+9] / aux;
        // S24
        aux = sqrt(compressed_spectral_matrix_f0[i*10]*compressed_spectral_matrix_f0[i*30+24]);
        compressed_spectral_matrix_f0[i*30+14] = compressed_spectral_matrix_f0[i*30+14] / aux;
        compressed_spectral_matrix_f0[i*30+15] = compressed_spectral_matrix_f0[i*30+15] / aux;
        // S25
        aux = sqrt(compressed_spectral_matrix_f0[i*10]*compressed_spectral_matrix_f0[i*30+28]);
        compressed_spectral_matrix_f0[i*30+16] = compressed_spectral_matrix_f0[i*30+16] / aux;
        compressed_spectral_matrix_f0[i*30+17] = compressed_spectral_matrix_f0[i*30+17] / aux;
        // S34
        aux = sqrt(compressed_spectral_matrix_f0[i*18]*compressed_spectral_matrix_f0[i*30+24]);
        compressed_spectral_matrix_f0[i*30+20] = compressed_spectral_matrix_f0[i*30+20] / aux;
        compressed_spectral_matrix_f0[i*30+21] = compressed_spectral_matrix_f0[i*30+21] / aux;
        // S35
        aux = sqrt(compressed_spectral_matrix_f0[i*18]*compressed_spectral_matrix_f0[i*30+28]);
        compressed_spectral_matrix_f0[i*30+22] = compressed_spectral_matrix_f0[i*30+22] / aux;
        compressed_spectral_matrix_f0[i*30+23] = compressed_spectral_matrix_f0[i*30+23] / aux;
    }
}



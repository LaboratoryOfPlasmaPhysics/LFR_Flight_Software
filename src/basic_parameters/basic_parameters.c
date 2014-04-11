// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1: 31/07/2013

#include "basic_parameters.h"
#include <math.h>
#include <stdio.h>

#define K44_PE    0
#define K55_PE    1
#define K45_PE_RE 2
#define K45_PE_IM 3

#define K14_SX_RE 4
#define K14_SX_IM 5
#define K15_SX_RE 6
#define K15_SX_IM 7
#define K24_SX_RE 8
#define K24_SX_IM 9
#define K25_SX_RE 10
#define K25_SX_IM 11
#define K34_SX_RE 12
#define K34_SX_IM 13
#define K35_SX_RE 14
#define K35_SX_IM 15

#define K24_NY_RE 16
#define K24_NY_IM 17
#define K25_NY_RE 18
#define K25_NY_IM 19
#define K34_NY_RE 20
#define K34_NY_IM 21
#define K35_NY_RE 22
#define K35_NY_IM 23

#define K24_NZ_RE 24
#define K24_NZ_IM 25
#define K25_NZ_RE 26
#define K25_NZ_IM 27
#define K34_NZ_RE 28
#define K34_NZ_IM 29
#define K35_NZ_RE 30
#define K35_NZ_IM 31

float k_f0[NB_BINS_COMPRESSED_MATRIX_f0][32];

void init_k_f0( void )
{
    unsigned char i;

    for(i=0; i<NB_BINS_COMPRESSED_MATRIX_f0; i++){
    k_f0[i][K44_PE]    = 1;
    k_f0[i][K55_PE]    = 1;
    k_f0[i][K45_PE_RE] = 1;
    k_f0[i][K45_PE_IM] = 1;

    k_f0[i][K14_SX_RE] = 1;
    k_f0[i][K14_SX_IM] = 1;
    k_f0[i][K15_SX_RE] = 1;
    k_f0[i][K15_SX_IM] = 1;
    k_f0[i][K24_SX_RE] = 1;
    k_f0[i][K24_SX_IM] = 1;
    k_f0[i][K25_SX_RE] = 1;
    k_f0[i][K25_SX_IM] = 1;
    k_f0[i][K34_SX_RE] = 1;
    k_f0[i][K34_SX_IM] = 1;
    k_f0[i][K35_SX_RE] = 1;
    k_f0[i][K35_SX_IM] = 1;

    k_f0[i][K24_NY_RE] = 1;
    k_f0[i][K24_NY_IM] = 1;
    k_f0[i][K25_NY_RE] = 1;
    k_f0[i][K25_NY_IM] = 1;
    k_f0[i][K34_NY_RE] = 1;
    k_f0[i][K34_NY_IM] = 1;
    k_f0[i][K35_NY_RE] = 1;
    k_f0[i][K35_NY_IM] = 1;

    k_f0[i][K24_NZ_RE] = 1;
    k_f0[i][K24_NZ_IM] = 1;
    k_f0[i][K25_NZ_RE] = 1;
    k_f0[i][K25_NZ_IM] = 1;
    k_f0[i][K34_NZ_RE] = 1;
    k_f0[i][K34_NZ_IM] = 1;
    k_f0[i][K35_NZ_RE] = 1;
    k_f0[i][K35_NZ_IM] = 1;
    }
}

float alpha_M = 45 * (3.1415927/180);

void BP1_set( float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1 ){
    int i, exponent;
    float PSDB;
    float PSDE;
    float tmp;
    float NVEC_V0;
    float NVEC_V1;
    float NVEC_V2;
    float aux;
    float tr_SB_SB;
    float e_cross_b_re;
    float e_cross_b_im;
    float n_cross_e_scal_b_re;
    float n_cross_e_scal_b_im;
    float ny;
    float nz;
    float bx_bx_star;
    float vphi;
    float significand;
    signed char nbitexp;
    signed char nbitsig;
    signed char expmin;
    signed char expmax;  //  8 bits
    short int rangesig;                            // 16 bits
    unsigned short int psd;
    unsigned short int tmp_u_short_int;       // 16 bits
    unsigned short int *pt_u_short_int;            // pointer on unsigned 16-bit words
    unsigned char tmp_u_char;                      //  8 bits
    unsigned char *pt_u_char;                      // pointer on unsigned 8-bit bytes

    init_k_f0();

#ifdef DEBUG_TCH
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
    printf("BP1 : \n");
#endif

    // initialization for managing the exponents of the floating point data:
    nbitexp = 5;                           // number of bits for the exponent
    expmax = 30;                           // maximum value of the exponent
    expmin = expmax - (1 << nbitexp) + 1;  // accordingly the minimum exponent value
    // for floating point data to be recorded on 12-bit words:
    nbitsig = 12 - nbitexp;  // number of bits for the significand
    rangesig = (1 << nbitsig)-1;  // == 2^nbitsig - 1

#ifdef DEBUG_TCH
    printf("nbitexp : %d, expmax : %d, expmin : %d\n", nbitexp, expmax, expmin);
    printf("nbitsig : %d, rangesig : %d\n", nbitsig, rangesig);
#endif

    for(i=0; i<nb_bins_compressed_spec_mat; i++){
        //==============================================
        // BP1 PSDB == PA_LFR_SC_BP1_PB_F0 == 12 bits = 5 bits (exponent) + 7 bits (significand)
        PSDB =  compressed_spec_mat[i*30]          // S11
              + compressed_spec_mat[i*30+9]        // S22
              + compressed_spec_mat[i*30+16];      // S33

        significand = frexpf(PSDB/3, &exponent);  // 0.5 <= significand < 1
                                                  // PSDB/3 = significand * 2^exponent
        // the division by 3 is to ensure that max value <= 2^30

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        psd = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                       // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP1[i*9+2]; // Affect an unsigned short int pointer with the
                                                                   // adress where the 16-bit word result will be stored
        *pt_u_short_int = psd | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("PSDB / 3    : %16.8e\n",PSDB/3);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("psd for PSDB significand : %d\n",psd);
        printf("tmp_u_short_int for PSDB exponent : %d\n",tmp_u_short_int);
        printf("*pt_u_short_int for PSDB exponent + significand: %.3d or %.4x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP1[i*9+3] : %.3d or %.2x\n",LFR_BP1[i*9+3], LFR_BP1[i*9+3]);
        printf("LFR_BP1[i*9+2] : %.3d or %.2x\n",LFR_BP1[i*9+2], LFR_BP1[i*9+2]);
#endif
        //==============================================
        // BP1 PSDE == PA_LFR_SC_BP1_PE_F0 == 12 bits = 5 bits (exponent) + 7 bits (significand)
        PSDE =  compressed_spec_mat[i*30+21] * k_f0[i][K44_PE]        // S44
              + compressed_spec_mat[i*30+24] * k_f0[i][K55_PE]        // S55
              + compressed_spec_mat[i*30+22] * k_f0[i][K45_PE_RE]     // S45 Re
              - compressed_spec_mat[i*30+23] * k_f0[i][K45_PE_IM];    // S45 Im

        significand = frexpf(PSDE/2, &exponent); // 0.5 <= significand < 1
                                                 // PSDE/2 = significand * 2^exponent
        // the division by 2 is to ensure that max value <= 2^30
        // should be reconsidered by taking into account the k-coefficients ...

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        psd = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                       // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP1[i*9+0]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = psd | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("PSDE / 2    : %16.8e\n",PSDE/2);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("psd for PSDE significand : %d\n",psd);
        printf("tmp_u_short_int for PSDE exponent : %d\n",tmp_u_short_int);
        printf("*pt_u_short_int for PSDE exponent + significand: %.3d or %.4x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP1[i*9+1] : %.3d or %.2x\n",LFR_BP1[i*9+1], LFR_BP1[i*9+1]);
        printf("LFR_BP1[i*9+0] : %.3d or %.2x\n",LFR_BP1[i*9+0], LFR_BP1[i*9+0]);
#endif
        //==============================================================================
        // BP1 normal wave vector == PA_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
                               // == PA_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
                               // == PA_LFR_SC_BP1_NVEC_V2_F0 == 1 sign bit
        tmp = sqrt( compressed_spec_mat[i*30+2] *compressed_spec_mat[i*30+2]   //Im S12
                   +compressed_spec_mat[i*30+4] *compressed_spec_mat[i*30+4]   //Im S13
                   +compressed_spec_mat[i*30+11]*compressed_spec_mat[i*30+11]  //Im S23
                   );
        NVEC_V0 =  compressed_spec_mat[i*30+11]/ tmp;  // S23 Im  => n1
        NVEC_V1 = -compressed_spec_mat[i*30+4] / tmp;  // S13 Im  => n2
        NVEC_V2 =  compressed_spec_mat[i*30+2] / tmp;  // S12 Im  => n3

        LFR_BP1[i*9+4] = (unsigned char) (NVEC_V0*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP1[i*9+5] = (unsigned char) (NVEC_V1*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        pt_u_char = (unsigned char*) &NVEC_V2;    // affect an unsigned char pointer with the adress of NVEC_V2
#ifdef LSB_FIRST_TCH
        LFR_BP1[i*9+6] = pt_u_char[3] & 0x80;  // extract the sign bit of NVEC_V2 (32-bit float, sign bit in the 4th octet:PC convention)
                                                  // record it at the 8th bit position (from the right to the left) of LFR_BP1[i*9+6]
#endif
#ifdef MSB_FIRST_TCH
        LFR_BP1[i*9+6] = pt_u_char[0] & 0x80;  // extract the sign bit of NVEC_V2 (32-bit float, sign bit in the 0th octet:SPARC convention)
                                                  // record it at the 8th bit position (from the right to the left) of LFR_BP1[i*9+6]
#endif
#ifdef DEBUG_TCH
        printf("NVEC_V0  : %16.8e\n",NVEC_V0);
        printf("NVEC_V1  : %16.8e\n",NVEC_V1);
        printf("NVEC_V2  : %16.8e\n",NVEC_V2);
        printf("LFR_BP1[i*9+4] for NVEC_V0 : %u\n",LFR_BP1[i*9+4]);
        printf("LFR_BP1[i*9+5] for NVEC_V1 : %u\n",LFR_BP1[i*9+5]);
        printf("LFR_BP1[i*9+6] for NVEC_V2 : %u\n",LFR_BP1[i*9+6]);
#endif
        //=======================================================
        // BP1 ellipticity == PA_LFR_SC_BP1_ELLIP_F0 == 4 bits
        aux = 2*tmp / PSDB;                        // compute the ellipticity

        tmp_u_char = (unsigned char) (aux*15 + 0.5); // shift and cast into a 8-bit unsigned char with rounding
                                                     // where just the first 4 bits are used (0, ..., 15)
        LFR_BP1[i*9+6] = LFR_BP1[i*9+6] | (tmp_u_char << 3); // put these 4 bits next to the right place
                                                                   // of the sign bit of NVEC_V2 (recorded
                                                                   // previously in LFR_BP1[i*9+6])
#ifdef DEBUG_TCH
        printf("ellipticity  : %16.8e\n",aux);
        printf("tmp_u_char for ellipticity : %u\n",tmp_u_char);
        printf("LFR_BP1[i*9+6] for NVEC_V2 + ellipticity : %u\n",LFR_BP1[i*9+6]);
#endif
        //==============================================================
        // BP1 degree of polarization == PA_LFR_SC_BP1_DOP_F0 == 3 bits
        tr_SB_SB = compressed_spec_mat[i*30]     *compressed_spec_mat[i*30]
                 + compressed_spec_mat[i*30+9]   *compressed_spec_mat[i*30+9]
                 + compressed_spec_mat[i*30+16]  *compressed_spec_mat[i*30+16]
                 + 2 * compressed_spec_mat[i*30+1] *compressed_spec_mat[i*30+1]
                 + 2 * compressed_spec_mat[i*30+2] *compressed_spec_mat[i*30+2]
                 + 2 * compressed_spec_mat[i*30+3] *compressed_spec_mat[i*30+3]
                 + 2 * compressed_spec_mat[i*30+4] *compressed_spec_mat[i*30+4]
                 + 2 * compressed_spec_mat[i*30+10]*compressed_spec_mat[i*30+10]
                 + 2 * compressed_spec_mat[i*30+11]*compressed_spec_mat[i*30+11];
        aux = PSDB*PSDB;
        tmp = ( 3*tr_SB_SB - aux ) / ( 2 * aux );   // compute the degree of polarisation

        tmp_u_char = (unsigned char) (tmp*7 + 0.5);// shift and cast into a 8-bit unsigned char with rounding
                                                   // where just the first 3 bits are used (0, ..., 7)
        LFR_BP1[i*9+6] = LFR_BP1[i*9+6] | tmp_u_char; // record these 3 bits at the 3 first bit positions
                                                            // (from the right to the left) of LFR_BP1[i*9+6]
#ifdef DEBUG_TCH
        printf("DOP  : %16.8e\n",tmp);
        printf("tmp_u_char for DOP : %u\n",tmp_u_char);
        printf("LFR_BP1[i*9+6] for NVEC_V2 + ellipticity + DOP : %u\n",LFR_BP1[i*9+6]);
#endif
        //=======================================================================================
        // BP1 X_SO-component of the Poynting flux == PA_LFR_SC_BP1_SX_F0 == 8 (+ 2) bits
        //                                          = 5 bits (exponent) + 3 bits (significand)
        //                                          + 1 sign bit + 1 argument bit (two sectors)
        e_cross_b_re =  compressed_spec_mat[i*30+17]*k_f0[i][K34_SX_RE]  //S34 Re
                      + compressed_spec_mat[i*30+19]*k_f0[i][K35_SX_RE]  //S35 Re
                      + compressed_spec_mat[i*30+5] *k_f0[i][K14_SX_RE]  //S14 Re
                      + compressed_spec_mat[i*30+7] *k_f0[i][K15_SX_RE]  //S15 Re
                      + compressed_spec_mat[i*30+12]*k_f0[i][K24_SX_RE]  //S24 Re
                      + compressed_spec_mat[i*30+14]*k_f0[i][K25_SX_RE]  //S25 Re
                      + compressed_spec_mat[i*30+18]*k_f0[i][K34_SX_IM]  //S34 Im
                      + compressed_spec_mat[i*30+20]*k_f0[i][K35_SX_IM]  //S35 Im
                      + compressed_spec_mat[i*30+6] *k_f0[i][K14_SX_IM]  //S14 Im
                      + compressed_spec_mat[i*30+8] *k_f0[i][K15_SX_IM]  //S15 Im
                      + compressed_spec_mat[i*30+13]*k_f0[i][K24_SX_IM]  //S24 Im
                      + compressed_spec_mat[i*30+15]*k_f0[i][K25_SX_IM]; //S25 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        e_cross_b_im =  compressed_spec_mat[i*30+17]*k_f0[i][K34_SX_IM]  //S34 Re
                      + compressed_spec_mat[i*30+19]*k_f0[i][K35_SX_IM]  //S35 Re
                      + compressed_spec_mat[i*30+5] *k_f0[i][K14_SX_IM]  //S14 Re
                      + compressed_spec_mat[i*30+7] *k_f0[i][K15_SX_IM]  //S15 Re
                      + compressed_spec_mat[i*30+12]*k_f0[i][K24_SX_IM]  //S24 Re
                      + compressed_spec_mat[i*30+14]*k_f0[i][K25_SX_IM]  //S25 Re
                      - compressed_spec_mat[i*30+18]*k_f0[i][K34_SX_RE]  //S34 Im
                      - compressed_spec_mat[i*30+20]*k_f0[i][K35_SX_RE]  //S35 Im
                      - compressed_spec_mat[i*30+6] *k_f0[i][K14_SX_RE]  //S14 Im
                      - compressed_spec_mat[i*30+8] *k_f0[i][K15_SX_RE]  //S15 Im
                      - compressed_spec_mat[i*30+13]*k_f0[i][K24_SX_RE]  //S24 Im
                      - compressed_spec_mat[i*30+15]*k_f0[i][K25_SX_RE]; //S25 Im
#ifdef DEBUG_TCH
        printf("ReaSX / 2   : %16.8e\n",e_cross_b_re/2);
#endif
        pt_u_char = (unsigned char*) &e_cross_b_re; // Affect an unsigned char pointer with the adress of e_cross_b_re
#ifdef LSB_FIRST_TCH
        LFR_BP1[i*9+1] = LFR_BP1[i*9+1] | (pt_u_char[3] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of LFR_BP1[i*9+1]
        pt_u_char[3] = (pt_u_char[3] & 0x7f);       // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
#ifdef MSB_FIRST_TCH
        LFR_BP1[i*9+1] = LFR_BP1[i*9+1] | (pt_u_char[0] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 0th octet:SPARC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of LFR_BP1[i*9+1]
        pt_u_char[0] = (pt_u_char[0] & 0x7f);       // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
        significand = frexpf(e_cross_b_re/2, &exponent);// 0.5 <= significand < 1
                                                        // ReaSX/2 = significand * 2^exponent
        // The division by 2 is to ensure that max value <= 2^30 (rough estimate)
        // Should be reconsidered by taking into account the k-coefficients ...

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        LFR_BP1[i*9+7] = (unsigned char) ((significand*2-1)*7 + 0.5); // Shift and cast into a 8-bit unsigned char with rounding
                                                                         // where just the first 3 bits are used (0, ..., 7)
        tmp_u_char = (unsigned char) (exponent-expmin); // Shift and cast into a 8-bit unsigned char where
                                                        // just the first 5 bits are used (0, ..., 2^5-1)
#ifdef DEBUG_TCH
        printf("|ReaSX| / 2 : %16.8e\n",e_cross_b_re/2);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("LFR_BP1[i*9+7] for ReaSX significand : %u\n",LFR_BP1[i*9+7]);
        printf("tmp_u_char        for ReaSX exponent    : %d\n",tmp_u_char);
#endif
        LFR_BP1[i*9+7] = LFR_BP1[i*9+7] | (tmp_u_char << 3); // shift these 5 bits to the left before logical addition
                                                                   // with LFR_BP1[i*9+7]
#ifdef DEBUG_TCH
        printf("LFR_BP1[i*9+7] for ReaSX exponent + significand : %u\n",LFR_BP1[i*9+7]);
        printf("LFR_BP1[i*9+1] for ReaSX sign + PSDE 'exponent' : %u\n",LFR_BP1[i*9+1]);
        printf("ImaSX / 2   : %16.8e\n",e_cross_b_im/2);
#endif
        pt_u_char = (unsigned char*) &e_cross_b_im; // Affect an unsigned char pointer with the adress of e_cross_b_im
#ifdef LSB_FIRST_TCH
        pt_u_char[3] = pt_u_char[3] & 0x7f;         // Make e_cross_b_im be positive in any case: |ImaSX|
#endif
#ifdef MSB_FIRST_TCH
        pt_u_char[0] = pt_u_char[0] & 0x7f;         // Make e_cross_b_im be positive in any case: |ImaSX|
#endif
        tmp_u_char = (e_cross_b_im > e_cross_b_re) ? 0x40 : 0x00; // Determine the sector argument of SX. If |Im| > |Re| affect
                                                                  // an unsigned 8-bit char with 01000000; otherwise with null.
        LFR_BP1[i*9+1] = LFR_BP1[i*9+1] |  tmp_u_char;      // Record it as a sign bit at the 7th bit position (from the right
                                                                  // to the left) of LFR_BP1[i*9+1], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|ImaSX| / 2 : %16.8e\n",e_cross_b_im/2);
        printf("ArgSX sign  : %u\n",tmp_u_char);
        printf("LFR_BP1[i*9+1] for ReaSX & ArgSX signs + PSDE 'exponent'  : %u\n",LFR_BP1[i*9+1]);
#endif
        //======================================================================
        // BP1 phase velocity estimator == PA_LFR_SC_BP1_VPHI_F0 == 8 (+ 2) bits
        //                                          = 5 bits (exponent) + 3 bits (significand)
        //                                          + 1 sign bit + 1 argument bit (two sectors)
        ny = sin(alpha_M)*NVEC_V1 + cos(alpha_M)*NVEC_V2;
        nz = NVEC_V0;
        bx_bx_star = cos(alpha_M)*cos(alpha_M)*compressed_spec_mat[i*30+9]   // S22 Re
                   + sin(alpha_M)*sin(alpha_M)*compressed_spec_mat[i*30+16]  // S33 Re
                 - 2*sin(alpha_M)*cos(alpha_M)*compressed_spec_mat[i*30+10]; // S23 Re

        n_cross_e_scal_b_re = ny * (compressed_spec_mat[i*30+12]*k_f0[i][K24_NY_RE]  //S24 Re
                                   +compressed_spec_mat[i*30+14]*k_f0[i][K25_NY_RE]  //S25 Re
                                   +compressed_spec_mat[i*30+17]*k_f0[i][K34_NY_RE]  //S34 Re
                                   +compressed_spec_mat[i*30+19]*k_f0[i][K35_NY_RE]  //S35 Re
                                   +compressed_spec_mat[i*30+13]*k_f0[i][K24_NY_IM]  //S24 Im
                                   +compressed_spec_mat[i*30+15]*k_f0[i][K25_NY_IM]  //S25 Im
                                   +compressed_spec_mat[i*30+18]*k_f0[i][K34_NY_IM]  //S34 Im
                                   +compressed_spec_mat[i*30+20]*k_f0[i][K35_NY_IM]) //S35 Im
                            + nz * (compressed_spec_mat[i*30+12]*k_f0[i][K24_NZ_RE]  //S24 Re
                                   +compressed_spec_mat[i*30+14]*k_f0[i][K25_NZ_RE]  //S25 Re
                                   +compressed_spec_mat[i*30+17]*k_f0[i][K34_NZ_RE]  //S34 Re
                                   +compressed_spec_mat[i*30+19]*k_f0[i][K35_NZ_RE]  //S35 Re
                                   +compressed_spec_mat[i*30+13]*k_f0[i][K24_NZ_IM]  //S24 Im
                                   +compressed_spec_mat[i*30+15]*k_f0[i][K25_NZ_IM]  //S25 Im
                                   +compressed_spec_mat[i*30+18]*k_f0[i][K34_NZ_IM]  //S34 Im
                                   +compressed_spec_mat[i*30+20]*k_f0[i][K35_NZ_IM]);//S35 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        n_cross_e_scal_b_im = ny * (compressed_spec_mat[i*30+12]*k_f0[i][K24_NY_IM]  //S24 Re
                                   +compressed_spec_mat[i*30+14]*k_f0[i][K25_NY_IM]  //S25 Re
                                   +compressed_spec_mat[i*30+17]*k_f0[i][K34_NY_IM]  //S34 Re
                                   +compressed_spec_mat[i*30+19]*k_f0[i][K35_NY_IM]  //S35 Re
                                   -compressed_spec_mat[i*30+13]*k_f0[i][K24_NY_RE]  //S24 Im
                                   -compressed_spec_mat[i*30+15]*k_f0[i][K25_NY_RE]  //S25 Im
                                   -compressed_spec_mat[i*30+18]*k_f0[i][K34_NY_RE]  //S34 Im
                                   -compressed_spec_mat[i*30+20]*k_f0[i][K35_NY_RE]) //S35 Im
                            + nz * (compressed_spec_mat[i*30+12]*k_f0[i][K24_NZ_IM]  //S24 Re
                                   +compressed_spec_mat[i*30+14]*k_f0[i][K25_NZ_IM]  //S25 Re
                                   +compressed_spec_mat[i*30+17]*k_f0[i][K34_NZ_IM]  //S34 Re
                                   +compressed_spec_mat[i*30+19]*k_f0[i][K35_NZ_IM]  //S35 Re
                                   -compressed_spec_mat[i*30+13]*k_f0[i][K24_NZ_RE]  //S24 Im
                                   -compressed_spec_mat[i*30+15]*k_f0[i][K25_NZ_RE]  //S25 Im
                                   -compressed_spec_mat[i*30+18]*k_f0[i][K34_NZ_RE]  //S34 Im
                                   -compressed_spec_mat[i*30+20]*k_f0[i][K35_NZ_RE]);//S35 Im
#ifdef DEBUG_TCH
        printf("n_cross_e_scal_b_re   : %16.8e\n",n_cross_e_scal_b_re);
        printf("n_cross_e_scal_b_im   : %16.8e\n",n_cross_e_scal_b_im);
#endif
        // vphi = n_cross_e_scal_b_re / bx_bx_star => sign(VPHI) = sign(n_cross_e_scal_b_re)
        pt_u_char = (unsigned char*) &n_cross_e_scal_b_re;     // Affect an unsigned char pointer with the adress of n_cross_e_scal_b_re
#ifdef LSB_FIRST_TCH
        LFR_BP1[i*9+3] = LFR_BP1[i*9+3] | (pt_u_char[3] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of LFR_BP1[i*9+3]
        pt_u_char[3] = (pt_u_char[3] & 0x7f);     // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
#ifdef MSB_FIRST_TCH
        LFR_BP1[i*9+3] = LFR_BP1[i*9+3] | (pt_u_char[0] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 0th octet:SPARC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of LFR_BP1[i*9+3]
        pt_u_char[0] = (pt_u_char[0] & 0x7f);     // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
        vphi = n_cross_e_scal_b_re / bx_bx_star;  // Compute |VPHI|

        significand = frexpf(vphi/2, &exponent);   // 0.5 <= significand < 1
                                                   // vphi/2 = significand * 2^exponent
        // The division by 2 is to ensure that max value <= 2^30 (rough estimate)
        // Should be reconsidered by taking into account the k-coefficients ...

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }
#ifdef DEBUG_TCH
        printf("|VPHI| / 2  : %16.8e\n",vphi/2);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif
        LFR_BP1[i*9+8] = (unsigned char) ((significand*2-1)*7 + 0.5); // Shift and cast into a 8-bit unsigned char with rounding
                                                                         // where just the first 3 bits are used (0, ..., 7)
        tmp_u_char = (unsigned char) (exponent-expmin); // Shift and cast into a 8-bit unsigned char where
                                                        // just the first 5 bits are used (0, ..., 2^5-1)
#ifdef DEBUG_TCH
        printf("LFR_BP1[i*9+8] for VPHI significand : %u\n",LFR_BP1[i*9+8]);
        printf("tmp_u_char        for VPHI exponent    : %d\n",tmp_u_char);
#endif
        LFR_BP1[i*9+8] = LFR_BP1[i*9+8] | (tmp_u_char << 3); // shift these 5 bits to the left before logical addition
                                                                   // with LFR_BP1[i*9+8]
#ifdef DEBUG_TCH
        printf("LFR_BP1[i*9+8] for VPHI exponent + significand : %u\n",LFR_BP1[i*9+8]);
        printf("LFR_BP1[i*9+3] for VPHI sign + PSDB 'exponent' : %u\n",LFR_BP1[i*9+3]);
#endif
        pt_u_char = (unsigned char*) &n_cross_e_scal_b_im; // Affect an unsigned char pointer with the adress of n_cross_e_scal_b_im
#ifdef LSB_FIRST_TCH
        pt_u_char[3] = pt_u_char[3] & 0x7f;         // Make n_cross_e_scal_b_im be positive in any case: |ImaSX|
#endif
#ifdef MSB_FIRST_TCH
        pt_u_char[0] = pt_u_char[0] & 0x7f;         // Make n_cross_e_scal_b_im be positive in any case: |ImaSX|
#endif
        tmp_u_char = (n_cross_e_scal_b_im > n_cross_e_scal_b_re) ? 0x40 : 0x00; // Determine the sector argument of SX. If |Im| > |Re| affect
                                                                  // an unsigned 8-bit char with 01000000; otherwise with null.
        LFR_BP1[i*9+3] = LFR_BP1[i*9+3] |  tmp_u_char;      // Record it as a sign bit at the 7th bit position (from the right
                                                                  // to the left) of LFR_BP1[i*9+3], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|n_cross_e_scal_b_im|             : %16.8e\n",n_cross_e_scal_b_im);
        printf("|n_cross_e_scal_b_im|/bx_bx_star/2: %16.8e\n",n_cross_e_scal_b_im/bx_bx_star/2);
        printf("ArgNEBX sign          : %u\n",tmp_u_char);
        printf("LFR_BP1[i*9+3] for VPHI & ArgNEBX signs + PSDB 'exponent'  : %u\n",LFR_BP1[i*9+3]);
#endif
    }
}

void BP2_set( float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP2 )
{
    int i, exponent;
    float aux, significand, cross_re, cross_im;
    signed char nbitexp, nbitsig, expmin, expmax;  //  8 bits
    short int rangesig;                            // 16 bits
    unsigned short int autocor, tmp_u_short_int;   // 16 bits
    unsigned short int *pt_u_short_int;            // pointer on unsigned 16-bit words

#ifdef DEBUG_TCH
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
    printf("BP2 : \n");
#endif

    // For floating point data to be recorded on 16-bit words :
    nbitexp = 6;             // number of bits for the exponent
    nbitsig = 16 - nbitexp;  // number of bits for the significand
    rangesig = (1 << nbitsig)-1;  // == 2^nbitsig - 1
    expmax = 32;
    expmin = expmax - (1 << nbitexp) + 1;

#ifdef DEBUG_TCH
    printf("nbitexp : %d, nbitsig : %d, rangesig : %d\n", nbitexp, nbitsig, rangesig);
    printf("expmin : %d, expmax : %d\n", expmin, expmax);
#endif

    for(i = 0; i<nb_bins_compressed_spec_mat; i++){
        //==============================================
        // BP2 normalized cross correlations == PA_LFR_SC_BP2_CROSS_F0 == 10 * (8+8) bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_0_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_0_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_1_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_1_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_2_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_2_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_3_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_3_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_4_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_4_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_5_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_5_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_6_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_6_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_7_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_7_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_8_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_8_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_RE_9_F0 == 8 bits
                                          // == PA_LFR_SC_BP2_CROSS_IM_9_F0 == 8 bits
        // S12
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[i*30+9]);
        cross_re = compressed_spec_mat[i*30+1] / aux;
        cross_im = compressed_spec_mat[i*30+2] / aux;
        LFR_BP2[i*30+10] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+20] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+10] for cross12_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+10]);
        printf("LFR_BP2[i*30+20] for cross12_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+20]);
#endif
        // S13
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[i*30+16]);
        cross_re = compressed_spec_mat[i*30+3] / aux;
        cross_im = compressed_spec_mat[i*30+4] / aux;
        LFR_BP2[i*30+11] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+21] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+11] for cross13_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+11]);
        printf("LFR_BP2[i*30+21] for cross13_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+21]);
#endif
        // S14
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[i*30+21]);
        cross_re = compressed_spec_mat[i*30+5] / aux;
        cross_im = compressed_spec_mat[i*30+6] / aux;
        LFR_BP2[i*30+12] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+22] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+12] for cross14_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+12]);
        printf("LFR_BP2[i*30+22] for cross14_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+22]);
#endif
        // S15
        aux = sqrt(compressed_spec_mat[i*30]*compressed_spec_mat[i*30+24]);
        cross_re = compressed_spec_mat[i*30+7] / aux;
        cross_im = compressed_spec_mat[i*30+8] / aux;
        LFR_BP2[i*30+13] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+23] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+13] for cross15_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+13]);
        printf("LFR_BP2[i*30+23] for cross15_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+23]);
#endif
        // S23
        aux = sqrt(compressed_spec_mat[i*30+9]*compressed_spec_mat[i*30+16]);
        cross_re = compressed_spec_mat[i*30+10] / aux;
        cross_im = compressed_spec_mat[i*30+11] / aux;
        LFR_BP2[i*30+14] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+24] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+14] for cross23_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+14]);
        printf("LFR_BP2[i*30+24] for cross23_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+24]);
#endif
        // S24
        aux = sqrt(compressed_spec_mat[i*30+9]*compressed_spec_mat[i*30+21]);
        cross_re = compressed_spec_mat[i*30+12] / aux;
        cross_im = compressed_spec_mat[i*30+13] / aux;
        LFR_BP2[i*30+15] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+25] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+15] for cross24_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+15]);
        printf("LFR_BP2[i*30+25] for cross24_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+25]);
#endif
        // S25
        aux = sqrt(compressed_spec_mat[i*30+9]*compressed_spec_mat[i*30+24]);
        cross_re = compressed_spec_mat[i*30+14] / aux;
        cross_im = compressed_spec_mat[i*30+15] / aux;
        LFR_BP2[i*30+16] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+26] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+16] for cross25_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+16]);
        printf("LFR_BP2[i*30+26] for cross25_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+26]);
#endif
        // S34
        aux = sqrt(compressed_spec_mat[i*30+16]*compressed_spec_mat[i*30+21]);
        cross_re = compressed_spec_mat[i*30+17] / aux;
        cross_im = compressed_spec_mat[i*30+18] / aux;
        LFR_BP2[i*30+17] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+27] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+17] for cross34_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+17]);
        printf("LFR_BP2[i*30+27] for cross34_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+27]);
#endif
        // S35
        aux = sqrt(compressed_spec_mat[i*30+16]*compressed_spec_mat[i*30+24]);
        cross_re = compressed_spec_mat[i*30+19] / aux;
        cross_im = compressed_spec_mat[i*30+20] / aux;
        LFR_BP2[i*30+18] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+28] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+18] for cross35_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+18]);
        printf("LFR_BP2[i*30+28] for cross35_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+28]);
#endif
        // S45
        aux = sqrt(compressed_spec_mat[i*30+21]*compressed_spec_mat[i*30+24]);
        cross_re = compressed_spec_mat[i*30+22] / aux;
        cross_im = compressed_spec_mat[i*30+23] / aux;
        LFR_BP2[i*30+19] = (unsigned char) (cross_re*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
        LFR_BP2[i*30+29] = (unsigned char) (cross_im*127.5 + 128); // shift and cast into a 8-bit unsigned char (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("LFR_BP2[i*30+19] for cross45_re (%16.8e) : %.3u\n",cross_re, LFR_BP2[i*30+19]);
        printf("LFR_BP2[i*30+29] for cross45_im (%16.8e) : %.3u\n",cross_im, LFR_BP2[i*30+29]);
#endif
        //==============================================
        // BP2  auto correlations == PA_LFR_SC_BP2_AUTO_F0 == 5*16 bits = 5*[6 bits (exponent) + 10 bits (significand)]
                               // == PA_LFR_SC_BP2_AUTO_A0_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A1_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A2_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A3_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A4_F0 == 16 bits
        // S11
        significand = frexpf(compressed_spec_mat[i*30], &exponent);  // 0.5 <= significand < 1
                                                                               // S11 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S11         : %16.8e\n",compressed_spec_mat[i*30]);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif
        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        autocor = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                           // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP2[i*30+0]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = autocor | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                              // left place of the significand bits (nbitsig), making
                                                              // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("autocor for S11 significand : %u\n",autocor );
        printf("tmp_u_char for S11 exponent : %u\n",tmp_u_short_int );
        printf("*pt_u_short_int for S11 exponent + significand : %.3d or %x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP2[i*30+1] : %u or %x\n",LFR_BP2[i*30+1], LFR_BP2[i*30+1]);
        printf("LFR_BP2[i*30+0] : %u or %x\n",LFR_BP2[i*30+0], LFR_BP2[i*30+0]);
#endif
        // S22
        significand = frexpf(compressed_spec_mat[i*30+9], &exponent);  // 0.5 <= significand < 1
                                                                                  // S22 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S22         : %16.8e\n",compressed_spec_mat[i*30+9]);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif
        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        autocor = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                           // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP2[i*30+2]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = autocor | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("autocor for S22 significand : %d\n",autocor );
        printf("tmp_u_char for S22 exponent : %d\n",tmp_u_short_int );
        printf("*pt_u_short_int for S22 exponent + significand : %.3d or %x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP2[i*30+3] : %.3d or %x\n",LFR_BP2[i*30+3], LFR_BP2[i*30+3]);
        printf("LFR_BP2[i*30+2] : %.3d or %x\n",LFR_BP2[i*30+2], LFR_BP2[i*30+2]);
#endif
        // S33
        significand = frexpf(compressed_spec_mat[i*30+16], &exponent);  // 0.5 <= significand < 1
                                                                                  // S33 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S33         : %16.8e\n",compressed_spec_mat[i*30+16]);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif
        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        autocor = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                           // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP2[i*30+4]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = autocor | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("autocor for S33 significand : %d\n",autocor );
        printf("tmp_u_char for S33 exponent : %d\n",tmp_u_short_int );
        printf("*pt_u_short_int for S33 exponent + significand : %.3d or %x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP2[i*30+5] : %.3d or %x\n",LFR_BP2[i*30+5], LFR_BP2[i*30+5]);
        printf("LFR_BP2[i*30+4] : %.3d or %x\n",LFR_BP2[i*30+4], LFR_BP2[i*30+4]);
#endif
        // S44
        significand = frexpf(compressed_spec_mat[i*30+21], &exponent);  // 0.5 <= significand < 1
                                                                                  // S44 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S44         : %16.8e\n",compressed_spec_mat[i*30+21]);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        autocor = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                           // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP2[i*30+6]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = autocor | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("autocor for S44 significand : %d\n",autocor );
        printf("tmp_u_char for S44 exponent : %d\n",tmp_u_short_int );
        printf("*pt_u_short_int for S44 exponent + significand : %.3d or %x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP2[i*30+7] : %.3d or %x\n",LFR_BP2[i*30+7], LFR_BP2[i*30+7]);
        printf("LFR_BP2[i*30+6] : %.3d or %x\n",LFR_BP2[i*30+6], LFR_BP2[i*30+6]);
#endif
        // S55
        significand = frexpf(compressed_spec_mat[i*30+24], &exponent);  // 0.5 <= significand < 1
                                                                                  // S55 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S55         : %16.8e\n",compressed_spec_mat[i*30+24]);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
#endif
        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {// in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;   // min value that can be recorded
        }

        autocor = (unsigned short int) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                           // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        tmp_u_short_int = (unsigned short int) (exponent-expmin); // Shift and cast into a 16-bit unsigned int
                                                                  // where just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        pt_u_short_int = (unsigned short int*) &LFR_BP2[i*30+8]; // Affect an unsigned short int pointer with the
                                                                    // adress where the 16-bit word result will be stored
        *pt_u_short_int = autocor | (tmp_u_short_int << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                          // left place of the significand bits (nbitsig), making
                                                          // the 16-bit word to be recorded, and record it using the pointer
#ifdef DEBUG_TCH
        printf("autocor for S55 significand : %d\n",autocor );
        printf("tmp_u_char for S55 exponent : %d\n",tmp_u_short_int );
        printf("*pt_u_short_int for S55 exponent + significand : %.3d or %x\n",*pt_u_short_int, *pt_u_short_int);
        printf("LFR_BP2[i*30+9] : %.3d or %x\n",LFR_BP2[i*30+9], LFR_BP2[i*30+9]);
        printf("LFR_BP2[i*30+8] : %.3d or %x\n",LFR_BP2[i*30+8], LFR_BP2[i*30+8]);
#endif
    }
}

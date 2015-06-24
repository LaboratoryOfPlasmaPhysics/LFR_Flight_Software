// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014
// version 1.4: 16/05/2014
// version 1.5: 20/05/2014
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB + indices [0,2] <=> [1,3])
// version 1.8: 02/02/2015 (gestion des divisions par zéro)
// In the frame of RPW LFR Sofware ICD Issue3 Rev6 (27/01/2015) => R3 FSW
// version 2.0: 19/06/2015
// version 2.1: 22/06/2015 (modifs de Paul)
// version 2.2: 23/06/2015 (modifs de l'ordre de déclaration/définition de init_k_coefficients dans basic_parameters.c ... + maintien des declarations dans le .h)

#ifndef BASIC_PARAMETERS_H_INCLUDED
#define BASIC_PARAMETERS_H_INCLUDED

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "basic_parameters_params.h"

static inline void BP1_set(float * compressed_spec_mat, float * k_coeff_intercalib, unsigned char nb_bins_compressed_spec_mat, unsigned char * lfr_bp1);
static inline void BP2_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * lfr_bp2);

void init_k_coefficients_f0( float *k_coeff_intercalib, unsigned char nb_binscompressed_matrix );
void init_k_coefficients_f1( float *k_coeff_intercalib, unsigned char nb_binscompressed_matrix );
void init_k_coefficients_f2( float *k_coeff_intercalib, unsigned char nb_binscompressed_matrix );

void init_k_coefficients( float *k_coeff_intercalib, unsigned char nb_binscompressed_matrix );

//***********************************
// STATIC INLINE FUNCTION DEFINITIONS

void BP1_set( float * compressed_spec_mat, float * k_coeff_intercalib, uint8_t nb_bins_compressed_spec_mat, uint8_t * lfr_bp1 ){
    float PSDB;                         // 32-bit floating point
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
    int exponent;                       // 32-bit signed integer
    float alpha_M;

    uint8_t nbitexp;                    // 8-bit unsigned integer
    uint8_t nbitsig;
    uint8_t tmp_uint8;
    uint8_t *pt_uint8;                  // pointer on unsigned 8-bit integer
    int8_t expmin;                      // 8-bit signed integer
    int8_t expmax;
    uint16_t rangesig;                  // 16-bit unsigned integer
    uint16_t psd;
    uint16_t exp;
    uint16_t tmp_uint16;
    uint16_t i;

    alpha_M = 45 * (3.1415927/180);

#ifdef DEBUG_TCH
    printf("BP1 : \n");
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
#endif

    // initialization for managing the exponents of the floating point data:
    nbitexp = 6;                           // number of bits for the exponent
    expmax = 32+5;                         // maximum value of the exponent
    expmin = expmax - (1 << nbitexp) + 1;  // accordingly the minimum exponent value
    // for floating point data to be recorded on 16-bit words:
    nbitsig = 16 - nbitexp;       // number of bits for the significand
    rangesig = (1 << nbitsig)-1;  // == 2^nbitsig - 1

#ifdef DEBUG_TCH
    printf("nbitexp : %d, expmax : %d, expmin : %d\n", nbitexp, expmax, expmin);
    printf("nbitsig : %d, rangesig : %d\n", nbitsig, rangesig);
#endif

    for(i=0; i<nb_bins_compressed_spec_mat; i++){
        //==============================================
        // BP1 PSDB == PA_LFR_SC_BP1_PB_F0 == 16 bits = 6 bits (exponent) + 10 bits (significand)
        PSDB =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]          // S11
              + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]        // S22
              + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16];      // S33

        significand = frexpf(PSDB, &exponent);  // 0.5 <= significand < 1
                                                  // PSDB = significand * 2^exponent

        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        psd = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                             // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = psd | (exp << nbitsig);     // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+2] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp1[i*NB_BYTES_BP1+3] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+2] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp1[i*NB_BYTES_BP1+3] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("\nBin number: %d\n", i);
        printf("PSDB        : %16.8e\n",PSDB);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("psd for PSDB significand : %d\n",psd);
        printf("exp for PSDB exponent : %d\n",exp);
        printf("pt_uint8[1] for PSDB exponent + significand: %.3d or %.2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for PSDB            significand: %.3d or %.2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+2] : %.3d or %.2x\n",lfr_bp1[i*NB_BYTES_BP1+2], lfr_bp1[i*NB_BYTES_BP1+2]);
        printf("lfr_bp1[i*NB_BYTES_BP1+3] : %.3d or %.2x\n",lfr_bp1[i*NB_BYTES_BP1+3], lfr_bp1[i*NB_BYTES_BP1+3]);
#endif
        //==============================================
        // BP1 PSDE == PA_LFR_SC_BP1_PE_F0 == 16 bits = 6 bits (exponent) + 10 bits (significand)
        PSDE =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21] * k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K44_PE]        // S44
              + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24] * k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K55_PE]        // S55
              + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+22] * k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K45_PE_RE]     // S45 Re
              - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+23] * k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K45_PE_IM];    // S45 Im

        significand = frexpf(PSDE, &exponent); // 0.5 <= significand < 1
                                               // PSDE = significand * 2^exponent

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

        psd = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                             // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = psd | (exp << nbitsig);     // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+0] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp1[i*NB_BYTES_BP1+1] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+0] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp1[i*NB_BYTES_BP1+1] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("PSDE        : %16.8e\n",PSDE);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("psd for PSDE significand : %d\n",psd);
        printf("exp for PSDE exponent : %d\n",exp);
        printf("pt_uint8[1] for PSDE exponent + significand: %.3d or %.2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for PSDE            significand: %.3d or %.2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+0] : %.3d or %.2x\n",lfr_bp1[i*NB_BYTES_BP1+0], lfr_bp1[i*NB_BYTES_BP1+0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+1] : %.3d or %.2x\n",lfr_bp1[i*NB_BYTES_BP1+1], lfr_bp1[i*NB_BYTES_BP1+1]);
#endif
        //==============================================================================
        // BP1 normal wave vector == PA_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
                               // == PA_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
                               // == PA_LFR_SC_BP1_NVEC_V2_F0 == 1 sign bit
        tmp = sqrt( compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2]   //Im S12
                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4]   //Im S13
                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11]  //Im S23
                   );
        if (tmp != 0.) { // no division by 0.
            NVEC_V0 =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11]/ tmp;  // S23 Im  => n1
            NVEC_V1 = -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4] / tmp;  // S13 Im  => n2
            NVEC_V2 =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2] / tmp;  // S12 Im  => n3
        }
        else
        {
            NVEC_V0 = 0.;
            NVEC_V1 = 0.;
            NVEC_V2 = 0.;
        }
        lfr_bp1[i*NB_BYTES_BP1+4] = (uint8_t) (NVEC_V0*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp1[i*NB_BYTES_BP1+5] = (uint8_t) (NVEC_V1*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        pt_uint8 = (uint8_t*) &NVEC_V2;                              // Affect an uint8_t pointer with the adress of NVEC_V2
#ifdef LSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+6] = pt_uint8[3] & 0x80;  // Extract the sign bit of NVEC_V2 (32-bit float, sign bit in the 4th octet:PC convention)
                                                         // Record it at the 8th bit position (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#endif
#ifdef MSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+6] = pt_uint8[0] & 0x80;  // Extract the sign bit of NVEC_V2 (32-bit float, sign bit in the 1th octet:SPARC convention)
                                                         // Record it at the 8th bit position (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#endif
#ifdef DEBUG_TCH
        printf("NVEC_V0  : %16.8e\n",NVEC_V0);
        printf("NVEC_V1  : %16.8e\n",NVEC_V1);
        printf("NVEC_V2  : %16.8e\n",NVEC_V2);
        printf("lfr_bp1[i*NB_BYTES_BP1+4] for NVEC_V0 : %u\n",lfr_bp1[i*NB_BYTES_BP1+4]);
        printf("lfr_bp1[i*NB_BYTES_BP1+5] for NVEC_V1 : %u\n",lfr_bp1[i*NB_BYTES_BP1+5]);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 : %u\n",lfr_bp1[i*NB_BYTES_BP1+6]);
#endif
        //=======================================================
        // BP1 ellipticity == PA_LFR_SC_BP1_ELLIP_F0 == 4 bits
        if (PSDB != 0.) { // no division by 0.
            aux = 2*tmp / PSDB;                   // Compute the ellipticity
        }
        else
        {
            aux = 0.;
        }
        tmp_uint8 = (uint8_t) (aux*15 + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                                              // where just the first 4 bits are used (0, ..., 15)
        lfr_bp1[i*NB_BYTES_BP1+6] = lfr_bp1[i*NB_BYTES_BP1+6] | (tmp_uint8 << 3); // Put these 4 bits next to the right place
                                                                                  // of the sign bit of NVEC_V2 (recorded
                                                                                  // previously in lfr_bp1[i*NB_BYTES_BP1+6])
#ifdef DEBUG_TCH
        printf("ellipticity  : %16.8e\n",aux);
        printf("tmp_uint8 for ellipticity : %u\n",tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 + ellipticity : %u\n",lfr_bp1[i*NB_BYTES_BP1+6]);
#endif
        //==============================================================
        // BP1 degree of polarization == PA_LFR_SC_BP1_DOP_F0 == 3 bits
        tr_SB_SB = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]     *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]
                 + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]   *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]
                 + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]  *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+1] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+1]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+3] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+3]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4] *compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+10]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+10]
                 + 2 * compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11];
        aux = PSDB*PSDB;
        if (aux != 0.) { // no division by 0.
            tmp = ( 3*tr_SB_SB - aux ) / ( 2 * aux );  // Compute the degree of polarisation
        }
        else
        {
            tmp = 0.;
        }
        tmp_uint8 = (uint8_t) (tmp*7 + 0.5);       // Shift and cast into a 8-bit uint8_t with rounding
                                                   // where just the first 3 bits are used (0, ..., 7)
        lfr_bp1[i*NB_BYTES_BP1+6] = lfr_bp1[i*NB_BYTES_BP1+6] | tmp_uint8; // Record these 3 bits at the 3 first bit positions
                                                                           // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#ifdef DEBUG_TCH
        printf("DOP  : %16.8e\n",tmp);
        printf("tmp_uint8 for DOP : %u\n",tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 + ellipticity + DOP : %u\n",lfr_bp1[i*NB_BYTES_BP1+6]);
#endif
        //=======================================================================================
        // BP1 X_SO-component of the Poynting flux == PA_LFR_SC_BP1_SX_F0 == 16 bits
        //                                          = 1 sign bit + 1 argument bit (two sectors)
        //                                          + 6 bits (exponent) + 8 bits (significand)
        e_cross_b_re =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_SX_RE]  //S34 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_SX_RE]  //S35 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+5] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K14_SX_RE]  //S14 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+7] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K15_SX_RE]  //S15 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_SX_RE]  //S24 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_SX_RE]  //S25 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_SX_IM]  //S34 Im
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_SX_IM]  //S35 Im
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+6] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K14_SX_IM]  //S14 Im
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+8] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K15_SX_IM]  //S15 Im
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_SX_IM]  //S24 Im
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_SX_IM]; //S25 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        e_cross_b_im =  compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_SX_IM]  //S34 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_SX_IM]  //S35 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+5] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K14_SX_IM]  //S14 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+7] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K15_SX_IM]  //S15 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_SX_IM]  //S24 Re
                      + compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_SX_IM]  //S25 Re
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_SX_RE]  //S34 Im
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_SX_RE]  //S35 Im
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+6] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K14_SX_RE]  //S14 Im
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+8] *k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K15_SX_RE]  //S15 Im
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_SX_RE]  //S24 Im
                      - compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_SX_RE]; //S25 Im
#ifdef DEBUG_TCH
        printf("ReaSX       : %16.8e\n",e_cross_b_re);
#endif
        pt_uint8 = (uint8_t*) &e_cross_b_re;      // Affect an uint8_t pointer with the adress of e_cross_b_re
#ifdef LSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+7] = lfr_bp1[i*NB_BYTES_BP1+7] | (pt_uint8[3] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                                                                          // Record it at the 8th bit position (from the right to the left)
                                                                          // of lfr_bp1[i*NB_BYTES_BP1+7]
        pt_uint8[3] = (pt_uint8[3] & 0x7f);       // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
#ifdef MSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+7] = lfr_bp1[i*NB_BYTES_BP1+7] | (pt_uint8[0] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 1th octet:SPARC convention)
                                                                          // Record it at the 8th bit position (from the right to the left)
                                                                          // of lfr_bp1[i*NB_BYTES_BP1+7]
        pt_uint8[0] = (pt_uint8[0] & 0x7f);       // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
        significand = frexpf(e_cross_b_re, &exponent); // 0.5 <= significand < 1
                                                       // ReaSX = significand * 2^exponent
        if (exponent < expmin) { // value should be >= 0.5 * 2^expmin
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }
        if (exponent > expmax) { // value should be <  0.5 * 2^(expmax+1)
          exponent = expmax;
          significand = 1.0;     // max value that can be recorded
        }
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        lfr_bp1[i*NB_BYTES_BP1+8] = (uint8_t) ((significand*2-1)*255 + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                                                                             // where all bits are used (0, ..., 255)
        tmp_uint8 = (uint8_t) (exponent-expmin); // Shift and cast into a 8-bit uint8_t where
                                                 // just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
#ifdef DEBUG_TCH
        printf("|ReaSX|     : %16.8e\n",e_cross_b_re);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("tmp_uint8        for ReaSX exponent    : %d\n",tmp_uint8);
#endif
        lfr_bp1[i*NB_BYTES_BP1+7] = lfr_bp1[i*NB_BYTES_BP1+7] | tmp_uint8; // Record these nbitexp bits in the nbitexp first bits
                                                                           // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+7]
#ifdef DEBUG_TCH
        printf("lfr_bp1[i*NB_BYTES_BP1+7] for ReaSX sign + RealSX exponent : %u\n",lfr_bp1[i*NB_BYTES_BP1+7]);
        printf("lfr_bp1[i*NB_BYTES_BP1+8] for ReaSX significand            : %u\n",lfr_bp1[i*NB_BYTES_BP1+8]);
        printf("ImaSX       : %16.8e\n",e_cross_b_im);
#endif
        pt_uint8 = (uint8_t*) &e_cross_b_im; // Affect an uint8_t pointer with the adress of e_cross_b_im
#ifdef LSB_FIRST_TCH
        pt_uint8[3] = pt_uint8[3] & 0x7f;    // Make e_cross_b_im be positive in any case: |ImaSX| (32-bit float, sign bit in the 4th octet:PC convention)
#endif
#ifdef MSB_FIRST_TCH
        pt_uint8[0] = pt_uint8[0] & 0x7f;    // Make e_cross_b_im be positive in any case: |ImaSX| (32-bit float, sign bit in the 1th octet:SPARC convention)
#endif
        tmp_uint8 = (e_cross_b_im > e_cross_b_re) ? 0x40 : 0x00; // Determine the sector argument of SX. If |Im| > |Re| affect
                                                                 // an unsigned 8-bit char with 01000000; otherwise with null.
        lfr_bp1[i*NB_BYTES_BP1+7] = lfr_bp1[i*NB_BYTES_BP1+7] |  tmp_uint8; // Record it as a sign bit at the 7th bit position (from the right
                                                                            // to the left) of lfr_bp1[i*NB_BYTES_BP1+7], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|ImaSX|     : %16.8e\n",e_cross_b_im);
        printf("ArgSX sign  : %u\n",tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+7] for ReaSX & ArgSX signs + ReaSX exponent  : %u\n",lfr_bp1[i*NB_BYTES_BP1+7]);
#endif
        //======================================================================
        // BP1 phase velocity estimator == PA_LFR_SC_BP1_VPHI_F0 == 16 bits
        //                                          = 1 sign bit + 1 argument bit (two sectors)
        //                                          + 6 bits (exponent) + 8 bits (significand)
        ny = sin(alpha_M)*NVEC_V1 + cos(alpha_M)*NVEC_V2;
        nz = NVEC_V0;
        bx_bx_star = cos(alpha_M)*cos(alpha_M)*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]   // S22 Re
                   + sin(alpha_M)*sin(alpha_M)*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]  // S33 Re
                 - 2*sin(alpha_M)*cos(alpha_M)*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+10]; // S23 Re

        n_cross_e_scal_b_re = ny * (compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NY_RE]  //S24 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NY_RE]  //S25 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NY_RE]  //S34 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NY_RE]  //S35 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NY_IM]  //S24 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NY_IM]  //S25 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NY_IM]  //S34 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NY_IM]) //S35 Im
                            + nz * (compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NZ_RE]  //S24 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NZ_RE]  //S25 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NZ_RE]  //S34 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NZ_RE]  //S35 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NZ_IM]  //S24 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NZ_IM]  //S25 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NZ_IM]  //S34 Im
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NZ_IM]);//S35 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        n_cross_e_scal_b_im = ny * (compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NY_IM]  //S24 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NY_IM]  //S25 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NY_IM]  //S34 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NY_IM]  //S35 Re
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NY_RE]  //S24 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NY_RE]  //S25 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NY_RE]  //S34 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NY_RE]) //S35 Im
                            + nz * (compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NZ_IM]  //S24 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NZ_IM]  //S25 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NZ_IM]  //S34 Re
                                   +compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NZ_IM]  //S35 Re
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K24_NZ_RE]  //S24 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K25_NZ_RE]  //S25 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K34_NZ_RE]  //S34 Im
                                   -compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]*k_coeff_intercalib[i*NB_K_COEFF_PER_BIN+K35_NZ_RE]);//S35 Im
#ifdef DEBUG_TCH
        printf("n_cross_e_scal_b_re   : %16.8e\n",n_cross_e_scal_b_re);
        printf("n_cross_e_scal_b_im   : %16.8e\n",n_cross_e_scal_b_im);
#endif
        // vphi = n_cross_e_scal_b_re / bx_bx_star => sign(VPHI) = sign(n_cross_e_scal_b_re)
        pt_uint8 = (uint8_t*) &n_cross_e_scal_b_re; // Affect an uint8_t pointer with the adress of n_cross_e_scal_b_re
#ifdef LSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+9] = lfr_bp1[i*NB_BYTES_BP1+9] | (pt_uint8[3] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of lfr_bp1[i*NB_BYTES_BP1+9]
        pt_uint8[3] = (pt_uint8[3] & 0x7f);     // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
#ifdef MSB_FIRST_TCH
        lfr_bp1[i*NB_BYTES_BP1+9] = lfr_bp1[i*NB_BYTES_BP1+9] | (pt_uint8[0] & 0x80);  // Extract its sign bit (32-bit float, sign bit in the 1th octet:SPARC convention)
                                                                        // Record it at the 8th bit position (from the right to the left)
                                                                        // of lfr_bp1[i*NB_BYTES_BP1+9]
        pt_uint8[0] = (pt_uint8[0] & 0x7f);     // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
        if (bx_bx_star != 0.) { // no division by 0.
            vphi = n_cross_e_scal_b_re / bx_bx_star;  // Compute |VPHI|
        }
        else
        {
            vphi = 1.e+20;                         // Put a huge value
        }
        significand = frexpf(vphi, &exponent);  // 0.5 <= significand < 1
                                                // vphi = significand * 2^exponent
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

        lfr_bp1[i*NB_BYTES_BP1+10] = (uint8_t) ((significand*2-1)*255 + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                                                                           // where all the bits are used (0, ..., 255)
        tmp_uint8 = (uint8_t) (exponent-expmin); // Shift and cast into a 8-bit uint8_t where
                                                 // just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
#ifdef DEBUG_TCH
        printf("|VPHI|      : %16.8e\n",vphi);
        printf("significand : %16.8e\n",significand);
        printf("exponent    : %d\n"    ,exponent);
        printf("tmp_uint8        for VPHI exponent    : %d\n",tmp_uint8);
#endif
        lfr_bp1[i*NB_BYTES_BP1+9] = lfr_bp1[i*NB_BYTES_BP1+9] | tmp_uint8; // Record these nbitexp bits in the nbitexp first bits
                                                                           // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+9]
#ifdef DEBUG_TCH
        printf("lfr_bp1[i*NB_BYTES_BP1+9]  for VPHI sign + VPHI exponent : %u\n",lfr_bp1[i*NB_BYTES_BP1+9]);
        printf("lfr_bp1[i*NB_BYTES_BP1+10] for VPHI significand          : %u\n",lfr_bp1[i*NB_BYTES_BP1+10]);
#endif
        pt_uint8 = (uint8_t*) &n_cross_e_scal_b_im; // Affect an uint8_t pointer with the adress of n_cross_e_scal_b_im
#ifdef LSB_FIRST_TCH
        pt_uint8[3] = pt_uint8[3] & 0x7f;           // Make n_cross_e_scal_b_im be positive in any case: |ImaNEBX| (32-bit float, sign bit in the 4th octet:PC convention)
#endif
#ifdef MSB_FIRST_TCH
        pt_uint8[0] = pt_uint8[0] & 0x7f;           // Make n_cross_e_scal_b_im be positive in any case: |ImaNEBX| (32-bit float, sign bit in the 1th octet:SPARC convention)
#endif
        tmp_uint8 = (n_cross_e_scal_b_im > n_cross_e_scal_b_re) ? 0x40 : 0x00; // Determine the sector argument of NEBX. If |Im| > |Re| affect
                                                                               // an unsigned 8-bit char with 01000000; otherwise with null.
        lfr_bp1[i*NB_BYTES_BP1+9] = lfr_bp1[i*NB_BYTES_BP1+9] |  tmp_uint8;    // Record it as a sign bit at the 7th bit position (from the right
                                                                               // to the left) of lfr_bp1[i*NB_BYTES_BP1+9], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|n_cross_e_scal_b_im|             : %16.8e\n",n_cross_e_scal_b_im);
        printf("|n_cross_e_scal_b_im|/bx_bx_star  : %16.8e\n",n_cross_e_scal_b_im/bx_bx_star);
        printf("ArgNEBX sign                      : %u\n",tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+9] for VPHI & ArgNEBX signs + VPHI exponent : %u\n",lfr_bp1[i*NB_BYTES_BP1+9]);
#endif
    }
}

void BP2_set( float * compressed_spec_mat, uint8_t nb_bins_compressed_spec_mat, uint8_t * lfr_bp2 )
{
    float cross_re;                     // 32-bit floating point
    float cross_im;
    float aux;
    float significand;
    int exponent;                       // 32-bit signed integer
    uint8_t nbitexp;                    // 8-bit unsigned integer
    uint8_t nbitsig;
    uint8_t *pt_uint8;                  // pointer on unsigned 8-bit integer
    int8_t expmin;                      // 8-bit signed integer
    int8_t expmax;
    uint16_t rangesig;                  // 16-bit unsigned integer
    uint16_t autocor;
    uint16_t exp;
    uint16_t tmp_uint16;
    uint16_t i;

#ifdef DEBUG_TCH
    printf("BP2 : \n");
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
#endif

    // For floating point data to be recorded on 16-bit words :
    nbitexp = 6;                  // number of bits for the exponent
    nbitsig = 16 - nbitexp;       // number of bits for the significand
    rangesig = (1 << nbitsig)-1;  // == 2^nbitsig - 1
    expmax = 32 + 5;
    expmin = expmax - (1 << nbitexp) + 1;

#ifdef DEBUG_TCH

    printf("nbitexp : %d, expmax : %d, expmin : %d\n", nbitexp, expmax, expmin);
    printf("nbitsig : %d, rangesig : %d\n", nbitsig, rangesig);
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
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+1] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+2] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+10] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+20] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("\nBin number: %d\n", i);
        printf("lfr_bp2[i*NB_BYTES_BP2+10] for cross12_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+10]);
        printf("lfr_bp2[i*NB_BYTES_BP2+20] for cross12_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+20]);
#endif
        // S13
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+3] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+4] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+11] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+21] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+11] for cross13_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+11]);
        printf("lfr_bp2[i*NB_BYTES_BP2+21] for cross13_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+21]);
#endif
        // S14
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+5] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+6] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+12] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+22] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+12] for cross14_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+12]);
        printf("lfr_bp2[i*NB_BYTES_BP2+22] for cross14_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+22]);
#endif
        // S15
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+7] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+8] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+13] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+23] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+13] for cross15_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+13]);
        printf("lfr_bp2[i*NB_BYTES_BP2+23] for cross15_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+23]);
#endif
        // S23
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+10] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+11] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+14] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+24] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+14] for cross23_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+14]);
        printf("lfr_bp2[i*NB_BYTES_BP2+24] for cross23_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+24]);
#endif
        // S24
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+12] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+13] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+15] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+25] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+15] for cross24_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+15]);
        printf("lfr_bp2[i*NB_BYTES_BP2+25] for cross24_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+25]);
#endif
        // S25
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+14] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+15] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+16] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+26] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+16] for cross25_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+16]);
        printf("lfr_bp2[i*NB_BYTES_BP2+26] for cross25_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+26]);
#endif
        // S34
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+17] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+18] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+17] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+27] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+17] for cross34_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+17]);
        printf("lfr_bp2[i*NB_BYTES_BP2+27] for cross34_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+27]);
#endif
        // S35
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+19] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+20] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+18] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+28] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+18] for cross35_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+18]);
        printf("lfr_bp2[i*NB_BYTES_BP2+28] for cross35_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+28]);
#endif
        // S45
        aux = sqrt(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21]*compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24]);
        if (aux != 0.) { // no division by 0.
        cross_re = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+22] / aux;
        cross_im = compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+23] / aux;
        }
        else
        {
        cross_re = 0.;
        cross_im = 0.;
        }
        lfr_bp2[i*NB_BYTES_BP2+19] = (uint8_t) (cross_re*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[i*NB_BYTES_BP2+29] = (uint8_t) (cross_im*127.5 + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+19] for cross45_re (%16.8e) : %.3u\n",cross_re, lfr_bp2[i*NB_BYTES_BP2+19]);
        printf("lfr_bp2[i*NB_BYTES_BP2+29] for cross45_im (%16.8e) : %.3u\n",cross_im, lfr_bp2[i*NB_BYTES_BP2+29]);
#endif
        //==============================================
        // BP2  auto correlations == PA_LFR_SC_BP2_AUTO_F0 == 5*16 bits = 5*[6 bits (exponent) + 10 bits (significand)]
                               // == PA_LFR_SC_BP2_AUTO_A0_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A1_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A2_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A3_F0 == 16 bits
                               // == PA_LFR_SC_BP2_AUTO_A4_F0 == 16 bits
        // S11
        significand = frexpf(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX], &exponent);  // 0.5 <= significand < 1
                                                                                                // S11 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S11         : %16.8e\n",compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX]);
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
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        autocor = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                 // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+0] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+1] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+0] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+1] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S11 significand : %u\n",autocor);
        printf("exp for S11 exponent : %u\n",exp);
        printf("pt_uint8[1] for S11 exponent + significand : %.3d or %2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for S11            significand : %.3d or %2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+0] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+0], lfr_bp2[i*NB_BYTES_BP2+0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+1] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+1], lfr_bp2[i*NB_BYTES_BP2+1]);
#endif
        // S22
        significand = frexpf(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9], &exponent);  // 0.5 <= significand < 1
                                                                                                  // S22 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S22         : %16.8e\n",compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+9]);
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
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        autocor = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                 // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+2] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+3] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+2] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+3] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S22 significand : %u\n",autocor);
        printf("exp for S11 exponent : %u\n",exp);
        printf("pt_uint8[1] for S22 exponent + significand : %.3d or %2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for S22            significand : %.3d or %2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+2] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+2], lfr_bp2[i*NB_BYTES_BP2+2]);
        printf("lfr_bp2[i*NB_BYTES_BP2+3] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+3], lfr_bp2[i*NB_BYTES_BP2+3]);
#endif
        // S33
        significand = frexpf(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16], &exponent);  // 0.5 <= significand < 1
                                                                                                   // S33 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S33         : %16.8e\n",compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+16]);
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
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        autocor = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                 // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+4] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+5] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+4] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+5] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S33 significand : %u\n",autocor);
        printf("exp for S33 exponent : %u\n",exp);
        printf("pt_uint8[1] for S33 exponent + significand : %.3d or %2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for S33            significand : %.3d or %2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+4] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+4], lfr_bp2[i*NB_BYTES_BP2+4]);
        printf("lfr_bp2[i*NB_BYTES_BP2+5] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+5], lfr_bp2[i*NB_BYTES_BP2+5]);
#endif
        // S44
        significand = frexpf(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21], &exponent);  // 0.5 <= significand < 1
                                                                                                   // S44 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S44         : %16.8e\n",compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+21]);
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
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        autocor = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                 // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+6] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+7] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+6] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+7] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S44 significand : %u\n",autocor);
        printf("exp for S44 exponent : %u\n",exp);
        printf("pt_uint8[1] for S44 exponent + significand : %.3d or %2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for S44            significand : %.3d or %2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+6] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+6], lfr_bp2[i*NB_BYTES_BP2+6]);
        printf("lfr_bp2[i*NB_BYTES_BP2+7] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+7], lfr_bp2[i*NB_BYTES_BP2+7]);
#endif
        // S55
        significand = frexpf(compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24], &exponent);  // 0.5 <= significand < 1
                                                                                                   // S55 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S55         : %16.8e\n",compressed_spec_mat[i*NB_VALUES_PER_SPECTRAL_MATRIX+24]);
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
        if (significand == 0) {  // in that case exponent == 0 too
          exponent = expmin;
          significand = 0.5;     // min value that can be recorded
        }

        autocor = (uint16_t) ((significand*2-1)*rangesig + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                                                                 // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t) (exponent-expmin);      // Shift and cast into a 16-bit unsigned int where just
                                                 // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*) &tmp_uint16;       // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+8] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+9] = pt_uint8[1]; // Record LSB of tmp_uint16
        //printf("MSB:\n");
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[i*NB_BYTES_BP2+8] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[i*NB_BYTES_BP2+9] = pt_uint8[0]; // Record LSB of tmp_uint16
        //printf("LSB:\n");
#endif
#ifdef DEBUG_TCH
        printf("autocor for S55 significand : %u\n",autocor);
        printf("exp for S55 exponent : %u\n",exp);
        printf("pt_uint8[1] for S55 exponent + significand : %.3d or %2x\n",pt_uint8[1], pt_uint8[1]);
        printf("pt_uint8[0] for S55            significand : %.3d or %2x\n",pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+8] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+8], lfr_bp2[i*NB_BYTES_BP2+8]);
        printf("lfr_bp2[i*NB_BYTES_BP2+9] : %3u or %2x\n",lfr_bp2[i*NB_BYTES_BP2+9], lfr_bp2[i*NB_BYTES_BP2+9]);
#endif
    }
}


#endif // BASIC_PARAMETERS_H_INCLUDED

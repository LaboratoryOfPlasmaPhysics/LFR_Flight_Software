// clang-format off
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
// version 2.3: 01/07/2015 (affectation initiale des octets 7 et 9 dans les BP1 corrigée ...)
// version 2.4: 05/10/2018 (mise en conformité LOGISCOPE)
// version 2.5: 09/10/2018 (dans main.c #include "basic_parameters_utilities.h" est changé par les déclarations extern correspondantes ...!
//                          + delta mise en conformité LOGISCOPE)
// clang-format on
/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Thomas Chust
--                   Contact : Thomas Chust
--                      Mail : thomas.chust@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

#ifndef BASIC_PARAMETERS_H_INCLUDED
#define BASIC_PARAMETERS_H_INCLUDED

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "basic_parameters_params.h"


#ifndef LFR_BIG_ENDIAN
#ifndef LFR_LITTLE_ENDIAN
#error "you must either define LFR_BIG_ENDIAN or LFR_LITTLE_ENDIAN"
#endif
#endif

#ifdef LFR_BIG_ENDIAN
#ifdef LFR_LITTLE_ENDIAN
#error "you must either define LFR_BIG_ENDIAN or LFR_LITTLE_ENDIAN but not both"
#endif
#endif

static inline void BP1_set(float* compressed_spec_mat, float* k_coeff_intercalib,
    unsigned char nb_bins_compressed_spec_mat, unsigned char* lfr_bp1);
static inline void BP2_set(
    float* compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char* lfr_bp2);

void init_k_coefficients_f0(float* k_coeff_intercalib, unsigned char nb_binscompressed_matrix);
void init_k_coefficients_f1(float* k_coeff_intercalib, unsigned char nb_binscompressed_matrix);
void init_k_coefficients_f2(float* k_coeff_intercalib, unsigned char nb_binscompressed_matrix);

void init_k_coefficients(float* k_coeff_intercalib, unsigned char nb_binscompressed_matrix);

//***********************************
// STATIC INLINE FUNCTION DEFINITIONS

void BP1_set(float* compressed_spec_mat, float* k_coeff_intercalib,
    uint8_t nb_bins_compressed_spec_mat, uint8_t* lfr_bp1);

void compute_BP1(const float* const spectral_matrices, const uint8_t spectral_matrices_count,
    uint8_t* bp1_buffer);

void BP2_set(float* compressed_spec_mat, uint8_t nb_bins_compressed_spec_mat, uint8_t* lfr_bp2)
{
    float cross_re; // 32-bit floating point
    float cross_im;
    float aux;
    float significand;
    int exponent; // 32-bit signed integer
    uint8_t nbitexp; // 8-bit unsigned integer
    uint8_t nbitsig;
    uint8_t* pt_uint8; // pointer on unsigned 8-bit integer
    int8_t expmin; // 8-bit signed integer
    int8_t expmax;
    uint16_t rangesig; // 16-bit unsigned integer
    uint16_t autocor;
    uint16_t exp;
    uint16_t tmp_uint16;
    uint16_t i;

#ifdef DEBUG_TCH
    printf("BP2 : \n");
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
#endif

    // For floating point data to be recorded on 16-bit words :
    nbitexp = 6; // number of bits for the exponent
    nbitsig = 16 - nbitexp; // number of bits for the significand
    rangesig = (1 << nbitsig) - 1; // == 2^nbitsig - 1
    expmax = 32 + 5;
    expmin = (expmax - (1 << nbitexp)) + 1;

#ifdef DEBUG_TCH

    printf("nbitexp : %d, expmax : %d, expmin : %d\n", nbitexp, expmax, expmin);
    printf("nbitsig : %d, rangesig : %d\n", nbitsig, rangesig);
#endif

    for (i = 0; i < nb_bins_compressed_spec_mat; i++)
    {
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
        aux = sqrtf(compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 1] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 10] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 20] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("\nBin number: %d\n", i);
        printf("lfr_bp2[i*NB_BYTES_BP2+10] for cross12_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 10]);
        printf("lfr_bp2[i*NB_BYTES_BP2+20] for cross12_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 20]);
#endif
        // S13
        aux = sqrtf(compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 3] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 11] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 21] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+11] for cross13_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 11]);
        printf("lfr_bp2[i*NB_BYTES_BP2+21] for cross13_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 21]);
#endif
        // S14
        aux = sqrtf(compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 5] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 6] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 12] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 22] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+12] for cross14_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 12]);
        printf("lfr_bp2[i*NB_BYTES_BP2+22] for cross14_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 22]);
#endif
        // S15
        aux = sqrtf(compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 7] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 8] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 13] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 23] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+13] for cross15_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 13]);
        printf("lfr_bp2[i*NB_BYTES_BP2+23] for cross15_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 23]);
#endif
        // S23
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 10] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 14] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 24] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+14] for cross23_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 14]);
        printf("lfr_bp2[i*NB_BYTES_BP2+24] for cross23_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 24]);
#endif
        // S24
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 15] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 25] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+15] for cross24_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 15]);
        printf("lfr_bp2[i*NB_BYTES_BP2+25] for cross24_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 25]);
#endif
        // S25
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 16] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 26] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+16] for cross25_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 16]);
        printf("lfr_bp2[i*NB_BYTES_BP2+26] for cross25_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 26]);
#endif
        // S34
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 17] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 27] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+17] for cross34_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 17]);
        printf("lfr_bp2[i*NB_BYTES_BP2+27] for cross34_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 27]);
#endif
        // S35
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 18] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 28] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+18] for cross35_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 18]);
        printf("lfr_bp2[i*NB_BYTES_BP2+28] for cross35_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 28]);
#endif
        // S45
        aux = sqrtf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21]
            * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24]);
        if (aux != 0.)
        { // no division by 0.
            cross_re = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 22] / aux;
            cross_im = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 23] / aux;
        }
        else
        {
            cross_re = 0.;
            cross_im = 0.;
        }
        lfr_bp2[(i * NB_BYTES_BP2) + 19] = (uint8_t)((cross_re * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp2[(i * NB_BYTES_BP2) + 29] = (uint8_t)((cross_im * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
#ifdef DEBUG_TCH
        printf("lfr_bp2[i*NB_BYTES_BP2+19] for cross45_re (%16.8e) : %.3u\n", cross_re,
            lfr_bp2[i * NB_BYTES_BP2 + 19]);
        printf("lfr_bp2[i*NB_BYTES_BP2+29] for cross45_im (%16.8e) : %.3u\n", cross_im,
            lfr_bp2[i * NB_BYTES_BP2 + 29]);
#endif
        //==============================================
        // BP2  auto correlations == PA_LFR_SC_BP2_AUTO_F0 == 5*16 bits = 5*[6 bits (exponent) + 10
        // bits (significand)]
        // == PA_LFR_SC_BP2_AUTO_A0_F0 == 16 bits
        // == PA_LFR_SC_BP2_AUTO_A1_F0 == 16 bits
        // == PA_LFR_SC_BP2_AUTO_A2_F0 == 16 bits
        // == PA_LFR_SC_BP2_AUTO_A3_F0 == 16 bits
        // == PA_LFR_SC_BP2_AUTO_A4_F0 == 16 bits
        // S11
        significand = frexpf(compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX],
            &exponent); // 0.5 <= significand < 1
                        // S11 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf("S11         : %16.8e\n", compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
#endif
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        autocor = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 0] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 1] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 0] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 1] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S11 significand : %u\n", autocor);
        printf("exp for S11 exponent : %u\n", exp);
        printf(
            "pt_uint8[1] for S11 exponent + significand : %.3d or %2x\n", pt_uint8[1], pt_uint8[1]);
        printf(
            "pt_uint8[0] for S11            significand : %.3d or %2x\n", pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+0] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 0],
            lfr_bp2[i * NB_BYTES_BP2 + 0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+1] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 1],
            lfr_bp2[i * NB_BYTES_BP2 + 1]);
#endif
        // S22
        significand = frexpf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9],
            &exponent); // 0.5 <= significand < 1
                        // S22 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf(
            "S22         : %16.8e\n", compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX + 9]);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
#endif
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        autocor = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 2] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 3] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 2] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 3] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S22 significand : %u\n", autocor);
        printf("exp for S11 exponent : %u\n", exp);
        printf(
            "pt_uint8[1] for S22 exponent + significand : %.3d or %2x\n", pt_uint8[1], pt_uint8[1]);
        printf(
            "pt_uint8[0] for S22            significand : %.3d or %2x\n", pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+2] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 2],
            lfr_bp2[i * NB_BYTES_BP2 + 2]);
        printf("lfr_bp2[i*NB_BYTES_BP2+3] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 3],
            lfr_bp2[i * NB_BYTES_BP2 + 3]);
#endif
        // S33
        significand = frexpf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16],
            &exponent); // 0.5 <= significand < 1
                        // S33 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf(
            "S33         : %16.8e\n", compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX + 16]);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
#endif
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        autocor = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 4] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 5] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 4] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 5] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S33 significand : %u\n", autocor);
        printf("exp for S33 exponent : %u\n", exp);
        printf(
            "pt_uint8[1] for S33 exponent + significand : %.3d or %2x\n", pt_uint8[1], pt_uint8[1]);
        printf(
            "pt_uint8[0] for S33            significand : %.3d or %2x\n", pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+4] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 4],
            lfr_bp2[i * NB_BYTES_BP2 + 4]);
        printf("lfr_bp2[i*NB_BYTES_BP2+5] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 5],
            lfr_bp2[i * NB_BYTES_BP2 + 5]);
#endif
        // S44
        significand = frexpf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21],
            &exponent); // 0.5 <= significand < 1
                        // S44 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf(
            "S44         : %16.8e\n", compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX + 21]);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
#endif

        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        autocor = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 6] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 7] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 6] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 7] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("autocor for S44 significand : %u\n", autocor);
        printf("exp for S44 exponent : %u\n", exp);
        printf(
            "pt_uint8[1] for S44 exponent + significand : %.3d or %2x\n", pt_uint8[1], pt_uint8[1]);
        printf(
            "pt_uint8[0] for S44            significand : %.3d or %2x\n", pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+6] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 6],
            lfr_bp2[i * NB_BYTES_BP2 + 6]);
        printf("lfr_bp2[i*NB_BYTES_BP2+7] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 7],
            lfr_bp2[i * NB_BYTES_BP2 + 7]);
#endif
        // S55
        significand = frexpf(compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24],
            &exponent); // 0.5 <= significand < 1
                        // S55 = significand * 2^exponent
#ifdef DEBUG_TCH
        printf(
            "S55         : %16.8e\n", compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX + 24]);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
#endif
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        autocor = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = autocor | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                                 // left place of the significand bits (nbitsig),
                                                 // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef MSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 8] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 9] = pt_uint8[1]; // Record LSB of tmp_uint16
        // printf("MSB:\n");
#endif
#ifdef LSB_FIRST_TCH
        lfr_bp2[(i * NB_BYTES_BP2) + 8] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp2[(i * NB_BYTES_BP2) + 9] = pt_uint8[0]; // Record LSB of tmp_uint16
        // printf("LSB:\n");
#endif
#ifdef DEBUG_TCH
        printf("autocor for S55 significand : %u\n", autocor);
        printf("exp for S55 exponent : %u\n", exp);
        printf(
            "pt_uint8[1] for S55 exponent + significand : %.3d or %2x\n", pt_uint8[1], pt_uint8[1]);
        printf(
            "pt_uint8[0] for S55            significand : %.3d or %2x\n", pt_uint8[0], pt_uint8[0]);
        printf("lfr_bp2[i*NB_BYTES_BP2+8] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 8],
            lfr_bp2[i * NB_BYTES_BP2 + 8]);
        printf("lfr_bp2[i*NB_BYTES_BP2+9] : %3u or %2x\n", lfr_bp2[i * NB_BYTES_BP2 + 9],
            lfr_bp2[i * NB_BYTES_BP2 + 9]);
#endif
    }
}


#endif // BASIC_PARAMETERS_H_INCLUDED

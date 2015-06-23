// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB + indices [0,2] <=> [1,3])
// version 1.8: 02/02/2015 (gestion des divisions par zéro)
// In the frame of RPW LFR Sofware ICD Issue3 Rev6 (27/01/2015) => R3 FSW
// version 2.0: 19/06/2015
// version 2.1: 22/06/2015 (modifs de Paul)


#ifndef BASIC_PARAMETERS_UTILITIES_H
#define BASIC_PARAMETERS_UTILITIES_H

#include <stdio.h>
#include <malloc.h>

#include "basic_parameters_params.h"

float compressed_spectral_matrix_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX];
float k_coefficients_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_K_COEFF_PER_BIN];
float k_coefficients_f1[NB_BINS_COMPRESSED_MATRIX_f1 * NB_K_COEFF_PER_BIN];
float k_coefficients_f2[NB_BINS_COMPRESSED_MATRIX_f2 * NB_K_COEFF_PER_BIN];

unsigned char LFR_BP1_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP1];
unsigned char LFR_BP2_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP2];

#endif // BASIC_PARAMETERS_UTILITIES_H

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

#include <stdio.h>
#include <stdint.h>

#include "basic_parameters_params.h"

void init_k_coefficients_f0(float *k_coefficients,
                            unsigned char nb_binscompressed_matrix )
{

    uint8_t i; // 8 bits unsigned
    for(i=0; i<nb_binscompressed_matrix; i++){
        k_coefficients[i*NB_K_COEFF_PER_BIN+K44_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K55_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_IM] = 1;
    }

}


void init_k_coefficients_f1(float *k_coefficients,
                            unsigned char nb_binscompressed_matrix )
{

    uint8_t i; // 8 bits unsigned
    for(i=0; i<nb_binscompressed_matrix; i++){
        k_coefficients[i*NB_K_COEFF_PER_BIN+K44_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K55_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_IM] = 1;
    }

}


void init_k_coefficients_f2(float *k_coefficients,
                            unsigned char nb_binscompressed_matrix )
{

    uint8_t i; // 8 bits unsigned
    for(i=0; i<nb_binscompressed_matrix; i++){
        k_coefficients[i*NB_K_COEFF_PER_BIN+K44_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K55_PE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K45_PE_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K14_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K15_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_SX_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NY_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K24_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K25_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K34_NZ_IM] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_RE] = 1;
        k_coefficients[i*NB_K_COEFF_PER_BIN+K35_NZ_IM] = 1;
    }

}


void init_k_coefficients(float *k_coefficients,
                         unsigned char nb_binscompressed_matrix )
{
    switch (nb_binscompressed_matrix)
    {
    case NB_BINS_COMPRESSED_MATRIX_f0:
#ifdef DEBUG_TCH
    printf("F0 data: initialization of the intercalibration k-coefficients\n");
#endif
    init_k_coefficients_f0(k_coefficients, nb_binscompressed_matrix);
    break;

    case NB_BINS_COMPRESSED_MATRIX_f1:
#ifdef DEBUG_TCH
    printf("F1 data: initialization of the intercalibration k-coefficients\n");
#endif
    init_k_coefficients_f1(k_coefficients, nb_binscompressed_matrix);
    break;

    case NB_BINS_COMPRESSED_MATRIX_f2:
#ifdef DEBUG_TCH
    printf("F2 data: initialization of the intercalibration k-coefficients\n");
#endif
    init_k_coefficients_f2(k_coefficients, nb_binscompressed_matrix);
    break;

    default:
#ifdef DEBUG_TCH
    printf("there is a problème !!?\n");
#endif
    break;
    }
}


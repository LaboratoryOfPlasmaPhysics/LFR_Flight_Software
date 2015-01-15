// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014
// version 1.4: 16/05/2014
// version 1.5: 20/05/2014
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB + indices [0,2] <=> [1,3])


#include <stdint.h>
#include "basic_parameters_params.h"

void init_k_coefficients(float *k_coefficients,
                         unsigned char nb_binscompressed_matrix )
{
    uint16_t i; // 16 bits unsigned
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

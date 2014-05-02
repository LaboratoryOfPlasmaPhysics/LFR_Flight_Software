// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014

#include "basic_parameters.h"

void init_k_f0( void )
{
    uint16_t i;                        // 16 bits unsigned

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


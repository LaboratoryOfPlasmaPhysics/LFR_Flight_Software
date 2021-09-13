#pragma once
#include "fsw_params_processing.h"

extern float
    mag_calibration_matrices_f0[NB_BINS_PER_SM * NB_ELEMENTS_MAG_CAL_MATRIX * FLOATS_PER_COMPLEX];
extern float
    mag_calibration_matrices_f1[NB_BINS_PER_SM * NB_ELEMENTS_MAG_CAL_MATRIX * FLOATS_PER_COMPLEX];
extern float
    mag_calibration_matrices_f2[NB_BINS_PER_SM * NB_ELEMENTS_MAG_CAL_MATRIX * FLOATS_PER_COMPLEX];

extern float
    elec_calibration_matrices_f0[NB_BINS_PER_SM * NB_ELEMENTS_ELEC_CAL_MATRIX * FLOATS_PER_COMPLEX];
extern float
    elec_calibration_matrices_f1[NB_BINS_PER_SM * NB_ELEMENTS_ELEC_CAL_MATRIX * FLOATS_PER_COMPLEX];
extern float
    elec_calibration_matrices_f2[NB_BINS_PER_SM * NB_ELEMENTS_ELEC_CAL_MATRIX * FLOATS_PER_COMPLEX];

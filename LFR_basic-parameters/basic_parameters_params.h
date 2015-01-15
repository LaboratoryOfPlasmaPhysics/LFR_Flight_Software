// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.4: 16/05/2014
// version 1.5: 20/05/2014
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB + indices [0,2] <=> [1,3])


#ifndef BASIC_PARAMETERS_PARAMS_H
#define BASIC_PARAMETERS_PARAMS_H

#define NB_VALUES_PER_SPECTRAL_MATRIX 25

#define NB_BYTES_BP1 9
#define NB_BYTES_BP2 30

//********************************************
// K-COEFFICIENTS FOR ONBOARD INTERCALIBRATION

#define NB_K_COEFF_PER_BIN 32

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

#endif // BASIC_PARAMETERS_PARAMS_H

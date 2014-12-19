// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014

#ifndef BASIC_PARAMETERS_H_INCLUDED
#define BASIC_PARAMETERS_H_INCLUDED

#define NB_VALUES_PER_SPECTRAL_MATRIX 25
#define NB_BINS_COMPRESSED_MATRIX_f0 11

#define NB_BYTES_BP1 9
#define NB_BYTES_BP2 30

void init_k_f0( void );
void BP1_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * lfr_bp1);
void BP2_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * lfr_bp2);

#endif // BASIC_PARAMETERS_H_INCLUDED

// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1: 31/07/2013

#ifndef BASIC_PARAMETERS_H_INCLUDED
#define BASIC_PARAMETERS_H_INCLUDED

#define NB_BINS_COMPRESSED_MATRIX_f0 1

void init_k_f0( void );
void BP1_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1);
void BP2_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP2);

#endif // BASIC_PARAMETERS_H_INCLUDED

// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014

#ifndef FILE_UTILITIES_H
#define FILE_UTILITIES_H

#include <stdio.h>
#include <malloc.h>
#include <basic_parameters.h>

extern float compressed_spectral_matrix_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX];

int lecture_file_sm(const char *fileName);

#endif // FILE_UTILITIES_H

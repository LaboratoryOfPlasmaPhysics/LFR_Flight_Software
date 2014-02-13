// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1: 31/07/2013

#ifndef FILE_UTILITIES_H
#define FILE_UTILITIES_H

#include <stdio.h>
#include <malloc.h>
#include <basic_parameters.h>

extern float compressed_spectral_matrix_f0[TOTAL_SIZE_COMPRESSED_MATRIX_f0];

int lecture_file_sm(const char *fileName);

#endif // FILE_UTILITIES_H

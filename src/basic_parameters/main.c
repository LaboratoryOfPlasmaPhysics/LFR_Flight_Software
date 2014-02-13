// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1: 31/07/2013

#include <stdio.h>
#include <basic_parameters.h>
#include <file_utilities.h>

float compressed_spectral_matrix_f0[TOTAL_SIZE_COMPRESSED_MATRIX_f0];

unsigned char LFR_BP1_F0[NB_BINS_COMPRESSED_MATRIX_f0*9];
unsigned char LFR_BP2_F0[NB_BINS_COMPRESSED_MATRIX_f0*30];

int main(void)
{
    printf("Hello World!\n");

    lecture_file_sm("sm_test1.dat");

    BP1_set();

    BP2_set();

    return 0;
}



















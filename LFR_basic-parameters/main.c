// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.O: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014

#include <stdio.h>
#include <basic_parameters.h>
#include <file_utilities.h>

float compressed_spectral_matrix_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX];

unsigned char LFR_BP1_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP1];
unsigned char LFR_BP2_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP2];

int main(void)
{
    const char *filename;
    printf("Hello World!\n\n");

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    //LSB FIRST
    printf("The multi-byte quantities are laid out in a LSB FIRST (little endian) fashion \n\n");
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    //MSB FIRST
    printf("The multi-byte quantities are laid out in a MSB FIRST (big endian) fashion\n\n");
#endif

    //filename="/WIN/Users/chust/DD CHUST/Missions/Solar Orbiter/LFR/Prog C/tests bp Paul/tests2/sm_test2.dat";
    filename="sm_test2.dat";

    lecture_file_sm(filename);

    printf("\n");

    BP1_set(compressed_spectral_matrix_f0, NB_BINS_COMPRESSED_MATRIX_f0, LFR_BP1_f0);

    printf("\n");

    BP2_set(compressed_spectral_matrix_f0, NB_BINS_COMPRESSED_MATRIX_f0, LFR_BP2_f0);

    return 0;
}



















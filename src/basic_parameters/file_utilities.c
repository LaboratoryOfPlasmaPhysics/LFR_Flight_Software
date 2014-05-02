// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013)
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014

#include <file_utilities.h>

int lecture_file_sm(const char *fileName)
{
    int i;

    FILE *infile;
    infile = fopen(fileName, "rb");  // open explicitely a binary file !!! ...
    if(infile == NULL) {
        printf("Hello I cannot open the file! \n");
        return 0;
    }
    (void) fread(compressed_spectral_matrix_f0, sizeof(compressed_spectral_matrix_f0), 1, infile);
    (void) fclose(infile);
    printf("size of compressed_spectral_matrix_f0 : %lu\n", sizeof(compressed_spectral_matrix_f0));
    printf("Number of bins: %d\n\n", NB_BINS_COMPRESSED_MATRIX_f0);

    printf("compressed_spectral_matrix_f0 : \n");

    for(i=0; i<NB_BINS_COMPRESSED_MATRIX_f0; i++){

      printf("Bin number: %d\n", i);

      printf("Element %.2d (S11) (%.2d & --) => Re:%16.8e  Im:%16.8e\n",   1, 0,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+0], 0.);
      printf("Element %.2d (S12) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 2, 1, 2,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+1], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+2]);
      printf("Element %.2d (S13) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 3, 3, 4,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+3], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+4]);
      printf("Element %.2d (S14) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 4, 5, 6,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+5], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+6]);
      printf("Element %.2d (S15) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 5, 7, 8,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+7], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+8]);
      printf("Element %.2d (S22) (%.2d & --) => Re:%16.8e  Im:%16.8e\n",   6, 9,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+9], 0.);
      printf("Element %.2d (S23) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 7, 10, 11,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+10], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+11]);
      printf("Element %.2d (S24) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 8, 12, 13,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+12], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+13]);
      printf("Element %.2d (S25) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 9, 14, 15,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+14], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+15]);
      printf("Element %.2d (S33) (%.2d & --) => Re:%16.8e  Im:%16.8e\n",   10, 16,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+16], 0.);
      printf("Element %.2d (S34) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 11, 17, 18,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+17], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+18]);
      printf("Element %.2d (S35) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 12, 19, 20,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+19], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+20]);
      printf("Element %.2d (S44) (%.2d & --) => Re:%16.8e  Im:%16.8e\n",   13, 21,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+21], 0.);
      printf("Element %.2d (S45) (%.2d & %.2d) => Re:%16.8e  Im:%16.8e\n", 14, 22, 23,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+22], compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+23]);
      printf("Element %.2d (S55) (%.2d & --) => Re:%16.8e  Im:%16.8e\n",   15, 24,
             compressed_spectral_matrix_f0[i*NB_VALUES_PER_SPECTRAL_MATRIX+24], 0.);

    }
    return 0;
}


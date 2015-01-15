#define NB_VALUES_PER_SM 25
#define NB_BINS_PER_SM   128

#define NB_BINS_COMPRESSED_SM_F0    11
#define ASM_F0_INDICE_START         17      // 88 bins
#define ASM_F0_INDICE_STOP          104     // 2 packets of 44 bins
#define NB_BINS_TO_AVERAGE_ASM_F0   8

void ASM_reorganize_and_divide( float *averaged_spec_mat, float *averaged_spec_mat_reorganized, float divider )
{
    int frequencyBin;
    int asmComponent;
    unsigned int offsetASM;
    unsigned int offsetASMReorganized;

    // BUILD DATA
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++ )
        {
            offsetASMReorganized =
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetASM            =
                    asmComponent * NB_BINS_PER_SM
                    + frequencyBin;
            averaged_spec_mat_reorganized[offsetASMReorganized  ] =
                    averaged_spec_mat[ offsetASM ] / divider;
        }
    }
}

void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat , float divider,
                                 unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage, unsigned char ASMIndexStart )
{
    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int k;

    // BUILD DATA
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix; frequencyBin++ )
        {
            offsetCompressed =  // NO TIME OFFSET
                    frequencyBin * NB_VALUES_PER_SM
                    + asmComponent;
            offsetASM =         // NO TIME OFFSET
                    asmComponent * NB_BINS_PER_SM
                    + ASMIndexStart
                    + frequencyBin * nbBinsToAverage;
            compressed_spec_mat[ offsetCompressed ] = 0;
            for ( k = 0; k < nbBinsToAverage; k++ )
            {
                compressed_spec_mat[offsetCompressed ] =
                        ( compressed_spec_mat[ offsetCompressed ]
                        + averaged_spec_mat[ offsetASM + k ] );
            }
            compressed_spec_mat[ offsetCompressed ] =
                    compressed_spec_mat[ offsetCompressed ] / (divider * nbBinsToAverage);
        }
    }
}

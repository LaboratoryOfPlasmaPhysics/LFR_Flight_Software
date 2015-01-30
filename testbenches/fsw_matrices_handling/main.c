#include <stdio.h>

#include "functions.h"

int main(void)
{
    printf("Hello World!\n");

    unsigned int asmComponent;
    unsigned int frequencyBin;
    unsigned int offset_input_ASM;

    float input_ASM             [ NB_VALUES_PER_SM * NB_BINS_PER_SM           ];
    float output_ASM            [ NB_VALUES_PER_SM * NB_BINS_PER_SM           ];
    float patched_ASM           [ NB_VALUES_PER_SM * NB_BINS_PER_SM           ];
    float output_ASM_compressed [ NB_VALUES_PER_SM * NB_BINS_COMPRESSED_SM_F0 ];

    //*******
    // TEST 1

    offset_input_ASM = 0;

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for (frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++)
        {
            offset_input_ASM = asmComponent * NB_BINS_PER_SM + frequencyBin;
            input_ASM[ offset_input_ASM ] = asmComponent;
        }
    }

    ASM_patch( input_ASM, patched_ASM );

    ASM_reorganize_and_divide( input_ASM, output_ASM,
                               1 ); // divider

    ASM_compress_reorganize_and_divide( input_ASM, output_ASM_compressed,
                                        1,  // divider
                                        NB_BINS_COMPRESSED_SM_F0,
                                        NB_BINS_TO_AVERAGE_ASM_F0,
                                        ASM_F0_INDICE_START);

    //*******
    // TEST 2
    offset_input_ASM = 0;

    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for (frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++)
        {
            offset_input_ASM = asmComponent * NB_BINS_PER_SM + frequencyBin;
            input_ASM[ offset_input_ASM ] = asmComponent * NB_BINS_PER_SM + frequencyBin;
        }
    }

    ASM_reorganize_and_divide( input_ASM, output_ASM,
                               1 ); // divider

    ASM_compress_reorganize_and_divide( input_ASM, output_ASM_compressed,
                                        10,  // divider
                                        NB_BINS_COMPRESSED_SM_F0,
                                        NB_BINS_TO_AVERAGE_ASM_F0,
                                        ASM_F0_INDICE_START);

    ASM_patch( input_ASM, patched_ASM );

    return 0;
}


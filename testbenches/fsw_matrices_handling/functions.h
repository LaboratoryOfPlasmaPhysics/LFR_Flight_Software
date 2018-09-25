/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

#define NB_VALUES_PER_SM 25
#define NB_BINS_PER_SM   128

#define NB_BINS_COMPRESSED_SM_F0    11
#define ASM_F0_INDICE_START         17      // 88 bins
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

void extractReImVectors( float *inputASM, float *outputASM, unsigned int asmComponent )
{
    unsigned int i;
    float re;
    float im;

    for (i=0; i<NB_BINS_PER_SM; i++){
        re = inputASM[ (asmComponent*NB_BINS_PER_SM) + i * 2    ];
        im = inputASM[ (asmComponent*NB_BINS_PER_SM) + i * 2 + 1];
        outputASM[ (asmComponent   *NB_BINS_PER_SM)  +  i] = re;
        outputASM[ (asmComponent+1)*NB_BINS_PER_SM   +  i] = im;
    }

}

void ASM_patch( float *inputASM, float *outputASM )
{
    extractReImVectors( inputASM, outputASM, 1);    // b1b2
    extractReImVectors( inputASM, outputASM, 3 );   // b1b3
    extractReImVectors( inputASM, outputASM, 5 );   // b1e1
    extractReImVectors( inputASM, outputASM, 7 );   // b1e2
    extractReImVectors( inputASM, outputASM, 10 );  // b2b3
    extractReImVectors( inputASM, outputASM, 12 );  // b2e1
    extractReImVectors( inputASM, outputASM, 14 );  // b2e2
    extractReImVectors( inputASM, outputASM, 17 );  // b3e1
    extractReImVectors( inputASM, outputASM, 19 );  // b3e2
    extractReImVectors( inputASM, outputASM, 22 );  // e1e2
}

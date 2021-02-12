/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2021, Plasma Physics Laboratory - CNRS
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
/*--                  Author : Alexis Jeandet
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/
#include "processing/ASM/spectralmatrices.h"

void SM_average(float *averaged_spec_mat_NORM, float *averaged_spec_mat_SBM, ring_node *ring_node_tab[], unsigned int nbAverageNORM, unsigned int nbAverageSBM, asm_msg *msgForMATR, unsigned char channel)
{
    float sum;
    unsigned int i;
    unsigned int k;
    unsigned char incomingSMIsValid[NB_SM_BEFORE_AVF0_F1];
    unsigned int numberOfValidSM;
    unsigned char isValid;

    //**************
    // PAS FILTERING
    // check acquisitionTime of the incoming data
    numberOfValidSM = 0;
    for (k=0; k<NB_SM_BEFORE_AVF0_F1; k++)
    {
        isValid = acquisitionTimeIsValid( ring_node_tab[k]->coarseTime, ring_node_tab[k]->fineTime, channel );
        incomingSMIsValid[k] = isValid;
        numberOfValidSM = numberOfValidSM + isValid;
    }

    //************************
    // AVERAGE SPECTRAL MATRIX
    for(i=0; i<TOTAL_SIZE_SM; i++)
    {
        sum = INIT_FLOAT;
        for ( k = 0; k < NB_SM_BEFORE_AVF0_F1; k++ )
        {
            if (incomingSMIsValid[k] == MATRIX_IS_NOT_POLLUTED)
            {
                sum = sum + ( (int *) (ring_node_tab[0]->buffer_address) ) [ i ] ;
            }
        }

        if ( (nbAverageNORM == 0) && (nbAverageSBM == 0) )
        {
            averaged_spec_mat_NORM[ i ] = sum;
            averaged_spec_mat_SBM[  i ] = sum;
            msgForMATR->coarseTimeNORM  = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeNORM    = ring_node_tab[0]->fineTime;
            msgForMATR->coarseTimeSBM   = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeSBM     = ring_node_tab[0]->fineTime;
        }
        else if ( (nbAverageNORM != 0) && (nbAverageSBM != 0) )
        {
            averaged_spec_mat_NORM[ i ] = ( averaged_spec_mat_NORM[  i ] + sum );
            averaged_spec_mat_SBM[  i ] = ( averaged_spec_mat_SBM[   i ] + sum );
        }
        else if ( (nbAverageNORM != 0) && (nbAverageSBM == 0) )
        {
            averaged_spec_mat_NORM[ i ] = ( averaged_spec_mat_NORM[ i ] + sum );
            averaged_spec_mat_SBM[  i ] = sum;
            msgForMATR->coarseTimeSBM   = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeSBM     = ring_node_tab[0]->fineTime;
        }
        else
        {
            averaged_spec_mat_NORM[ i ] = sum;
            averaged_spec_mat_SBM[  i ] = ( averaged_spec_mat_SBM[   i ] + sum );
            msgForMATR->coarseTimeNORM   = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeNORM     = ring_node_tab[0]->fineTime;
            //            PRINTF2("ERR *** in SM_average *** unexpected parameters %d %d\n", nbAverageNORM, nbAverageSBM)
        }
    }

    //*******************
    // UPDATE SM COUNTERS
    if ( (nbAverageNORM == 0) && (nbAverageSBM == 0) )
    {
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM  = numberOfValidSM;
    }
    else if ( (nbAverageNORM != 0) && (nbAverageSBM != 0) )
    {
        msgForMATR->numberOfSMInASMNORM = msgForMATR->numberOfSMInASMNORM   + numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM  = msgForMATR->numberOfSMInASMSBM    + numberOfValidSM;
    }
    else if ( (nbAverageNORM != 0) && (nbAverageSBM == 0) )
    {
        msgForMATR->numberOfSMInASMNORM = msgForMATR->numberOfSMInASMNORM   + numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM  = numberOfValidSM;
    }
    else
    {
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM  = msgForMATR->numberOfSMInASMSBM    + numberOfValidSM;
    }
}


void ASM_compress_reorganize_and_divide_mask(float *averaged_spec_mat, float *compressed_spec_mat , float divider,
                                             unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage,
                                             unsigned char ASMIndexStart,
                                             unsigned char channel )
{
    //*************
    // input format
    // component0[0 .. 127] component1[0 .. 127] .. component24[0 .. 127]
    //**************
    // output format
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr127[0 .. 24]
    //************
    // compression
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr11[0 .. 24] => f0 NORM
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr22[0 .. 24] => f0 BURST, SBM

    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int offsetFBin;
    int fBinMask;
    int k;

    // BUILD DATA
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++)
    {
        for( frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix; frequencyBin++ )
        {
            offsetCompressed =  // NO TIME OFFSET
                    (frequencyBin * NB_VALUES_PER_SM)
                    + asmComponent;
            offsetASM =         // NO TIME OFFSET
                    (asmComponent * NB_BINS_PER_SM)
                    + ASMIndexStart
                    + (frequencyBin * nbBinsToAverage);
            offsetFBin = ASMIndexStart
                    + (frequencyBin * nbBinsToAverage);
            compressed_spec_mat[ offsetCompressed ] = 0;
            for ( k = 0; k < nbBinsToAverage; k++ )
            {
                fBinMask = getFBinMask( offsetFBin + k, channel );
                compressed_spec_mat[offsetCompressed ] = compressed_spec_mat[ offsetCompressed ]
                        + (averaged_spec_mat[ offsetASM + k ] * fBinMask);
            }
            if (divider != 0)
            {
                compressed_spec_mat[ offsetCompressed ] = compressed_spec_mat[ offsetCompressed ] / (divider * nbBinsToAverage);
            }
            else
            {
                compressed_spec_mat[ offsetCompressed ] = INIT_FLOAT;
            }
        }
    }

}

void ASM_convert(volatile float *input_matrix, char *output_matrix) {
    unsigned int frequencyBin;
    unsigned int asmComponent;
    char *pt_char_input;
    char *pt_char_output;
    unsigned int offsetInput;
    unsigned int offsetOutput;

    pt_char_input = (char *)&input_matrix;
    pt_char_output = (char *)&output_matrix;

    // convert all other data
    for (frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++) {
        for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++) {
            offsetInput = (frequencyBin * NB_VALUES_PER_SM) + asmComponent;
            offsetOutput =
                    SM_BYTES_PER_VAL * ((frequencyBin * NB_VALUES_PER_SM) + asmComponent);
            pt_char_input = (char *)&input_matrix[offsetInput];
            pt_char_output = (char *)&output_matrix[offsetOutput];
            pt_char_output[0] = pt_char_input[0]; // bits 31 downto 24 of the float
            pt_char_output[1] = pt_char_input[1]; // bits 23 downto 16 of the float
        }
    }
}

void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat, float divider, unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage, unsigned char ASMIndexStart) {
    int frequencyBin;
    int asmComponent;
    int offsetASM;
    int offsetCompressed;
    int k;

    // BUILD DATA
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++) {
        for (frequencyBin = 0; frequencyBin < nbBinsCompressedMatrix;
             frequencyBin++) {
            offsetCompressed = // NO TIME OFFSET
                    (frequencyBin * NB_VALUES_PER_SM) + asmComponent;
            offsetASM = // NO TIME OFFSET
                    (asmComponent * NB_BINS_PER_SM) + ASMIndexStart +
                    (frequencyBin * nbBinsToAverage);
            compressed_spec_mat[offsetCompressed] = 0;
            for (k = 0; k < nbBinsToAverage; k++) {
                compressed_spec_mat[offsetCompressed] =
                        (compressed_spec_mat[offsetCompressed] +
                         averaged_spec_mat[offsetASM + k]);
            }
            compressed_spec_mat[offsetCompressed] =
                    compressed_spec_mat[offsetCompressed] / (divider * nbBinsToAverage);
        }
    }
}

void ASM_reorganize_and_divide(float *averaged_spec_mat, float *averaged_spec_mat_reorganized, const float divider) {
    int frequencyBin;
    int asmComponent;
    unsigned int offsetASM;
    unsigned int offsetASMReorganized;

    // BUILD DATA
    for (asmComponent = 0; asmComponent < NB_VALUES_PER_SM; asmComponent++) {
        for (frequencyBin = 0; frequencyBin < NB_BINS_PER_SM; frequencyBin++) {
            offsetASMReorganized = (frequencyBin * NB_VALUES_PER_SM) + asmComponent;
            offsetASM = (asmComponent * NB_BINS_PER_SM) + frequencyBin;
            if (divider != INIT_FLOAT) {
                averaged_spec_mat_reorganized[offsetASMReorganized] =
                        averaged_spec_mat[offsetASM] / divider;
            } else {
                averaged_spec_mat_reorganized[offsetASMReorganized] = INIT_FLOAT;
            }
        }
    }
}


/**
 * @brief extractReImVectors converts a given ASM component from interleaved to split representation
 * @param inputASM
 * @param outputASM
 * @param asmComponent
 */
void extractReImVectors( float *inputASM, float *outputASM, unsigned int asmComponent )
{
    unsigned int i;
    float re;
    float im;

    for (i=0; i<NB_BINS_PER_SM; i++){
        re = inputASM[ (asmComponent*NB_BINS_PER_SM) + (i * SM_BYTES_PER_VAL)    ];
        im = inputASM[ (asmComponent*NB_BINS_PER_SM) + (i * SM_BYTES_PER_VAL) + 1];
        outputASM[ ( asmComponent   *NB_BINS_PER_SM) +  i] = re;
        outputASM[ ((asmComponent+1)*NB_BINS_PER_SM) +  i] = im;
    }
}

/**
 * @brief copyReVectors copies real part of a given ASM from inputASM to outputASM
 * @param inputASM
 * @param outputASM
 * @param asmComponent
 */
void copyReVectors( float *inputASM, float *outputASM, unsigned int asmComponent )
{
    unsigned int i;
    float re;

    for (i=0; i<NB_BINS_PER_SM; i++){
        re = inputASM[ (asmComponent*NB_BINS_PER_SM) + i];
        outputASM[ (asmComponent*NB_BINS_PER_SM)  +  i] = re;
    }
}

/**
 * @brief ASM_patch, converts ASM from interleaved to split representation
 * @param inputASM
 * @param outputASM
 * @note inputASM and outputASM must be different, in other words this function can't do in place convertion
 * @see extractReImVectors
 */
void ASM_patch( float *inputASM, float *outputASM )
{
    extractReImVectors( inputASM, outputASM, ASM_COMP_B1B2);    // b1b2
    extractReImVectors( inputASM, outputASM, ASM_COMP_B1B3 );   // b1b3
    extractReImVectors( inputASM, outputASM, ASM_COMP_B1E1 );   // b1e1
    extractReImVectors( inputASM, outputASM, ASM_COMP_B1E2 );   // b1e2
    extractReImVectors( inputASM, outputASM, ASM_COMP_B2B3 );   // b2b3
    extractReImVectors( inputASM, outputASM, ASM_COMP_B2E1 );   // b2e1
    extractReImVectors( inputASM, outputASM, ASM_COMP_B2E2 );   // b2e2
    extractReImVectors( inputASM, outputASM, ASM_COMP_B3E1 );   // b3e1
    extractReImVectors( inputASM, outputASM, ASM_COMP_B3E2 );   // b3e2
    extractReImVectors( inputASM, outputASM, ASM_COMP_E1E2 );   // e1e2

    copyReVectors(inputASM, outputASM, ASM_COMP_B1B1 );     // b1b1
    copyReVectors(inputASM, outputASM, ASM_COMP_B2B2 );     // b2b2
    copyReVectors(inputASM, outputASM, ASM_COMP_B3B3);      // b3b3
    copyReVectors(inputASM, outputASM, ASM_COMP_E1E1);      // e1e1
    copyReVectors(inputASM, outputASM, ASM_COMP_E2E2);      // e2e2
}

void SM_calibrate(float *input_asm, float *calibration_matrix,float *output_asm) {

}

void Matrix_change_of_basis(float *input_matrix, float *transition_matrix, float *output_matrix) {

}

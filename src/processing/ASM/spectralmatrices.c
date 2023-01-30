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
#include "fsw_debug.h"
#include "mitigations/PAS_filtering.h"
#include "mitigations/reaction_wheel_filtering.h"
#include "processing/calibration_matrices.h"

inline void clear_spectral_matrix(float* spectral_matrix) __attribute__((always_inline));
void clear_spectral_matrix(float* spectral_matrix)
{
    DEBUG_CHECK_PTR(spectral_matrix);
    for (int i = 0; i < TOTAL_SIZE_SM; i++)
    {
        spectral_matrix[i] = 0.f;
    }
}

void SM_average(float* averaged_spec_mat_NORM, float* averaged_spec_mat_SBM,
    ring_node* ring_node_tab[], unsigned int nbAverageNORM, unsigned int nbAverageSBM,
    asm_msg* msgForMATR, unsigned char channel, unsigned int start_bin, unsigned int bins_count)
{
    DEBUG_CHECK_PTR(averaged_spec_mat_NORM);
    DEBUG_CHECK_PTR(averaged_spec_mat_SBM);
    DEBUG_CHECK_PTR(ring_node_tab);
    DEBUG_CHECK_PTR(msgForMATR);

    unsigned int numberOfValidSM = 0U;
    int* valid_matrices[8];
    for (int SM_index = 0; SM_index < NB_SM_BEFORE_AVF0_F1; SM_index++)
    {
        if (acquisitionTimeIsValid(
                ring_node_tab[SM_index]->coarseTime, ring_node_tab[SM_index]->fineTime, channel))
        {
            valid_matrices[numberOfValidSM] = (int*)(ring_node_tab[SM_index]->buffer_address);
            numberOfValidSM++;
        }
    }

    if (nbAverageNORM == 0)
    {
        clear_spectral_matrix(averaged_spec_mat_NORM);
        msgForMATR->coarseTimeNORM = ring_node_tab[0]->coarseTime;
        msgForMATR->fineTimeNORM = ring_node_tab[0]->fineTime;
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
    }
    else
    {

        msgForMATR->numberOfSMInASMNORM += numberOfValidSM;
    }

    if (nbAverageSBM == 0)
    {
        clear_spectral_matrix(averaged_spec_mat_SBM);
        msgForMATR->coarseTimeSBM = ring_node_tab[0]->coarseTime;
        msgForMATR->fineTimeSBM = ring_node_tab[0]->fineTime;
        msgForMATR->numberOfSMInASMSBM = numberOfValidSM;
    }
    else
    {
        msgForMATR->numberOfSMInASMSBM += numberOfValidSM;
    }

    unsigned int block_offset = 0;
    for (unsigned int line = 0; line < 5; line++)
    {
        for (unsigned int column = line; column < 5; column++)
        {
            // Looping this way is 40% faster ~ 32ms Vs ~54ms
            // Skiping discarded frequencies gives another speedup
            unsigned int start_index;
            unsigned int stop_index;
            if (line == column) // pure reals
            {
                start_index = block_offset + start_bin;
                stop_index = start_index + bins_count;
            }
            else // complexes
            {
                start_index = block_offset + (start_bin * 2);
                stop_index = start_index + (bins_count * 2);
            }

            for (unsigned int i = start_index; i < stop_index; i += 2)
            {
                register float sum1 = 0.;
                register float sum2 = 0.;
                for (unsigned int SM_index = 0; SM_index < numberOfValidSM; SM_index++)
                {
                    sum1 += (float)(valid_matrices[SM_index][i]);
                    sum2 += (float)(valid_matrices[SM_index][i + 1]);
                }
                averaged_spec_mat_SBM[i] += sum1;
                averaged_spec_mat_NORM[i] += sum1;

                averaged_spec_mat_SBM[i + 1] += sum2;
                averaged_spec_mat_NORM[i + 1] += sum2;
            }
            if (line == column) // pure reals
            {
                block_offset += NB_BINS_PER_SM;
            }
            else // complexes
            {
                block_offset += NB_BINS_PER_SM * 2;
            }
        }
    }
}

void SM_average_f2(float* averaged_spec_mat_f2, ring_node* ring_node, unsigned int nbAverageNormF2,
    asm_msg* msgForMATR)
{
    DEBUG_CHECK_PTR(averaged_spec_mat_f2);
    DEBUG_CHECK_PTR(ring_node);
    DEBUG_CHECK_PTR(msgForMATR);
    DEBUG_PRINTF("in SM_average_f2");
    float sm_bin;
    unsigned char keepMatrix;

    // test acquisitionTime validity
    keepMatrix = acquisitionTimeIsValid(ring_node->coarseTime, ring_node->fineTime, CHANNELF2);

    for (unsigned int i = 0; i < TOTAL_SIZE_SM; i++)
    {
        sm_bin = (float)(((int*)(ring_node->buffer_address))[i]);
        if (nbAverageNormF2 == 0) // average initialization
        {
            if (keepMatrix == MATRIX_IS_NOT_POLLUTED) // keep the matrix and add it to the average
            {
                averaged_spec_mat_f2[i] = sm_bin;
            }
            else // drop the matrix and initialize the average
            {
                averaged_spec_mat_f2[i] = INIT_FLOAT;
            }
            msgForMATR->coarseTimeNORM = ring_node->coarseTime;
            msgForMATR->fineTimeNORM = ring_node->fineTime;
        }
        else
        {
            if (keepMatrix == MATRIX_IS_NOT_POLLUTED) // keep the matrix and add it to the average
            {
                averaged_spec_mat_f2[i] = (averaged_spec_mat_f2[i] + sm_bin);
            }
            else
            {
                // nothing to do, the matrix is not valid
            }
        }
    }

    if (keepMatrix == MATRIX_IS_NOT_POLLUTED)
    {
        if (nbAverageNormF2 == 0)
        {
            msgForMATR->numberOfSMInASMNORM = 1;
        }
        else
        {
            msgForMATR->numberOfSMInASMNORM++;
        }
    }
    else
    {
        if (nbAverageNormF2 == 0)
        {
            msgForMATR->numberOfSMInASMNORM = 0;
        }
        else
        {
            // nothing to do
        }
    }
    DEBUG_PRINTF("leaving SM_average_f2");
}


void ASM_compress_divide_and_mask(const float* const averaged_spec_mat, float* compressed_spec_mat,
    const float divider, const unsigned char nbBinsCompressedMatrix,
    const unsigned char nbBinsToAverage, const unsigned char ASMIndexStart,
    const unsigned char channel)
{
    DEBUG_CHECK_PTR(averaged_spec_mat);
    DEBUG_CHECK_PTR(compressed_spec_mat);
    //*************
    // input format
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr127[0 .. 24]
    //************
    // compression
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr11[0 .. 24] => f0 NORM
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr22[0 .. 24] => f0 BURST,
    // SBM

    // BUILD DATA
    // Just zero the whole compressed ASM
    // has to be done either way if divider==0. or to initialize average
    {
        for (int i = 0; i < nbBinsCompressedMatrix * NB_FLOATS_PER_SM; i++)
        {
            compressed_spec_mat[i] = 0.;
        }
    }
    if (divider != 0.f)
    {
        int freq_offset = ASMIndexStart;
        const float* input_asm_ptr = averaged_spec_mat + (ASMIndexStart * NB_FLOATS_PER_SM);
        float* compressed_asm_ptr = compressed_spec_mat;
        for (int compressed_frequency_bin = 0; compressed_frequency_bin < nbBinsCompressedMatrix;
             compressed_frequency_bin++)
        {
            for (int frequency_bin = 0; frequency_bin < nbBinsToAverage; frequency_bin++)
            {
                int fBinMask = getFBinMask(freq_offset, channel);
                compressed_asm_ptr
                    = compressed_spec_mat + (compressed_frequency_bin * NB_FLOATS_PER_SM);
                for (int asm_component = 0; asm_component < NB_FLOATS_PER_SM; asm_component++)
                {
                    *compressed_asm_ptr += (*input_asm_ptr * (float)fBinMask);
                    compressed_asm_ptr++;
                    input_asm_ptr++;
                }
                freq_offset++;
            }
        }
        for (int i = 0; i < nbBinsCompressedMatrix * NB_FLOATS_PER_SM; i++)
        {
            compressed_spec_mat[i] = compressed_spec_mat[i] / (divider * nbBinsToAverage);
        }
    }
}

void ASM_divide(const float* averaged_spec_mat, float* averaged_spec_mat_normalized,
    const float divider, unsigned int start_indice, unsigned int stop_indice)
{
    DEBUG_CHECK_PTR(averaged_spec_mat);
    DEBUG_CHECK_PTR(averaged_spec_mat_normalized);
    // BUILD DATA
    if (divider == 0.)
    {
        for (unsigned int i = (start_indice * NB_FLOATS_PER_SM);
             i < (stop_indice * NB_FLOATS_PER_SM); i++)
        {
            averaged_spec_mat_normalized[i] = 0.;
        }
    }
    else
    {
        const float coef = 1.f / divider;
        for (unsigned int i = (start_indice * NB_FLOATS_PER_SM);
             i < (stop_indice * NB_FLOATS_PER_SM); i++)
        {
            averaged_spec_mat_normalized[i] = averaged_spec_mat[i] * coef;
        }
    }
}


// clang-format off
/*
 *   This function performs a specific 5x5 matrix change of basis with the folowing assumptions
                                                                                                                              T*
          output_matrix                mag_transition_matrix               input_matrix                 mag_transition_matrix
               |                              |                                 |                              |
               v                              v                                 v                              v
 | |SM11|  SM12  SM13  SM14  SM15  |   | B11 B21 B31   0   0  |   | |SM11|  SM12  SM13  SM14  SM15  |   | B11 B12 B13   0   0  |
 |        |SM22| SM23  SM24  SM25  |   | B12 B22 B32   0   0  |   |        |SM22| SM23  SM24  SM25  |   | B21 B22 B23   0   0  |
 |              |SM33| SM34  SM35  | = | B13 B23 B33   0   0  | X |              |SM33| SM34  SM35  | X | B31 B32 B33   0   0  |
 |                    |SM44| SM45  |   |  0   0   0   E11 E21 |   |                    |SM44| SM45  |   |  0   0   0   E11 E12 |
 |                          |SM55| |   |  0   0   0   E12 E22 |   |                          |SM55| |   |  0   0   0   E21 E22 |
                                                         ^                                                                ^
                                                         |                                                                |            T*
                                              elec_transition_matrix                                             elec_transition_matrix
Where each matrix product is done sequencialy for mag and elec transition matrices
Note that this code is generated with LFR_Flight_Software/python_scripts/Matrix_calibration_code_gen.ipynb
*/
// clang-format on
void Matrix_change_of_basis(_Complex float _intermediary[25], const float* input_matrix,
    const float* mag_transition_matrix, const float* elec_transition_matrix, float* output_matrix)
{
    DEBUG_CHECK_PTR(_intermediary);
    DEBUG_CHECK_PTR(input_matrix);
    DEBUG_CHECK_PTR(mag_transition_matrix);
    DEBUG_CHECK_PTR(elec_transition_matrix);
    DEBUG_CHECK_PTR(output_matrix);

    float* intermediary = (float*)_intermediary;
    intermediary[0] = input_matrix[0] * mag_transition_matrix[0]
        + input_matrix[1] * mag_transition_matrix[2] + input_matrix[3] * mag_transition_matrix[4]
        - (-input_matrix[2]) * mag_transition_matrix[3]
        - (-input_matrix[4]) * mag_transition_matrix[5];
    intermediary[1] = input_matrix[0] * mag_transition_matrix[1]
        + input_matrix[1] * mag_transition_matrix[3] + input_matrix[3] * mag_transition_matrix[5]
        + mag_transition_matrix[2] * (-input_matrix[2])
        + mag_transition_matrix[4] * (-input_matrix[4]);
    intermediary[2] = input_matrix[9] * mag_transition_matrix[2]
        + input_matrix[1] * mag_transition_matrix[0] + input_matrix[10] * mag_transition_matrix[4]
        - input_matrix[2] * mag_transition_matrix[1]
        - (-input_matrix[11]) * mag_transition_matrix[5];
    intermediary[3] = input_matrix[9] * mag_transition_matrix[3]
        + input_matrix[1] * mag_transition_matrix[1] + input_matrix[10] * mag_transition_matrix[5]
        + mag_transition_matrix[0] * input_matrix[2]
        + mag_transition_matrix[4] * (-input_matrix[11]);
    intermediary[4] = input_matrix[16] * mag_transition_matrix[4]
        + input_matrix[3] * mag_transition_matrix[0] + input_matrix[10] * mag_transition_matrix[2]
        - input_matrix[4] * mag_transition_matrix[1] - input_matrix[11] * mag_transition_matrix[3];
    intermediary[5] = input_matrix[16] * mag_transition_matrix[5]
        + input_matrix[3] * mag_transition_matrix[1] + input_matrix[10] * mag_transition_matrix[3]
        + mag_transition_matrix[0] * input_matrix[4] + mag_transition_matrix[2] * input_matrix[11];
    intermediary[6] = input_matrix[5] * mag_transition_matrix[0]
        + input_matrix[12] * mag_transition_matrix[2] + input_matrix[17] * mag_transition_matrix[4]
        - input_matrix[6] * mag_transition_matrix[1] - input_matrix[13] * mag_transition_matrix[3]
        - input_matrix[18] * mag_transition_matrix[5];
    intermediary[7] = input_matrix[5] * mag_transition_matrix[1]
        + input_matrix[12] * mag_transition_matrix[3] + input_matrix[17] * mag_transition_matrix[5]
        + mag_transition_matrix[0] * input_matrix[6] + mag_transition_matrix[2] * input_matrix[13]
        + mag_transition_matrix[4] * input_matrix[18];
    intermediary[8] = input_matrix[7] * mag_transition_matrix[0]
        + input_matrix[14] * mag_transition_matrix[2] + input_matrix[19] * mag_transition_matrix[4]
        - input_matrix[8] * mag_transition_matrix[1] - input_matrix[15] * mag_transition_matrix[3]
        - input_matrix[20] * mag_transition_matrix[5];
    intermediary[9] = input_matrix[7] * mag_transition_matrix[1]
        + input_matrix[14] * mag_transition_matrix[3] + input_matrix[19] * mag_transition_matrix[5]
        + mag_transition_matrix[0] * input_matrix[8] + mag_transition_matrix[2] * input_matrix[15]
        + mag_transition_matrix[4] * input_matrix[20];
    intermediary[10] = input_matrix[0] * mag_transition_matrix[6]
        + input_matrix[1] * mag_transition_matrix[8] + input_matrix[3] * mag_transition_matrix[10]
        - (-input_matrix[2]) * mag_transition_matrix[9]
        - (-input_matrix[4]) * mag_transition_matrix[11];
    intermediary[11] = input_matrix[0] * mag_transition_matrix[7]
        + input_matrix[1] * mag_transition_matrix[9] + input_matrix[3] * mag_transition_matrix[11]
        + mag_transition_matrix[8] * (-input_matrix[2])
        + mag_transition_matrix[10] * (-input_matrix[4]);
    intermediary[12] = input_matrix[9] * mag_transition_matrix[8]
        + input_matrix[1] * mag_transition_matrix[6] + input_matrix[10] * mag_transition_matrix[10]
        - input_matrix[2] * mag_transition_matrix[7]
        - (-input_matrix[11]) * mag_transition_matrix[11];
    intermediary[13] = input_matrix[9] * mag_transition_matrix[9]
        + input_matrix[1] * mag_transition_matrix[7] + input_matrix[10] * mag_transition_matrix[11]
        + mag_transition_matrix[6] * input_matrix[2]
        + mag_transition_matrix[10] * (-input_matrix[11]);
    intermediary[14] = input_matrix[16] * mag_transition_matrix[10]
        + input_matrix[3] * mag_transition_matrix[6] + input_matrix[10] * mag_transition_matrix[8]
        - input_matrix[4] * mag_transition_matrix[7] - input_matrix[11] * mag_transition_matrix[9];
    intermediary[15] = input_matrix[16] * mag_transition_matrix[11]
        + input_matrix[3] * mag_transition_matrix[7] + input_matrix[10] * mag_transition_matrix[9]
        + mag_transition_matrix[6] * input_matrix[4] + mag_transition_matrix[8] * input_matrix[11];
    intermediary[16] = input_matrix[5] * mag_transition_matrix[6]
        + input_matrix[12] * mag_transition_matrix[8] + input_matrix[17] * mag_transition_matrix[10]
        - input_matrix[6] * mag_transition_matrix[7] - input_matrix[13] * mag_transition_matrix[9]
        - input_matrix[18] * mag_transition_matrix[11];
    intermediary[17] = input_matrix[5] * mag_transition_matrix[7]
        + input_matrix[12] * mag_transition_matrix[9] + input_matrix[17] * mag_transition_matrix[11]
        + mag_transition_matrix[6] * input_matrix[6] + mag_transition_matrix[8] * input_matrix[13]
        + mag_transition_matrix[10] * input_matrix[18];
    intermediary[18] = input_matrix[7] * mag_transition_matrix[6]
        + input_matrix[14] * mag_transition_matrix[8] + input_matrix[19] * mag_transition_matrix[10]
        - input_matrix[8] * mag_transition_matrix[7] - input_matrix[15] * mag_transition_matrix[9]
        - input_matrix[20] * mag_transition_matrix[11];
    intermediary[19] = input_matrix[7] * mag_transition_matrix[7]
        + input_matrix[14] * mag_transition_matrix[9] + input_matrix[19] * mag_transition_matrix[11]
        + mag_transition_matrix[6] * input_matrix[8] + mag_transition_matrix[8] * input_matrix[15]
        + mag_transition_matrix[10] * input_matrix[20];
    intermediary[20] = input_matrix[0] * mag_transition_matrix[12]
        + input_matrix[1] * mag_transition_matrix[14] + input_matrix[3] * mag_transition_matrix[16]
        - (-input_matrix[2]) * mag_transition_matrix[15]
        - (-input_matrix[4]) * mag_transition_matrix[17];
    intermediary[21] = input_matrix[0] * mag_transition_matrix[13]
        + input_matrix[1] * mag_transition_matrix[15] + input_matrix[3] * mag_transition_matrix[17]
        + mag_transition_matrix[14] * (-input_matrix[2])
        + mag_transition_matrix[16] * (-input_matrix[4]);
    intermediary[22] = input_matrix[9] * mag_transition_matrix[14]
        + input_matrix[1] * mag_transition_matrix[12] + input_matrix[10] * mag_transition_matrix[16]
        - input_matrix[2] * mag_transition_matrix[13]
        - (-input_matrix[11]) * mag_transition_matrix[17];
    intermediary[23] = input_matrix[9] * mag_transition_matrix[15]
        + input_matrix[1] * mag_transition_matrix[13] + input_matrix[10] * mag_transition_matrix[17]
        + mag_transition_matrix[12] * input_matrix[2]
        + mag_transition_matrix[16] * (-input_matrix[11]);
    intermediary[24] = input_matrix[16] * mag_transition_matrix[16]
        + input_matrix[3] * mag_transition_matrix[12] + input_matrix[10] * mag_transition_matrix[14]
        - input_matrix[4] * mag_transition_matrix[13]
        - input_matrix[11] * mag_transition_matrix[15];
    intermediary[25] = input_matrix[16] * mag_transition_matrix[17]
        + input_matrix[3] * mag_transition_matrix[13] + input_matrix[10] * mag_transition_matrix[15]
        + mag_transition_matrix[12] * input_matrix[4]
        + mag_transition_matrix[14] * input_matrix[11];
    intermediary[26] = input_matrix[5] * mag_transition_matrix[12]
        + input_matrix[12] * mag_transition_matrix[14]
        + input_matrix[17] * mag_transition_matrix[16] - input_matrix[6] * mag_transition_matrix[13]
        - input_matrix[13] * mag_transition_matrix[15]
        - input_matrix[18] * mag_transition_matrix[17];
    intermediary[27] = input_matrix[5] * mag_transition_matrix[13]
        + input_matrix[12] * mag_transition_matrix[15]
        + input_matrix[17] * mag_transition_matrix[17] + mag_transition_matrix[12] * input_matrix[6]
        + mag_transition_matrix[14] * input_matrix[13]
        + mag_transition_matrix[16] * input_matrix[18];
    intermediary[28] = input_matrix[7] * mag_transition_matrix[12]
        + input_matrix[14] * mag_transition_matrix[14]
        + input_matrix[19] * mag_transition_matrix[16] - input_matrix[8] * mag_transition_matrix[13]
        - input_matrix[15] * mag_transition_matrix[15]
        - input_matrix[20] * mag_transition_matrix[17];
    intermediary[29] = input_matrix[7] * mag_transition_matrix[13]
        + input_matrix[14] * mag_transition_matrix[15]
        + input_matrix[19] * mag_transition_matrix[17] + mag_transition_matrix[12] * input_matrix[8]
        + mag_transition_matrix[14] * input_matrix[15]
        + mag_transition_matrix[16] * input_matrix[20];
    intermediary[30] = elec_transition_matrix[0] * input_matrix[5]
        + elec_transition_matrix[2] * input_matrix[7]
        - elec_transition_matrix[1] * (-input_matrix[6])
        - elec_transition_matrix[3] * (-input_matrix[8]);
    intermediary[31] = elec_transition_matrix[0] * (-input_matrix[6])
        + elec_transition_matrix[2] * (-input_matrix[8])
        + input_matrix[5] * elec_transition_matrix[1] + input_matrix[7] * elec_transition_matrix[3];
    intermediary[32] = elec_transition_matrix[0] * input_matrix[12]
        + elec_transition_matrix[2] * input_matrix[14]
        - elec_transition_matrix[1] * (-input_matrix[13])
        - elec_transition_matrix[3] * (-input_matrix[15]);
    intermediary[33] = elec_transition_matrix[0] * (-input_matrix[13])
        + elec_transition_matrix[2] * (-input_matrix[15])
        + input_matrix[12] * elec_transition_matrix[1]
        + input_matrix[14] * elec_transition_matrix[3];
    intermediary[34] = elec_transition_matrix[0] * input_matrix[17]
        + elec_transition_matrix[2] * input_matrix[19]
        - elec_transition_matrix[1] * (-input_matrix[18])
        - elec_transition_matrix[3] * (-input_matrix[20]);
    intermediary[35] = elec_transition_matrix[0] * (-input_matrix[18])
        + elec_transition_matrix[2] * (-input_matrix[20])
        + input_matrix[17] * elec_transition_matrix[1]
        + input_matrix[19] * elec_transition_matrix[3];
    intermediary[36] = input_matrix[21] * elec_transition_matrix[0]
        + elec_transition_matrix[2] * input_matrix[22]
        - elec_transition_matrix[3] * (-input_matrix[23]);
    intermediary[37] = input_matrix[21] * elec_transition_matrix[1]
        + elec_transition_matrix[2] * (-input_matrix[23])
        + input_matrix[22] * elec_transition_matrix[3];
    intermediary[38] = input_matrix[24] * elec_transition_matrix[2]
        + elec_transition_matrix[0] * input_matrix[22]
        - elec_transition_matrix[1] * input_matrix[23];
    intermediary[39] = input_matrix[24] * elec_transition_matrix[3]
        + elec_transition_matrix[0] * input_matrix[23]
        + input_matrix[22] * elec_transition_matrix[1];
    intermediary[40] = elec_transition_matrix[4] * input_matrix[5]
        + elec_transition_matrix[6] * input_matrix[7]
        - elec_transition_matrix[5] * (-input_matrix[6])
        - elec_transition_matrix[7] * (-input_matrix[8]);
    intermediary[41] = elec_transition_matrix[4] * (-input_matrix[6])
        + elec_transition_matrix[6] * (-input_matrix[8])
        + input_matrix[5] * elec_transition_matrix[5] + input_matrix[7] * elec_transition_matrix[7];
    intermediary[42] = elec_transition_matrix[4] * input_matrix[12]
        + elec_transition_matrix[6] * input_matrix[14]
        - elec_transition_matrix[5] * (-input_matrix[13])
        - elec_transition_matrix[7] * (-input_matrix[15]);
    intermediary[43] = elec_transition_matrix[4] * (-input_matrix[13])
        + elec_transition_matrix[6] * (-input_matrix[15])
        + input_matrix[12] * elec_transition_matrix[5]
        + input_matrix[14] * elec_transition_matrix[7];
    intermediary[44] = elec_transition_matrix[4] * input_matrix[17]
        + elec_transition_matrix[6] * input_matrix[19]
        - elec_transition_matrix[5] * (-input_matrix[18])
        - elec_transition_matrix[7] * (-input_matrix[20]);
    intermediary[45] = elec_transition_matrix[4] * (-input_matrix[18])
        + elec_transition_matrix[6] * (-input_matrix[20])
        + input_matrix[17] * elec_transition_matrix[5]
        + input_matrix[19] * elec_transition_matrix[7];
    intermediary[46] = input_matrix[21] * elec_transition_matrix[4]
        + elec_transition_matrix[6] * input_matrix[22]
        - elec_transition_matrix[7] * (-input_matrix[23]);
    intermediary[47] = input_matrix[21] * elec_transition_matrix[5]
        + elec_transition_matrix[6] * (-input_matrix[23])
        + input_matrix[22] * elec_transition_matrix[7];
    intermediary[48] = input_matrix[24] * elec_transition_matrix[6]
        + elec_transition_matrix[4] * input_matrix[22]
        - elec_transition_matrix[5] * input_matrix[23];
    intermediary[49] = input_matrix[24] * elec_transition_matrix[7]
        + elec_transition_matrix[4] * input_matrix[23]
        + input_matrix[22] * elec_transition_matrix[5];
    output_matrix[0] = mag_transition_matrix[0] * intermediary[0]
        + mag_transition_matrix[2] * intermediary[2] + mag_transition_matrix[4] * intermediary[4]
        + mag_transition_matrix[1] * intermediary[1] + mag_transition_matrix[3] * intermediary[3]
        + mag_transition_matrix[5] * intermediary[5];
    output_matrix[1] = mag_transition_matrix[6] * intermediary[0]
        + mag_transition_matrix[8] * intermediary[2] + mag_transition_matrix[10] * intermediary[4]
        + mag_transition_matrix[7] * intermediary[1] + mag_transition_matrix[9] * intermediary[3]
        + mag_transition_matrix[11] * intermediary[5];
    output_matrix[2] = mag_transition_matrix[6] * intermediary[1]
        + mag_transition_matrix[8] * intermediary[3] + mag_transition_matrix[10] * intermediary[5]
        - intermediary[0] * mag_transition_matrix[7] - intermediary[2] * mag_transition_matrix[9]
        - intermediary[4] * mag_transition_matrix[11];
    output_matrix[3] = mag_transition_matrix[12] * intermediary[0]
        + mag_transition_matrix[14] * intermediary[2] + mag_transition_matrix[16] * intermediary[4]
        + mag_transition_matrix[13] * intermediary[1] + mag_transition_matrix[15] * intermediary[3]
        + mag_transition_matrix[17] * intermediary[5];
    output_matrix[4] = mag_transition_matrix[12] * intermediary[1]
        + mag_transition_matrix[14] * intermediary[3] + mag_transition_matrix[16] * intermediary[5]
        - intermediary[0] * mag_transition_matrix[13] - intermediary[2] * mag_transition_matrix[15]
        - intermediary[4] * mag_transition_matrix[17];
    output_matrix[5] = elec_transition_matrix[0] * intermediary[6]
        + elec_transition_matrix[2] * intermediary[8] + elec_transition_matrix[1] * intermediary[7]
        + elec_transition_matrix[3] * intermediary[9];
    output_matrix[6] = elec_transition_matrix[0] * intermediary[7]
        + elec_transition_matrix[2] * intermediary[9] - intermediary[6] * elec_transition_matrix[1]
        - intermediary[8] * elec_transition_matrix[3];
    output_matrix[7] = elec_transition_matrix[4] * intermediary[6]
        + elec_transition_matrix[6] * intermediary[8] + elec_transition_matrix[5] * intermediary[7]
        + elec_transition_matrix[7] * intermediary[9];
    output_matrix[8] = elec_transition_matrix[4] * intermediary[7]
        + elec_transition_matrix[6] * intermediary[9] - intermediary[6] * elec_transition_matrix[5]
        - intermediary[8] * elec_transition_matrix[7];
    output_matrix[9] = mag_transition_matrix[6] * intermediary[10]
        + mag_transition_matrix[8] * intermediary[12] + mag_transition_matrix[10] * intermediary[14]
        + mag_transition_matrix[7] * intermediary[11] + mag_transition_matrix[9] * intermediary[13]
        + mag_transition_matrix[11] * intermediary[15];
    output_matrix[10] = mag_transition_matrix[12] * intermediary[10]
        + mag_transition_matrix[14] * intermediary[12]
        + mag_transition_matrix[16] * intermediary[14]
        + mag_transition_matrix[13] * intermediary[11]
        + mag_transition_matrix[15] * intermediary[13]
        + mag_transition_matrix[17] * intermediary[15];
    output_matrix[11] = mag_transition_matrix[12] * intermediary[11]
        + mag_transition_matrix[14] * intermediary[13]
        + mag_transition_matrix[16] * intermediary[15]
        - intermediary[10] * mag_transition_matrix[13]
        - intermediary[12] * mag_transition_matrix[15]
        - intermediary[14] * mag_transition_matrix[17];
    output_matrix[12] = elec_transition_matrix[0] * intermediary[16]
        + elec_transition_matrix[2] * intermediary[18]
        + elec_transition_matrix[1] * intermediary[17]
        + elec_transition_matrix[3] * intermediary[19];
    output_matrix[13] = elec_transition_matrix[0] * intermediary[17]
        + elec_transition_matrix[2] * intermediary[19]
        - intermediary[16] * elec_transition_matrix[1]
        - intermediary[18] * elec_transition_matrix[3];
    output_matrix[14] = elec_transition_matrix[4] * intermediary[16]
        + elec_transition_matrix[6] * intermediary[18]
        + elec_transition_matrix[5] * intermediary[17]
        + elec_transition_matrix[7] * intermediary[19];
    output_matrix[15] = elec_transition_matrix[4] * intermediary[17]
        + elec_transition_matrix[6] * intermediary[19]
        - intermediary[16] * elec_transition_matrix[5]
        - intermediary[18] * elec_transition_matrix[7];
    output_matrix[16] = mag_transition_matrix[12] * intermediary[20]
        + mag_transition_matrix[14] * intermediary[22]
        + mag_transition_matrix[16] * intermediary[24]
        + mag_transition_matrix[13] * intermediary[21]
        + mag_transition_matrix[15] * intermediary[23]
        + mag_transition_matrix[17] * intermediary[25];
    output_matrix[17] = elec_transition_matrix[0] * intermediary[26]
        + elec_transition_matrix[2] * intermediary[28]
        + elec_transition_matrix[1] * intermediary[27]
        + elec_transition_matrix[3] * intermediary[29];
    output_matrix[18] = elec_transition_matrix[0] * intermediary[27]
        + elec_transition_matrix[2] * intermediary[29]
        - intermediary[26] * elec_transition_matrix[1]
        - intermediary[28] * elec_transition_matrix[3];
    output_matrix[19] = elec_transition_matrix[4] * intermediary[26]
        + elec_transition_matrix[6] * intermediary[28]
        + elec_transition_matrix[5] * intermediary[27]
        + elec_transition_matrix[7] * intermediary[29];
    output_matrix[20] = elec_transition_matrix[4] * intermediary[27]
        + elec_transition_matrix[6] * intermediary[29]
        - intermediary[26] * elec_transition_matrix[5]
        - intermediary[28] * elec_transition_matrix[7];
    output_matrix[21] = elec_transition_matrix[0] * intermediary[36]
        + elec_transition_matrix[2] * intermediary[38]
        + elec_transition_matrix[1] * intermediary[37]
        + elec_transition_matrix[3] * intermediary[39];
    output_matrix[22] = elec_transition_matrix[4] * intermediary[36]
        + elec_transition_matrix[6] * intermediary[38]
        + elec_transition_matrix[5] * intermediary[37]
        + elec_transition_matrix[7] * intermediary[39];
    output_matrix[23] = elec_transition_matrix[4] * intermediary[37]
        + elec_transition_matrix[6] * intermediary[39]
        - intermediary[36] * elec_transition_matrix[5]
        - intermediary[38] * elec_transition_matrix[7];
    output_matrix[24] = elec_transition_matrix[4] * intermediary[46]
        + elec_transition_matrix[6] * intermediary[48]
        + elec_transition_matrix[5] * intermediary[47]
        + elec_transition_matrix[7] * intermediary[49];
}

void SM_calibrate_and_reorder(_Complex float intermediary[25], float work_matrix[NB_FLOATS_PER_SM],
    const float* input_asm, const float* mag_calibration_matrices, const float* elec_calibration_matrices,
    float* output_asm, unsigned int start_indice, unsigned int stop_indice)
{
    DEBUG_CHECK_PTR(intermediary);
    DEBUG_CHECK_PTR(work_matrix);
    DEBUG_CHECK_PTR(mag_calibration_matrices);
    DEBUG_CHECK_PTR(elec_calibration_matrices);
    DEBUG_CHECK_PTR(output_asm);
    output_asm += (start_indice * NB_FLOATS_PER_SM);
    for (unsigned int frequency_offset = start_indice; frequency_offset < stop_indice;
         frequency_offset++)
    {
        extract_bin_vhdl_repr(input_asm, work_matrix, frequency_offset);
        Matrix_change_of_basis(intermediary, work_matrix, mag_calibration_matrices,
            elec_calibration_matrices, output_asm);

        mag_calibration_matrices += (3 * 3 * 2);
        elec_calibration_matrices += (2 * 2 * 2);
        output_asm += NB_FLOATS_PER_SM;
    }
}

void SM_calibrate_and_reorder_f0(float* input_asm, float* output_asm)
{
    DEBUG_CHECK_PTR(input_asm);
    DEBUG_CHECK_PTR(output_asm);
    static float work_matrix[NB_FLOATS_PER_SM];
    static _Complex float intermediary[25];
    SM_calibrate_and_reorder(intermediary, work_matrix, input_asm, mag_calibration_matrices_f0,
        elec_calibration_matrices_f0, output_asm, ASM_F0_INDICE_START,
        ASM_F0_INDICE_START + ASM_F0_KEEP_BINS);
}

void SM_calibrate_and_reorder_f1(float* input_asm, float* output_asm)
{
    DEBUG_CHECK_PTR(input_asm);
    DEBUG_CHECK_PTR(output_asm);
    static float work_matrix[NB_FLOATS_PER_SM];
    static _Complex float intermediary[25];
    SM_calibrate_and_reorder(intermediary, work_matrix, input_asm, mag_calibration_matrices_f1,
        elec_calibration_matrices_f1, output_asm, ASM_F1_INDICE_START,
        ASM_F1_INDICE_START + ASM_F1_KEEP_BINS);
}

void SM_calibrate_and_reorder_f2(float* input_asm, float* output_asm)
{
    DEBUG_CHECK_PTR(input_asm);
    DEBUG_CHECK_PTR(output_asm);
    static float work_matrix[NB_FLOATS_PER_SM];
    static _Complex float intermediary[25];
    SM_calibrate_and_reorder(intermediary, work_matrix, input_asm, mag_calibration_matrices_f2,
        elec_calibration_matrices_f2, output_asm, ASM_F2_INDICE_START,
        ASM_F2_INDICE_START + ASM_F2_KEEP_BINS);
}

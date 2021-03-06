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


inline void clear_spectral_matrix(float* spectral_matrix) __attribute__((always_inline));
void clear_spectral_matrix(float* spectral_matrix)
{
    for (int i = 0; i < TOTAL_SIZE_SM; i++)
    {
        spectral_matrix[i] = 0.f;
    }
}

void SM_average(float* averaged_spec_mat_NORM, float* averaged_spec_mat_SBM,
    ring_node* ring_node_tab[], unsigned int nbAverageNORM, unsigned int nbAverageSBM,
    asm_msg* msgForMATR, unsigned char channel)
{
    if (nbAverageNORM == 0)
    {
        clear_spectral_matrix(averaged_spec_mat_NORM);
        msgForMATR->coarseTimeNORM = ring_node_tab[0]->coarseTime;
        msgForMATR->fineTimeNORM = ring_node_tab[0]->fineTime;
    }
    if (nbAverageSBM == 0)
    {
        clear_spectral_matrix(averaged_spec_mat_SBM);
        for (int i = 0; i < TOTAL_SIZE_SM; i++)
        {
            averaged_spec_mat_SBM[i] = 0.f;
        }
        msgForMATR->coarseTimeSBM = ring_node_tab[0]->coarseTime;
        msgForMATR->fineTimeSBM = ring_node_tab[0]->fineTime;
    }
    unsigned int numberOfValidSM = 0U;
    for (int SM_index = 0; SM_index < NB_SM_BEFORE_AVF0_F1; SM_index++)
    {
        if (acquisitionTimeIsValid(
                ring_node_tab[SM_index]->coarseTime, ring_node_tab[SM_index]->fineTime, channel))
        {
            numberOfValidSM++;
            int* input_SM_ptr = (int*)(ring_node_tab[SM_index]->buffer_address);
            for (int i = 0; i < TOTAL_SIZE_SM; i++)
            {
                averaged_spec_mat_SBM[i] += (float)input_SM_ptr[i];
                averaged_spec_mat_NORM[i] += (float)input_SM_ptr[i];
            }
        }
    }
    if (nbAverageNORM == 0)
    {
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
    }
    else
    {
        msgForMATR->numberOfSMInASMNORM += numberOfValidSM;
    }
    if (nbAverageSBM == 0)
    {
        msgForMATR->numberOfSMInASMSBM = numberOfValidSM;
    }
    else
    {
        msgForMATR->numberOfSMInASMSBM += numberOfValidSM;
    }
}

void SM_average_old(float* averaged_spec_mat_NORM, float* averaged_spec_mat_SBM,
    ring_node* ring_node_tab[], unsigned int nbAverageNORM, unsigned int nbAverageSBM,
    asm_msg* msgForMATR, unsigned char channel)
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
    for (k = 0; k < NB_SM_BEFORE_AVF0_F1; k++)
    {
        isValid = acquisitionTimeIsValid(
            ring_node_tab[k]->coarseTime, ring_node_tab[k]->fineTime, channel);
        incomingSMIsValid[k] = isValid;
        numberOfValidSM = numberOfValidSM + isValid;
    }

    //************************
    // AVERAGE SPECTRAL MATRIX
    for (i = 0; i < TOTAL_SIZE_SM; i++)
    {
        sum = INIT_FLOAT;
        for (k = 0; k < NB_SM_BEFORE_AVF0_F1; k++)
        {
            if (incomingSMIsValid[k] == MATRIX_IS_NOT_POLLUTED)
            {
                // TODO check this, why looping over k and accumulating the
                // very same SM element
                sum = sum + ((int*)(ring_node_tab[k]->buffer_address))[i];
            }
        }

        if ((nbAverageNORM == 0) && (nbAverageSBM == 0))
        {
            averaged_spec_mat_NORM[i] = sum;
            averaged_spec_mat_SBM[i] = sum;
            msgForMATR->coarseTimeNORM = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeNORM = ring_node_tab[0]->fineTime;
            msgForMATR->coarseTimeSBM = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeSBM = ring_node_tab[0]->fineTime;
        }
        else if ((nbAverageNORM != 0) && (nbAverageSBM != 0))
        {
            averaged_spec_mat_NORM[i] = (averaged_spec_mat_NORM[i] + sum);
            averaged_spec_mat_SBM[i] = (averaged_spec_mat_SBM[i] + sum);
        }
        else if ((nbAverageNORM != 0) && (nbAverageSBM == 0))
        {
            averaged_spec_mat_NORM[i] = (averaged_spec_mat_NORM[i] + sum);
            averaged_spec_mat_SBM[i] = sum;
            msgForMATR->coarseTimeSBM = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeSBM = ring_node_tab[0]->fineTime;
        }
        else
        {
            averaged_spec_mat_NORM[i] = sum;
            averaged_spec_mat_SBM[i] = (averaged_spec_mat_SBM[i] + sum);
            msgForMATR->coarseTimeNORM = ring_node_tab[0]->coarseTime;
            msgForMATR->fineTimeNORM = ring_node_tab[0]->fineTime;
            //            PRINTF2("ERR *** in SM_average *** unexpected parameters %d %d\n",
            //            nbAverageNORM, nbAverageSBM)
        }
    }

    //*******************
    // UPDATE SM COUNTERS
    if ((nbAverageNORM == 0) && (nbAverageSBM == 0))
    {
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM = numberOfValidSM;
    }
    else if ((nbAverageNORM != 0) && (nbAverageSBM != 0))
    {
        msgForMATR->numberOfSMInASMNORM = msgForMATR->numberOfSMInASMNORM + numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM = msgForMATR->numberOfSMInASMSBM + numberOfValidSM;
    }
    else if ((nbAverageNORM != 0) && (nbAverageSBM == 0))
    {
        msgForMATR->numberOfSMInASMNORM = msgForMATR->numberOfSMInASMNORM + numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM = numberOfValidSM;
    }
    else
    {
        msgForMATR->numberOfSMInASMNORM = numberOfValidSM;
        msgForMATR->numberOfSMInASMSBM = msgForMATR->numberOfSMInASMSBM + numberOfValidSM;
    }
}

// TODO add unit test
void ASM_compress_divide_and_mask(const float* const averaged_spec_mat, float* compressed_spec_mat,
    const float divider, const unsigned char nbBinsCompressedMatrix,
    const unsigned char nbBinsToAverage, const unsigned char ASMIndexStart,
    const unsigned char channel)
{
    //*************
    // input format
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr127[0 .. 24]
    //************
    // compression
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr11[0 .. 24] => f0 NORM
    // matr0[0 .. 24]      matr1[0 .. 24]       .. matr22[0 .. 24] => f0 BURST, SBM

    // BUILD DATA
    // Just zero the whole compressed ASM
    // has to be done either way if divider==0. or to initialize average
    {
        for (int _ = 0; _ < nbBinsCompressedMatrix * NB_FLOATS_PER_SM; _++)
        {
            *compressed_spec_mat = 0.;
            compressed_spec_mat++;
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
                compressed_asm_ptr = compressed_spec_mat + compressed_frequency_bin;
                for (int asm_component = 0; asm_component < NB_FLOATS_PER_SM; asm_component++)
                {
                    *compressed_asm_ptr += (*input_asm_ptr * fBinMask);
                    compressed_asm_ptr++;
                }
                freq_offset++;
            }
        }
        compressed_asm_ptr = compressed_spec_mat;
        for (int _ = 0; _ < nbBinsCompressedMatrix * NB_FLOATS_PER_SM; _++)
        {
            *compressed_asm_ptr = *compressed_asm_ptr / (divider * nbBinsToAverage);
            compressed_asm_ptr++;
        }
    }
}

void ASM_divide(
    const float* averaged_spec_mat, float* averaged_spec_mat_normalized, const float divider)
{
    // BUILD DATA
    if (divider == 0.)
    {
        for (int _ = 0; _ < NB_FLOATS_PER_SM * NB_BINS_PER_SM; _++)
        {
            *averaged_spec_mat_normalized = 0.;
            averaged_spec_mat_normalized++;
        }
    }
    else
    {
        for (int _ = 0; _ < NB_FLOATS_PER_SM * NB_BINS_PER_SM; _++)
        {
            *averaged_spec_mat_normalized = *averaged_spec_mat / divider;
            averaged_spec_mat_normalized++;
            averaged_spec_mat++;
        }
    }
}


static inline float* spectral_matrix_re_element(
    float* spectral_matrix, int frequency, int line, int column) __attribute__((always_inline));
float* spectral_matrix_re_element(float* spectral_matrix, int frequency, int line, int column)
{
    const int indexes[5][5] = { { 0, 1, 3, 5, 7 }, { 1, 9, 10, 12, 14 }, { 3, 10, 16, 17, 19 },
        { 5, 12, 17, 21, 22 }, { 7, 14, 19, 22, 24 } };
    return spectral_matrix + (indexes[line][column] * NB_BINS_PER_SM)
        + (frequency * (line == column ? 1 : 2));
}

static inline float* spectral_matrix_im_element(
    float* spectral_matrix, int frequency, int line, int column) __attribute__((always_inline));
float* spectral_matrix_im_element(float* spectral_matrix, int frequency, int line, int column)
{
    return spectral_matrix_re_element(spectral_matrix, frequency, line, column) + 1;
}

static inline float* triangular_matrix_re_element(float* matrix, int line, int column)
    __attribute__((always_inline));
float* triangular_matrix_re_element(float* matrix, int line, int column)
{
    const int indexes[5][5] = { { 0, 1, 3, 5, 7 }, { 1, 9, 10, 12, 14 }, { 3, 10, 16, 17, 19 },
        { 5, 12, 17, 21, 22 }, { 7, 14, 19, 22, 24 } };
    return matrix + indexes[line][column];
}

static inline float* triangular_matrix_im_element(float* matrix, int line, int column)
    __attribute__((always_inline));
float* triangular_matrix_im_element(float* matrix, int line, int column)
{
    return triangular_matrix_re_element(matrix, line, column) + 1;
}

inline float* matrix_re_element(float* matrix, int line, int column, const int size)
    __attribute__((always_inline));
float* matrix_re_element(float* matrix, int line, int column, const int size)
{
    return matrix + 2 * (column + line * size);
}

static inline float* matrix_im_element(float* matrix, int line, int column, const int size)
    __attribute__((always_inline));
float* matrix_im_element(float* matrix, int line, int column, const int size)
{
    return matrix_re_element(matrix, line, column, size) + 1;
}


static inline _Complex float triangular_matrix_element(float* matrix, int line, int column)
    __attribute__((always_inline));
_Complex float triangular_matrix_element(float* matrix, int line, int column)
{
    _Complex float res;
    __real__ res = *triangular_matrix_re_element(matrix, line, column);
    if (line != column)
    {
        if (line > column)
        {
            __imag__ res = -*triangular_matrix_im_element(matrix, line, column);
        }
        else
        {
            __imag__ res = *triangular_matrix_im_element(matrix, line, column);
        }
    }
    else
        __imag__ res = 0.f;
    return res;
}

static inline _Complex float matrix_element(float* matrix, int line, int column, const int size)
    __attribute__((always_inline));
_Complex float matrix_element(float* matrix, int line, int column, const int size)
{
    _Complex float res;
    __real__ res = *matrix_re_element(matrix, line, column, size);
    __imag__ res = *matrix_im_element(matrix, line, column, size);
    return res;
}


// clang-format off
/*
 *   This function performs a specific 5x5 matrix change of basis with the folowing assumptions
                                                            T
          output_matrix                mag_transition_matrix               input_matrix                 mag_transition_matrix
               |                              |                                 |                              |
               v                              v                                 v                              v
 | |SM11|  SM12  SM13  SM14  SM15  |   | B11 B21 B31   0   0  |   | |SM11|  SM12  SM13  SM14  SM15  |   | B11 B12 B13   0   0  |
 |        |SM22| SM23  SM24  SM25  |   | B12 B22 B32   0   0  |   |        |SM22| SM23  SM24  SM25  |   | B21 B22 B23   0   0  |
 |              |SM33| SM34  SM35  | = | B13 B23 B33   0   0  | X |              |SM33| SM34  SM35  | X | B31 B32 B33   0   0  |
 |                    |SM44| SM45  |   |  0   0   0   E11 E21 |   |                    |SM44| SM45  |   |  0   0   0   E11 E12 |
 |                          |SM55| |   |  0   0   0   E12 E22 |   |                          |SM55| |   |  0   0   0   E21 E22 |
                                                         ^                                                                ^
                                                         |          T                                                     |
                                              elec_transition_matrix                                             elec_transition_matrix
Where each matrix product is done sequencialy for mag and elec transition matrices
For a more readable but equivalent impl jump bellow to Matrix_change_of_basis_old
*/
// clang-format on

void Matrix_change_of_basis(float* input_matrix, float* mag_transition_matrix,
    float* elec_transition_matrix, float* output_matrix)
{

    // does  transpose(P)xMxP see https://en.wikipedia.org/wiki/Change_of_basis
    _Complex float intermediary[25] = { 0.f };
    _Complex float* intermediary_ptr = intermediary;
    // first part: intermediary = transpose(transition_matrix) x input_matrix
    for (int line = 0; line < 3; line++)
    {
        for (int column = 0; column < 5; column++)
        {
            float* mag_ptr = mag_transition_matrix + (line * 2);
            _Complex float product = 0.f;
            for (int k = 0; k < 3; k++)
            {
                _Complex float mag;
                __real__ mag = *mag_ptr;
                __imag__ mag = *(mag_ptr + 1);
                mag_ptr += 6;
                product += mag * triangular_matrix_element(input_matrix, k, column);
            }
            *intermediary_ptr = product;
            intermediary_ptr++;
        }
    }
    intermediary_ptr = intermediary + 3 * 5;
    for (int line = 0; line < 2; line++)
    {
        for (int column = 0; column < 5; column++)
        {
            float* elec_ptr = elec_transition_matrix + (line * 2);
            _Complex float product = 0.f;
            for (int k = 0; k < 2; k++)
            {
                _Complex float elec;
                __real__ elec = *elec_ptr;
                __imag__ elec = *(elec_ptr + 1);
                elec_ptr += 4;
                product += elec * triangular_matrix_element(input_matrix, k + 3, column);
            }
            *intermediary_ptr = product;
            intermediary_ptr++;
        }
    }

    // second part: output_matrix = intermediary x transition_matrix
    for (int line = 0; line < 3; line++)
    {
        for (int column = line; column < 3; column++)
        {
            intermediary_ptr = intermediary + line * 5;
            float* mag_ptr = mag_transition_matrix + (column * 2);
            _Complex float product = 0.f;
            for (int k = 0; k < 3; k++)
            {
                _Complex float mag;
                __real__ mag = *mag_ptr;
                __imag__ mag = *(mag_ptr + 1);
                mag_ptr += 6;
                product += *intermediary_ptr * mag;
                intermediary_ptr++;
            }
            *triangular_matrix_re_element(output_matrix, line, column) = __real__ product;
            if (line != column)
            {
                *triangular_matrix_im_element(output_matrix, line, column) = __imag__ product;
            }
        }
    }
    for (int column = 3; column < 5; column++)
    {
        for (int line = 0; line <= column; line++)
        {
            intermediary_ptr = intermediary + line * 5 + 3;
            float* elec_ptr = elec_transition_matrix + ((column - 3) * 2);
            _Complex float product = 0.f;
            for (int k = 0; k < 2; k++)
            {
                _Complex float elec;
                __real__ elec = *elec_ptr;
                __imag__ elec = *(elec_ptr + 1);
                elec_ptr += 4;
                product += *intermediary_ptr * elec;
                intermediary_ptr++;
            }
            *triangular_matrix_re_element(output_matrix, line, column) = __real__ product;
            if (line != column)
            {
                *triangular_matrix_im_element(output_matrix, line, column) = __imag__ product;
            }
        }
    }
}

void SM_calibrate_and_reorder(float* input_asm, float* mag_calibration_matrices,
    float* elec_calibration_matrices, float* output_asm)
{
    float work_matrix[NB_FLOATS_PER_SM];
    for (int frequency_offset = 0; frequency_offset < NB_BINS_PER_SM; frequency_offset++)
    {
        float* out_ptr = work_matrix;
        float* in_ptr = input_asm + frequency_offset;
        for (unsigned int line = 0; line < 5; line++)
        {
            for (unsigned int column = line; column < 5; column++)
            {
                *out_ptr = *in_ptr;
                out_ptr += 1;
                if (line != column) // imaginary part
                {
                    *out_ptr = *(in_ptr + 1);
                    in_ptr += 2 * NB_BINS_PER_SM;
                    out_ptr += 1;
                }
                else
                {
                    in_ptr += NB_BINS_PER_SM + frequency_offset;
                }
            }
            in_ptr -= frequency_offset;
        }

        Matrix_change_of_basis(
            work_matrix, mag_calibration_matrices, elec_calibration_matrices, output_asm);
        mag_calibration_matrices += 3 * 3 * 2;
        elec_calibration_matrices += 2 * 2 * 2;
        output_asm += NB_FLOATS_PER_SM;
    }
}


#ifdef ENABLE_BENCHMARKS

void Matrix_change_of_basis_old(float* input_matrix, float* mag_transition_matrix,
    float* elec_transition_matrix, float* output_matrix)
{

    // does  transpose(P)xMxP see https://en.wikipedia.org/wiki/Change_of_basis
    _Complex float intermediary[5][5] = { { 0.f } };
    // first part: intermediary = transpose(transition_matrix) x input_matrix
    for (int line = 0; line < 3; line++)
    {
        for (int column = 0; column < 5; column++)
        {
            _Complex float product = 0.f;
            for (int k = 0; k < 3; k++)
            {
                product += matrix_element(mag_transition_matrix, k, line, 3)
                    * triangular_matrix_element(input_matrix, k, column);
            }
            intermediary[line][column] = product;
        }
    }

    for (int line = 0; line < 2; line++)
    {
        for (int column = 0; column < 5; column++)
        {
            _Complex float product = 0.f;
            for (int k = 0; k < 2; k++)
            {
                product += matrix_element(elec_transition_matrix, k, line, 2)
                    * triangular_matrix_element(input_matrix, k + 3, column);
            }
            intermediary[line + 3][column] = product;
        }
    }

    // second part: output_matrix = intermediary x transition_matrix
    for (int line = 0; line < 3; line++)
    {
        for (int column = line; column < 3; column++)
        {
            _Complex float product = 0.f;
            for (int k = 0; k < 3; k++)
            {
                product
                    += intermediary[line][k] * matrix_element(mag_transition_matrix, k, column, 3);
            }
            *triangular_matrix_re_element(output_matrix, line, column) = __real__ product;
            if (line != column)
            {
                *triangular_matrix_im_element(output_matrix, line, column) = __imag__ product;
            }
        }
    }
    for (int column = 3; column < 5; column++)
    {
        for (int line = 0; line <= column; line++)
        {
            _Complex float product = 0.f;
            for (int k = 0; k < 2; k++)
            {
                product += intermediary[line][k + 3]
                    * matrix_element(elec_transition_matrix, k, column - 3, 2);
            }
            *triangular_matrix_re_element(output_matrix, line, column) = __real__ product;
            if (line != column)
            {
                *triangular_matrix_im_element(output_matrix, line, column) = __imag__ product;
            }
        }
    }
}

#endif

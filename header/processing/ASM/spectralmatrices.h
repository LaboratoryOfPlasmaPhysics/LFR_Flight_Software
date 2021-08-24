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
#pragma once
#include "lfr_common_headers/fsw_params.h"
#include <rtems.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct ring_node_asm
    {
        struct ring_node_asm* next;
        float matrix[TOTAL_SIZE_SM];
        unsigned int status;
    } ring_node_asm;

    typedef struct asm_msg
    {
        ring_node_asm* norm;
        ring_node_asm* burst_sbm;
        rtems_event_set event;
        unsigned int coarseTimeNORM;
        unsigned int fineTimeNORM;
        unsigned int coarseTimeSBM;
        unsigned int fineTimeSBM;
        unsigned int numberOfSMInASMNORM;
        unsigned int numberOfSMInASMSBM;
    } asm_msg;

    void Matrix_change_of_basis(_Complex float intermediary[25], float* input_matrix, float* mag_transition_matrix,
        float* elec_transition_matrix, float* output_matrix);

    void SM_calibrate_and_reorder_f0(float* input_asm, float* mag_calibration_matrices,
        float* elec_calibration_matrices, float* output_asm);
    void SM_calibrate_and_reorder_f1(float* input_asm, float* mag_calibration_matrices,
        float* elec_calibration_matrices, float* output_asm);
    void SM_calibrate_and_reorder_f2(float* input_asm, float* mag_calibration_matrices,
        float* elec_calibration_matrices, float* output_asm);

    void SM_average(float* averaged_spec_mat_NORM, float* averaged_spec_mat_SBM,
        ring_node* ring_node_tab[], unsigned int nbAverageNORM, unsigned int nbAverageSBM,
        asm_msg* msgForMATR, unsigned char channel);

    void ASM_compress_divide_and_mask(const float* const averaged_spec_mat,
        float* compressed_spec_mat, const float divider, const unsigned char nbBinsCompressedMatrix,
        const unsigned char nbBinsToAverage, const unsigned char ASMIndexStart,
        const unsigned char channel);

    void ASM_divide(
        const float* averaged_spec_mat, float* averaged_spec_mat_normalized, const float divider);

#ifdef __cplusplus
}
#endif

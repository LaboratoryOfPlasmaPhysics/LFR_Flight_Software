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
#include "fsw_params.h"
#include <rtems.h>

typedef struct ring_node_asm {
  struct ring_node_asm *next;
  float matrix[TOTAL_SIZE_SM];
  unsigned int status;
} ring_node_asm;

typedef struct asm_msg {
  ring_node_asm *norm;
  ring_node_asm *burst_sbm;
  rtems_event_set event;
  unsigned int coarseTimeNORM;
  unsigned int fineTimeNORM;
  unsigned int coarseTimeSBM;
  unsigned int fineTimeSBM;
  unsigned int numberOfSMInASMNORM;
  unsigned int numberOfSMInASMSBM;
} asm_msg;

void ASM_patch(float *inputASM, float *outputASM);

void SM_average(float *averaged_spec_mat_NORM, float *averaged_spec_mat_SBM,
                ring_node *ring_node_tab[], unsigned int nbAverageNORM,
                unsigned int nbAverageSBM, asm_msg *msgForMATR,
                unsigned char channel);

void ASM_compress_reorganize_and_divide_mask(
    float *averaged_spec_mat, float *compressed_spec_mat, float divider,
    unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage,
    unsigned char ASMIndexStart, unsigned char channel);

void ASM_convert(volatile float *input_matrix, char *output_matrix);

void ASM_reorganize_and_divide(float *averaged_spec_mat,
                               float *averaged_spec_mat_reorganized,
                               const float divider);

void ASM_compress_reorganize_and_divide(float *averaged_spec_mat,
                                        float *compressed_spec_mat,
                                        float divider,
                                        unsigned char nbBinsCompressedMatrix,
                                        unsigned char nbBinsToAverage,
                                        unsigned char ASMIndexStart);

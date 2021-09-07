// clang-format off
// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
// version 1.0: 31/07/2013
// version 1.1: 02/04/2014
// version 1.2: 30/04/2014
// version 1.3: 02/05/2014
// version 1.4: 16/05/2014
// version 1.5: 20/05/2014
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB + indices [0,2] <=> [1,3])
// version 1.8: 02/02/2015 (gestion des divisions par zéro)
// In the frame of RPW LFR Sofware ICD Issue3 Rev6 (27/01/2015) => R3 FSW
// version 2.0: 19/06/2015
// version 2.1: 22/06/2015 (modifs de Paul)
// version 2.2: 23/06/2015 (modifs de l'ordre de déclaration/définition de init_k_coefficients dans basic_parameters.c ... + maintien des declarations dans le .h)
// version 2.3: 01/07/2015 (affectation initiale des octets 7 et 9 dans les BP1 corrigée ...)
// version 2.4: 05/10/2018 (mise en conformité LOGISCOPE)
// version 2.5: 09/10/2018 (dans main.c #include "basic_parameters_utilities.h" est changé par les déclarations extern correspondantes ...!
//                          + delta mise en conformité LOGISCOPE)
// clang-format on
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
/*--                  Author : Thomas Chust
--                   Contact : Thomas Chust
--                      Mail : thomas.chust@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

#ifndef BASIC_PARAMETERS_H_INCLUDED
#define BASIC_PARAMETERS_H_INCLUDED

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "basic_parameters_params.h"


#ifndef LFR_BIG_ENDIAN
    #ifndef LFR_LITTLE_ENDIAN
        #error "you must either define LFR_BIG_ENDIAN or LFR_LITTLE_ENDIAN"
    #endif
#endif


void BP1_set(float* compressed_spec_mat, float* k_coeff_intercalib,
    uint8_t nb_bins_compressed_spec_mat, uint8_t* lfr_bp1);

void compute_BP1(const float* const spectral_matrices, const uint8_t spectral_matrices_count,
    uint8_t* bp1_buffer);

void BP2_set(float* compressed_spec_mat, uint8_t nb_bins_compressed_spec_mat, uint8_t* lfr_bp2);

void compute_BP2(const float* const spectral_matrices, const uint8_t spectral_matrices_count,
    uint8_t* bp2_buffer);


#endif // BASIC_PARAMETERS_H_INCLUDED

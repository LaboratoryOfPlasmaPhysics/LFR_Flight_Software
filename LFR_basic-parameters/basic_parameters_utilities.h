// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
// version 1.6: 19/12/2014
// version 1.7: 15/01/2015 (modifs de Paul + correction erreurs qui se compensaient (LSB <=> MSB +
// indices [0,2] <=> [1,3]) version 1.8: 02/02/2015 (gestion des divisions par zéro) In the frame of
// RPW LFR Sofware ICD Issue3 Rev6 (27/01/2015) => R3 FSW version 2.0: 19/06/2015 version 2.1:
// 22/06/2015 (modifs de Paul) version 2.2: 23/06/2015 (modifs de l'ordre de déclaration/définition
// de init_k_coefficients dans basic_parameters.c ... + maintien des declarations dans le .h)
// version 2.3: 01/07/2015 (affectation initiale des octets 7 et 9 dans les BP1 corrigée ...)
// version 2.4: 05/10/2018 (mise en conformité LOGISCOPE)
// version 2.5: 09/10/2018 (dans main.c #include "basic_parameters_utilities.h" est changé par les
// déclarations extern correspondantes ...!
//                          + delta mise en conformité LOGISCOPE)

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

#ifndef BASIC_PARAMETERS_UTILITIES_H
#define BASIC_PARAMETERS_UTILITIES_H

#include <malloc.h>
#include <stdio.h>

#include "basic_parameters_params.h"

float compressed_spectral_matrix_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX]
    = { 0.0 };
float k_coefficients_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_K_COEFF_PER_BIN] = { 0.0 };
float k_coefficients_f1[NB_BINS_COMPRESSED_MATRIX_f1 * NB_K_COEFF_PER_BIN] = { 0.0 };
float k_coefficients_f2[NB_BINS_COMPRESSED_MATRIX_f2 * NB_K_COEFF_PER_BIN] = { 0.0 };

unsigned char LFR_BP1_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_BYTES_BP1] = { 0 };
unsigned char LFR_BP2_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_BYTES_BP2] = { 0 };

#endif // BASIC_PARAMETERS_UTILITIES_H

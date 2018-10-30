// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
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
// version 2.4: 05/10/2018 (added GPL headers)
// version 2.5: 09/10/2018 (dans main.c #include "basic_parameters_utilities.h" est changé par les déclarations extern correspondantes ...!
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

#ifndef BASIC_PARAMETERS_PARAMS_H
#define BASIC_PARAMETERS_PARAMS_H

#define NB_VALUES_PER_SPECTRAL_MATRIX 25

#define NB_BINS_COMPRESSED_MATRIX_f0 11
#define NB_BINS_COMPRESSED_MATRIX_f1 13
#define NB_BINS_COMPRESSED_MATRIX_f2 12

#define NB_BYTES_BP1 11
#define NB_BYTES_BP2 30

//********************************************
// K-COEFFICIENTS FOR ONBOARD INTERCALIBRATION

#define NB_K_COEFF_PER_BIN 32

#define K44_PE    0
#define K55_PE    1
#define K45_PE_RE 2
#define K45_PE_IM 3

#define K14_SX_RE 4
#define K14_SX_IM 5
#define K15_SX_RE 6
#define K15_SX_IM 7
#define K24_SX_RE 8
#define K24_SX_IM 9
#define K25_SX_RE 10
#define K25_SX_IM 11
#define K34_SX_RE 12
#define K34_SX_IM 13
#define K35_SX_RE 14
#define K35_SX_IM 15

#define K24_NY_RE 16
#define K24_NY_IM 17
#define K25_NY_RE 18
#define K25_NY_IM 19
#define K34_NY_RE 20
#define K34_NY_IM 21
#define K35_NY_RE 22
#define K35_NY_IM 23

#define K24_NZ_RE 24
#define K24_NZ_IM 25
#define K25_NZ_RE 26
#define K25_NZ_IM 27
#define K34_NZ_RE 28
#define K34_NZ_IM 29
#define K35_NZ_RE 30
#define K35_NZ_IM 31

#endif // BASIC_PARAMETERS_PARAMS_H

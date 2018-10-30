// In the frame of RPW LFR Sofware ICD Issue1 Rev8 (05/07/2013) => R2 FSW
// version 1.O: 31/07/2013
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


#include <stdio.h>

#include "file_utilities.h"
#include "basic_parameters.h"

extern float compressed_spectral_matrix_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX];
extern float k_coefficients_f0[NB_BINS_COMPRESSED_MATRIX_f0 * NB_K_COEFF_PER_BIN];
extern float k_coefficients_f1[NB_BINS_COMPRESSED_MATRIX_f1 * NB_K_COEFF_PER_BIN];
extern float k_coefficients_f2[NB_BINS_COMPRESSED_MATRIX_f2 * NB_K_COEFF_PER_BIN];

extern unsigned char LFR_BP1_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP1];
extern unsigned char LFR_BP2_f0[NB_BINS_COMPRESSED_MATRIX_f0*NB_BYTES_BP2];

int main(void)
{
    const char *filename;
    printf("Hello World!\n\n");

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    //LSB FIRST
    printf("The multi-byte quantities are laid out in a LSB FIRST (little endian) fashion \n\n");
#endif

#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    //MSB FIRST
    printf("The multi-byte quantities are laid out in a MSB FIRST (big endian) fashion\n\n");
#endif

    //filename="/WIN/Users/chust/DD CHUST/Missions/Solar Orbiter/LFR/Prog C/tests bp Paul/tests7/sm_test2_R3.dat";
    filename="/home/chust/DD pc-p-chust/Missions/Solar Orbiter/LFR/Prog C/tests bp Paul/tests7/sm_test2_R3.dat";

    lecture_file_sm(filename);

    printf("\n");

    init_k_coefficients(k_coefficients_f0, NB_BINS_COMPRESSED_MATRIX_f0);
    init_k_coefficients(k_coefficients_f1, NB_BINS_COMPRESSED_MATRIX_f1);
    init_k_coefficients(k_coefficients_f2, NB_BINS_COMPRESSED_MATRIX_f2);

    printf("\n\n");

    BP1_set(compressed_spectral_matrix_f0, k_coefficients_f0, NB_BINS_COMPRESSED_MATRIX_f0, LFR_BP1_f0);

    printf("\n");

    BP2_set(compressed_spectral_matrix_f0, NB_BINS_COMPRESSED_MATRIX_f0, LFR_BP2_f0);

    return 0;
}



















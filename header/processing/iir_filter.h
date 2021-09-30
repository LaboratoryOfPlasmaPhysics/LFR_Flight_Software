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
#include <stdint.h>

#define B00 196
#define B01 196
#define B02 0
#define B10 131
#define B11 -244
#define B12 131
#define B20 161
#define B21 -314
#define B22 161

#define A00 1
#define A01 -925
#define A02 0
#define A10 1
#define A11 -947
#define A12 439
#define A20 1
#define A21 -993
#define A22 486

#define GAIN_B0 12
#define GAIN_B1 11
#define GAIN_B2 10

#define GAIN_A0 10
#define GAIN_A1 9
#define GAIN_A2 9

#define NB_COEFFS 3
#define COEFF0    0
#define COEFF1    1
#define COEFF2    2


typedef struct filter_ctx
{
    int W[NB_COEFFS][NB_COEFFS];
} filter_ctx;


/**
 * @brief filter is a Direct-Form-II filter implementation, mostly used to filter electric field for
 * HK
 * @param x, new sample
 * @param ctx, filter context, used to store previous input and output samples
 * @return a new filtered sample
 */
inline int filter(int x, filter_ctx* ctx)
{
    static const int b[NB_COEFFS][NB_COEFFS]
        = { { B00, B01, B02 }, { B10, B11, B12 }, { B20, B21, B22 } };
    static const int a[NB_COEFFS][NB_COEFFS]
        = { { A00, A01, A02 }, { A10, A11, A12 }, { A20, A21, A22 } };
    static const int b_gain[NB_COEFFS] = { GAIN_B0, GAIN_B1, GAIN_B2 };
    static const int a_gain[NB_COEFFS] = { GAIN_A0, GAIN_A1, GAIN_A2 };

    // Direct-Form-II
    for (int i = 0; i < NB_COEFFS; i++)
    {
        x = x << a_gain[i];
        int32_t W = (x - (a[i][COEFF1] * ctx->W[i][COEFF0]) - (a[i][COEFF2] * ctx->W[i][COEFF1]))
            >> a_gain[i];
        x = (b[i][COEFF0] * W) + (b[i][COEFF1] * ctx->W[i][COEFF0])
            + (b[i][COEFF2] * ctx->W[i][COEFF1]);
        x = x >> b_gain[i];
        ctx->W[i][1] = ctx->W[i][0];
        ctx->W[i][0] = W;
    }
    return x;
}

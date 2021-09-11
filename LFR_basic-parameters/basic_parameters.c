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
// version 2.4: 05/10/2018 (added GPL headers)
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

#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "basic_parameters_params.h"
#include "custom_floats.h"
#include "processing/ASM/spectralmatrices.h"


inline const float* next_matrix(const float* const spectral_matrix) __attribute__((always_inline));
const float* next_matrix(const float* const spectral_matrix)
{
    return spectral_matrix + NB_FLOATS_PER_SM;
}

inline float elec_power_spectrum_density(const float* const spectral_matrix)
    __attribute__((always_inline));
float elec_power_spectrum_density(const float* const spectral_matrix)
{
    return spectral_matrix[ASM_COMP_E1E1] + spectral_matrix[ASM_COMP_E2E2];
}

inline float mag_power_spectrum_density(const float* const spectral_matrix)
    __attribute__((always_inline));
inline float mag_power_spectrum_density(const float* const spectral_matrix)
{
    return spectral_matrix[ASM_COMP_B1B1] + spectral_matrix[ASM_COMP_B2B2]
        + spectral_matrix[ASM_COMP_B3B3];
}

typedef struct
{
    float x;
    float y;
    float z;
    float ab; // Ellipse a.b product
} normal_wave_vector_t;

inline float square(const float value) __attribute__((always_inline));
float square(const float value)
{
    return value * value;
}

inline normal_wave_vector_t normal_wave_vector(const float* const spectral_matrix)
    __attribute__((always_inline));
normal_wave_vector_t normal_wave_vector(const float* const spectral_matrix)
{
    const float ab = sqrtf(square(spectral_matrix[ASM_COMP_B1B2_imag])
        + square(spectral_matrix[ASM_COMP_B1B3_imag])
        + square(spectral_matrix[ASM_COMP_B2B3_imag]));
    if (ab != 0.)
    {
        normal_wave_vector_t nvec = { .x = spectral_matrix[ASM_COMP_B2B3_imag] / ab,
            .y = -spectral_matrix[ASM_COMP_B1B3_imag] / ab,
            .z = spectral_matrix[ASM_COMP_B1B2_imag] / ab,
            .ab = ab };
        return nvec;
    }
    else
    {
        normal_wave_vector_t nvec = { .x = 0., .y = 0., .z = 0., .ab = 0. };
        return nvec;
    }
}

inline float wave_ellipticity_estimator(const float mag_PSD, const float nvec_denom)
    __attribute__((always_inline));
float wave_ellipticity_estimator(const float mag_PSD, const float nvec_denom)
{
    if (mag_PSD != 0.f)
        return 2.f * nvec_denom / mag_PSD;
    else
        return 0.;
}

inline float degree_of_polarization(const float mag_PSD, const float* const spectral_matrix)
    __attribute__((always_inline));
float degree_of_polarization(const float mag_PSD, const float* const spectral_matrix)
{
    const float B_square_trace = square(spectral_matrix[ASM_COMP_B1B1])
        + square(spectral_matrix[ASM_COMP_B2B2]) + square(spectral_matrix[ASM_COMP_B3B3])
        + 2.f
            * (square(spectral_matrix[ASM_COMP_B1B2]) + square(spectral_matrix[ASM_COMP_B1B2_imag])
                + square(spectral_matrix[ASM_COMP_B1B3])
                + square(spectral_matrix[ASM_COMP_B1B3_imag])
                + square(spectral_matrix[ASM_COMP_B2B3])
                + square(spectral_matrix[ASM_COMP_B2B3_imag]));
    const float square_B_trace = square(mag_PSD);
    if (square_B_trace != 0.)
        return sqrtf((3.f * B_square_trace - square_B_trace) / (2.f * square_B_trace));
    else
        return 0.f;
}

inline compressed_complex X_poynting_vector(const float* const spectral_matrix)
    __attribute__((always_inline));
compressed_complex X_poynting_vector(const float* const spectral_matrix)
{
    // E1B3 - E2B2
    compressed_complex X_PV;
    X_PV.real = spectral_matrix[ASM_COMP_B3E1] - spectral_matrix[ASM_COMP_B2E2];
    const float imag = spectral_matrix[ASM_COMP_B3E1_imag] + spectral_matrix[ASM_COMP_B2E2_imag];
    X_PV.arg = fabs(imag) > fabs(X_PV.real);
    return X_PV;
}

inline float modulus(const float a, const float b) __attribute__((always_inline));
float modulus(const float a, const float b)
{
    return sqrtf(square(a) + square(b));
}

inline float cplx_modulus(const _Complex float value) __attribute__((always_inline));
float cplx_modulus(const _Complex float value)
{
    return modulus(__real__ value, __imag__ value);
}


inline compressed_complex phase_velocity_estimator(const float* const spectral_matrix,
    const normal_wave_vector_t nvec) __attribute__((always_inline));
compressed_complex phase_velocity_estimator(
    const float* const spectral_matrix, const normal_wave_vector_t nvec)
{
    /*
    VPHI = abs(NEBX) * sign( Re[NEBX] ) / BXBX
with:
    NEBX = nY<EZBX*>/rho_EZBX  - nZ<EYBX*>/rho_EYBX = n2<E2B1*>/rho_E2B1 - n3<E1B1*>/rho_E1B1
    rho_E2B1 = |<E2B1*>| / sqrt(<E2E2*><B1B1*>)
    rho_E1B1 = |<E1B1*>| / sqrt(<E1E1*><B1B1*>)
    BXBX = <BXBX*> = <B1B1*>
*/
    compressed_complex vphi = { .real = 0, .arg = 0 };
    const float sqrt_E2E2B1B1
        = sqrtf(spectral_matrix[ASM_COMP_E2E2] * spectral_matrix[ASM_COMP_B1B1]);
    const float sqrt_E1E1B1B1
        = sqrtf(spectral_matrix[ASM_COMP_E1E1] * spectral_matrix[ASM_COMP_B1B1]);
    const float mod_E2B1
        = modulus(spectral_matrix[ASM_COMP_B1E2], spectral_matrix[ASM_COMP_B1E2_imag]);
    const float mod_E1B1
        = modulus(spectral_matrix[ASM_COMP_B1E1], spectral_matrix[ASM_COMP_B1E1_imag]);
    _Complex float NEBX;
    if (mod_E2B1 != 0. && mod_E1B1 != 0.)
    {
        __real__ NEBX = nvec.y * spectral_matrix[ASM_COMP_B1E2] * sqrt_E2E2B1B1 / mod_E2B1
            - nvec.z * spectral_matrix[ASM_COMP_B1E1] * sqrt_E1E1B1B1 / mod_E1B1;

        __imag__ NEBX = nvec.z * spectral_matrix[ASM_COMP_B1E1_imag] * sqrt_E1E1B1B1 / mod_E1B1
            - nvec.y * spectral_matrix[ASM_COMP_B1E2_imag] * sqrt_E2E2B1B1 / mod_E2B1;

        const float BXBX = spectral_matrix[ASM_COMP_B1B1];
        if (BXBX != 0.f)
        {
            vphi.real = cplx_modulus(NEBX) / BXBX;
            if (__real__ NEBX < 0.)
            {
                vphi.real = -vphi.real;
            }
        }
        else
        {
            if (__real__ NEBX >= 0.)
                vphi.real = 1.e+20;
            else
                vphi.real = -1.e+20;
        }
        vphi.arg = fabs(__imag__ NEBX) > fabs(__real__ NEBX);
    }
    return vphi;
}

inline uint8_t* encode_nvec_z_ellip_dop(const float nvec_z, const float ellipticity,
    const float DOP, uint8_t* const bp_buffer_frame) __attribute__((always_inline));
uint8_t* encode_nvec_z_ellip_dop(
    const float nvec_z, const float ellipticity, const float DOP, uint8_t* const bp_buffer_frame)
{
    const str_float_t z = { .value = nvec_z };
#ifdef LFR_BIG_ENDIAN
    union __attribute__((__packed__))
    {
        uint8_t value;
        struct __attribute__((__packed__))
        {
            uint8_t nvec_z_sign : 1;
            uint8_t ellipticity : 4;
            uint8_t DOP : 3;
        } str;
    } result;
#endif
#ifdef LFR_LITTLE_ENDIAN
    union __attribute__((__packed__))
    {
        uint8_t value;
        struct __attribute__((__packed__))
        {
            uint8_t DOP : 3;
            uint8_t ellipticity : 4;
            uint8_t nvec_z_sign : 1;
        } str;
    } result;
#endif
    result.str.nvec_z_sign = z.str.sign;
    result.str.ellipticity = (uint8_t)(ellipticity * 15.f + 0.5f);
    result.str.DOP = (uint8_t)(DOP * 7.f + 0.5f);
    bp_buffer_frame[0] = result.value;
    return bp_buffer_frame + 1;
}

inline uint8_t* encode_uint16_t(const uint16_t value, uint8_t* const bp1_buffer_frame)
    __attribute__((always_inline));
uint8_t* encode_uint16_t(const uint16_t value, uint8_t* const bp1_buffer_frame)
{
    const str_uint16_t value_split = { .value = value };
    bp1_buffer_frame[0] = value_split.str.MSB;
    bp1_buffer_frame[1] = value_split.str.LSB;
    return bp1_buffer_frame + 2;
}


inline uint8_t* encode_float_uint8_t(float value, uint8_t* const bp_buffer_frame)
    __attribute__((always_inline));
uint8_t* encode_float_uint8_t(float value, uint8_t* const bp_buffer_frame)
{
    bp_buffer_frame[0] = (uint8_t)(value * 127.5 + 128);
    return bp_buffer_frame + 1;
}

inline uint8_t* encode_BP1(const float mag_PSD, const float elec_PSD,
    const normal_wave_vector_t nvec, const float ellipticity, const float DOP,
    const compressed_complex X_PV, const compressed_complex VPHI, uint8_t* bp1_buffer_frame)
    __attribute__((always_inline));
uint8_t* encode_BP1(const float mag_PSD, const float elec_PSD, const normal_wave_vector_t nvec,
    const float ellipticity, const float DOP, const compressed_complex X_PV, const compressed_complex VPHI,
    uint8_t* bp1_buffer_frame)
{
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_6_10(elec_PSD), bp1_buffer_frame);
    }
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_6_10(mag_PSD), bp1_buffer_frame);
    }
    {
        bp1_buffer_frame = encode_float_uint8_t(nvec.x, bp1_buffer_frame);
        bp1_buffer_frame = encode_float_uint8_t(nvec.y, bp1_buffer_frame);
        bp1_buffer_frame = encode_nvec_z_ellip_dop(nvec.z, ellipticity, DOP, bp1_buffer_frame);
    }
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_1_1_6_8(X_PV), bp1_buffer_frame);
    }
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_1_1_6_8(VPHI), bp1_buffer_frame);
    }
    return bp1_buffer_frame;
}

void compute_BP1(const float* const spectral_matrices, const uint8_t spectral_matrices_count,
    uint8_t* bp1_buffer)
{

    const float* spectral_matrix_ptr = spectral_matrices;
    uint8_t* bp1_buffer_frame = bp1_buffer;
    for (int i = 0; i < spectral_matrices_count; i++)
    {
        const float mag_PSD = mag_power_spectrum_density(spectral_matrix_ptr);
        const float elec_PSD = elec_power_spectrum_density(spectral_matrix_ptr);
        const normal_wave_vector_t nvec = normal_wave_vector(spectral_matrix_ptr);
        const float ellipticity = wave_ellipticity_estimator(mag_PSD, nvec.ab);
        const float DOP = degree_of_polarization(mag_PSD, spectral_matrix_ptr);
        const compressed_complex X_PV = X_poynting_vector(spectral_matrix_ptr);
        const compressed_complex VPHI = phase_velocity_estimator(spectral_matrix_ptr, nvec);
        bp1_buffer_frame
            = encode_BP1(mag_PSD, elec_PSD, nvec, ellipticity, DOP, X_PV, VPHI, bp1_buffer_frame);
        spectral_matrix_ptr = next_matrix(spectral_matrix_ptr);
    }
}


inline uint8_t* _compute_BP2_cross_component(uint8_t* const bp2_frame, float auto1, float auto2,
    float cross_re, float cross_imag) __attribute__((always_inline));
uint8_t* _compute_BP2_cross_component(
    uint8_t* const bp2_frame, float auto1, float auto2, float cross_re, float cross_imag)
{
    const float aux = sqrtf(auto1 * auto2);
    encode_float_uint8_t(cross_re / aux, bp2_frame);
    encode_float_uint8_t(cross_imag / aux, bp2_frame + 10);
    return bp2_frame + 1;
}

void compute_BP2(const float* const spectral_matrices, const uint8_t spectral_matrices_count,
    uint8_t* bp2_buffer)
{
    const float* sm_ptr = spectral_matrices;
    uint8_t* bp2_frame = bp2_buffer;
    for (int i = 0; i < spectral_matrices_count; i++)
    {
        bp2_frame = encode_uint16_t(to_custom_float_6_10(sm_ptr[ASM_COMP_B1B1]), bp2_frame);
        bp2_frame = encode_uint16_t(to_custom_float_6_10(sm_ptr[ASM_COMP_B2B2]), bp2_frame);
        bp2_frame = encode_uint16_t(to_custom_float_6_10(sm_ptr[ASM_COMP_B3B3]), bp2_frame);
        bp2_frame = encode_uint16_t(to_custom_float_6_10(sm_ptr[ASM_COMP_E1E1]), bp2_frame);
        bp2_frame = encode_uint16_t(to_custom_float_6_10(sm_ptr[ASM_COMP_E2E2]), bp2_frame);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B1B1],
            sm_ptr[ASM_COMP_B2B2], sm_ptr[ASM_COMP_B1B2], sm_ptr[ASM_COMP_B1B2_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B1B1],
            sm_ptr[ASM_COMP_B3B3], sm_ptr[ASM_COMP_B1B3], sm_ptr[ASM_COMP_B1B3_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B2B2],
            sm_ptr[ASM_COMP_B3B3], sm_ptr[ASM_COMP_B2B3], sm_ptr[ASM_COMP_B2B3_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_E1E1],
            sm_ptr[ASM_COMP_E2E2], sm_ptr[ASM_COMP_E1E2], sm_ptr[ASM_COMP_E1E2_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B1B1],
            sm_ptr[ASM_COMP_E1E1], sm_ptr[ASM_COMP_B1E1], sm_ptr[ASM_COMP_B1E1_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B1B1],
            sm_ptr[ASM_COMP_E2E2], sm_ptr[ASM_COMP_B1E2], sm_ptr[ASM_COMP_B1E2_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B2B2],
            sm_ptr[ASM_COMP_E1E1], sm_ptr[ASM_COMP_B2E1], sm_ptr[ASM_COMP_B2E1_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B2B2],
            sm_ptr[ASM_COMP_E2E2], sm_ptr[ASM_COMP_B2E2], sm_ptr[ASM_COMP_B2E2_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B3B3],
            sm_ptr[ASM_COMP_E1E1], sm_ptr[ASM_COMP_B3E1], sm_ptr[ASM_COMP_B3E1_imag]);

        bp2_frame = _compute_BP2_cross_component(bp2_frame, sm_ptr[ASM_COMP_B3B3],
            sm_ptr[ASM_COMP_E2E2], sm_ptr[ASM_COMP_B3E2], sm_ptr[ASM_COMP_B3E2_imag]);
    }
}

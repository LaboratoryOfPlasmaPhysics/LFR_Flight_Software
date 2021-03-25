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

//#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "basic_parameters_params.h"
#include "custom_floats.h"
#include "processing/ASM/spectralmatrices.h"


void init_k_coefficients_f0(float* k_coefficients, unsigned char nb_binscompressed_matrix)
{

    uint8_t i; // 8 bits unsigned
    for (i = 0; i < nb_binscompressed_matrix; i++)
    {
        k_coefficients[i * NB_K_COEFF_PER_BIN + K44_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K55_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_IM] = 1;
    }
}


void init_k_coefficients_f1(float* k_coefficients, unsigned char nb_binscompressed_matrix)
{

    uint8_t i; // 8 bits unsigned
    for (i = 0; i < nb_binscompressed_matrix; i++)
    {
        k_coefficients[i * NB_K_COEFF_PER_BIN + K44_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K55_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_IM] = 1;
    }
}


void init_k_coefficients_f2(float* k_coefficients, unsigned char nb_binscompressed_matrix)
{

    uint8_t i; // 8 bits unsigned
    for (i = 0; i < nb_binscompressed_matrix; i++)
    {
        k_coefficients[i * NB_K_COEFF_PER_BIN + K44_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K55_PE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K45_PE_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K14_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K15_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_SX_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NY_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K24_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K25_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K34_NZ_IM] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_RE] = 1;
        k_coefficients[i * NB_K_COEFF_PER_BIN + K35_NZ_IM] = 1;
    }
}


void init_k_coefficients(float* k_coefficients, unsigned char nb_binscompressed_matrix)
{
    switch (nb_binscompressed_matrix)
    {
        case NB_BINS_COMPRESSED_MATRIX_f0:
#ifdef DEBUG_TCH
            printf("F0 data: initialization of the intercalibration k-coefficients\n");
#endif
            init_k_coefficients_f0(k_coefficients, nb_binscompressed_matrix);
            break;

        case NB_BINS_COMPRESSED_MATRIX_f1:
#ifdef DEBUG_TCH
            printf("F1 data: initialization of the intercalibration k-coefficients\n");
#endif
            init_k_coefficients_f1(k_coefficients, nb_binscompressed_matrix);
            break;

        case NB_BINS_COMPRESSED_MATRIX_f2:
#ifdef DEBUG_TCH
            printf("F2 data: initialization of the intercalibration k-coefficients\n");
#endif
            init_k_coefficients_f2(k_coefficients, nb_binscompressed_matrix);
            break;

        default:
#ifdef DEBUG_TCH
            printf("there is a problème !!?\n");
#endif
            break;
    }
}

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
        normal_wave_vector_t nvec = { .x = spectral_matrix[ASM_COMP_B2B3] / ab,
            .y = -spectral_matrix[ASM_COMP_B1B3] / ab,
            .z = spectral_matrix[ASM_COMP_B1B2] / ab,
            .ab = ab };
        return nvec;
    }
    else
    {
        normal_wave_vector_t nvec = { .x = 0., .y = 0., .z = 0., .ab = 0. };
        return nvec;
    }
}

inline float wave_ellipticity_estimator(const float mag_psd, const float nvec_denom)
    __attribute__((always_inline));
float wave_ellipticity_estimator(const float mag_psd, const float nvec_denom)
{
    return 2.f * nvec_denom / mag_psd;
}

inline float degree_of_polarization(const float B_trace, const float* const spectral_matrix)
    __attribute__((always_inline));
float degree_of_polarization(const float B_trace, const float* const spectral_matrix)
{
    const float B_square_trace = square(spectral_matrix[ASM_COMP_B1B1])
        + square(spectral_matrix[ASM_COMP_B2B2]) + square(spectral_matrix[ASM_COMP_B3B3]);
    return sqrtf((3.f * B_square_trace - square(B_trace)) / (2.f * B_square_trace));
}

inline _Complex float X_poynting_vector(const float* const spectral_matrix)
    __attribute__((always_inline));
_Complex float X_poynting_vector(const float* const spectral_matrix)
{
    // E1B3 - E2B2
    _Complex float X_PV;
    __real__ X_PV = spectral_matrix[ASM_COMP_B3E1] - spectral_matrix[ASM_COMP_B2E2];
    __imag__ X_PV = spectral_matrix[ASM_COMP_B3E1_imag] - spectral_matrix[ASM_COMP_B2E2_imag];
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


inline float phase_velocity_estimator(const float* const spectral_matrix,
    const normal_wave_vector_t nvec) __attribute__((always_inline));
float phase_velocity_estimator(const float* const spectral_matrix, const normal_wave_vector_t nvec)
{
    /*
    VPHI = abs(NEBX) * sign( Re[NEBX] ) / BXBX
with:
    NEBX = nY<EZBX*>/rho_EZBX  - nZ<EYBX*>/rho_EYBX = n2<E2B1*>/rho_E2B1 - n3<E1B1*>/rho_E1B1
    rho_E2B1 = |<E2B1*>| / sqrt(<E2E2*><B1B1*>)
    rho_E1B1 = |<E1B1*>| / sqrt(<E1E1*><B1B1*>)
    BXBX = <BXBX*> = <B1B1*>
*/
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
        float vphi = cplx_modulus(NEBX) / BXBX;
        if (__real__ NEBX >= 0.)
            return vphi;
        else
            return -vphi;
    }
    else
    {
        return 0.;
    }
}

inline uint8_t encode_nvec_z_ellip_dop(const float nvec_z, const float ellipticity, const float DOP)
    __attribute__((always_inline));
uint8_t encode_nvec_z_ellip_dop(const float nvec_z, const float ellipticity, const float DOP)
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
    return result.value;
}

inline uint8_t* encode_uint16_t(const uint16_t value, uint8_t* bp1_buffer_frame)
    __attribute__((always_inline));
uint8_t* encode_uint16_t(const uint16_t value, uint8_t* bp1_buffer_frame)
{
    const str_uint16_t value_split = { .value = value };
    *bp1_buffer_frame = value_split.str.MSB;
    bp1_buffer_frame++;
    *bp1_buffer_frame = value_split.str.LSB;
    bp1_buffer_frame++;
    return bp1_buffer_frame;
}

inline uint8_t* encode_BP1(const float mag_PSD, const float elec_PSD,
    const normal_wave_vector_t nvec, const float ellipticity, const float DOP,
    const _Complex float X_PV, const float VPHI, uint8_t* bp1_buffer_frame)
    __attribute__((always_inline));
uint8_t* encode_BP1(const float mag_PSD, const float elec_PSD, const normal_wave_vector_t nvec,
    const float ellipticity, const float DOP, const _Complex float X_PV, const float VPHI,
    uint8_t* bp1_buffer_frame)
{
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_6_10(elec_PSD), bp1_buffer_frame);
    }
    {
        bp1_buffer_frame = encode_uint16_t(to_custom_float_6_10(mag_PSD), bp1_buffer_frame);
    }
    {
        *bp1_buffer_frame = (uint8_t)(nvec.x * 127.5f + 128.f);
        bp1_buffer_frame++;
        *bp1_buffer_frame = (uint8_t)(nvec.y * 127.5f + 128.f);
        bp1_buffer_frame++;
        *bp1_buffer_frame = encode_nvec_z_ellip_dop(nvec.z, ellipticity, DOP);
        bp1_buffer_frame++;
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
        const _Complex float X_PV = X_poynting_vector(spectral_matrix_ptr);
        const float VPHI = phase_velocity_estimator(spectral_matrix_ptr, nvec);
        bp1_buffer_frame
            = encode_BP1(mag_PSD, elec_PSD, nvec, ellipticity, DOP, X_PV, VPHI, bp1_buffer_frame);
        spectral_matrix_ptr = next_matrix(spectral_matrix_ptr);
    }
}

#ifdef ENABLE_DEAD_CODE
void BP1_set(float* compressed_spec_mat, float* k_coeff_intercalib,
    uint8_t nb_bins_compressed_spec_mat, uint8_t* lfr_bp1)
{
    float PSDB; // 32-bit floating point
    float PSDE;
    float tmp;
    float NVEC_V0;
    float NVEC_V1;
    float NVEC_V2;
    float aux;
    float tr_SB_SB;
    float e_cross_b_re;
    float e_cross_b_im;
    float n_cross_e_scal_b_re;
    float n_cross_e_scal_b_im;
    float ny;
    float nz;
    float bx_bx_star;
    float vphi;
    float significand;
    int exponent; // 32-bit signed integer
    float alpha_M;

    uint8_t nbitexp; // 8-bit unsigned integer
    uint8_t nbitsig;
    uint8_t tmp_uint8;
    uint8_t* pt_uint8; // pointer on unsigned 8-bit integer
    int8_t expmin; // 8-bit signed integer
    int8_t expmax;
    uint16_t rangesig; // 16-bit unsigned integer
    uint16_t psd;
    uint16_t exp;
    uint16_t tmp_uint16;
    uint16_t i;

    alpha_M = 45 * (3.1415927 / 180);

#ifdef DEBUG_TCH
    printf("BP1 : \n");
    printf("Number of bins: %d\n", nb_bins_compressed_spec_mat);
#endif

    // initialization for managing the exponents of the floating point data:
    nbitexp = 6; // number of bits for the exponent
    expmax = 32 + 5; // maximum value of the exponent
    expmin = (expmax - (1 << nbitexp)) + 1; // accordingly the minimum exponent value
    // for floating point data to be recorded on 16-bit words:
    nbitsig = 16 - nbitexp; // number of bits for the significand
    rangesig = (1 << nbitsig) - 1; // == 2^nbitsig - 1

#ifdef DEBUG_TCH
    printf("nbitexp : %d, expmax : %d, expmin : %d\n", nbitexp, expmax, expmin);
    printf("nbitsig : %d, rangesig : %d\n", nbitsig, rangesig);
#endif

    for (i = 0; i < nb_bins_compressed_spec_mat; i++)
    {
        //==============================================
        // BP1 PSDB == PA_LFR_SC_BP1_PB_F0 == 16 bits = 6 bits (exponent) + 10 bits (significand)
        PSDB = compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX] // S11
            + compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9] // S22
            + compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]; // S33

        significand = frexpf(PSDB, &exponent); // 0.5 <= significand < 1
                                               // PSDB = significand * 2^exponent

        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        psd = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = psd | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                             // left place of the significand bits (nbitsig),
                                             // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef LFR_BIG_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 2] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp1[(i * NB_BYTES_BP1) + 3] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LFR_LITTLE_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 2] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp1[(i * NB_BYTES_BP1) + 3] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("\nBin number: %d\n", i);
        printf("PSDB        : %16.8e\n", PSDB);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
        printf("psd for PSDB significand : %d\n", psd);
        printf("exp for PSDB exponent : %d\n", exp);
        printf("pt_uint8[1] for PSDB exponent + significand: %.3d or %.2x\n", pt_uint8[1],
            pt_uint8[1]);
        printf("pt_uint8[0] for PSDB            significand: %.3d or %.2x\n", pt_uint8[0],
            pt_uint8[0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+2] : %.3d or %.2x\n", lfr_bp1[i * NB_BYTES_BP1 + 2],
            lfr_bp1[i * NB_BYTES_BP1 + 2]);
        printf("lfr_bp1[i*NB_BYTES_BP1+3] : %.3d or %.2x\n", lfr_bp1[i * NB_BYTES_BP1 + 3],
            lfr_bp1[i * NB_BYTES_BP1 + 3]);
#endif
        //==============================================
        // BP1 PSDE == PA_LFR_SC_BP1_PE_F0 == 16 bits = 6 bits (exponent) + 10 bits (significand)
        PSDE = (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 21]
                   * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K44_PE]) // S44
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 24]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K55_PE]) // S55
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 22]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K45_PE_RE]) // S45 Re
            - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 23]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K45_PE_IM]); // S45 Im

        significand = frexpf(PSDE, &exponent); // 0.5 <= significand < 1
                                               // PSDE = significand * 2^exponent

        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        psd = (uint16_t)((((significand * 2) - 1) * rangesig)
            + 0.5); // Shift and cast into a 16-bit unsigned int with rounding
                    // where just the first nbitsig bits are used (0, ..., 2^nbitsig-1)
        exp = (uint16_t)(exponent
            - expmin); // Shift and cast into a 16-bit unsigned int where just
                       // the first nbitexp bits are used (0, ..., 2^nbitexp-1)
        tmp_uint16 = psd | (exp << nbitsig); // Put the exponent bits (nbitexp) next to the
                                             // left place of the significand bits (nbitsig),
                                             // making the 16-bit word to be recorded
        pt_uint8 = (uint8_t*)&tmp_uint16; // Affect an uint8_t pointer with the adress of tmp_uint16
#ifdef LFR_BIG_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 0] = pt_uint8[0]; // Record MSB of tmp_uint16
        lfr_bp1[(i * NB_BYTES_BP1) + 1] = pt_uint8[1]; // Record LSB of tmp_uint16
#endif
#ifdef LFR_LITTLE_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 0] = pt_uint8[1]; // Record MSB of tmp_uint16
        lfr_bp1[(i * NB_BYTES_BP1) + 1] = pt_uint8[0]; // Record LSB of tmp_uint16
#endif
#ifdef DEBUG_TCH
        printf("PSDE        : %16.8e\n", PSDE);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
        printf("psd for PSDE significand : %d\n", psd);
        printf("exp for PSDE exponent : %d\n", exp);
        printf("pt_uint8[1] for PSDE exponent + significand: %.3d or %.2x\n", pt_uint8[1],
            pt_uint8[1]);
        printf("pt_uint8[0] for PSDE            significand: %.3d or %.2x\n", pt_uint8[0],
            pt_uint8[0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+0] : %.3d or %.2x\n", lfr_bp1[i * NB_BYTES_BP1 + 0],
            lfr_bp1[i * NB_BYTES_BP1 + 0]);
        printf("lfr_bp1[i*NB_BYTES_BP1+1] : %.3d or %.2x\n", lfr_bp1[i * NB_BYTES_BP1 + 1],
            lfr_bp1[i * NB_BYTES_BP1 + 1]);
#endif
        //==============================================================================
        // BP1 normal wave vector == PA_LFR_SC_BP1_NVEC_V0_F0 == 8 bits
        // == PA_LFR_SC_BP1_NVEC_V1_F0 == 8 bits
        // == PA_LFR_SC_BP1_NVEC_V2_F0 == 1 sign bit
        tmp = sqrt((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2]
                       * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2]) // Im S12
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4]) // Im S13
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11]) // Im S23
        );
        if (tmp != 0.)
        { // no division by 0.
            NVEC_V0 = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11]
                / tmp; // S23 Im  => n1
            NVEC_V1 = (-compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4])
                / tmp; // S13 Im  => n2
            NVEC_V2 = compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2]
                / tmp; // S12 Im  => n3
        }
        else
        {
            NVEC_V0 = 0.;
            NVEC_V1 = 0.;
            NVEC_V2 = 0.;
        }
        lfr_bp1[(i * NB_BYTES_BP1) + 4] = (uint8_t)((NVEC_V0 * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        lfr_bp1[(i * NB_BYTES_BP1) + 5] = (uint8_t)((NVEC_V1 * 127.5)
            + 128); // Shift and cast into a 8-bit uint8_t (0, ..., 255) with rounding
        pt_uint8 = (uint8_t*)&NVEC_V2; // Affect an uint8_t pointer with the adress of NVEC_V2
#ifdef LFR_LITTLE_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 6]
            = pt_uint8[3] & 0x80; // Extract the sign bit of NVEC_V2 (32-bit float, sign bit in the
                                  // 4th octet:PC convention) Record it at the 8th bit position
                                  // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#endif
#ifdef LFR_BIG_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 6]
            = pt_uint8[0] & 0x80; // Extract the sign bit of NVEC_V2 (32-bit float, sign bit in the
                                  // 1th octet:SPARC convention) Record it at the 8th bit position
                                  // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#endif
#ifdef DEBUG_TCH
        printf("NVEC_V0  : %16.8e\n", NVEC_V0);
        printf("NVEC_V1  : %16.8e\n", NVEC_V1);
        printf("NVEC_V2  : %16.8e\n", NVEC_V2);
        printf("lfr_bp1[i*NB_BYTES_BP1+4] for NVEC_V0 : %u\n", lfr_bp1[i * NB_BYTES_BP1 + 4]);
        printf("lfr_bp1[i*NB_BYTES_BP1+5] for NVEC_V1 : %u\n", lfr_bp1[i * NB_BYTES_BP1 + 5]);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 : %u\n", lfr_bp1[i * NB_BYTES_BP1 + 6]);
#endif
        //=======================================================
        // BP1 ellipticity == PA_LFR_SC_BP1_ELLIP_F0 == 4 bits
        if (PSDB != 0.)
        { // no division by 0.
            aux = 2 * tmp / PSDB; // Compute the ellipticity
        }
        else
        {
            aux = 0.;
        }
        tmp_uint8
            = (uint8_t)((aux * 15) + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                                           // where just the first 4 bits are used (0, ..., 15)
        lfr_bp1[(i * NB_BYTES_BP1) + 6] = lfr_bp1[(i * NB_BYTES_BP1) + 6]
            | (tmp_uint8 << 3); // Put these 4 bits next to the right place
                                // of the sign bit of NVEC_V2 (recorded
                                // previously in lfr_bp1[i*NB_BYTES_BP1+6])
#ifdef DEBUG_TCH
        printf("ellipticity  : %16.8e\n", aux);
        printf("tmp_uint8 for ellipticity : %u\n", tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 + ellipticity : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 6]);
#endif
        //==============================================================
        // BP1 degree of polarization == PA_LFR_SC_BP1_DOP_F0 == 3 bits
        tr_SB_SB = (compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX]
                       * compressed_spec_mat[i * NB_VALUES_PER_SPECTRAL_MATRIX])
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9])
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 1]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 1])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 2])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 3]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 3])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 4])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 10]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 10])
            + (2 * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11]
                * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 11]);
        aux = PSDB * PSDB;
        if (aux != 0.)
        { // no division by 0.
            tmp = (3 * tr_SB_SB - aux) / (2 * aux); // Compute the degree of polarisation
        }
        else
        {
            tmp = 0.;
        }
        tmp_uint8 = (uint8_t)((tmp * 7) + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                                                // where just the first 3 bits are used (0, ..., 7)
        lfr_bp1[(i * NB_BYTES_BP1) + 6] = lfr_bp1[(i * NB_BYTES_BP1) + 6]
            | tmp_uint8; // Record these 3 bits at the 3 first bit positions
                         // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+6]
#ifdef DEBUG_TCH
        printf("DOP  : %16.8e\n", tmp);
        printf("tmp_uint8 for DOP : %u\n", tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+6] for NVEC_V2 + ellipticity + DOP : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 6]);
#endif
        //=======================================================================================
        // BP1 X_SO-component of the Poynting flux == PA_LFR_SC_BP1_SX_F0 == 16 bits
        //                                          = 1 sign bit + 1 argument bit (two sectors)
        //                                          + 6 bits (exponent) + 8 bits (significand)
        e_cross_b_re = (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                           * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_SX_RE]) // S34 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_SX_RE]) // S35 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 5]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K14_SX_RE]) // S14 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 7]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K15_SX_RE]) // S15 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_SX_RE]) // S24 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_SX_RE]) // S25 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_SX_IM]) // S34 Im
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_SX_IM]) // S35 Im
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 6]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K14_SX_IM]) // S14 Im
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 8]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K15_SX_IM]) // S15 Im
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_SX_IM]) // S24 Im
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_SX_IM]); // S25 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        e_cross_b_im = (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                           * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_SX_IM]) // S34 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_SX_IM]) // S35 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 5]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K14_SX_IM]) // S14 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 7]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K15_SX_IM]) // S15 Re
            + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_SX_IM]) // S24 Re
            + ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                   * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_SX_IM]) // S25 Re
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_SX_RE]) // S34 Im
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_SX_RE]) // S35 Im
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 6]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K14_SX_RE]) // S14 Im
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 8]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K15_SX_RE]) // S15 Im
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_SX_RE]) // S24 Im
                - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                    * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_SX_RE])); // S25 Im
#ifdef DEBUG_TCH
        printf("ReaSX       : %16.8e\n", e_cross_b_re);
#endif
        pt_uint8
            = (uint8_t*)&e_cross_b_re; // Affect an uint8_t pointer with the adress of e_cross_b_re
#ifdef LFR_LITTLE_ENDIAN

        lfr_bp1[(i * NB_BYTES_BP1) + 7] = (uint8_t)(pt_uint8[3]
            & 0x80); // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                     // Record it at the 8th bit position (from the right to the left)
                     // of lfr_bp1[i*NB_BYTES_BP1+7]
        pt_uint8[3] = (pt_uint8[3] & 0x7f); // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
#ifdef LFR_BIG_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 7] = (uint8_t)(pt_uint8[0]
            & 0x80); // Extract its sign bit (32-bit float, sign bit in the 1th octet:SPARC
                     // convention) Record it at the 8th bit position (from the right to the left)
                     // of lfr_bp1[i*NB_BYTES_BP1+7]
        pt_uint8[0] = (pt_uint8[0] & 0x7f); // Make e_cross_b_re be positive in any case: |ReaSX|
#endif
        significand = frexpf(e_cross_b_re, &exponent); // 0.5 <= significand < 1
                                                       // ReaSX = significand * 2^exponent
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        lfr_bp1[(i * NB_BYTES_BP1) + 8] = (uint8_t)((((significand * 2) - 1) * 255)
            + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                    // where all bits are used (0, ..., 255)
        tmp_uint8 = (uint8_t)(exponent
            - expmin); // Shift and cast into a 8-bit uint8_t where
                       // just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
#ifdef DEBUG_TCH
        printf("|ReaSX|     : %16.8e\n", e_cross_b_re);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
        printf("tmp_uint8        for ReaSX exponent    : %d\n", tmp_uint8);
#endif
        lfr_bp1[(i * NB_BYTES_BP1) + 7] = lfr_bp1[(i * NB_BYTES_BP1) + 7]
            | tmp_uint8; // Record these nbitexp bits in the nbitexp first bits
                         // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+7]
#ifdef DEBUG_TCH
        printf("lfr_bp1[i*NB_BYTES_BP1+7] for ReaSX sign + RealSX exponent : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 7]);
        printf("lfr_bp1[i*NB_BYTES_BP1+8] for ReaSX significand            : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 8]);
        printf("ImaSX       : %16.8e\n", e_cross_b_im);
#endif
        pt_uint8
            = (uint8_t*)&e_cross_b_im; // Affect an uint8_t pointer with the adress of e_cross_b_im
#ifdef LFR_LITTLE_ENDIAN
        pt_uint8[3] = pt_uint8[3] & 0x7f; // Make e_cross_b_im be positive in any case: |ImaSX|
                                          // (32-bit float, sign bit in the 4th octet:PC convention)
#endif
#ifdef LFR_BIG_ENDIAN
        pt_uint8[0]
            = pt_uint8[0] & 0x7f; // Make e_cross_b_im be positive in any case: |ImaSX| (32-bit
                                  // float, sign bit in the 1th octet:SPARC convention)
#endif
        // Determine the sector argument of SX. If |Im| > |Re| affect
        // an unsigned 8-bit char with 01000000; otherwise with null.
        if (e_cross_b_im > e_cross_b_re)
        {
            tmp_uint8 = 0x40;
        }
        else
        {
            tmp_uint8 = 0x00;
        }

        lfr_bp1[(i * NB_BYTES_BP1) + 7] = lfr_bp1[(i * NB_BYTES_BP1) + 7]
            | tmp_uint8; // Record it as a sign bit at the 7th bit position (from the right
                         // to the left) of lfr_bp1[i*NB_BYTES_BP1+7], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|ImaSX|     : %16.8e\n", e_cross_b_im);
        printf("ArgSX sign  : %u\n", tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+7] for ReaSX & ArgSX signs + ReaSX exponent  : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 7]);
#endif
        //======================================================================
        // BP1 phase velocity estimator == PA_LFR_SC_BP1_VPHI_F0 == 16 bits
        //                                          = 1 sign bit + 1 argument bit (two sectors)
        //                                          + 6 bits (exponent) + 8 bits (significand)
        ny = (sin(alpha_M) * NVEC_V1) + (cos(alpha_M) * NVEC_V2);
        nz = NVEC_V0;
        bx_bx_star = (cos(alpha_M) * cos(alpha_M)
                         * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 9]) // S22 Re
            + ((sin(alpha_M) * sin(alpha_M)
                   * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 16]) // S33 Re
                - (2 * sin(alpha_M) * cos(alpha_M)
                    * compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 10])); // S23 Re

        n_cross_e_scal_b_re
            = (ny
                  * ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                         * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NY_RE]) // S24 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NY_RE]) // S25 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NY_RE]) // S34 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NY_RE]) // S35 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NY_IM]) // S24 Im
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NY_IM]) // S25 Im
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NY_IM]) // S34 Im
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NY_IM]))) // S35 Im
            + (nz
                * ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                       * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NZ_RE]) // S24 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NZ_RE]) // S25 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NZ_RE]) // S34 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NZ_RE]) // S35 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NZ_IM]) // S24 Im
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NZ_IM]) // S25 Im
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NZ_IM]) // S34 Im
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NZ_IM]))); // S35 Im
        // Im(S_ji) = -Im(S_ij)
        // k_ji = k_ij
        n_cross_e_scal_b_im
            = (ny
                  * ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                         * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NY_IM]) // S24 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NY_IM]) // S25 Re
                      + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                          * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NY_IM]) // S34 Re
                      + ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                             * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NY_IM]) // S35 Re
                          - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                              * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NY_RE]) // S24 Im
                          - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                              * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NY_RE]) // S25 Im
                          - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                              * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NY_RE]) // S34 Im
                          - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                              * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NY_RE])))) // S35
                                                                                             // Im
            + (nz
                * ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 12]
                       * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NZ_IM]) // S24 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 14]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NZ_IM]) // S25 Re
                    + (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 17]
                        * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NZ_IM]) // S34 Re
                    + ((compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 19]
                           * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NZ_IM]) // S35 Re
                        - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 13]
                            * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K24_NZ_RE]) // S24 Im
                        - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 15]
                            * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K25_NZ_RE]) // S25 Im
                        - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 18]
                            * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K34_NZ_RE]) // S34 Im
                        - (compressed_spec_mat[(i * NB_VALUES_PER_SPECTRAL_MATRIX) + 20]
                            * k_coeff_intercalib[(i * NB_K_COEFF_PER_BIN) + K35_NZ_RE])))); // S35
                                                                                            // Im
#ifdef DEBUG_TCH
        printf("n_cross_e_scal_b_re   : %16.8e\n", n_cross_e_scal_b_re);
        printf("n_cross_e_scal_b_im   : %16.8e\n", n_cross_e_scal_b_im);
#endif
        // vphi = n_cross_e_scal_b_re / bx_bx_star => sign(VPHI) = sign(n_cross_e_scal_b_re)
        pt_uint8 = (uint8_t*)&n_cross_e_scal_b_re; // Affect an uint8_t pointer with the adress of
                                                   // n_cross_e_scal_b_re
#ifdef LFR_LITTLE_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 9] = (uint8_t)(pt_uint8[3]
            & 0x80); // Extract its sign bit (32-bit float, sign bit in the 4th octet:PC convention)
                     // Record it at the 8th bit position (from the right to the left)
                     // of lfr_bp1[i*NB_BYTES_BP1+9]
        pt_uint8[3] = (pt_uint8[3]
            & 0x7f); // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
#ifdef LFR_BIG_ENDIAN
        lfr_bp1[(i * NB_BYTES_BP1) + 9] = (uint8_t)(pt_uint8[0]
            & 0x80); // Extract its sign bit (32-bit float, sign bit in the 1th octet:SPARC
                     // convention) Record it at the 8th bit position (from the right to the left)
                     // of lfr_bp1[i*NB_BYTES_BP1+9]
        pt_uint8[0] = (pt_uint8[0]
            & 0x7f); // Make n_cross_e_scal_b_re be positive in any case: |n_cross_e_scal_b_re|
#endif
        if (bx_bx_star != 0.)
        { // no division by 0.
            vphi = n_cross_e_scal_b_re / bx_bx_star; // Compute |VPHI|
        }
        else
        {
            vphi = 1.e+20; // Put a huge value
        }
        significand = frexpf(vphi, &exponent); // 0.5 <= significand < 1
                                               // vphi = significand * 2^exponent
        if (exponent < expmin)
        { // value should be >= 0.5 * 2^expmin
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }
        if (exponent > expmax)
        { // value should be <  0.5 * 2^(expmax+1)
            exponent = expmax;
            significand = 1.0; // max value that can be recorded
        }
        if (significand == 0)
        { // in that case exponent == 0 too
            exponent = expmin;
            significand = 0.5; // min value that can be recorded
        }

        lfr_bp1[(i * NB_BYTES_BP1) + 10] = (uint8_t)((((significand * 2) - 1) * 255)
            + 0.5); // Shift and cast into a 8-bit uint8_t with rounding
                    // where all the bits are used (0, ..., 255)
        tmp_uint8 = (uint8_t)(exponent
            - expmin); // Shift and cast into a 8-bit uint8_t where
                       // just the first nbitexp bits are used (0, ..., 2^nbitexp-1)
#ifdef DEBUG_TCH
        printf("|VPHI|      : %16.8e\n", vphi);
        printf("significand : %16.8e\n", significand);
        printf("exponent    : %d\n", exponent);
        printf("tmp_uint8        for VPHI exponent    : %d\n", tmp_uint8);
#endif
        lfr_bp1[(i * NB_BYTES_BP1) + 9] = lfr_bp1[(i * NB_BYTES_BP1) + 9]
            | tmp_uint8; // Record these nbitexp bits in the nbitexp first bits
                         // (from the right to the left) of lfr_bp1[i*NB_BYTES_BP1+9]
#ifdef DEBUG_TCH
        printf("lfr_bp1[i*NB_BYTES_BP1+9]  for VPHI sign + VPHI exponent : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 9]);
        printf("lfr_bp1[i*NB_BYTES_BP1+10] for VPHI significand          : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 10]);
#endif
        pt_uint8 = (uint8_t*)&n_cross_e_scal_b_im; // Affect an uint8_t pointer with the adress of
                                                   // n_cross_e_scal_b_im
#ifdef LFR_LITTLE_ENDIAN
        pt_uint8[3]
            = pt_uint8[3] & 0x7f; // Make n_cross_e_scal_b_im be positive in any case: |ImaNEBX|
                                  // (32-bit float, sign bit in the 4th octet:PC convention)
#endif
#ifdef LFR_BIG_ENDIAN
        pt_uint8[0]
            = pt_uint8[0] & 0x7f; // Make n_cross_e_scal_b_im be positive in any case: |ImaNEBX|
                                  // (32-bit float, sign bit in the 1th octet:SPARC convention)
#endif

        // Determine the sector argument of NEBX. If |Im| > |Re| affect
        // an unsigned 8-bit char with 01000000; otherwise with null.
        if (n_cross_e_scal_b_im > n_cross_e_scal_b_re)
        {
            tmp_uint8 = 0x40;
        }
        else
        {
            tmp_uint8 = 0x00;
        }

        lfr_bp1[(i * NB_BYTES_BP1) + 9] = lfr_bp1[(i * NB_BYTES_BP1) + 9]
            | tmp_uint8; // Record it as a sign bit at the 7th bit position (from the right
                         // to the left) of lfr_bp1[i*NB_BYTES_BP1+9], by simple logical addition.
#ifdef DEBUG_TCH
        printf("|n_cross_e_scal_b_im|             : %16.8e\n", n_cross_e_scal_b_im);
        printf("|n_cross_e_scal_b_im|/bx_bx_star  : %16.8e\n", n_cross_e_scal_b_im / bx_bx_star);
        printf("ArgNEBX sign                      : %u\n", tmp_uint8);
        printf("lfr_bp1[i*NB_BYTES_BP1+9] for VPHI & ArgNEBX signs + VPHI exponent : %u\n",
            lfr_bp1[i * NB_BYTES_BP1 + 9]);
#endif
    }
}

#endif

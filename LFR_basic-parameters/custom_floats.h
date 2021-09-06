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
#ifdef LFR_BIG_ENDIAN

typedef union
{
    uint16_t value;
    struct __attribute__((__packed__))
    {
        uint16_t MSB : 8;
        uint16_t LSB : 8;
    } str;
} str_uint16_t;

typedef union
{
    uint16_t value;
    struct __attribute__((__packed__))
    {
        uint16_t exponent : 6;
        uint16_t mantissa : 10;
    } str;
} float_6_10_t;

typedef union
{
    uint16_t value;
    struct __attribute__((__packed__))
    {
        uint16_t sign : 1;
        uint16_t arg : 1;
        uint16_t exponent : 6;
        uint16_t mantissa : 8;
    } str;
} float_1_1_6_8_t;

typedef union
{
    float value;
    struct __attribute__((__packed__))
    {
        uint32_t sign : 1;
        uint32_t exponent : 8;
        uint32_t mantissa : 23;
    } str;
} str_float_t;
#endif
#ifdef LFR_LITTLE_ENDIAN
typedef union
{
    uint16_t value;
    struct __attribute__((__packed__))
    {
        uint16_t mantissa : 10;
        uint16_t exponent : 6;
    } str;
} float_6_10_t;

typedef union
{
    uint16_t value;
    struct __attribute__((__packed__))
    {
        uint16_t mantissa : 8;
        uint16_t exponent : 6;
        uint16_t arg : 1;
        uint16_t sign : 1;
    } str;
} float_1_1_6_8_t;

typedef union
{
    float value;
    struct __attribute__((__packed__))
    {
        uint32_t mantissa : 23;
        uint32_t exponent : 8;
        uint32_t sign : 1;
    } str;
} str_float_t;
#endif


inline uint16_t to_custom_float_6_10(const float value) __attribute__((always_inline));
uint16_t to_custom_float_6_10(const float value)
{
    float_6_10_t result = { .value = 0 };
    str_float_t v = { .value = value };

    if ((v.str.exponent - 127) < -27)
        return 0;
    if ((v.str.exponent - 127) > 37)
        return 0xFFFF;

    result.str.exponent = v.str.exponent - 127 + 27;
    result.str.mantissa = v.str.mantissa >> 13;

    return result.value;
}

inline uint16_t to_custom_float_1_1_6_8(const _Complex float value) __attribute__((always_inline));
uint16_t to_custom_float_1_1_6_8(const _Complex float value)
{
    float_1_1_6_8_t result = { .value = 0 };
    str_float_t v_re = { .value = __real__ value };
    str_float_t v_imag = { .value = __imag__ value };

    if ((v.str.exponent - 127) < -27)
        return 0;
    if ((v.str.exponent - 127) > 37)
        return 0xFFFF;

    result.str.sign = v_re.str.sign;
    v_re.str.sign = 0;
    v_imag.str.sign = 0;
    result.str.exponent = v_re.str.exponent - 127 + 27;
    result.str.mantissa = v_re.str.mantissa >> 15;
    result.str.arg = v_imag.value > v_re.value;
    return result.value;
}

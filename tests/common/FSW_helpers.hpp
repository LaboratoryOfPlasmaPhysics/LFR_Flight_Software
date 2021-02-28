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
#include <array>
#include <complex>

using lfr_asm_t = std::array<float, 25 * 128>;

template <bool transpose = false>
struct full_spectral_matrix_t
{
    float* _data;
    full_spectral_matrix_t(float* data) : _data { data } { }
    float& real(int line, int column)
    {
        if constexpr (transpose)
            return _data[(line + column * 5) * 2];
        else
            return _data[(column + line * 5) * 2];
    }
    float& img(int line, int column)
    {
        if constexpr (transpose)
            return _data[(line + column * 5) * 2 + 1];
        else
            return _data[(column + line * 5) * 2 + 1];
    }
};

struct triangular_spectral_matrix_t
{
    static constexpr int _indexes[5][5] = { { 0, 1, 3, 5, 7 }, { 1, 9, 10, 12, 14 },
        { 3, 10, 16, 17, 19 }, { 5, 12, 17, 21, 22 }, { 7, 14, 19, 22, 24 } };
    float _z = 0.f;
    float* _data;
    triangular_spectral_matrix_t(float* data) : _data { data } { }
    float& real(int line, int column) { return _data[_indexes[line][column]]; }
    float& img(int line, int column)
    {
        if (line == column)
        {
            _z = 0.f;
            return _z;
        }
        return _data[_indexes[line][column] + 1];
    }
};

template <int matrix_size = 5>
inline std::size_t to_triangular_matrix_offset(int line, int column)
{
    return column + line * matrix_size;
}

template <bool transpose = false>
inline void extract_upper_triangle(
    std::complex<float>* src_full_matrix, float* dest_triangular_matrix)
{
    auto triang_m = triangular_spectral_matrix_t { dest_triangular_matrix };
    auto full_m = full_spectral_matrix_t<transpose> { reinterpret_cast<float*>(src_full_matrix) };
    for (int line = 0; line < 5; line++)
    {
        for (int column = line; column < 5; column++)
        {
            triang_m.real(line, column) = full_m.real(line, column);
            if (line != column)
                triang_m.img(line, column) = full_m.img(line, column);
        }
    }
}

inline lfr_asm_t to_lfr_asm(float* full_spectral_matrix)
{
    constexpr auto full_asm_comp_size = 5 * 5 * 2 * sizeof(float);
    constexpr auto triangular_asm_comp_size = 25 * sizeof(float);
    lfr_asm_t trianglular_asm;
    for (int freq_index = 0; freq_index < 128; freq_index++)
    {
        auto full_matrix = reinterpret_cast<std::complex<float>*>(
            full_spectral_matrix + freq_index * full_asm_comp_size);
        auto triangular_matrix = trianglular_asm.data() + triangular_asm_comp_size * freq_index;
        extract_upper_triangle(full_matrix, triangular_matrix);
    }
    return trianglular_asm;
}

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

template <bool swap_lines_columns = false>
struct full_spectral_matrix_t
{
    float* _data;
    full_spectral_matrix_t(float* data) : _data { data } { }
    float& real(int line, int column)
    {
        if constexpr (swap_lines_columns)
            return _data[(line + column * 5) * 2];
        else
            return _data[(column + line * 5) * 2];
    }
    float& img(int line, int column)
    {
        if constexpr (swap_lines_columns)
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
    void real(int line, int column, float value)
    {
        if (line > column)
            throw std::range_error("can't write lower part of triangular matrix");
        _data[_indexes[line][column]] = value;
    }
    void img(int line, int column, float value)
    {
        if (line >= column)
            throw std::range_error("can't write lower part of triangular matrix");
        _data[_indexes[line][column] + 1] = value;
    }

    float real(int line, int column) const { return _data[_indexes[line][column]]; }
    float img(int line, int column) const
    {
        if (line > column)
            return -_data[_indexes[line][column] + 1];
        if (line == column)
        {
            return 0.f;
        }
        return _data[_indexes[line][column] + 1];
    }
};

struct lfr_triangular_spectral_matrix_t
{
    static constexpr int _indexes[5][5] = { { 0, 1, 3, 5, 7 }, { 1, 9, 10, 12, 14 },
        { 3, 10, 16, 17, 19 }, { 5, 12, 17, 21, 22 }, { 7, 14, 19, 22, 24 } };
    float _z = 0.f;
    float* _data;

    std::size_t index(int frequency, int line, int column) const
    {
        return (_indexes[line][column] * 2 * 128) + frequency;
    }

    lfr_triangular_spectral_matrix_t(float* data) : _data { data } { }

    void real(int frequency, int line, int column, float value)
    {
        if (line > column)
            throw std::range_error("can't write lower part of triangular matrix");
        _data[index(frequency, line, column)] = value;
    }
    void img(int frequency, int line, int column, float value)
    {
        if (line >= column)
            throw std::range_error("can't write lower part of triangular matrix");
        _data[index(frequency, line, column) + 1] = value;
    }

    float real(int frequency, int line, int column) const
    {
        return _data[index(frequency, line, column)];
    }
    float img(int frequency, int line, int column) const
    {
        if (line > column)
            return -_data[index(frequency, line, column) + 1];
        if (line == column)
        {
            return 0.f;
        }
        return _data[index(frequency, line, column) + 1];
    }
};

template <int matrix_size = 5>
inline std::size_t to_triangular_matrix_offset(int line, int column)
{
    return column + line * matrix_size;
}

template <bool swap_lines_columns = false>
inline void extract_upper_triangle(
    std::complex<float>* src_full_matrix, float* dest_triangular_matrix)
{
    auto triang_m = triangular_spectral_matrix_t { dest_triangular_matrix };
    auto full_m
        = full_spectral_matrix_t<swap_lines_columns> { reinterpret_cast<float*>(src_full_matrix) };
    for (int line = 0; line < 5; line++)
    {
        for (int column = line; column < 5; column++)
        {
            triang_m.real(line, column, full_m.real(line, column));
            if (line != column)
                triang_m.img(line, column, full_m.img(line, column));
        }
    }
}

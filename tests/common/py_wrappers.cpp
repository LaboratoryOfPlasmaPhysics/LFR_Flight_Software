#include <vector>

#include <pybind11/pybind11.h>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "common/FSW_helpers.hpp"
#include "processing/ASM/spectralmatrices.h"

namespace py = pybind11;

template <bool is_triangular = true>
std::vector<float> to_lfr_matrix_repr(py::array_t<std::complex<float>>& input_matrix)
{
    py::buffer_info input_matrix_buff = input_matrix.request();
    auto result = std::vector<float>(is_triangular ? 25 : 50);
    if (input_matrix_buff.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 5, 5 })
        throw std::runtime_error("Shape must be (5,5)");
    if constexpr (is_triangular)
    {
        auto view = triangular_spectral_matrix_t { result.data() };
        for (int line = 0; line < 5; line++)
        {
            for (int column = line; column < 5; column++)
            {
                view.real(line, column, input_matrix.data(line, column)->real());
                if (line != column)
                    view.img(line, column, input_matrix.data(line, column)->imag());
            }
        }
    }
    else
    {
        for (int line = 0; line < 5; line++)
        {
            for (int column = 0; column < 5; column++)
            {
                result[2 * (column + line * 5)] = input_matrix.data(line, column)->real();
                result[2 * (column + line * 5) + 1] = input_matrix.data(line, column)->imag();
            }
        }
    }
    return result;
}

template <bool is_triangular = true>
std::vector<float> to_lfr_spectral_matrix_repr(py::array_t<std::complex<float>>& input_matrix)
{
    py::buffer_info input_matrix_buff = input_matrix.request();
    auto result = std::vector<float>(is_triangular ? 25 * 128 : 50 * 128);
    if (input_matrix_buff.ndim != 3)
        throw std::runtime_error("Number of dimensions must be 3");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 128, 5, 5 })
        throw std::runtime_error("Shape must be (128, 5, 5)");

    auto view = lfr_triangular_spectral_matrix_t { result.data() };
    for (auto frequency = 0ul; frequency < 128; frequency++)
    {
        if constexpr (is_triangular)
        {
            for (int line = 0; line < 5; line++)
            {
                for (int column = line; column < 5; column++)
                {
                    view.real(frequency, line, column,
                        input_matrix.data(frequency, line, column)->real());
                    if (line != column)
                        view.img(frequency, line, column,
                            input_matrix.data(frequency, line, column)->imag());
                }
            }
        }
        else
        {
            constexpr auto offset = is_triangular ? 25 : 50;
            for (int line = 0; line < 5; line++)
            {
                for (int column = 0; column < 5; column++)
                {

                    result[frequency * offset + 2 * (column + line * 5)]
                        = input_matrix.data(frequency, line, column)->real();
                    result[frequency * offset + 2 * (column + line * 5) + 1]
                        = input_matrix.data(frequency, line, column)->imag();
                }
            }
        }
    }
    return result;
}

template <std::size_t size, std::size_t pos>
std::vector<float> extract_spectral_transition_matrix(
    py::array_t<std::complex<float>>& input_matrix)
{
    py::buffer_info input_matrix_buff = input_matrix.request();
    auto result = std::vector<float>(size * size * 2 * 128);
    if (input_matrix_buff.ndim != 3)
        throw std::runtime_error("Number of dimensions must be 3");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 128, 5, 5 })
        throw std::runtime_error("Shape must be (128, 5, 5)");

    for (auto i = 0ul; i < 128; i++)
    {
        {
            constexpr auto offset = size * size * 2;
            for (auto line = 0UL; line < size; line++)
            {
                for (auto column = 0UL; column < size; column++)
                {
                    auto flat_index = i * offset + 2 * (column + line * size);
                    result[flat_index] = input_matrix.data(i, line + pos, column + pos)->real();
                    result[flat_index + 1] = input_matrix.data(i, line + pos, column + pos)->imag();
                }
            }
        }
    }
    return result;
}

template <std::size_t size, std::size_t pos>
std::vector<float> extract_transition_matrix(py::array_t<std::complex<float>>& input_matrix)
{
    py::buffer_info input_matrix_buff = input_matrix.request();
    auto result = std::vector<float>(size * size * 2);
    if (input_matrix_buff.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 5, 5 })
        throw std::runtime_error("Shape must be (5, 5)");

    {
        for (auto line = 0UL; line < size; line++)
        {
            for (auto column = 0UL; column < size; column++)
            {
                auto flat_index = 2 * (column + line * size);
                result[flat_index] = input_matrix.data(line + pos, column + pos)->real();
                result[flat_index + 1] = input_matrix.data(line + pos, column + pos)->imag();
            }
        }
    }
    return result;
}

void from_lfr_matrix_repr(std::vector<float>& src, py::array_t<std::complex<float>>& dest)
{
    auto triang_m = triangular_spectral_matrix_t { src.data() };
    for (int line = 0; line < 5; line++)
    {
        for (int column = 0; column < 5; column++)
        {
            dest.mutable_data(line, column)->real(triang_m.real(line, column));
            dest.mutable_data(line, column)->imag(triang_m.img(line, column));
        }
    }
}

void from_lfr_spectral_matrix_repr(std::vector<float>& src, py::array_t<std::complex<float>>& dest)
{
    auto triang_m = lfr_triangular_spectral_matrix_t { src.data() };
    for (auto frequency = 0ul; frequency < 128; frequency++)
    {
        for (int line = 0; line < 5; line++)
        {
            for (int column = 0; column < 5; column++)
            {
                dest.mutable_data(frequency, line, column)
                    ->real(triang_m.real(frequency, line, column));
                dest.mutable_data(frequency, line, column)
                    ->imag(triang_m.img(frequency, line, column));
            }
        }
    }
}

PYBIND11_MODULE(lfr, m)
{
    m.doc() = "lfr module";

    m.def("SM_calibrate",
        [](py::array_t<std::complex<float>> input_asm,
            py::array_t<std::complex<float>> calibration_matrix)
        {
            auto output_matrix
                = py::array_t<std::complex<float>>(py::buffer_info { input_asm.request() }.shape);
            auto _matrix = to_lfr_spectral_matrix_repr<true>(input_asm);
            auto mag_transition_matrix
                = extract_spectral_transition_matrix<3, 0>(calibration_matrix);
            auto elec_transition_matrix
                = extract_spectral_transition_matrix<2, 3>(calibration_matrix);
            SM_calibrate_and_reorder(_matrix.data(), mag_transition_matrix.data(),
                elec_transition_matrix.data(), _matrix.data());
            from_lfr_spectral_matrix_repr(_matrix, output_matrix);
            return output_matrix;
        });

    m.def("Matrix_change_of_basis",
        [](py::array_t<std::complex<float>> input_matrix,
            py::array_t<std::complex<float>> transition_matrix)
        {
            auto output_matrix = py::array_t<std::complex<float>>(
                py::buffer_info { input_matrix.request() }.shape);
            auto _matrix = to_lfr_matrix_repr<true>(input_matrix);
            auto mag_transition_matrix = extract_transition_matrix<3, 0>(transition_matrix);
            auto elec_transition_matrix = extract_transition_matrix<2, 3>(transition_matrix);
            Matrix_change_of_basis(_matrix.data(), mag_transition_matrix.data(),
                elec_transition_matrix.data(), _matrix.data());
            from_lfr_matrix_repr(_matrix, output_matrix);
            return output_matrix;
        });
}

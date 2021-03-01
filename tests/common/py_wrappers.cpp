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
    auto in_ptr = reinterpret_cast<std::complex<float>*>(input_matrix_buff.ptr);
    auto result = std::vector<float>(is_triangular ? 25 : 50);
    if (input_matrix_buff.ndim != 2)
        throw std::runtime_error("Number of dimensions must be 2");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 5, 5 })
        throw std::runtime_error("Shape must be (5,5)");
    if constexpr (is_triangular)
        extract_upper_triangle<true>(in_ptr, result.data());
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
    std::complex<float>* in_ptr = reinterpret_cast<std::complex<float>*>(input_matrix_buff.ptr);
    auto result = std::vector<float>(is_triangular ? 25 * 128 : 50 * 128);
    if (input_matrix_buff.ndim != 3)
        throw std::runtime_error("Number of dimensions must be 2");
    if (input_matrix_buff.shape != std::vector<ssize_t> { 128, 5, 5 })
        throw std::runtime_error("Shape must be (128, 5, 5)");

    for (auto i = 0ul; i < 128; i++)
    {
        if constexpr (is_triangular)
            extract_upper_triangle<true>(in_ptr + i * 25, result.data() + i * 25);
        else
        {
            constexpr auto offset = is_triangular ? 25 : 50;
            for (int line = 0; line < 5; line++)
            {
                for (int column = 0; column < 5; column++)
                {

                    result[i * offset + 2 * (column + line * 5)]
                        = input_matrix.data(i, line, column)->real();
                    result[i * offset + 2 * (column + line * 5) + 1]
                        = input_matrix.data(i, line, column)->imag();
                }
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
            if (line > column)
                dest.mutable_data(line, column)->imag(-triang_m.img(line, column));
            else
                dest.mutable_data(line, column)->imag(triang_m.img(line, column));
        }
    }
}

void from_lfr_spectral_matrix_repr(std::vector<float>& src, py::array_t<std::complex<float>>& dest)
{
    for (auto i = 0ul; i < 128; i++)
    {
        auto triang_m = triangular_spectral_matrix_t { src.data() + i * 25 };
        for (int line = 0; line < 5; line++)
        {
            for (int column = 0; column < 5; column++)
            {
                dest.mutable_data(i, line, column)->real(triang_m.real(line, column));
                if (line > column)
                    dest.mutable_data(i, line, column)->imag(-triang_m.img(line, column));
                else
                    dest.mutable_data(i, line, column)->imag(triang_m.img(line, column));
            }
        }
    }
}

PYBIND11_MODULE(lfr, m)
{
    m.doc() = "lfr module";
    // void SM_calibrate(float *input_asm, float *calibration_matrix,
    //                  float *output_asm);
    m.def("SM_calibrate",
        [](py::array_t<std::complex<float>> input_asm,
            py::array_t<std::complex<float>> calibration_matrix) {
            auto output_matrix
                = py::array_t<std::complex<float>>(py::buffer_info { input_asm.request() }.shape);
            auto _matrix = to_lfr_spectral_matrix_repr<true>(input_asm);
            auto _transition_matrix = to_lfr_spectral_matrix_repr<false>(calibration_matrix);
            std::vector<float> output(25 * 128);
            SM_calibrate(_matrix.data(), _transition_matrix.data(), output.data());
            from_lfr_spectral_matrix_repr(output, output_matrix);
            return output_matrix;
        });

    m.def("Matrix_change_of_basis",
        [](py::array_t<std::complex<float>> input_matrix,
            py::array_t<std::complex<float>> transition_matrix) {
            auto output_matrix = py::array_t<std::complex<float>>(
                py::buffer_info { input_matrix.request() }.shape);
            auto _matrix = to_lfr_matrix_repr<true>(input_matrix);
            auto _transition_matrix = to_lfr_matrix_repr<false>(transition_matrix);
            std::vector<float> output(25);
            Matrix_change_of_basis(_matrix.data(), _transition_matrix.data(), output.data());
            from_lfr_matrix_repr(output, output_matrix);
            return output_matrix;
        });
}

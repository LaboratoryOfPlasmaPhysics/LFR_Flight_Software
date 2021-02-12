#include <pybind11/pybind11.h>

#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include "processing/ASM/spectralmatrices.h"

namespace py = pybind11;

PYBIND11_MODULE(lfr, m) {
  m.doc() = "lfr module";
  // void SM_calibrate(float *input_asm, float *calibration_matrix,
  //                  float *output_asm);
  m.def("SM_calibrate", [](py::array_t<float> input_asm,
                           py::array_t<float> calibration_matrix) {
    py::buffer_info input_asm_buff = input_asm.request();
    py::buffer_info calibration_matrix_buff = calibration_matrix.request();

    if (input_asm_buff.ndim != 3 || calibration_matrix_buff.ndim != 3)
      throw std::runtime_error("Number of dimensions must be 3");

    auto output_asm = py::array_t<float>(input_asm_buff.shape);
    py::buffer_info output_asm_buff = output_asm.request();

    SM_calibrate(static_cast<float *>(input_asm_buff.ptr),
                 static_cast<float *>(calibration_matrix_buff.ptr),
                 static_cast<float *>(output_asm_buff.ptr));
    return output_asm;
  });

  m.def("Matrix_change_of_basis",
        [](py::array_t<std::complex<float>> input_matrix,
           py::array_t<std::complex<float>> transition_matrix) {
          py::buffer_info input_matrix_buff = input_matrix.request();
          py::buffer_info transition_matrix_buff = transition_matrix.request();

          if (input_matrix_buff.ndim != 2 || transition_matrix_buff.ndim != 2)
            throw std::runtime_error("Number of dimensions must be 3");
          if (input_matrix_buff.shape != std::vector<ssize_t>{5, 5} ||
              transition_matrix_buff.shape != std::vector<ssize_t>{5, 5})
            throw std::runtime_error("Shape must be (5,5)");

          auto output_matrix =
              py::array_t<std::complex<float>>(input_matrix_buff.shape);
          py::buffer_info output_matrix_buff = output_matrix.request();

          Matrix_change_of_basis(
              static_cast<float *>(input_matrix_buff.ptr),
              static_cast<float *>(transition_matrix_buff.ptr),
              static_cast<float *>(output_matrix_buff.ptr));
          return output_matrix;
        });
}

#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
#include <catch2/catch.hpp>
#else
#include <catch.hpp>
#endif

#include "processing/ASM/spectralmatrices.h"

SCENARIO("SM calibration", "[]") {

  float SM[256 * 15];
  float calibration_matrix[256 * 15];

  SM_calibrate(SM, calibration_matrix, SM);
}

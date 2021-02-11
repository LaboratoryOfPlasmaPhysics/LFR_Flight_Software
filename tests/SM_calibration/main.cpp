#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
#include <catch2/catch.hpp>
#else
#include <catch.hpp>
#endif

#include "processing/ASM/spectralmatrices.h"

SCENARIO("SM calibration", "[]") { REQUIRE(1 == 1); }

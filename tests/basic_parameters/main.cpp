#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
    #include <catch2/catch.hpp>
#else
    #include <catch.hpp>
#endif

#include <algorithm>
#include <complex>
#include <random>
#include <vector>

#include "basic_parameters.h"
#include "common/FSW_helpers.hpp"
#include "lfr_common_headers/ccsds_types.h"
#include "lfr_common_headers/fsw_params_processing.h"


std::random_device rd;
std::mt19937 mt(rd());
std::uniform_real_distribution<float> dist(-32000., 32000.);

inline float make_random_ASM_element()
{
    return dist(mt);
}

void fill_ASM_bin(float* SM, std::complex<float> B1, std::complex<float> B2, std::complex<float> B3,
    std::complex<float> E1, std::complex<float> E2)
{
    triangular_matrix_t view { SM };
    {
        auto B1B1 = B1 * std::conj(B1);
        auto B1B2 = B1 * std::conj(B2);
        auto B1B3 = B1 * std::conj(B3);
        auto B1E1 = B1 * std::conj(E1);
        auto B1E2 = B1 * std::conj(E2);

        view.real(0, 0, B1B1.real());

        view.real(0, 1, B1B2.real());
        view.img(0, 1, B1B2.imag());

        view.real(0, 2, B1B3.real());
        view.img(0, 2, B1B3.imag());

        view.real(0, 3, B1E1.real());
        view.img(0, 3, B1E1.imag());

        view.real(0, 4, B1E2.real());
        view.img(0, 4, B1E2.imag());
    }
    {
        auto B2B2 = B2 * std::conj(B2);
        auto B2B3 = B2 * std::conj(B3);
        auto B2E1 = B2 * std::conj(E1);
        auto B2E2 = B2 * std::conj(E2);

        view.real(1, 1, B2B2.real());

        view.real(1, 2, B2B3.real());
        view.img(1, 2, B2B3.imag());

        view.real(1, 3, B2E1.real());
        view.img(1, 3, B2E1.imag());

        view.real(1, 4, B2E2.real());
        view.img(1, 4, B2E2.imag());
    }
    {
        auto B3B3 = B3 * std::conj(B3);
        auto B3E1 = B3 * std::conj(E1);
        auto B3E2 = B3 * std::conj(E2);

        view.real(2, 2, B3B3.real());

        view.real(2, 3, B3E1.real());
        view.img(2, 3, B3E1.imag());

        view.real(2, 4, B3E2.real());
        view.img(2, 4, B3E2.imag());
    }
    {
        auto E1E1 = E1 * std::conj(E1);
        auto E1E2 = E1 * std::conj(E2);

        view.real(3, 3, E1E1.real());

        view.real(3, 4, E1E2.real());
        view.img(3, 4, E1E2.imag());
    }
    {
        auto E2E2 = E2 * std::conj(E2);

        view.real(4, 4, E2E2.real());
    }
}

void populate_random(std::vector<float>& asms)
{
    using namespace std::complex_literals;
    auto asm_count = std::size(asms) / NB_FLOATS_PER_SM;
    lfr_triangular_spectral_matrix_t<false> ASM_view { asms.data() };
    for (auto asm_index = 0UL; asm_index < asm_count; asm_index++)
    {
        fill_ASM_bin(asms.data() + (NB_FLOATS_PER_SM * asm_index),
            { make_random_ASM_element(), make_random_ASM_element() },
            { make_random_ASM_element(), make_random_ASM_element() },
            { make_random_ASM_element(), make_random_ASM_element() },
            { make_random_ASM_element(), make_random_ASM_element() },
            { make_random_ASM_element(), make_random_ASM_element() });
    }
}

SCENARIO("LFR Basic Parameters Set 1", "[]")
{
    GIVEN("some random input")
    {
        THEN("LFR should be able to compute BP1 without crash")
        {
            for (int asm_count = 0; asm_count < 100; asm_count++)
            {
                std::vector<float> asms(NB_FLOATS_PER_SM * asm_count);
                std::vector<uint8_t> tm_packet(
                    MAX_SRC_DATA / NB_BINS_COMPRESSED_SM_SBM_F1 * asm_count);

                populate_random(asms);
                compute_BP1(asms.data(), asm_count, tm_packet.data());
            }
        }
    }
}


SCENARIO("LFR Basic Parameters Set 2", "[]")
{
    GIVEN("some random input")
    {
        THEN("LFR should be able to compute BP2 without crash")
        {
            for (int asm_count = 0; asm_count < 100; asm_count++)
            {
                std::vector<float> asms(NB_FLOATS_PER_SM * asm_count);
                std::vector<uint8_t> tm_packet(
                    MAX_SRC_DATA / NB_BINS_COMPRESSED_SM_SBM_F1 * asm_count);

                populate_random(asms);
                compute_BP2(asms.data(), asm_count, tm_packet.data());
            }
        }
    }
}

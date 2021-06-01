#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
    #include <catch2/catch.hpp>
#else
    #include <catch.hpp>
#endif

#include "common/FSW_helpers.hpp"
#include <complex>
#include <math.h>

SCENARIO("SM calibration", "[]")
{
    std::complex<float> full_matrix[5 * 5] = { { 10.f, -1.f }, { 20.f, 12.f }, { 30.f, 13.f },
        { 40.f, 14.f }, { 50.f, 15.f }, { 100.f, 21.f }, { 200.f, -1.f }, { 300.f, 23.f },
        { 400.f, 24.f }, { 500.f, 25.f }, { 1000.f, 31.f }, { 2000.f, 32.f }, { 3000.f, -1.f },
        { 4000.f, 34.f }, { 5000.f, 35.f }, { 10000.f, 41.f }, { 20000.f, 42.f }, { 30000.f, 43.f },
        { 40000.f, -1.f }, { 50000.f, 45.f }, { 100000.f, 51.f }, { 200000.f, 52.f },
        { 300000.f, 53.f }, { 400000.f, 54.f }, { 500000.f, -1.f } };
    float triangular_matrix[25];
    float expected_triangular_matrix[25]
        = { 10.f, 20.f, 12.f, 30.f, 13.f, 40.f, 14.f, 50.f, 15.f, 200.f, 300.f, 23.f, 400.f, 24.f,
              500.f, 25.f, 3000.f, 4000.f, 34.f, 5000.f, 35.f, 40000.f, 50000.f, 45.f, 500000.f };

    GIVEN("A full Matrix view")
    {
        full_matrix_t view(reinterpret_cast<float*>(full_matrix));
        for (int line = 0; line < 5; line++)
        {
            for (int column = 0; column < 5; column++)
            {
                REQUIRE(view.real(line, column) == float((column + 1) * pow(10, line + 1)));
                if (line != column)
                    REQUIRE(view.img(line, column) == float((line + 1) * 10 + column + 1));
                else
                    REQUIRE(view.img(line, column) == -1.f);
            }
        }
    }

    GIVEN("A Triangular Matrix view")
    {
        triangular_matrix_t view(expected_triangular_matrix);
        for (int line = 0; line < 5; line++)
        {
            for (int column = line; column < 5; column++)
            {
                REQUIRE(view.real(line, column) == float((column + 1) * pow(10, line + 1)));
                if (line != column)
                    REQUIRE(view.img(line, column) == float((line + 1) * 10 + column + 1));
                else
                    REQUIRE(view.img(line, column) == 0.f);
            }
        }
    }

    GIVEN("An LFR Spectral Matrix view")
    {
        std::vector<float> flat_sm(128 * 25);
        {
            auto cursor = flat_sm.begin();
            for (int line = 0; line < 5; line++)
            {
                for (int column = line; column < 5; column++)
                {
                    for (int freq = 0; freq < 128; freq++)
                    {
                        *cursor = line * 10000 + column * 1000 + freq;
                        cursor++;
                        if (line != column)
                        {
                            *cursor = -(line * 10000 + column * 1000 + freq);
                            cursor++;
                        }
                    }
                }
            }
        }
        lfr_triangular_spectral_matrix_t view(flat_sm.data());

        for (int frequency = 0; frequency < 128; frequency++)
        {
            for (int line = 0; line < 5; line++)
            {
                for (int column = line; column < 5; column++)
                {
                    auto expected = (line * 10000 + column * 1000 + frequency);
                    REQUIRE(view.real(frequency, line, column) == expected);
                    if (line != column)
                    {
                        REQUIRE(view.img(frequency, line, column) == -expected);
                    }
                    else
                        REQUIRE(view.img(frequency, line, column) == 0.f);
                }
            }
        }
    }

    extract_upper_triangle(full_matrix, triangular_matrix);

    for (int i = 0; i < 25; i++)
    {
        REQUIRE(triangular_matrix[i] == expected_triangular_matrix[i]);
    }
}

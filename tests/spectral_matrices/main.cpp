#define CATCH_CONFIG_MAIN
#if __has_include(<catch2/catch.hpp>)
    #include <catch2/catch.hpp>
#else
    #include <catch.hpp>
#endif

#include <algorithm>

#include "common/mock_globals.h"
#include "processing/ASM/spectralmatrices.h"
#include "processing/fsw_processing.h"

SCENARIO("F0/F1 SM averaging", "[]")
{
    lfr_test_init();
    ring_node* ring_node_tab[NB_SM_BEFORE_AVF0_F1];
    ring_node_asm asm_sbm;
    ring_node_asm asm_norm;
    asm_msg msgForPRC;

    GIVEN("some random SMs at F0")
    {
        THEN("LFR should be able to compute SM average @F0 without crash")
        {
            ring_node* nodeForAveraging = getRingNodeForAveraging(0);
            for (int i = NB_SM_BEFORE_AVF0_F1 - 1; i >= 0; i--)
            {
                ring_node_tab[i] = nodeForAveraging;
                nodeForAveraging = nodeForAveraging->previous;
            }
            for (int i = 0; i < 8; i++)
            {
                unsigned int nb_norm_bp1 = i;
                unsigned int nb_sbm_bp1 = i;
                SM_average(asm_norm.matrix, asm_sbm.matrix, ring_node_tab, nb_norm_bp1, nb_sbm_bp1,
                    &msgForPRC, 0, ASM_F0_INDICE_START, ASM_F0_INDICE_START + ASM_F0_KEEP_BINS);
            }
        }
    }
    GIVEN("some random SMs at F1")
    {
        THEN("LFR should be able to compute SM average @F1 without crash")
        {
            ring_node* nodeForAveraging = getRingNodeForAveraging(1);
            for (int i = NB_SM_BEFORE_AVF0_F1 - 1; i >= 0; i--)
            {
                ring_node_tab[i] = nodeForAveraging;
                nodeForAveraging = nodeForAveraging->previous;
            }

            for (int i = 0; i < 8; i++)
            {
                unsigned int nb_norm_bp1 = i;
                unsigned int nb_sbm_bp1 = i;
                SM_average(asm_norm.matrix, asm_sbm.matrix, ring_node_tab, nb_norm_bp1, nb_sbm_bp1,
                    &msgForPRC, 0, ASM_F1_INDICE_START, ASM_F1_INDICE_START + ASM_F1_KEEP_BINS);
            }
        }
    }
    GIVEN("some random SMs at F2")
    {
        THEN("LFR should be able to compute SM average @F2 without crash")
        {
            ring_node* nodeForAveraging = getRingNodeForAveraging(1);
            for (int i = 0; i < 8; i++)
            {
                unsigned int nb_norm_bp1 = i;
                SM_average_f2(asm_norm.matrix, nodeForAveraging, nb_norm_bp1, &msgForPRC);
            }
        }
    }
}

SCENARIO("Picking matrices from VHDL repr SMs", "[]")
{
    GIVEN("some SM")
    {
        float SM[128 * 25];
        int index = 0;
        for (int row = 0; row < 5; row++)
        {
            for (int col = row; col < 5; col++)
            {
                if (row == col)
                {
                    for (int fbin = 0; fbin < 128; fbin++)
                    {
                        SM[index++] = fbin * 1000 + col * 10 + row;
                    }
                }
                else
                {
                    for (int fbin = 0; fbin < 128; fbin++)
                    {
                        SM[index++] = fbin * 1000 + col * 10 + row;
                        SM[index++] = -(fbin * 1000 + col * 10 + row);
                    }
                }
            }
        }

        THEN("LFR should be able to extract any bin")
        {
            float mat[25];
            for (int fbin = 0; fbin < 128; fbin++)
            {
                extract_bin_vhdl_repr(SM, mat, fbin);
                int i=0;
                for (int row = 0; row < 5; row++)
                {
                    for (int col = row; col < 5; col++)
                    {
                        if (row == col)
                        {
                            REQUIRE(mat[i++] == fbin * 1000 + col * 10 + row);
                        }
                        else
                        {
                            REQUIRE(mat[i++] == fbin * 1000 + col * 10 + row);
                            REQUIRE(mat[i++] == -(fbin * 1000 + col * 10 + row));
                        }
                    }
                }
            }
        }
    }
}

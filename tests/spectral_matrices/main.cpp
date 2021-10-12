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
    unsigned int nb_norm_bp1 = 0;
    unsigned int nb_sbm_bp1 = 0;

    GIVEN("some random SMs at F0")
    {
        THEN("LFR should be able to compute BP1 without crash")
        {
            ring_node* nodeForAveraging = getRingNodeForAveraging(0);
            for (int i = NB_SM_BEFORE_AVF0_F1 - 1; i >= 0; i--)
            {
                ring_node_tab[i] = nodeForAveraging;
                nodeForAveraging = nodeForAveraging->previous;
            }
            SM_average(asm_norm.matrix, asm_sbm.matrix, ring_node_tab, nb_norm_bp1, nb_sbm_bp1,
                &msgForPRC, 0, ASM_F0_INDICE_START, ASM_F0_INDICE_START + ASM_F0_KEEP_BINS);
        }
    }
    GIVEN("some random SMs at F1")
    {
        THEN("LFR should be able to compute BP1 without crash")
        {
            ring_node* nodeForAveraging = getRingNodeForAveraging(1);
            for (int i = NB_SM_BEFORE_AVF0_F1 - 1; i >= 0; i--)
            {
                ring_node_tab[i] = nodeForAveraging;
                nodeForAveraging = nodeForAveraging->previous;
            }
            SM_average(asm_norm.matrix, asm_sbm.matrix, ring_node_tab, nb_norm_bp1, nb_sbm_bp1,
                &msgForPRC, 0, ASM_F1_INDICE_START, ASM_F1_INDICE_START + ASM_F1_KEEP_BINS);
        }
    }
}

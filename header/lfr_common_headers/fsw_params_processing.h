#ifndef FSW_PARAMS_PROCESSING_H
#define FSW_PARAMS_PROCESSING_H

#define CHANNELF0 0
#define CHANNELF1 1
#define CHANNELF2 2
#define CHANNELF3 3

#define FINETIME_PER_SM_F0   683     // 65536 / 96 = 682.6667
#define FINETIME_PER_SM_F1   4096    // 65536 / 16
#define FINETIME_PER_SM_F2   65536   // 65536 / 1
#define NB_SM_PER_S_F0      96
#define NB_SM_PER_S_F1      16
#define NB_SM_PER_S_F2      1
#define NB_SM_PER_S1_BP_P0  24

#define ASM_COMP_B1B2 1
#define ASM_COMP_B1B3 3
#define ASM_COMP_B1E1 5
#define ASM_COMP_B1E2 7
#define ASM_COMP_B2B3 10
#define ASM_COMP_B2E1 12
#define ASM_COMP_B2E2 14
#define ASM_COMP_B3E1 17
#define ASM_COMP_B3E2 19
#define ASM_COMP_E1E2 22
#define ASM_COMP_B1B1 0
#define ASM_COMP_B2B2 9
#define ASM_COMP_B3B3 16
#define ASM_COMP_E1E1 21
#define ASM_COMP_E2E2 24

#define SM_BYTES_PER_VAL        2
#define NB_BINS_PER_SM          128
#define NB_VALUES_PER_SM        25
#define TOTAL_SIZE_SM           3200    // 25 * 128 = 0xC80
#define TOTAL_SIZE_NORM_BP1_F0   99     // 11 * 9 = 99
#define TOTAL_SIZE_NORM_BP1_F1  117     // 13 * 9 = 117
#define TOTAL_SIZE_NORM_BP1_F2  108     // 12 * 9 = 108
#define TOTAL_SIZE_SBM1_BP1_F0  198     // 22 * 9 = 198
// F0
#define NB_RING_NODES_SM_F0             20  // AT LEAST 8 due to the way the averaging is done
#define NB_RING_NODES_ASM_BURST_SBM_F0  10  // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F0       10  // AT LEAST 3
#define NB_RING_NODES_ASM_F0            3   // AT LEAST 3
// F1
#define NB_RING_NODES_SM_F1             12  // AT LEAST 8 due to the way the averaging is done
#define NB_RING_NODES_ASM_BURST_SBM_F1  5   // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F1       5   // AT LEAST 3
#define NB_RING_NODES_ASM_F1            3   // AT LEAST 3
// F2
#define NB_RING_NODES_SM_F2             5   // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F2       3   // AT LEAST 3
#define NB_RING_NODES_ASM_F2            3   // AT LEAST 3
//
#define NB_BINS_PER_ASM_F0          88
#define NB_BINS_PER_PKT_ASM_F0_1    32
#define NB_BINS_PER_PKT_ASM_F0_2    24
#define DLEN_ASM_F0_PKT_1           3200    // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define DLEN_ASM_F0_PKT_2           2400    // 24 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F0_INDICE_START         16      // 17  - 1, (-1) due to the VHDL behaviour
#define ASM_F0_INDICE_STOP          103     // 104 - 1, 2 packets of 44 bins
//
#define NB_BINS_PER_ASM_F1          104
#define NB_BINS_PER_PKT_ASM_F1_1    36
#define NB_BINS_PER_PKT_ASM_F1_2    32
#define DLEN_ASM_F1_PKT_1           3600    // 36 * 25 * 4, 25 components per matrix, 4 bytes per float
#define DLEN_ASM_F1_PKT_2           3200    // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F1_INDICE_START         5       //   6 - 1, (-1) due to the VHDL behaviour
#define ASM_F1_INDICE_STOP          108     // 109 - 1, 2 packets of 52 bins
//
#define NB_BINS_PER_ASM_F2          96
#define NB_BINS_PER_PKT_ASM_F2      32
#define DLEN_ASM_F2_PKT             3200    // 32 * 25 * 4, 25 components per matrix, 4 bytes per float
#define ASM_F2_INDICE_START         6       //   7 - 1, (-1) due to the VHDL behaviour
#define ASM_F2_INDICE_STOP          101     // 102 - 1, 2 packets of 48 bins
//
#define KCOEFF_BLK_SIZE                 130
#define KCOEFF_FREQ                     2
#define NB_BINS_COMPRESSED_SM_F0        11
#define NB_BINS_COMPRESSED_SM_F1        13
#define NB_BINS_COMPRESSED_SM_F2        12
#define NB_BINS_COMPRESSED_SM           36 // 11 + 12 + 13
#define NB_BINS_COMPRESSED_SM_SBM_F0    22
#define NB_BINS_COMPRESSED_SM_SBM_F1    26
#define NB_BINS_COMPRESSED_SM_SBM_F2    24
//
#define NB_BYTES_PER_BP1                9
#define NB_BYTES_PER_BP2                30
//
#define NB_BINS_TO_AVERAGE_ASM_F0       8
#define NB_BINS_TO_AVERAGE_ASM_F1       8
#define NB_BINS_TO_AVERAGE_ASM_F2       8
#define NB_BINS_TO_AVERAGE_ASM_SBM_F0   4
#define NB_BINS_TO_AVERAGE_ASM_SBM_F1   4
#define NB_BINS_TO_AVERAGE_ASM_SBM_F2   4
//
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F0   275     // 11 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F1   325     // 13 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_NORM_F2   300     // 12 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_SBM_F0    550     // 22 * 25 WORDS
#define TOTAL_SIZE_COMPRESSED_ASM_SBM_F1    650     // 26 * 25 WORDS
#define TOTAL_SIZE_BP1_NORM_F0              99      // 9  * 11 UNSIGNED CHAR
#define TOTAL_SIZE_BP2_NORM_F0              330     // 30 * 11 UNSIGNED CHAR
#define TOTAL_SIZE_BP1_SBM_F0               198     // 9  * 22 UNSIGNED CHAR
// GENERAL
#define NB_SM_BEFORE_AVF0_F1            8   // must be 8 due to the SM_average() function
#define NB_SM_BEFORE_AVF2               1   // must be 1 due to the SM_average_f2() function

#endif // FSW_PARAMS_PROCESSING_H


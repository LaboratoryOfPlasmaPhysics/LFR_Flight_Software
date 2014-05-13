#ifndef FSW_PARAMS_PROCESSING_H
#define FSW_PARAMS_PROCESSING_H

#define NB_BINS_PER_SM          128
#define NB_VALUES_PER_SM        25
#define TOTAL_SIZE_SM           3200    // 25 * 128
#define TOTAL_SIZE_NORM_BP1_F0   99     // 11 * 9 = 99
#define TOTAL_SIZE_NORM_BP1_F1  117     // 13 * 9 = 117
#define TOTAL_SIZE_NORM_BP1_F2  108     // 12 * 9 = 108
#define TOTAL_SIZE_SBM1_BP1_F0  198     // 22 * 9 = 198
//
#define NB_RING_NODES_SM_F0             12  // AT LEAST 3
#define NB_RING_NODES_ASM_BURST_SBM_F0  10  // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F0       10  // AT LEAST 3
#define NB_RING_NODES_SM_F1             3   // AT LEAST 3
#define NB_RING_NODES_ASM_BURST_SBM_F1  5   // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F1       5   // AT LEAST 3
#define NB_RING_NODES_SM_F2             3   // AT LEAST 3
#define NB_RING_NODES_ASM_BURST_SBM_F2  3   // AT LEAST 3
#define NB_RING_NODES_ASM_NORM_F2       3   // AT LEAST 3
//
#define NB_BINS_PER_ASM_F0  88
#define NB_BINS_PER_PKT_ASM_F0  44
#define TOTAL_SIZE_ASM_F0_IN_BYTES   4400    // 25 * 88 * 2
#define ASM_F0_INDICE_START 17      // 88 bins
#define ASM_F0_INDICE_STOP  104     // 2 packets of 44 bins
//
#define NB_BINS_PER_ASM_F1  104
#define NB_BINS_PER_PKT_ASM_F1  52
#define TOTAL_SIZE_ASM_F1_IN_BYTES   5200    // 25 * 104 * 2
#define ASM_F1_INDICE_START 6       // 104 bins
#define ASM_F1_INDICE_STOP  109     // 2 packets of 52 bins
//
#define NB_BINS_PER_ASM_F2  96
#define NB_BINS_PER_PKT_ASM_F2  48
#define TOTAL_SIZE_ASM_F2_IN_BYTES   4800    // 25 * 96 * 2
#define ASM_F2_INDICE_START 7       // 96 bins
#define ASM_F2_INDICE_STOP  102     // 2 packets of 48 bins
//
#define NB_BINS_COMPRESSED_SM_F0        11
#define NB_BINS_COMPRESSED_SM_F1        13
#define NB_BINS_COMPRESSED_SM_F2        12
#define NB_BINS_COMPRESSED_SM_SBM_F0    22
#define NB_BINS_COMPRESSED_SM_SBM_F1    26
#define NB_BINS_COMPRESSED_SM_SBM_F2    24
//
#define NB_BYTES_PER_BP1                9
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
#define TOTAL_SIZE_COMPRESSED_ASM_SBM_F2    600     // 24 * 25 WORDS
#define TOTAL_SIZE_BP1_NORM_F0              99      // 9  * 11 UNSIGNED CHAR
#define TOTAL_SIZE_BP1_SBM_F0               198     // 9  * 22 UNSIGNED CHAR
// GENERAL
#define NB_SM_BEFORE_AVF0               8   // must be 8 due to the SM_average() function
#define NB_SM_BEFORE_AVF1               8   // must be 8 due to the SM_average() function
#define NB_SM_BEFORE_AVF2               1   // must be 1 due to the SM_average_f2() function

#endif // FSW_PARAMS_PROCESSING_H


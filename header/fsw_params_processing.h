#ifndef FSW_PARAMS_PROCESSING_H
#define FSW_PARAMS_PROCESSING_H

#define NB_BINS_PER_SM      128     //
#define NB_VALUES_PER_SM    25      //
#define TOTAL_SIZE_SM       3200    // 25 * 128
#define SM_HEADER           0       //
//
#define NB_BINS_PER_ASM_F0  88
#define TOTAL_SIZE_ASM_F0   2200    // 25 * 88
#define ASM_F0_INDICE_START 17      // 88 bins
#define ASM_F0_INDICE_STOP  104     // 2 packets of 44 bins
//
#define NB_BINS_PER_ASM_F1  104
#define TOTAL_SIZE_ASM_F1   2600    // 25 * 104
#define ASM_F1_INDICE_START 6       // 104 bins
#define ASM_F1_INDICE_STOP  109     // 2 packets of 52 bins
//
#define NB_BINS_PER_ASM_F2  96
#define TOTAL_SIZE_ASM_F2   2400    // 25 * 96
#define ASM_F2_INDICE_START 7       // 96 bins
#define ASM_F2_INDICE_STOP  102     // 2 packets of 48 bins
//
#define NB_BINS_COMPRESSED_SM_F0 11
#define NB_BINS_COMPRESSED_SM_F1 13
#define NB_BINS_COMPRESSED_SM_F2 12
//
#define TOTAL_SIZE_COMPRESSED_MATRIX_f0 (NB_BINS_COMPRESSED_SM_F0 * NB_VALUES_PER_SM)
#define NB_AVERAGE_NORMAL_f0 96*4
#define NB_SM_TO_RECEIVE_BEFORE_AVF0 8

typedef struct {
    volatile unsigned char PE[2];
    volatile unsigned char PB[2];
    volatile unsigned char V0;
    volatile unsigned char V1;
    volatile unsigned char V2_ELLIP_DOP;
    volatile unsigned char SZ;
    volatile unsigned char VPHI;
} BP1_t;

#endif // FSW_PARAMS_PROCESSING_H

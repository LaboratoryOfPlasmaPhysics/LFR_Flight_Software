#ifndef TM_BYTE_POSITIONS_H
#define TM_BYTE_POSITIONS_H

#define PACKET_POS_SEQUENCE_CNT             6   // 4 + 2
#define PACKET_POS_PA_LFR_SID_PKT           20   // 4 + 16

// TC_LFR_LOAD_COMMON_PAR

// TC_LFR_LOAD_NORMAL_PAR
#define DATAFIELD_POS_SY_LFR_N_SWF_L        0
#define DATAFIELD_POS_SY_LFR_N_SWF_P        2
#define DATAFIELD_POS_SY_LFR_N_ASM_P        4
#define DATAFIELD_POS_SY_LFR_N_BP_P0        6
#define DATAFIELD_POS_SY_LFR_N_BP_P1        7
#define DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3  8

// TC_LFR_LOAD_BURST_PAR
#define DATAFIELD_POS_SY_LFR_B_BP_P0        0
#define DATAFIELD_POS_SY_LFR_B_BP_P1        1

// TC_LFR_LOAD_SBM1_PAR
#define DATAFIELD_POS_SY_LFR_S1_BP_P0       0
#define DATAFIELD_POS_SY_LFR_S1_BP_P1       1

// TC_LFR_LOAD_SBM2_PAR
#define DATAFIELD_POS_SY_LFR_S2_BP_P0       0
#define DATAFIELD_POS_SY_LFR_S2_BP_P1       1

// TC_LFR_UPDATE_INFO
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET1    10
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET2    11
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET5    34
#define BYTE_POS_UPDATE_INFO_PARAMETERS_SET6    35

// TC_LFR_ENTER_MODE
#define BYTE_POS_CP_MODE_LFR_SET            11
#define BYTE_POS_CP_LFR_ENTER_MODE_TIME     12

//TC_LFR_LOAD_FBINS_MASK
#define NB_FBINS_MASKS          12
#define NB_BYTES_PER_FREQ_MASK  16
#define NB_BYTES_PER_FBINS_MASK 4

// TC_LFR_LOAD_KCOEFFICIENTS
#define NB_BYTES_PER_FLOAT 4
#define DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY 0 // 10 - 10
#define DATAFIELD_POS_SY_LFR_KCOEFF_1  2        // 12 - 10

#endif // TM_BYTE_POSITIONS_H

#ifndef FSW_PARAMS_H_INCLUDED
#define FSW_PARAMS_H_INCLUDED

#include "fsw_params_processing.h"
#include "fsw_params_nb_bytes.h"
#include "tm_byte_positions.h"
#include "ccsds_types.h"

#define GRSPW_DEVICE_NAME "/dev/grspw0"
#define UART_DEVICE_NAME "/dev/console"

//*******
// MACROS
#ifdef PRINT_MESSAGES_ON_CONSOLE
#define PRINTF(x) printf(x);
#define PRINTF1(x,y) printf(x,y);
#define PRINTF2(x,y,z) printf(x,y,z);
#else
#define PRINTF(x) ;
#define PRINTF1(x,y) ;
#define PRINTF2(x,y,z) ;
#endif

#ifdef BOOT_MESSAGES
#define BOOT_PRINTF(x) printf(x);
#define BOOT_PRINTF1(x,y) printf(x,y);
#define BOOT_PRINTF2(x,y,z) printf(x,y,z);
#else
#define BOOT_PRINTF(x) ;
#define BOOT_PRINTF1(x,y) ;
#define BOOT_PRINTF2(x,y,z) ;
#endif

#ifdef DEBUG_MESSAGES
#define DEBUG_PRINTF(x) printf(x);
#define DEBUG_PRINTF1(x,y) printf(x,y);
#define DEBUG_PRINTF2(x,y,z) printf(x,y,z);
#else
#define DEBUG_PRINTF(x) ;
#define DEBUG_PRINTF1(x,y) ;
#define DEBUG_PRINTF2(x,y,z) ;
#endif

#define CONST_65536 65536   // 2^16
#define CONST_2048  2048    // 2^11
#define CONST_512   512     // 2^9
#define CONST_256   256     // 2^8
#define CONST_128   128     // 2^7
#define UINT8_MAX   255

#define FLOAT_MSBYTE    0
#define FLOAT_LSBYTE    3
#define BITS_PER_BYTE   8
#define INIT_FLOAT      0.
#define INIT_CHAR       0x00
#define INIT_INT        0
#define INT8_ALL_F      0xff
#define INT16_ALL_F     0xffff
#define INT32_ALL_F     0xffffffff
#define INT32_ALL_0     0x00000000
#define SHIFT_1_BYTE    8
#define SHIFT_2_BYTES   16
#define SHIFT_3_BYTES   24
#define SHIFT_4_BYTES   32
#define SHIFT_5_BYTES   40
#define SHIFT_2_BITS    2
#define SHIFT_3_BITS    3
#define SHIFT_4_BITS    4
#define SHIFT_5_BITS    5
#define SHIFT_6_BITS    6
#define SHIFT_7_BITS    7
#define BYTE_0  0
#define BYTE_1  1
#define BYTE_2  2
#define BYTE_3  3
#define BYTE_4  4
#define BYTE_5  5
#define BYTE_6  6
#define BYTE_7  7
#define BYTE0_MASK  0xff00
#define BYTE1_MASK  0x00ff

enum lfr_transition_type_t{
    TRANSITION_NOT_SPECIFIC,
    TRANSITION_NORM_TO_S1,
    TRANSITION_NORM_TO_S2,
    TRANSITION_S1_TO_NORM,
    TRANSITION_S2_TO_NORM,
    TRANSITION_S1_TO_S2,
    TRANSITION_S2_TO_S1
};

typedef struct ring_node
{
    struct ring_node *previous;
    struct ring_node *next;
    unsigned int sid;
    unsigned int coarseTime;
    unsigned int fineTime;
    int buffer_address;
    unsigned int status;
} ring_node;

//************************
// flight software version
// this parameters is handled by the Qt project options

#define NB_PACKETS_PER_GROUP_OF_CWF         8   // 8 packets containing 336 blk
#define NB_PACKETS_PER_GROUP_OF_CWF_LIGHT   4   // 4 packets containing 672 blk
#define NB_SAMPLES_PER_SNAPSHOT 2688    // 336 * 8 = 672 * 4 = 2688
#define TIME_OFFSET 2
#define TIME_OFFSET_IN_BYTES 8
//#define WAVEFORM_EXTENDED_HEADER_OFFSET 22
#define NB_BYTES_SWF_BLK (2 * 6)
#define NB_WORDS_SWF_BLK 3
#define NB_BYTES_CWF3_LIGHT_BLK 6
//#define WFRM_INDEX_OF_LAST_PACKET 6  // waveforms are transmitted in groups of 2048 blocks, 6 packets of 340 and 1 of 8
#define NB_RING_NODES_F0 3      // AT LEAST 3
#define NB_RING_NODES_F1 5      // AT LEAST 3
#define NB_RING_NODES_F2 5      // AT LEAST 3
#define NB_RING_NODES_F3 3      // AT LEAST 3

//**********
// LFR MODES
#define LFR_MODE_STANDBY    0
#define LFR_MODE_NORMAL     1
#define LFR_MODE_BURST      2
#define LFR_MODE_SBM1       3
#define LFR_MODE_SBM2       4

#define TDS_MODE_LFM        5
#define TDS_MODE_STANDBY    0
#define TDS_MODE_NORMAL     1
#define TDS_MODE_BURST      2
#define TDS_MODE_SBM1       3
#define TDS_MODE_SBM2       4

#define THR_MODE_STANDBY    0
#define THR_MODE_NORMAL     1
#define THR_MODE_BURST      2

#define RTEMS_EVENT_MODE_STANDBY    RTEMS_EVENT_0
#define RTEMS_EVENT_MODE_NORMAL     RTEMS_EVENT_1
#define RTEMS_EVENT_MODE_BURST      RTEMS_EVENT_2
#define RTEMS_EVENT_MODE_SBM1       RTEMS_EVENT_3
#define RTEMS_EVENT_MODE_SBM2       RTEMS_EVENT_4
#define RTEMS_EVENT_MODE_NORM_S1_S2 RTEMS_EVENT_5
#define RTEMS_EVENT_NORM_BP1_F0     RTEMS_EVENT_6
#define RTEMS_EVENT_NORM_BP2_F0     RTEMS_EVENT_7
#define RTEMS_EVENT_NORM_ASM_F0     RTEMS_EVENT_8   // ASM only in NORM mode
#define RTEMS_EVENT_NORM_BP1_F1     RTEMS_EVENT_9
#define RTEMS_EVENT_NORM_BP2_F1     RTEMS_EVENT_10
#define RTEMS_EVENT_NORM_ASM_F1     RTEMS_EVENT_11  // ASM only in NORM mode
#define RTEMS_EVENT_NORM_BP1_F2     RTEMS_EVENT_12
#define RTEMS_EVENT_NORM_BP2_F2     RTEMS_EVENT_13
#define RTEMS_EVENT_NORM_ASM_F2     RTEMS_EVENT_14  // ASM only in NORM mode
#define RTEMS_EVENT_SBM_BP1_F0      RTEMS_EVENT_15
#define RTEMS_EVENT_SBM_BP2_F0      RTEMS_EVENT_16
#define RTEMS_EVENT_SBM_BP1_F1      RTEMS_EVENT_17
#define RTEMS_EVENT_SBM_BP2_F1      RTEMS_EVENT_18
#define RTEMS_EVENT_BURST_BP1_F0    RTEMS_EVENT_19
#define RTEMS_EVENT_BURST_BP2_F0    RTEMS_EVENT_20
#define RTEMS_EVENT_BURST_BP1_F1    RTEMS_EVENT_21
#define RTEMS_EVENT_BURST_BP2_F1    RTEMS_EVENT_22
#define RTEMS_EVENT_SWF_RESYNCH     RTEMS_EVENT_23

//********************************************
//********************************************
// LFR PARAMETERS: DEFAULT, MIN AND MAX VALUES

#define DEFAULT_LAST_VALID_TRANSITION_DATE 0xffffffff

// COMMON
#define DEFAULT_SY_LFR_COMMON0 0x00
#define DEFAULT_SY_LFR_COMMON1 0x20 // default value bw sp0 sp1 r0 r1 r2 = 1 0 0 0 0 0

// NORM
#define DFLT_SY_LFR_N_SWF_L 2048 // nb sample
#define DFLT_SY_LFR_N_SWF_P 300  // sec
#define MIN_SY_LFR_N_SWF_P  22  // sec
#define DFLT_SY_LFR_N_ASM_P 3600 // sec
#define DFLT_SY_LFR_N_BP_P0 4    // sec
#define DFLT_SY_LFR_N_BP_P1 20   // sec
#define DFLT_SY_LFR_N_CWF_LONG_F3 0      // 0 => production of light continuous waveforms at f3
#define MIN_DELTA_SNAPSHOT 16       // sec

// BURST
#define DEFAULT_SY_LFR_B_BP_P0 1    // sec
#define DEFAULT_SY_LFR_B_BP_P1 5    // sec

// SBM1
#define S1_BP_P0_SCALE          0.25
#define DEFAULT_SY_LFR_S1_BP_P0 1   // 0.25 sec
#define DEFAULT_SY_LFR_S1_BP_P1 1   // sec

// SBM2
#define DEFAULT_SY_LFR_S2_BP_P0 1   // sec
#define DEFAULT_SY_LFR_S2_BP_P1 5   // sec

// ADDITIONAL PARAMETERS
#define TIME_BETWEEN_TWO_SWF_PACKETS 30     // nb x 10 ms => 300 ms
#define TIME_BETWEEN_TWO_CWF3_PACKETS 1000  // nb x 10 ms => 10 s

// STATUS WORD
#define DEFAULT_STATUS_WORD_BYTE0 0x0d  // [0000] [1] [101] mode 4 bits / SPW enabled 1 bit / state is run 3 bits

#define DEFAULT_STATUS_WORD_BYTE1 0x00
// TC_LFR_LOAD_FILTER_PAR
#define MIN_PAS_FILTER_MODULUS  4
#define MAX_PAS_FILTER_MODULUS  8
#define MIN_PAS_FILTER_TBAD     0.0
#define MAX_PAS_FILTER_TBAD     4.0
#define MIN_PAS_FILTER_OFFSET   0
#define MAX_PAS_FILTER_OFFSET   7
#define MIN_PAS_FILTER_SHIFT    0.0
#define MAX_PAS_FILTER_SHIFT    1.0
#define MIN_SY_LFR_SC_RW_DELTA_F    0
#define MIN_SY_LFR_RW_K             0
#define MIN_SY_LFR_RW_F             0
//
#define SY_LFR_DPU_CONNECT_TIMEOUT 100  // 100 * 10 ms = 1 s
#define SY_LFR_DPU_CONNECT_ATTEMPT 3
//****************************

//*****************************
// APB REGISTERS BASE ADDRESSES
#define REGS_ADDR_APBUART           0x80000100
#define REGS_ADDR_GPTIMER           0x80000300
#define REGS_ADDR_GRSPW             0x80000500
#define APB_OFFSET_GRSPW_STATUS_REGISTER    0x04
#define APB_OFFSET_GRSPW_TIME_REGISTER      0x14
#define REGS_ADDR_TIME_MANAGEMENT   0x80000600
#define REGS_ADDR_GRGPIO            0x80000b00

#define REGS_ADDR_SPECTRAL_MATRIX   0x80000f00
#define REGS_ADDR_WAVEFORM_PICKER   0x80000f54  // PDB >= 0.1.28
#define APB_OFFSET_VHDL_REV         0xb0
#define REGS_ADDR_VHDL_VERSION      0x80000ff0

#define APBUART_CTRL_REG_MASK_DB    0xfffff7ff
#define APBUART_CTRL_REG_MASK_TE    0x00000002
// scaler value = system_clock_frequency / ( baud_rate * 8 ) - 1
#define APBUART_SCALER_RELOAD_VALUE 0x00000050      // 25 MHz => about 38400

//**********
// IRQ LINES
#define IRQ_GPTIMER_WATCHDOG 9
#define IRQ_SPARC_GPTIMER_WATCHDOG 0x19     // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WAVEFORM_PICKER 14
#define IRQ_SPARC_WAVEFORM_PICKER 0x1e  // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_SPECTRAL_MATRIX 6
#define IRQ_SPARC_SPECTRAL_MATRIX 0x16  // see sparcv8.pdf p.76 for interrupt levels

//*****
// TIME
#define CLKDIV_WATCHDOG     (10000000 - 1)       // 10.0s => 10 000 000
#define TIMER_WATCHDOG      1
#define WATCHDOG_PERIOD     100                 // 1s
#define HK_PERIOD                           100     // 100 * 10ms => 1s
#define AVGV_PERIOD                         6       //   6 * 10ms => 60ms (1 / 16 = 62.5ms)
#define SY_LFR_TIME_SYN_TIMEOUT_in_ticks    200     // 200 * 10 ms = 2 s
#define HK_SYNC_WAIT                        10      //  10 * 10 ms = 100 ms
#define SPW_LINK_WAIT                       10      //  10 * 10 ms = 100 ms
#define TIMECODE_TIMER_TIMEOUT              120     // 120 * 10 ms = 1.2 s
#define TIMECODE_TIMER_TIMEOUT_INIT         200     // 200 * 10 ms = 2.0 s
#define TIMECODE_MASK                       0x3f    // 0011 1111

//**********
// LPP CODES
#define LFR_SUCCESSFUL  0
#define LFR_DEFAULT     1
#define LFR_EXE_ERROR   2
#define LFR_DEFAULT_ALT -1

//******
// RTEMS
#define STACK_SIZE_MULT 2

#define TASKID_AVGV 0
#define TASKID_RECV 1
#define TASKID_ACTN 2
#define TASKID_SPIQ 3
#define TASKID_LOAD 4
#define TASKID_AVF0 5
#define TASKID_SWBD 6
#define TASKID_WFRM 7
#define TASKID_DUMB 8
#define TASKID_HOUS 9
#define TASKID_PRC0 10
#define TASKID_CWF3 11
#define TASKID_CWF2 12
#define TASKID_CWF1 13
#define TASKID_SEND 14
#define TASKID_LINK 15
#define TASKID_AVF1 16
#define TASKID_PRC1 17
#define TASKID_AVF2 18
#define TASKID_PRC2 19

#define TASK_PRIORITY_SPIQ 5
#define TASK_PRIORITY_LINK 20
#define TASK_PRIORITY_AVGV 25
#define TASK_PRIORITY_HOUS 30
#define TASK_PRIORITY_CWF1 35   // CWF1 and CWF2 are never running together
#define TASK_PRIORITY_CWF2 35   //
#define TASK_PRIORITY_SWBD 37   // SWBD has a lower priority than WFRM, this is to extract the snapshot before sending it
#define TASK_PRIORITY_WFRM 40
#define TASK_PRIORITY_CWF3 40   // there is a printf in this function, be careful with its priority wrt CWF1
#define TASK_PRIORITY_SEND 45
#define TASK_PRIORITY_RECV 50
#define TASK_PRIORITY_ACTN 50
#define TASK_PRIORITY_AVF0 60
#define TASK_PRIORITY_AVF1 70
#define TASK_PRIORITY_PRC0 100
#define TASK_PRIORITY_PRC1 100
#define TASK_PRIORITY_AVF2 110
#define TASK_PRIORITY_PRC2 110
#define TASK_PRIORITY_LOAD 190
#define TASK_PRIORITY_DUMB 200

#define MSG_QUEUE_COUNT_RECV  10
#define MSG_QUEUE_COUNT_SEND  50
#define MSG_QUEUE_COUNT_PRC0  10
#define MSG_QUEUE_COUNT_PRC1  10
#define MSG_QUEUE_COUNT_PRC2  5
#define MSG_QUEUE_SIZE_SEND             812 // 808 + 4 => TM_LFR_SCIENCE_BURST_BP2_F1
#define ACTION_MSG_SPW_IOCTL_SEND_SIZE  24  // hlen *hdr dlen *data sent options
#define MSG_QUEUE_SIZE_PRC0             36  // two pointers, one rtems_event + 6 integers
#define MSG_QUEUE_SIZE_PRC1             36  // two pointers, one rtems_event + 6 integers
#define MSG_QUEUE_SIZE_PRC2             36  // two pointers, one rtems_event + 6 integers

#define QUEUE_RECV 0
#define QUEUE_SEND 1
#define QUEUE_PRC0 2
#define QUEUE_PRC1 3
#define QUEUE_PRC2 4

#define CPU_USAGE_REPORT_PERIOD 6   // * 10 s = period

struct param_local_str{
    unsigned int local_sbm1_nb_cwf_sent;
    unsigned int local_sbm1_nb_cwf_max;
    unsigned int local_sbm2_nb_cwf_sent;
    unsigned int local_sbm2_nb_cwf_max;
};

//************
// FBINS MASKS

#define BYTES_PER_FBINS_MASK 16

typedef struct {
    unsigned char merged_fbins_mask_f0[BYTES_PER_FBINS_MASK];
    unsigned char merged_fbins_mask_f1[BYTES_PER_FBINS_MASK];
    unsigned char merged_fbins_mask_f2[BYTES_PER_FBINS_MASK];
} fbins_masks_t;

#define DEFAULT_SY_LFR_PAS_FILTER_ENABLED   0
#define DEFAULT_SY_LFR_PAS_FILTER_MODULUS   4
#define DEFAULT_SY_LFR_PAS_FILTER_TBAD      1.0
#define DEFAULT_SY_LFR_PAS_FILTER_OFFSET    0
#define DEFAULT_SY_LFR_PAS_FILTER_SHIFT     0.5
#define DEFAULT_SY_LFR_SC_RW_DELTA_F        0.045
#define DEFAULT_SY_LFR_RW_K1                1.
#define DEFAULT_SY_LFR_RW_K2                8.
#define DEFAULT_SY_LFR_RW_K3                24.
#define DEFAULT_SY_LFR_RW_K4                48.

typedef struct{
    unsigned char spare_sy_lfr_pas_filter_enabled;
    unsigned char sy_lfr_pas_filter_modulus;
    float sy_lfr_pas_filter_tbad;
    unsigned char sy_lfr_pas_filter_offset;
    float sy_lfr_pas_filter_shift;
    float sy_lfr_sc_rw_delta_f;
    // rw1_k
    float sy_lfr_rw1_k1;
    float sy_lfr_rw1_k2;
    float sy_lfr_rw1_k3;
    float sy_lfr_rw1_k4;
    // rw2_k
    float sy_lfr_rw2_k1;
    float sy_lfr_rw2_k2;
    float sy_lfr_rw2_k3;
    float sy_lfr_rw2_k4;
    // rw3_k
    float sy_lfr_rw3_k1;
    float sy_lfr_rw3_k2;
    float sy_lfr_rw3_k3;
    float sy_lfr_rw3_k4;
    // rw4_k
    float sy_lfr_rw4_k1;
    float sy_lfr_rw4_k2;
    float sy_lfr_rw4_k3;
    float sy_lfr_rw4_k4;
} filterPar_t;

typedef struct{
    // rw1_f
    float cp_rpw_sc_rw1_f1;
    float cp_rpw_sc_rw1_f2;
    float cp_rpw_sc_rw1_f3;
    float cp_rpw_sc_rw1_f4;
    // rw2_f
    float cp_rpw_sc_rw2_f1;
    float cp_rpw_sc_rw2_f2;
    float cp_rpw_sc_rw2_f3;
    float cp_rpw_sc_rw2_f4;
    // rw3_f
    float cp_rpw_sc_rw3_f1;
    float cp_rpw_sc_rw3_f2;
    float cp_rpw_sc_rw3_f3;
    float cp_rpw_sc_rw3_f4;
    // rw4_f
    float cp_rpw_sc_rw4_f1;
    float cp_rpw_sc_rw4_f2;
    float cp_rpw_sc_rw4_f3;
    float cp_rpw_sc_rw4_f4;
} rw_f_t;

#define MATRIX_IS_POLLUTED      0
#define MATRIX_IS_NOT_POLLUTED  1
#define ACQUISITION_DURATION_F0 683     // 256 / 24576 * 65536
#define ACQUISITION_DURATION_F1 4096    // 256 /  4096 * 65536
#define ACQUISITION_DURATION_F2 65536   // 256 /   256 * 65536
#define HALF_ACQUISITION_DURATION_F0 341     // 256 / 24576 * 65536 / 2
#define HALF_ACQUISITION_DURATION_F1 2048    // 256 /  4096 * 65536 / 2
#define HALF_ACQUISITION_DURATION_F2 32768   // 256 /   256 * 65536 / 2

#endif // FSW_PARAMS_H_INCLUDED

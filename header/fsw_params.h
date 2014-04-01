#ifndef FSW_PARAMS_H_INCLUDED
#define FSW_PARAMS_H_INCLUDED

#include "grlib_regs.h"
#include "fsw_params_processing.h"
#include "fsw_params_nb_bytes.h"
#include "tm_byte_positions.h"
#include "ccsds_types.h"

#define GRSPW_DEVICE_NAME "/dev/grspw0"
#define UART_DEVICE_NAME "/dev/console"

typedef struct ring_node
{
    struct ring_node *previous;
    int buffer_address;
    struct ring_node *next;
    unsigned int status;
} ring_node;

typedef struct ring_node_sm
{
    struct ring_node_sm *previous;
    int buffer_address;
    struct ring_node_sm *next;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
} ring_node_sm;

//************************
// flight software version
// this parameters is handled by the Qt project options

#define NB_PACKETS_PER_GROUP_OF_CWF         8   // 8 packets containing 336 blk
#define NB_PACKETS_PER_GROUP_OF_CWF_LIGHT   4   // 4 packets containing 672 blk
#define NB_SAMPLES_PER_SNAPSHOT 2688    // 336 * 8 = 672 * 4 = 2688
#define TIME_OFFSET 2
#define TIME_OFFSET_IN_BYTES 8
#define WAVEFORM_EXTENDED_HEADER_OFFSET 22
#define NB_BYTES_SWF_BLK (2 * 6)
#define NB_WORDS_SWF_BLK 3
#define NB_BYTES_CWF3_LIGHT_BLK 6
#define WFRM_INDEX_OF_LAST_PACKET 6  // waveforms are transmitted in groups of 2048 blocks, 6 packets of 340 and 1 of 8
#define NB_RING_NODES_F0 3      // AT LEAST 3
#define NB_RING_NODES_F1 5      // AT LEAST 3
#define NB_RING_NODES_F2 5      // AT LEAST 3

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

#define RTEMS_EVENT_MODE_STANDBY        RTEMS_EVENT_0
#define RTEMS_EVENT_MODE_NORMAL         RTEMS_EVENT_1
#define RTEMS_EVENT_MODE_BURST          RTEMS_EVENT_2
#define RTEMS_EVENT_MODE_SBM1           RTEMS_EVENT_3
#define RTEMS_EVENT_MODE_SBM2           RTEMS_EVENT_4
#define RTEMS_EVENT_MODE_SBM2_WFRM      RTEMS_EVENT_5
#define RTEMS_EVENT_MODE_NORMAL_SWF_F0  RTEMS_EVENT_6
#define RTEMS_EVENT_MODE_NORMAL_SWF_F1  RTEMS_EVENT_7
#define RTEMS_EVENT_MODE_NORMAL_SWF_F2  RTEMS_EVENT_8

//****************************
// LFR DEFAULT MODE PARAMETERS
// COMMON
#define DEFAULT_SY_LFR_COMMON0 0x00
#define DEFAULT_SY_LFR_COMMON1 0x10 // default value 0 0 0 1 0 0 0 0
// NORM
#define SY_LFR_N_SWF_L 2048 // nb sample
#define SY_LFR_N_SWF_P 300  // sec
#define SY_LFR_N_ASM_P 3600 // sec
#define SY_LFR_N_BP_P0 4    // sec
#define SY_LFR_N_BP_P1 20   // sec
#define SY_LFR_N_CWF_LONG_F3 0      // 0 => production of light continuous waveforms at f3
#define MIN_DELTA_SNAPSHOT 16       // sec
// BURST
#define DEFAULT_SY_LFR_B_BP_P0 1    // sec
#define DEFAULT_SY_LFR_B_BP_P1 5    // sec
// SBM1
#define DEFAULT_SY_LFR_S1_BP_P0 1   // sec
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
//
#define SY_LFR_DPU_CONNECT_TIMEOUT 100  // 100 * 10 ms = 1 s
#define SY_LFR_DPU_CONNECT_ATTEMPT 3
//****************************

//*****************************
// APB REGISTERS BASE ADDRESSES
#define REGS_ADDR_APBUART           0x80000100
#define REGS_ADDR_GPTIMER           0x80000300
#define REGS_ADDR_GRSPW             0x80000500
#define REGS_ADDR_TIME_MANAGEMENT   0x80000600
#define REGS_ADDR_GRGPIO            0x80000b00

#define REGS_ADDR_SPECTRAL_MATRIX   0x80000f00
#define REGS_ADDR_WAVEFORM_PICKER   0x80000f40

#define APBUART_CTRL_REG_MASK_DB    0xfffff7ff
#define APBUART_CTRL_REG_MASK_TE    0x00000002
#define APBUART_SCALER_RELOAD_VALUE 0x00000050      // 25 MHz => about 38400 (0x50)

//**********
// IRQ LINES
#define IRQ_SM_SIMULATOR 9
#define IRQ_SPARC_SM_SIMULATOR 0x19     // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WAVEFORM_PICKER 14
#define IRQ_SPARC_WAVEFORM_PICKER 0x1e  // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_SPECTRAL_MATRIX 6
#define IRQ_SPARC_SPECTRAL_MATRIX 0x16  // see sparcv8.pdf p.76 for interrupt levels

//*****
// TIME
#define CLKDIV_SM_SIMULATOR (10000 - 1)     // 10 ms
#define TIMER_SM_SIMULATOR 1
#define HK_PERIOD                           100     // 100 * 10ms => 1s
#define SY_LFR_TIME_SYN_TIMEOUT_in_ms       2000
#define SY_LFR_TIME_SYN_TIMEOUT_in_ticks    200     // 200 * 10 ms = 2 s

//**********
// LPP CODES
#define LFR_SUCCESSFUL 0
#define LFR_DEFAULT 1

//******
// RTEMS
#define TASKID_RECV 1
#define TASKID_ACTN 2
#define TASKID_SPIQ 3
#define TASKID_SMIQ 4
#define TASKID_STAT 5
#define TASKID_AVF0 6
#define TASKID_SWBD 7
#define TASKID_WFRM 8
#define TASKID_DUMB 9
#define TASKID_HOUS 10
#define TASKID_MATR 11
#define TASKID_CWF3 12
#define TASKID_CWF2 13
#define TASKID_CWF1 14
#define TASKID_SEND 15
#define TASKID_WTDG 16

#define TASK_PRIORITY_SPIQ 5
#define TASK_PRIORITY_SMIQ 10
#define TASK_PRIORITY_WTDG 20
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
#define TASK_PRIORITY_BPF0 60
#define TASK_PRIORITY_MATR 100
#define TASK_PRIORITY_STAT 200
#define TASK_PRIORITY_DUMB 200

#define ACTION_MSG_QUEUE_COUNT 10
#define ACTION_MSG_PKTS_COUNT 50
#define ACTION_MSG_PKTS_MAX_SIZE (PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES)
#define ACTION_MSG_SPW_IOCTL_SEND_SIZE 24                   // hlen *hdr dlen *data sent options

#define QUEUE_RECV 0
#define QUEUE_SEND 1

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

#define CPU_USAGE_REPORT_PERIOD 6   // * 10 s = period

struct param_local_str{
    unsigned int local_sbm1_nb_cwf_sent;
    unsigned int local_sbm1_nb_cwf_max;
    unsigned int local_sbm2_nb_cwf_sent;
    unsigned int local_sbm2_nb_cwf_max;
    unsigned int local_nb_interrupt_f0_MAX;
};

#endif // FSW_PARAMS_H_INCLUDED

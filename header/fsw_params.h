#ifndef FSW_RTEMS_CONFIG_H_INCLUDED
#define FSW_RTEMS_CONFIG_H_INCLUDED

#include <fsw_params_processing.h>

#define GRSPW_DEVICE_NAME "/dev/grspw0"
#define UART_DEVICE_NAME "/dev/console"

//************************
// flight software version
// this parameters is handled by the Qt project options

//**********
// LFR MODES
#define LFR_MODE_STANDBY 0
#define LFR_MODE_NORMAL 1
#define LFR_MODE_BURST 2
#define LFR_MODE_SBM1 3
#define LFR_MODE_SBM2 4

#define RTEMS_EVENT_MODE_STANDBY RTEMS_EVENT_0
#define RTEMS_EVENT_MODE_NORMAL RTEMS_EVENT_1
#define RTEMS_EVENT_MODE_BURST RTEMS_EVENT_2
#define RTEMS_EVENT_MODE_SBM1 RTEMS_EVENT_3
#define RTEMS_EVENT_MODE_SBM2 RTEMS_EVENT_4

//****************************
// LFR DEFAULT MODE PARAMETERS
// COMMON
#define DEFAULT_SY_LFR_COMMON0 0x00
#define DEFAULT_SY_LFR_COMMON1 0x10 // default value 0 0 0 1 0 0 0 0
// NORM
#define DEFAULT_SY_LFR_N_SWF_L 2048 // nb sample
#define DEFAULT_SY_LFR_N_SWF_P 16   // sec
#define DEFAULT_SY_LFR_N_ASM_P 16 // sec
#define DEFAULT_SY_LFR_N_BP_P0 4    // sec
#define DEFAULT_SY_LFR_N_BP_P1 20   // sec
// BURST
#define DEFAULT_SY_LFR_B_BP_P0 1    // sec
#define DEFAULT_SY_LFR_B_BP_P1 5    // sec
// SBM1
#define DEFAULT_SY_LFR_S1_BP_P0 1   // sec
#define DEFAULT_SY_LFR_S1_BP_P1 1   // sec
// SBM2
#define DEFAULT_SY_LFR_S2_BP_P0 1   // sec
#define DEFAULT_SY_LFR_S2_BP_P1 5   // sec

//*****************************
// APB REGISTERS BASE ADDRESSES
#define REGS_ADDR_APBUART 0x80000100
#define REGS_ADDR_GPTIMER 0x80000300
#define REGS_ADDR_GRSPW 0x80000500
#define REGS_ADDR_TIME_MANAGEMENT 0x80000600
#define REGS_ADDR_SPECTRAL_MATRIX 0x80000f00

#ifdef GSA
#else
    #define REGS_ADDR_WAVEFORM_PICKER 0x80000f20
#endif

#define APBUART_CTRL_REG_MASK_DB 0xfffff7ff
#define APBUART_SCALER_RELOAD_VALUE 0x00000050      // 25 MHz => about 38400 (0x50)

//**********
// IRQ LINES
#define IRQ_SM 9
#define IRQ_SPARC_SM 0x19               // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WF 10
#define IRQ_SPARC_WF 0x1a               // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_TIME1 12
#define IRQ_SPARC_TIME1 0x1c            // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_TIME2 13
#define IRQ_SPARC_TIME2 0x1d            // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WAVEFORM_PICKER 14
#define IRQ_SPARC_WAVEFORM_PICKER 0x1e  // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_SPECTRAL_MATRIX 6
#define IRQ_SPARC_SPECTRAL_MATRIX 0x16  // see sparcv8.pdf p.76 for interrupt levels

//*****
// TIME
#define CLKDIV_SM_SIMULATOR (10000 - 1)     // 10 ms
#define CLKDIV_WF_SIMULATOR (10000000 - 1)  // 10 000 000 * 1 us = 10 s
#define TIMER_SM_SIMULATOR 1
#define TIMER_WF_SIMULATOR 2
#define HK_PERIOD 100 // 100 * 10ms => 1sec

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
#define TASKID_BPF0 7
#define TASKID_WFRM 8
#define TASKID_DUMB 9
#define TASKID_HOUS 10
#define TASKID_MATR 11

#define ACTION_MSG_QUEUE_COUNT 10

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

#define NB_SAMPLES_PER_SNAPSHOT 2048
#define TIME_OFFSET 2
#define WAVEFORM_EXTENDED_HEADER_OFFSET 22
#define NB_BYTES_SWF_BLK 2 * 6
#define NB_WORDS_SWF_BLK 3

//******************
// SEQUENCE COUNTERS
#define SEQ_CNT_NB_PID 2
#define SEQ_CNT_NB_CAT 4
#define SEQ_CNT_NB_DEST_ID 11
// pid
#define SEQ_CNT_PID_76 0
#define SEQ_CNT_PID_79 1
//cat
#define SEQ_CNT_CAT_1 0
#define SEQ_CNT_CAT_4 1
#define SEQ_CNT_CAT_9 2
#define SEQ_CNT_CAT_12 3
// destination id
#define SEQ_CNT_DST_ID_GROUND 0
#define SEQ_CNT_DST_ID_MISSION_TIMELINE 1
#define SEQ_CNT_DST_ID_TC_SEQUENCES 2
#define SEQ_CNT_DST_ID_RECOVERY_ACTION_CMD 3
#define SEQ_CNT_DST_ID_BACKUP_MISSION_TIMELINE 4
#define SEQ_CNT_DST_ID_DIRECT_CMD 5
#define SEQ_CNT_DST_ID_SPARE_GRD_SRC1 6
#define SEQ_CNT_DST_ID_SPARE_GRD_SRC2 7
#define SEQ_CNT_DST_ID_OBCP 8
#define SEQ_CNT_DST_ID_SYSTEM_CONTROL 9
#define SEQ_CNT_DST_ID_AOCS 10

struct param_local_str{
    unsigned int local_sbm1_nb_cwf_sent;
    unsigned int local_sbm1_nb_cwf_max;
    unsigned int local_sbm2_nb_cwf_sent;
    unsigned int local_sbm2_nb_cwf_max;
};

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

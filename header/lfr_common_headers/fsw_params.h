/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

#ifndef FSW_PARAMS_H_INCLUDED
#define FSW_PARAMS_H_INCLUDED

#include "ccsds_types.h"
#include "fsw_params_nb_bytes.h"
#include "fsw_params_processing.h"
#include "stdint.h"
#include "tm_byte_positions.h"

/*
 * RTEMS CONFIG
 *
 */
#define GRSPW_DEVICE_NAME "/dev/grspw0"

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS 23 // number of tasks concurrently active including INIT
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS              (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY             1 // instead of 100
#define CONFIGURE_INIT_TASK_MODE                 (RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT)
#define CONFIGURE_INIT_TASK_ATTRIBUTES           (RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT)
#define CONFIGURE_MAXIMUM_DRIVERS                16
#define CONFIGURE_MAXIMUM_PERIODS                6 // [hous] [load] [avgv]
#define CONFIGURE_MAXIMUM_TIMERS                 6 // [spiq] [link] [spacewire_reset_link]
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES         5
#ifdef PRINT_STACK_REPORT
    #define CONFIGURE_STACK_CHECKER_ENABLED
#endif


//*******
// MACROS
#ifdef PRINT_MESSAGES_ON_CONSOLE
    #define PRINTF(x)        printf(x);
    #define PRINTF1(x, y)    printf(x, y);
    #define PRINTF2(x, y, z) printf(x, y, z);
#else
    #define PRINTF(x)        ;
    #define PRINTF1(x, y)    ;
    #define PRINTF2(x, y, z) ;
#endif

#ifdef BOOT_MESSAGES
    #define BOOT_PRINTF(x)        printf(x);
    #define BOOT_PRINTF1(x, y)    printf(x, y);
    #define BOOT_PRINTF2(x, y, z) printf(x, y, z);
#else
    #define BOOT_PRINTF(x)        ;
    #define BOOT_PRINTF1(x, y)    ;
    #define BOOT_PRINTF2(x, y, z) ;
#endif

#ifdef DEBUG_MESSAGES
    #define DEBUG_PRINTF(x)        printf(x);
    #define DEBUG_PRINTF1(x, y)    printf(x, y);
    #define DEBUG_PRINTF2(x, y, z) printf(x, y, z);
#else
    #define DEBUG_PRINTF(x)        ;
    #define DEBUG_PRINTF1(x, y)    ;
    #define DEBUG_PRINTF2(x, y, z) ;
#endif

#define CONST_65536 65536 // 2^16
#define CONST_2048  2048 // 2^11
#define CONST_512   512 // 2^9
#define CONST_256   256 // 2^8
#ifndef UINT8_MAX
    #define UINT8_MAX 255
#endif

#define FLOAT_LSBYTE  3
#define BITS_PER_BYTE 8
#define INIT_FLOAT    0.
#define INIT_CHAR     0x00
#define INIT_INT      0
#define INT8_ALL_F    0xff
#define INT16_ALL_F   0xffff
#define INT32_ALL_F   0xffffffff
#define INT32_ALL_0   0x00000000
#define SHIFT_1_BYTE  8
#define SHIFT_2_BYTES 16
#define SHIFT_3_BYTES 24
#define SHIFT_4_BYTES 32
#define SHIFT_5_BYTES 40
#define SHIFT_2_BITS  2
#define SHIFT_3_BITS  3
#define SHIFT_4_BITS  4
#define SHIFT_5_BITS  5
#define SHIFT_7_BITS  7
#define BYTE_0        0
#define BYTE_1        1
#define BYTE_2        2
#define BYTE_3        3
#define BYTE_4        4
#define BYTE_5        5
#define BYTE_6        6
#define BYTE_7        7
#define BYTE0_MASK    0xff00
#define BYTE1_MASK    0x00ff

enum lfr_transition_type_t
{
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
    struct ring_node* previous;
    struct ring_node* next;
    unsigned int sid;
    unsigned int coarseTime;
    unsigned int fineTime;
    void* buffer_address;
    unsigned int status;
} ring_node;

//************************
// flight software version
// this parameters is handled by the Qt project options

#define NB_PACKETS_PER_GROUP_OF_CWF       8 // 8 packets containing 336 blk
#define NB_PACKETS_PER_GROUP_OF_CWF_LIGHT 4 // 4 packets containing 672 blk
#define NB_SAMPLES_PER_SNAPSHOT           2688 // 336 * 8 = 672 * 4 = 2688
#define TIME_OFFSET                       2
#define NB_BYTES_SWF_BLK                  (2 * 6)
#define NB_WORDS_SWF_BLK                  3
#define NB_BYTES_CWF3_LIGHT_BLK           6
#define NB_RING_NODES_F0                  3 // AT LEAST 3
#define NB_RING_NODES_F1                  5 // AT LEAST 3
#define NB_RING_NODES_F2                  5 // AT LEAST 3
#define NB_RING_NODES_F3                  3 // AT LEAST 3

//**********
// LFR MODES
#define LFR_MODE_STANDBY 0
#define LFR_MODE_NORMAL  1
#define LFR_MODE_BURST   2
#define LFR_MODE_SBM1    3
#define LFR_MODE_SBM2    4

#define TDS_MODE_LFM     5
#define TDS_MODE_STANDBY 0
#define TDS_MODE_NORMAL  1
#define TDS_MODE_BURST   2
#define TDS_MODE_SBM1    3
#define TDS_MODE_SBM2    4

#define THR_MODE_STANDBY 0
#define THR_MODE_NORMAL  1
#define THR_MODE_BURST   2

#define RTEMS_EVENT_MODE_NORMAL     RTEMS_EVENT_1
#define RTEMS_EVENT_MODE_BURST      RTEMS_EVENT_2
#define RTEMS_EVENT_MODE_SBM2       RTEMS_EVENT_4
#define RTEMS_EVENT_MODE_NORM_S1_S2 RTEMS_EVENT_5
#define RTEMS_EVENT_NORM_BP1_F0     RTEMS_EVENT_6
#define RTEMS_EVENT_NORM_BP2_F0     RTEMS_EVENT_7
#define RTEMS_EVENT_NORM_ASM_F0     RTEMS_EVENT_8 // ASM only in NORM mode
#define RTEMS_EVENT_NORM_BP1_F1     RTEMS_EVENT_9
#define RTEMS_EVENT_NORM_BP2_F1     RTEMS_EVENT_10
#define RTEMS_EVENT_NORM_ASM_F1     RTEMS_EVENT_11 // ASM only in NORM mode
#define RTEMS_EVENT_NORM_BP1_F2     RTEMS_EVENT_12
#define RTEMS_EVENT_NORM_BP2_F2     RTEMS_EVENT_13
#define RTEMS_EVENT_NORM_ASM_F2     RTEMS_EVENT_14 // ASM only in NORM mode
#define RTEMS_EVENT_SBM_BP1_F0      RTEMS_EVENT_15
#define RTEMS_EVENT_SBM_BP2_F0      RTEMS_EVENT_16
#define RTEMS_EVENT_SBM_BP1_F1      RTEMS_EVENT_17
#define RTEMS_EVENT_SBM_BP2_F1      RTEMS_EVENT_18
#define RTEMS_EVENT_BURST_BP1_F0    RTEMS_EVENT_19
#define RTEMS_EVENT_BURST_BP2_F0    RTEMS_EVENT_20
#define RTEMS_EVENT_BURST_BP1_F1    RTEMS_EVENT_21
#define RTEMS_EVENT_BURST_BP2_F1    RTEMS_EVENT_22
#define RTEMS_EVENT_SWF_RESYNCH     RTEMS_EVENT_23
#define RTEMS_EVENT_CAL_SWEEP_WAKE  RTEMS_EVENT_24

//********************************************
//********************************************
// LFR PARAMETERS: DEFAULT, MIN AND MAX VALUES

#define DEFAULT_LAST_VALID_TRANSITION_DATE 0xffffffff

// COMMON
#define DEFAULT_SY_LFR_COMMON0 0x00
#define DEFAULT_SY_LFR_COMMON1 0x20 // default value bw sp0 sp1 r0 r1 r2 = 1 0 0 0 0 0

// NORM
#define DFLT_SY_LFR_N_SWF_L       2048 // nb sample
#define DFLT_SY_LFR_N_SWF_P       300 // sec
#define MIN_SY_LFR_N_SWF_P        22 // sec
#define DFLT_SY_LFR_N_ASM_P       3600 // sec
#define DFLT_SY_LFR_N_BP_P0       4 // sec
#define DFLT_SY_LFR_N_BP_P1       20 // sec
#define DFLT_SY_LFR_N_CWF_LONG_F3 0 // 0 => production of light continuous waveforms at f3

// BURST
#define DEFAULT_SY_LFR_B_BP_P0 1 // sec
#define DEFAULT_SY_LFR_B_BP_P1 5 // sec

// SBM1
#define S1_BP_P0_SCALE          0.25
#define DEFAULT_SY_LFR_S1_BP_P0 1 // 0.25 sec
#define DEFAULT_SY_LFR_S1_BP_P1 1 // sec

// SBM2
#define DEFAULT_SY_LFR_S2_BP_P0 1 // sec
#define DEFAULT_SY_LFR_S2_BP_P1 5 // sec

// STATUS WORD
#define DEFAULT_STATUS_WORD_BYTE0                                                                  \
    0x0d // [0000] [1] [101] mode 4 bits / SPW enabled 1 bit / state is run 3 bits

#define DEFAULT_STATUS_WORD_BYTE1 0x00
// TC_LFR_LOAD_FILTER_PAR
#define MIN_PAS_FILTER_MODULUS   4
#define MAX_PAS_FILTER_MODULUS   8
#define MIN_PAS_FILTER_TBAD      0.0
#define MAX_PAS_FILTER_TBAD      4.0
#define MIN_PAS_FILTER_OFFSET    0
#define MAX_PAS_FILTER_OFFSET    7
#define MIN_PAS_FILTER_SHIFT     0.0
#define MAX_PAS_FILTER_SHIFT     1.0
#define MIN_SY_LFR_SC_RW_DELTA_F 0
#define MIN_SY_LFR_RW_F          0
//
#define SY_LFR_DPU_CONNECT_TIMEOUT 100 // 100 * 10 ms = 1 s
#define SY_LFR_DPU_CONNECT_ATTEMPT 3
//****************************

//*****************************
// APB REGISTERS BASE ADDRESSES
#define REGS_ADDR_APBUART                0x80000100
#define REGS_ADDR_GPTIMER                0x80000300
#define REGS_ADDR_GRSPW                  0x80000500
#define APB_OFFSET_GRSPW_STATUS_REGISTER 0x04
#define APB_OFFSET_GRSPW_TIME_REGISTER   0x14
#define REGS_ADDR_TIME_MANAGEMENT        0x80000600

#define REGS_ADDR_SPECTRAL_MATRIX 0x80000f00
#define REGS_ADDR_WAVEFORM_PICKER 0x80000f54 // PDB >= 0.1.28
#define APB_OFFSET_VHDL_REV       0xb0
#define REGS_ADDR_VHDL_VERSION    0x80000ff0

#define APBUART_CTRL_REG_MASK_TE 0x00000002
// scaler value = system_clock_frequency / ( baud_rate * 8 ) - 1
#define APBUART_SCALER_RELOAD_VALUE 0x0000001B // 25 MHz => about 115200

//**********
// IRQ LINES
#define IRQ_GPTIMER_WATCHDOG       9
#define IRQ_SPARC_GPTIMER_WATCHDOG 0x19 // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WAVEFORM_PICKER        14
#define IRQ_SPARC_WAVEFORM_PICKER  0x1e // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_SPECTRAL_MATRIX        6
#define IRQ_SPARC_SPECTRAL_MATRIX  0x16 // see sparcv8.pdf p.76 for interrupt levels

//*****
// TIME
#define CLKDIV_WATCHDOG                  (10000000 - 1) // 10.0s => 10 000 000
#define TIMER_WATCHDOG                   1
#define WATCHDOG_PERIOD                  100 // 1s
#define HK_PERIOD                        100 // 100 * 10ms => 1s
#define AVGV_PERIOD                      6 //   6 * 10ms => 60ms (1 / 16 = 62.5ms)
#define SY_LFR_TIME_SYN_TIMEOUT_in_ticks 200 // 200 * 10 ms = 2 s
#define HK_SYNC_WAIT                     10 //  10 * 10 ms = 100 ms
#define SPW_LINK_WAIT                    10 //  10 * 10 ms = 100 ms
#define TIMECODE_TIMER_TIMEOUT           120 // 120 * 10 ms = 1.2 s
#define TIMECODE_TIMER_TIMEOUT_INIT      200 // 200 * 10 ms = 2.0 s
#define TIMECODE_MASK                    0x3f // 0011 1111

//**********
// LPP CODES
#define LFR_SUCCESSFUL 0
#define LFR_DEFAULT    1
#define LFR_EXE_ERROR  2

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
#define TASKID_SCRB 20
#define TASKID_CALI 21

#define TASK_PRIORITY_SPIQ 5
#define TASK_PRIORITY_LINK 20
#define TASK_PRIORITY_AVGV 25
#define TASK_PRIORITY_HOUS 30
#define TASK_PRIORITY_CWF1 35 // CWF1 and CWF2 are never running together
#define TASK_PRIORITY_CWF2 35 //
#define TASK_PRIORITY_SWBD                                                                         \
    37 // SWBD has a lower priority than WFRM, this is to extract the snapshot before sending it
#define TASK_PRIORITY_WFRM 40
#define TASK_PRIORITY_CWF3                                                                         \
    40 // there is a printf in this function, be careful with its priority wrt CWF1
#define TASK_PRIORITY_SEND 45
#define TASK_PRIORITY_RECV 50
#define TASK_PRIORITY_CALI 50
#define TASK_PRIORITY_ACTN 50
#define TASK_PRIORITY_AVF0 60
#define TASK_PRIORITY_AVF1 70
#define TASK_PRIORITY_PRC0 100
#define TASK_PRIORITY_PRC1 100
#define TASK_PRIORITY_AVF2 110
#define TASK_PRIORITY_PRC2 110
#define TASK_PRIORITY_LOAD 190
#define TASK_PRIORITY_DUMB 200
#define TASK_PRIORITY_SCRB 210

#define MSG_QUEUE_COUNT_RECV 10
#define MSG_QUEUE_COUNT_SEND 50
#define MSG_QUEUE_COUNT_PRC0 10
#define MSG_QUEUE_COUNT_PRC1 10
#define MSG_QUEUE_COUNT_PRC2 5
#define MSG_QUEUE_SIZE_SEND  812 // 808 + 4 => TM_LFR_SCIENCE_BURST_BP2_F1
#define MSG_QUEUE_SIZE_PRC0  36 // two pointers, one rtems_event + 6 integers
#define MSG_QUEUE_SIZE_PRC1  36 // two pointers, one rtems_event + 6 integers
#define MSG_QUEUE_SIZE_PRC2  36 // two pointers, one rtems_event + 6 integers

#define QUEUE_RECV 0
#define QUEUE_SEND 1
#define QUEUE_PRC0 2
#define QUEUE_PRC1 3
#define QUEUE_PRC2 4

struct param_local_str
{
    unsigned int local_sbm1_nb_cwf_sent;
    unsigned int local_sbm1_nb_cwf_max;
    unsigned int local_sbm2_nb_cwf_sent;
    unsigned int local_sbm2_nb_cwf_max;
};

//************
// FBINS MASKS

#define BYTES_PER_FBINS_MASK 16

typedef struct
{
    unsigned char merged_fbins_mask_f0[BYTES_PER_FBINS_MASK];
    unsigned char merged_fbins_mask_f1[BYTES_PER_FBINS_MASK];
    unsigned char merged_fbins_mask_f2[BYTES_PER_FBINS_MASK];
} fbins_masks_t;

#define DEFAULT_SY_LFR_PAS_FILTER_ENABLED 0
#define DEFAULT_SY_LFR_PAS_FILTER_MODULUS 4
#define DEFAULT_SY_LFR_PAS_FILTER_TBAD    1.0f
#define DEFAULT_SY_LFR_PAS_FILTER_OFFSET  0
#define DEFAULT_SY_LFR_PAS_FILTER_SHIFT   0.5f
#define DEFAULT_MODULUS                   262144 // 65536 * 4
#define DEFAULT_TBAD                      65536 // 65536
#define DEFAULT_OFFSET                    0 // 65536 * 0
#define DEFAULT_SHIFT                     32768 // 65536 / 2
#define DEFAULT_SY_LFR_SC_RW_DELTA_F      0.045f
#define DEFAULT_SY_LFR_RW_K1              1.f
#define DEFAULT_SY_LFR_RW_K2              8.f
#define DEFAULT_SY_LFR_RW_K3              24.f
#define DEFAULT_SY_LFR_RW_K4              48.f

typedef struct
{
    unsigned char spare_sy_lfr_pas_filter_enabled;
    float sy_lfr_pas_filter_tbad;
    float sy_lfr_pas_filter_shift;
    uint64_t modulus_in_finetime;
    uint64_t tbad_in_finetime;
    uint64_t offset_in_finetime;
    uint64_t shift_in_finetime;
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

typedef struct
{
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

#define MATRIX_IS_POLLUTED           0
#define MATRIX_IS_NOT_POLLUTED       1
#define ACQUISITION_DURATION_F0      683 // 256 / 24576 * 65536
#define ACQUISITION_DURATION_F1      4096 // 256 /  4096 * 65536
#define ACQUISITION_DURATION_F2      65536 // 256 /   256 * 65536
#define HALF_ACQUISITION_DURATION_F0 341 // 256 / 24576 * 65536 / 2
#define HALF_ACQUISITION_DURATION_F1 2048 // 256 /  4096 * 65536 / 2
#define HALF_ACQUISITION_DURATION_F2 32768 // 256 /   256 * 65536 / 2

#endif // FSW_PARAMS_H_INCLUDED

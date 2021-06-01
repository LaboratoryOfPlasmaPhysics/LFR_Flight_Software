#ifndef GRLIB_REGS_H_INCLUDED
#define GRLIB_REGS_H_INCLUDED

#define NB_GPTIMER 3

#include <stdint.h>

struct apbuart_regs_str
{
    volatile unsigned int data;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
    volatile unsigned int fifoDebug;
};

struct grgpio_regs_str
{
    volatile int io_port_data_register;
    int io_port_output_register;
    int io_port_direction_register;
    int interrupt_mak_register;
    int interrupt_polarity_register;
    int interrupt_edge_register;
    int bypass_register;
    int reserved;
    // 0x20-0x3c interrupt map register(s)
};

typedef struct
{
    volatile unsigned int counter;
    volatile unsigned int reload;
    volatile unsigned int ctrl;
    volatile unsigned int unused;
} timer_regs_t;

//*************
//*************
// GPTIMER_REGS

#define GPTIMER_CLEAR_IRQ 0x00000010 // clear pending IRQ if any
#define GPTIMER_LD        0x00000004 // LD load value from the reload register
#define GPTIMER_EN        0x00000001 // EN enable the timer
#define GPTIMER_EN_MASK   0xfffffffe // EN enable the timer
#define GPTIMER_RS        0x00000002 // RS restart
#define GPTIMER_IE        0x00000008 // IE interrupt enable
#define GPTIMER_IE_MASK   0xffffffef // IE interrupt enable

typedef struct
{
    volatile unsigned int scaler_value;
    volatile unsigned int scaler_reload;
    volatile unsigned int conf;
    volatile unsigned int unused0;
    timer_regs_t timer[NB_GPTIMER];
} gptimer_regs_t;

//*********************
//*********************
// TIME_MANAGEMENT_REGS

#define VAL_SOFTWARE_RESET   0x02 // [0010] software reset
#define VAL_LFR_SYNCHRONIZED 0x80000000
#define BIT_SYNCHRONIZATION  31
#define COARSE_TIME_MASK     0x7fffffff
#define SYNC_BIT_MASK        0x7f
#define SYNC_BIT             0x80
#define BIT_CAL_RELOAD       0x00000010
#define MASK_CAL_RELOAD      0xffffffef // [1110 1111]
#define BIT_CAL_ENABLE       0x00000040
#define MASK_CAL_ENABLE      0xffffffbf // [1011 1111]
#define BIT_SET_INTERLEAVED  0x00000020 // [0010 0000]
#define MASK_SET_INTERLEAVED 0xffffffdf // [1101 1111]
#define BIT_SOFT_RESET       0x00000004 // [0100]
#define MASK_SOFT_RESET      0xfffffffb // [1011]

typedef struct
{
    volatile int
        ctrl; // bit 0 forces the load of the coarse_time_load value and resets the fine_time
              // bit 1 is the soft reset for the time management module
              // bit 2 is the soft reset for the waveform picker and the spectral matrix modules,
              // set to 1 after HW reset
    volatile int coarse_time_load;
    volatile int coarse_time;
    volatile int fine_time;
    // TEMPERATURES
    volatile int temp_pcb; // SEL1 = 0 SEL0 = 0
    volatile int temp_fpga; // SEL1 = 0 SEL0 = 1
    volatile int temp_scm; // SEL1 = 1 SEL0 = 0
    // CALIBRATION
    volatile unsigned int calDACCtrl;
    volatile unsigned int calPrescaler;
    volatile unsigned int calDivisor;
    volatile unsigned int calDataPtr;
    volatile unsigned int calData;
} time_management_regs_t;

//*********************
//*********************
// WAVEFORM_PICKER_REGS

#define BITS_WFP_STATUS_F3 0xc0 // [1100 0000] check the f3 full bits
#define BIT_WFP_BUF_F3_0   0x40 // [0100 0000] f3 buffer 0 is full
#define BIT_WFP_BUF_F3_1   0x80 // [1000 0000] f3 buffer 1 is full
#define RST_WFP_F3_0       0x00008840 // [1000 1000 0100 0000]
#define RST_WFP_F3_1       0x00008880 // [1000 1000 1000 0000]

#define BITS_WFP_STATUS_F2  0x30 // [0011 0000] get the status bits for f2
#define SHIFT_WFP_STATUS_F2 4
#define BIT_WFP_BUF_F2_0    0x10 // [0001 0000] f2 buffer 0 is full
#define BIT_WFP_BUF_F2_1    0x20 // [0010 0000] f2 buffer 1 is full
#define RST_WFP_F2_0        0x00004410 // [0100 0100 0001 0000]
#define RST_WFP_F2_1        0x00004420 // [0100 0100 0010 0000]

#define BITS_WFP_STATUS_F1 0x0c // [0000 1100] check the f1 full bits
#define BIT_WFP_BUF_F1_0   0x04 // [0000 0100] f1 buffer 0 is full
#define BIT_WFP_BUF_F1_1   0x08 // [0000 1000] f1 buffer 1 is full
#define RST_WFP_F1_0       0x00002204 // [0010 0010 0000 0100] f1 bits = 0
#define RST_WFP_F1_1       0x00002208 // [0010 0010 0000 1000] f1 bits = 0

#define BITS_WFP_STATUS_F0 0x03 // [0000 0011] check the f0 full bits
#define RST_WFP_F0_0       0x00001101 // [0001 0001 0000 0001]
#define RST_WFP_F0_1       0x00001102 // [0001 0001 0000 0010]

#define BIT_WFP_BUFFER_0 0x01
#define BIT_WFP_BUFFER_1 0x02

#define RST_BITS_RUN_BURST_EN  0x80 // [1000 0000] burst f2, f1, f0     enable f3, f2, f1, f0
#define BITS_WFP_ENABLE_ALL    0x0f // [0000 1111] enable f3, f2, f1, f0
#define BITS_WFP_ENABLE_BURST  0x0c // [0000 1100] enable f3, f2
#define RUN_BURST_ENABLE_SBM2  0x60 // [0110 0000] enable f2 and f1 burst
#define RUN_BURST_ENABLE_BURST 0x40 // [0100 0000] f2 burst enabled

#define DFLT_WFP_NB_DATA_BY_BUFFER 0xa7f // 0x30 *** 2688 - 1 => nb samples -1
#define DFLT_WFP_SNAPSHOT_PARAM    0xa80 // 0x34 *** 2688 => nb samples
#define DFLT_WFP_BUFFER_LENGTH     0x1f8 // buffer length in burst = 3 * 2688 / 16 = 504 = 0x1f8
#define DFLT_WFP_DELTA_F0_2        0x30 // 48 = 11 0000, max 7 bits

// PDB >= 0.1.28, 0x80000f54
typedef struct
{
    int data_shaping; // 0x00 00 *** R2 R1 R0 SP1 SP0 BW
    int run_burst_enable; // 0x04 01 *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]
    int addr_data_f0_0; // 0x08
    int addr_data_f0_1; // 0x0c
    int addr_data_f1_0; // 0x10
    int addr_data_f1_1; // 0x14
    int addr_data_f2_0; // 0x18
    int addr_data_f2_1; // 0x1c
    int addr_data_f3_0; // 0x20
    int addr_data_f3_1; // 0x24
    volatile int status; // 0x28
    volatile int delta_snapshot; // 0x2c
    int delta_f0; // 0x30
    int delta_f0_2; // 0x34
    int delta_f1; // 0x38
    int delta_f2; // 0x3c
    int nb_data_by_buffer; // 0x40 number of samples in a buffer = 2688
    int snapshot_param; // 0x44
    int start_date; // 0x48
    //
    volatile unsigned int f0_0_coarse_time; // 0x4c
    volatile unsigned int f0_0_fine_time; // 0x50
    volatile unsigned int f0_1_coarse_time; // 0x54
    volatile unsigned int f0_1_fine_time; // 0x58
    //
    volatile unsigned int f1_0_coarse_time; // 0x5c
    volatile unsigned int f1_0_fine_time; // 0x60
    volatile unsigned int f1_1_coarse_time; // 0x64
    volatile unsigned int f1_1_fine_time; // 0x68
    //
    volatile unsigned int f2_0_coarse_time; // 0x6c
    volatile unsigned int f2_0_fine_time; // 0x70
    volatile unsigned int f2_1_coarse_time; // 0x74
    volatile unsigned int f2_1_fine_time; // 0x78
    //
    volatile unsigned int f3_0_coarse_time; // 0x7c => 0x7c + 0xf54 = 0xd0
    volatile unsigned int f3_0_fine_time; // 0x80
    volatile unsigned int f3_1_coarse_time; // 0x84
    volatile unsigned int f3_1_fine_time; // 0x88
    //
    unsigned int buffer_length; // 0x8c = buffer length in burst 2688 / 16 = 168
    //
    volatile int16_t v_dummy; // 0x90
    volatile int16_t v; // 0x90
    volatile int16_t e1_dummy; // 0x94
    volatile int16_t e1; // 0x94
    volatile int16_t e2_dummy; // 0x98
    volatile int16_t e2; // 0x98
} waveform_picker_regs_0_1_18_t;

//*********************
//*********************
// SPECTRAL_MATRIX_REGS

#define BITS_STATUS_F0         0x03 // [0011]
#define BITS_STATUS_F1         0x0c // [1100]
#define BITS_STATUS_F2         0x30 // [0011 0000]
#define BITS_HK_AA_SM          0x780 // [0111 1000 0000]
#define BITS_SM_ERR            0x7c0 // [0111 1100 0000]
#define BITS_STATUS_REG        0x7ff // [0111 1111 1111]
#define BIT_READY_0            0x1 // [01]
#define BIT_READY_1            0x2 // [10]
#define BIT_READY_0_1          0x3 // [11]
#define BIT_STATUS_F1_0        0x04 // [0100]
#define BIT_STATUS_F1_1        0x08 // [1000]
#define BIT_STATUS_F2_0        0x10 // [0001 0000]
#define BIT_STATUS_F2_1        0x20 // [0010 0000]
#define DEFAULT_MATRIX_LENGTH  0xc8 // 25 * 128 / 16 = 200 = 0xc8
#define BIT_IRQ_ON_NEW_MATRIX  0x01
#define MASK_IRQ_ON_NEW_MATRIX 0xfffffffe
#define BIT_IRQ_ON_ERROR       0x02
#define MASK_IRQ_ON_ERROR      0xfffffffd

typedef struct
{
    volatile int config; // 0x00
    volatile int status; // 0x04
    volatile int f0_0_address; // 0x08
    volatile int f0_1_address; // 0x0C
    //
    volatile int f1_0_address; // 0x10
    volatile int f1_1_address; // 0x14
    volatile int f2_0_address; // 0x18
    volatile int f2_1_address; // 0x1C
    //
    volatile unsigned int f0_0_coarse_time; // 0x20
    volatile unsigned int f0_0_fine_time; // 0x24
    volatile unsigned int f0_1_coarse_time; // 0x28
    volatile unsigned int f0_1_fine_time; // 0x2C
    //
    volatile unsigned int f1_0_coarse_time; // 0x30
    volatile unsigned int f1_0_fine_time; // 0x34
    volatile unsigned int f1_1_coarse_time; // 0x38
    volatile unsigned int f1_1_fine_time; // 0x3C
    //
    volatile unsigned int f2_0_coarse_time; // 0x40
    volatile unsigned int f2_0_fine_time; // 0x44
    volatile unsigned int f2_1_coarse_time; // 0x48
    volatile unsigned int f2_1_fine_time; // 0x4C
    //
    unsigned int matrix_length; // 0x50, length of a spectral matrix in burst 3200 / 16 = 200 = 0xc8
} spectral_matrix_regs_t;

#endif // GRLIB_REGS_H_INCLUDED

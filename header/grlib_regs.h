#ifndef GRLIB_REGS_H_INCLUDED
#define GRLIB_REGS_H_INCLUDED

#define NB_GPTIMER 3

struct apbuart_regs_str{
    volatile unsigned int data;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
    volatile unsigned int fifoDebug;
};

struct ahbuart_regs_str{
    volatile unsigned int unused;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
};

typedef struct {
    volatile unsigned int counter;
    volatile unsigned int reload;
    volatile unsigned int ctrl;
    volatile unsigned int unused;
} timer_regs_t;

typedef struct {
    volatile unsigned int scaler_value;
    volatile unsigned int scaler_reload;
    volatile unsigned int conf;
    volatile unsigned int unused0;
    timer_regs_t timer[NB_GPTIMER];
} gptimer_regs_t;

typedef struct {
    volatile int ctrl; // bit 0 forces the load of the coarse_time_load value and resets the fine_time
    volatile int coarse_time_load;
    volatile int coarse_time;
    volatile int fine_time;
} time_management_regs_t;

typedef struct {
    volatile int data_shaping;      // 0x00 00      *** R1 R0 SP1 SP0 BW
    volatile int burst_enable;      // 0x04 01      *** burst f2, f1, f0     enable f3, f2, f1, f0
    volatile int addr_data_f0;      // 0x08 10      ***
    volatile int addr_data_f1;      // 0x0c 11      ***
    volatile int addr_data_f2;      // 0x10 100     ***
    volatile int addr_data_f3;      // 0x14 101     ***
    volatile int status;            // 0x18 110     ***
    volatile int delta_snapshot;    // 0x1c 111     ***
    volatile int delta_f2_f1;       // 0x20 0000    ***
    volatile int delta_f2_f0;       // 0x24 0001    ***
    volatile int nb_burst_available;// 0x28 0010    ***
    volatile int nb_snapshot_param; // 0x2c 0011    ***
} waveform_picker_regs_t;

typedef struct{
    int data_shaping; // 0x00 00 *** R1 R0 SP1 SP0 BW
    int run_burst_enable; // 0x04 01 *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]
    int addr_data_f0; // 0x08
    int addr_data_f1; // 0x0c
    int addr_data_f2; // 0x10
    int addr_data_f3; // 0x14
    volatile int status; // 0x18
    int delta_snapshot; // 0x1c
    int delta_f0; // 0x20
    int delta_f0_2; // 0x24
    int delta_f1; // 0x28
    int delta_f2; // 0x2c
    int nb_data_by_buffer; // 0x30
    int snapshot_param; // 0x34
    int start_date; // 0x38
    int nb_word_in_buffer; // 0x3c
} waveform_picker_regs_new_t;

typedef struct {
    volatile int config;
    volatile int status;
    volatile int matrixF0_Address0;
    volatile int matrixFO_Address1;
    volatile int matrixF1_Address;
    volatile int matrixF2_Address;
} spectral_matrix_regs_t;

#endif // GRLIB_REGS_H_INCLUDED

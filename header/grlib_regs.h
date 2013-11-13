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

struct timer_regs_str
{
    volatile unsigned int counter;
    volatile unsigned int reload;
    volatile unsigned int ctrl;
    volatile unsigned int unused;
};
typedef struct timer_regs_str timer_regs_t;

struct gptimer_regs_str
{
    volatile unsigned int scaler_value;
    volatile unsigned int scaler_reload;
    volatile unsigned int conf;
    volatile unsigned int unused0;
    timer_regs_t timer[NB_GPTIMER];
};
typedef struct gptimer_regs_str gptimer_regs_t;

struct time_management_regs_str{
    volatile int ctrl; // bit 0 forces the load of the coarse_time_load value and resets the fine_time
    volatile int coarse_time_load;
    volatile int coarse_time;
    volatile int fine_time;
};
typedef struct time_management_regs_str time_management_regs_t;

struct new_waveform_picker_regs_str{
    volatile int data_shaping;      // 0x00 00      *** R1 R0 SP1 SP0 BW
    volatile int run_burst_enable;  // 0x04 01      *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]
    volatile int addr_data_f0;      // 0x08
    volatile int addr_data_f1;      // 0x0c
    volatile int addr_data_f2;      // 0x10
    volatile int addr_data_f3;      // 0x14
    volatile int status;            // 0x18
    volatile int delta_snapshot;    // 0x1c
    volatile int delta_f0;          // 0x20
    volatile int delta_f0_2;
    volatile int delta_f1;
    volatile int delta_f2;
    volatile int nb_data_by_buffer;
    volatile int snapshot_param;
    volatile int start_date;
};
typedef struct new_waveform_picker_regs_str new_waveform_picker_regs_t;

struct spectral_matrix_regs_str{
    volatile int config;
    volatile int status;
    volatile int matrixF0_Address0;
    volatile int matrixFO_Address1;
    volatile int matrixF1_Address;
    volatile int matrixF2_Address;
};
typedef struct spectral_matrix_regs_str spectral_matrix_regs_t;

#endif // GRLIB_REGS_H_INCLUDED

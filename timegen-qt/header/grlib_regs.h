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

struct grgpio_regs_str{
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
    volatile int config;            // 0x00
    volatile int status;            // 0x04
    volatile int f0_0_address;      // 0x08
    volatile int f0_1_address;      // 0x0C
    //
    volatile int f1_0_address;      // 0x10
    volatile int f1_1_address;      // 0x14
    volatile int f2_0_address;      // 0x18
    volatile int f2_1_address;      // 0x1C
    //
    volatile unsigned int f0_0_coarse_time;  // 0x20
    volatile unsigned int f0_0_fine_time;    // 0x24
    volatile unsigned int f0_1_coarse_time;  // 0x28
    volatile unsigned int f0_1_fine_time;    // 0x2C
    //
    volatile unsigned int f1_0_coarse_time;  // 0x30
    volatile unsigned int f1_0_fine_time;    // 0x34
    volatile unsigned int f1_1_coarse_time;  // 0x38
    volatile unsigned int f1_1_time_time;    // 0x3C
    //
    volatile unsigned int f2_0_coarse_time;  // 0x40
    volatile unsigned int f2_0_fine_time;    // 0x44
    volatile unsigned int f2_1_coarse_time;  // 0x48
    volatile unsigned int f2_1_fine_time;    // 0x4C
} spectral_matrix_regs_t;

#endif // GRLIB_REGS_H_INCLUDED

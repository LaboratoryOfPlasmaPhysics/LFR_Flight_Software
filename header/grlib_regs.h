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
    volatile int ctrl;  // bit 0 forces the load of the coarse_time_load value and resets the fine_time
                        // bit 1 is the soft reset for the time management module
                        // bit 2 is the soft reset for the waveform picker and the spectral matrix modules, set to 1 after HW reset
    volatile int coarse_time_load;
    volatile int coarse_time;
    volatile int fine_time;
    volatile int temp_scm;
    volatile int temp_pcb;
    volatile int temp_fpga;
} time_management_regs_t;

// PDB >= 0.1.28
typedef struct{
    int data_shaping;       // 0x00 00 *** R1 R0 SP1 SP0 BW
    int run_burst_enable;   // 0x04 01 *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]
    int addr_data_f0_0;     // 0x08
    int addr_data_f0_1;     // 0x0c
    int addr_data_f1_0;     // 0x10
    int addr_data_f1_1;     // 0x14
    int addr_data_f2_0;     // 0x18
    int addr_data_f2_1;     // 0x1c
    int addr_data_f3_0;     // 0x20
    int addr_data_f3_1;     // 0x24
    volatile int status;    // 0x28
    int delta_snapshot;     // 0x2c
    int delta_f0;           // 0x30
    int delta_f0_2;         // 0x34
    int delta_f1;           // 0x38
    int delta_f2;           // 0x3c
    int nb_data_by_buffer;  // 0x40 number of samples in a buffer = 2688
    int snapshot_param;     // 0x44
    int start_date;         // 0x48
    //
    volatile unsigned int f0_0_coarse_time; // 0x4c
    volatile unsigned int f0_0_fine_time;   // 0x50
    volatile unsigned int f0_1_coarse_time; // 0x54
    volatile unsigned int f0_1_fine_time;   // 0x58
    //
    volatile unsigned int f1_0_coarse_time; // 0x5c
    volatile unsigned int f1_0_fine_time;   // 0x60
    volatile unsigned int f1_1_coarse_time; // 0x64
    volatile unsigned int f1_1_fine_time;   // 0x68
    //
    volatile unsigned int f2_0_coarse_time; // 0x6c
    volatile unsigned int f2_0_fine_time;   // 0x70
    volatile unsigned int f2_1_coarse_time; // 0x74
    volatile unsigned int f2_1_fine_time;   // 0x78
    //
    volatile unsigned int f3_0_coarse_time; // 0x7c
    volatile unsigned int f3_0_fine_time;   // 0x80
    volatile unsigned int f3_1_coarse_time; // 0x84
    volatile unsigned int f3_1_fine_time;   // 0x88
    //
    unsigned int buffer_length;             // 0x8c = buffer length in burst 2688 / 16 = 168
    //
    volatile unsigned int v;                // 0x90
    volatile unsigned int e1;               // 0x94
    volatile unsigned int e2;               // 0x98
} waveform_picker_regs_0_1_18_t;

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
    volatile unsigned int f0_0_coarse_time; // 0x20
    volatile unsigned int f0_0_fine_time;   // 0x24
    volatile unsigned int f0_1_coarse_time; // 0x28
    volatile unsigned int f0_1_fine_time;   // 0x2C
    //
    volatile unsigned int f1_0_coarse_time; // 0x30
    volatile unsigned int f1_0_fine_time;   // 0x34
    volatile unsigned int f1_1_coarse_time; // 0x38
    volatile unsigned int f1_1_fine_time;   // 0x3C
    //
    volatile unsigned int f2_0_coarse_time; // 0x40
    volatile unsigned int f2_0_fine_time;   // 0x44
    volatile unsigned int f2_1_coarse_time; // 0x48
    volatile unsigned int f2_1_fine_time;   // 0x4C
    //
    unsigned int matrix_length;             // 0x50, length of a spectral matrix in burst 3200 / 16 = 200 = 0xc8
} spectral_matrix_regs_t;

#endif // GRLIB_REGS_H_INCLUDED

#ifndef GRLIBREGS_H_INCLUDED
#define GRLIBREGS_H_INCLUDED

struct apbuart_regs_str{
    volatile unsigned int data;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
    volatile unsigned int fifoDebug;
};

struct gptimer_regs_str
{
    volatile unsigned int scaler_value;
    volatile unsigned int scaler_reload;
    volatile unsigned int conf;
    volatile unsigned int unused0;
    volatile unsigned int timer1_counter;
    volatile unsigned int timer1_reload;
    volatile unsigned int timer1_ctrl;
    volatile unsigned int unused1;
    volatile unsigned int timer2_counter;
    volatile unsigned int timer2_reload;
    volatile unsigned int timer2_ctrl;
    volatile unsigned int unused2;
    volatile unsigned int timer3_counter;
    volatile unsigned int timer3_reload;
    volatile unsigned int timer3_ctrl;
    volatile unsigned int unused3;
};
typedef struct gptimer_regs_str gptimer_regs_t;

struct spectral_matrices_regs_str{
    volatile int ctrl;
    volatile int address0;
    volatile int address1;
};
typedef struct spectral_matrices_regs_str spectral_matrices_regs_t;

#endif // GRLIBREGS_H_INCLUDED

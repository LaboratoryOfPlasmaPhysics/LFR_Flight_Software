#ifndef GRLIBREGS_H_INCLUDED
#define GRLIBREGS_H_INCLUDED

#define NB_GPTIMER 3

struct apbuart_regs_str{
    volatile unsigned int data;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
    volatile unsigned int fifoDebug;
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

struct spectral_matrices_regs_str{
    volatile int ctrl;
    volatile int address0;
    volatile int address1;
};
typedef struct spectral_matrices_regs_str spectral_matrices_regs_t;

struct time_management_regs_str{
    volatile int ctrl; // bit 0 forces the load of the coarse_time_load value and resets the fine_time
    volatile int coarse_time_load;
    volatile int coarse_time;
    volatile int fine_time;
};
typedef struct time_management_regs_str time_management_regs_t;

#endif // GRLIBREGS_H_INCLUDED

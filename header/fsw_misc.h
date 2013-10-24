#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>
#include <stdio.h>
#include <grspw.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"

rtems_name name_hk_rate_monotonic;     // name of the HK rate monotonic
rtems_id HK_id;         // id of the HK rate monotonic period

extern rtems_name  misc_name[5];
time_management_regs_t *time_management_regs;
extern Packet_TM_LFR_HK_t housekeeping_packet;

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
int timer_start( gptimer_regs_t *gptimer_regs, unsigned char timer );
int timer_stop( gptimer_regs_t *gptimer_regs, unsigned char timer );
int timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider);

// SERIAL LINK
int send_console_outputs_on_apbuart_port( void );
void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value);

// RTEMS TASKS
rtems_task stat_task( rtems_task_argument argument );
rtems_task hous_task( rtems_task_argument argument );
rtems_task dumb_task( rtems_task_argument unused );

void init_housekeeping_parameters( void );

#endif // FSW_MISC_H_INCLUDED

#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include "fsw_init.h"

rtems_name HK_name;     // name of the HK rate monotonic
rtems_id HK_id;         // id of the HK rate monotonic period
extern Packet_TM_LFR_HK_t housekeeping_packet;

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
int timer_start( gptimer_regs_t *gptimer_regs, unsigned char timer );
int timer_stop( gptimer_regs_t *gptimer_regs, unsigned char timer );
int timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider);
void update_spacewire_statistics();

// SERIAL LINK
int send_console_outputs_on_apbuart_port( void );
void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value);

// RTEMS TASKS
rtems_task stat_task(rtems_task_argument argument);
rtems_task hous_task(rtems_task_argument argument);
rtems_task send_task(rtems_task_argument argument);

rtems_id get_pkts_queue_id( void );

#endif // FSW_MISC_H_INCLUDED

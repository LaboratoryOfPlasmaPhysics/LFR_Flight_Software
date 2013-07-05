#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>
#include <stdio.h>

#include <fsw_init.h>
#include <fsw_params.h>
#include <grlib_regs.h>
#include <grspw.h>
#include <ccsds_types.h>

rtems_name HK_name;     // name of the HK rate monotonic
rtems_id HK_id;         // id of the HK rate monotonic period
extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
void update_spacewire_statistics();

// SERIAL LINK
int send_console_outputs_on_apbuart_port( void );
int set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value);

// RTEMS TASKS
rtems_task stat_task(rtems_task_argument argument);
rtems_task hous_task(rtems_task_argument argument);

#endif // FSW_MISC_H_INCLUDED

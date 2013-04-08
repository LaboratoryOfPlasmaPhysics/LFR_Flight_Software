#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>
#include <stdio.h>

#include <fsw_init.h>
#include <fsw_params.h>
#include <grlib_regs.h>
#include <grspw.h>
#include <ccsds_types.h>

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
void print_statistics(spw_stats *stats);
int send_console_outputs_on_serial_port();
rtems_task stat_task(rtems_task_argument argument);
rtems_task hous_task(rtems_task_argument argument);

#endif // FSW_MISC_H_INCLUDED

#ifndef FSW_RTEMS_H_INCLUDED
#define FSW_RTEMS_H_INCLUDED

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <grspw.h>
#include <apbuart.h>

#include <fsw_params.h>
#include <fsw_misc.h>
#include <fsw_processing.h>
#include <tc_handler.h>
#include <wf_handler.h>
#include <grlib_regs.h>

extern int sched_yield();
extern int errno;

// RTEMS TASKS
rtems_task Init( rtems_task_argument argument);	/* forward declaration needed */
rtems_task spw_recv_task(rtems_task_argument argument);
rtems_task spw_spiq_task(rtems_task_argument argument);
rtems_task spw_stat_task(rtems_task_argument argument);
rtems_task spw_wfrm_task(rtems_task_argument argument);
int create_all_tasks();
int start_all_tasks();

// OTHER functions
int configure_spw_link();
int send_console_outputs_on_serial_port();
extern int rtems_cpu_usage_report();
extern int rtems_cpu_usage_reset();
void print_statistics(spw_stats *);
rtems_status_code write_spw(spw_ioctl_pkt_send* spw_ioctl_send);

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

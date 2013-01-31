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
extern rtems_id   Task_id[ ];       /* array of task ids */
extern rtems_name Task_name[ ];     /* array of task names */
extern rtems_name misc_name[ ];    /* arry of miscellaneous names for rtems objects */
extern int fdSPW;   // grspw file descriptor
extern int fdUART;  // uart file descriptor

// RTEMS TASKS
rtems_task Init( rtems_task_argument argument);	/* forward declaration needed */
rtems_task recv_task(rtems_task_argument argument);
rtems_task spiq_task(rtems_task_argument argument);
rtems_task stat_task(rtems_task_argument argument);
rtems_task wfrm_task(rtems_task_argument argument);
int create_all_tasks();
int start_all_tasks();
int create_message_queue();

// OTHER functions
int configure_spw_link();
int send_console_outputs_on_serial_port();
extern int rtems_cpu_usage_report();
extern int rtems_cpu_usage_reset();
void print_statistics(spw_stats *);
rtems_status_code write_spw(spw_ioctl_pkt_send* spw_ioctl_send);

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

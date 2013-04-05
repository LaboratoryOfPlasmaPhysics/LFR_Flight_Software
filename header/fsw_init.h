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

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc);

// MODE PARAMETERS
extern struct param_norm_str param_norm;
extern struct param_burst_str param_burst;
extern struct param_sbm1_str param_sbm1;
extern struct param_sbm2_str param_sbm2;
extern unsigned char param_common[];

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
void init_default_mode_parameters();

int configure_spw_link();
void configure_spacewire_set_NP(unsigned char val, unsigned int regAddr); // No Port force
void configure_spacewire_set_RE(unsigned char val, unsigned int regAddr); // RMAP Enable

extern int rtems_cpu_usage_report();
extern int rtems_cpu_usage_reset();
void print_statistics(spw_stats *);
rtems_status_code write_spw(spw_ioctl_pkt_send* spw_ioctl_send);
void (*grspw_timecode_callback) (void *pDev, void *regs, int minor, unsigned int tc);

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

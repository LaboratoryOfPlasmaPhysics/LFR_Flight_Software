#ifndef FSW_RTEMS_H_INCLUDED
#define FSW_RTEMS_H_INCLUDED

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

#include <grspw.h>
#include <apbuart.h>

#include "fsw_params.h"
#include "fsw_misc.h"
#include "tm_byte_positions.h"
#include "fsw_processing.h"
#include "tc_handler.h"
#include "wf_handler.h"
#include "grlib_regs.h"
#include "ccsds_types.h"

#include "fsw_spacewire.h"

extern rtems_name  misc_name[5];
extern rtems_id    misc_id[5];
extern rtems_name  Task_name[20];       /* array of task names */
extern rtems_id    Task_id[20];         /* array of task ids */
extern unsigned int maxCount;
extern int fdSPW;   // grspw file descriptor
extern int fdUART;  // uart file descriptor
extern unsigned char lfrCurrentMode;

// MODE PARAMETERS
extern struct param_local_str param_local;
extern Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
extern unsigned short sequenceCounters[SEQ_CNT_NB_PID][SEQ_CNT_NB_CAT][SEQ_CNT_NB_DEST_ID];

// RTEMS TASKS
rtems_task Init( rtems_task_argument argument);	/* forward declaration needed */
rtems_task recv_task(rtems_task_argument argument);
rtems_task stat_task(rtems_task_argument argument);
rtems_task wfrm_task(rtems_task_argument argument);

// OTHER functions
int create_names( void );
int create_all_tasks( void );
int start_all_tasks( void );
rtems_status_code create_message_queues( void );
//
void init_parameter_dump( void );
void init_local_mode_parameters( void );
void init_housekeeping_parameters( void );

extern int rtems_cpu_usage_report( void );
extern int rtems_cpu_usage_reset( void );
extern void rtems_stack_checker_report_usage( void );

extern int sched_yield( void );
extern int errno;

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

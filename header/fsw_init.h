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
#include "fsw_processing.h"
#include "tc_handler.h"
#include "wf_handler.h"
#include "grlib_regs.h"
#include "ccsds_types.h"

#include "fsw_spacewire.h"

extern int sched_yield( void );
extern int errno;
extern rtems_id   Task_id[ ];       /* array of task ids */
extern rtems_name Task_name[ ];     /* array of task names */
extern rtems_name misc_name[ ];    /* arry of miscellaneous names for rtems objects */
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
int create_names( void );
int create_all_tasks( void );
int start_all_tasks( void );
int create_message_queue( void );
int create_message_queues( void );

// OTHER functions
void init_parameter_dump( void );
void init_local_mode_parameters( void );
void init_housekeeping_parameters( void );

extern int rtems_cpu_usage_report( void );
extern int rtems_cpu_usage_reset( void );

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

#ifndef TIMEGEN_INIT_H_INCLUDED
#define TIMEGEN_INIT_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "fsw_params.h"
#include "fsw_misc.h"
#include "wf_handler.h"

#include "timegen_spacewire.h"
#include "timegen_misc.h"

extern rtems_name  Task_name[20];       /* array of task names */
extern rtems_id    Task_id[20];         /* array of task ids */
extern rtems_name  misc_name[5];

// RTEMS TASKS
rtems_task Init( rtems_task_argument argument);

// OTHER functions
void create_names( void );
int create_all_tasks( void );
int start_all_tasks( void );
//
rtems_status_code create_message_queues( void );
rtems_status_code get_message_queue_id_send( rtems_id *queue_id );
rtems_status_code get_message_queue_id_recv( rtems_id *queue_id );
//
int start_recv_send_tasks( void );
//
void init_local_mode_parameters( void );
void reset_local_time( void );

extern void rtems_stack_checker_report_usage( void );

extern int sched_yield( void );

#endif // TIMEGEN_INIT_H_INCLUDED

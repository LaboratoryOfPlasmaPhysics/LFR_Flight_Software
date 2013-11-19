#ifndef FSW_INIT_H_INCLUDED
#define FSW_INIT_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "fsw_params.h"
#include "fsw_misc.h"
#include "fsw_processing.h"
#include "tc_handler.h"
#include "wf_handler.h"

#include "fsw_spacewire.h"

extern rtems_name  Task_name[20];       /* array of task names */
extern rtems_id    Task_id[20];         /* array of task ids */

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

extern int rtems_cpu_usage_report( void );
extern int rtems_cpu_usage_reset( void );
extern void rtems_stack_checker_report_usage( void );

extern int sched_yield( void );

#endif // FSW_INIT_H_INCLUDED

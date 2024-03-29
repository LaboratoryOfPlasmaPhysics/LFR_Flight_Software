#ifndef FSW_INIT_H_INCLUDED
#define FSW_INIT_H_INCLUDED

//#include <leon.h>
#include <rtems.h>

#include "fsw_misc.h"
#include "lfr_common_headers/fsw_params.h"
#include "fsw_processing.h"

#include "fsw_globals.h"

#include "fsw_spacewire.h"
#include "tc_tm/tc_handler.h"
#include "hw/wf_handler.h"

#include "avf0_prc0.h"
#include "avf1_prc1.h"
#include "avf2_prc2.h"

extern rtems_name Task_name[]; /* array of task names */
extern rtems_id Task_id[]; /* array of task ids */
extern rtems_name timecode_timer_name;
extern rtems_id timecode_timer_id;
extern unsigned char pa_bia_status_info;

// RTEMS TASKS
rtems_task Init(rtems_task_argument argument);

// OTHER functions
void create_names(void);
int create_all_tasks(void);
int start_all_tasks(void);
//
rtems_status_code create_message_queues(void);
rtems_status_code create_timecode_timer(void);
rtems_status_code get_message_queue_id_send(rtems_id* queue_id);
rtems_status_code get_message_queue_id_recv(rtems_id* queue_id);
rtems_status_code get_message_queue_id_prc0(rtems_id* queue_id);
rtems_status_code get_message_queue_id_prc1(rtems_id* queue_id);
rtems_status_code get_message_queue_id_prc2(rtems_id* queue_id);
void update_queue_max_count(rtems_id queue_id, unsigned char* fifo_size_max);
void init_ring(
    ring_node ring[], unsigned char nbNodes, volatile int buffer[], unsigned int bufferSize);
//
int start_recv_send_tasks(void);
//
void init_local_mode_parameters(void);
void reset_local_time(void);

extern void rtems_cpu_usage_report(void);
extern void rtems_cpu_usage_reset(void);
extern void rtems_stack_checker_report_usage(void);

extern int sched_yield(void);

#endif // FSW_INIT_H_INCLUDED

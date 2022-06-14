#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>

//#include <grspw.h>

#include <stdio.h>


#include "lfr_common_headers/fsw_params.h"
#include "fsw_globals.h"

#define WATCHDOG_LOOP_PRINTF 10
#define WATCHDOG_LOOP_DEBUG  3

#define LFR_NO_RETURN __attribute__((noreturn))

#define NB_RTEMS_EVENTS 32
#define EVENT_12        12
#define EVENT_13        13
#define EVENT_14        14
#define DUMB_MESSAGE_1  "in DUMB *** timecode_irq_handler"
#define DUMB_MESSAGE_12 "WATCHDOG timer"
#define DUMB_MESSAGE_13 "TIMECODE timer"

#ifdef PRINT_MESSAGES_ON_CONSOLE
    #define DUMB_TASK_ENABLED
#endif

#define MAX_OF(type)                                                                               \
    (((type)(~0LLU) > (type)((1LLU << ((sizeof(type) << 3) - 1)) - 1LLU))                          \
            ? (long long unsigned int)(type)(~0LLU)                                                \
            : (long long unsigned int)(type)((1LLU << ((sizeof(type) << 3) - 1)) - 1LLU))


extern void ASR16_get_FPRF_IURF_ErrorCounters(unsigned int*, unsigned int*);
extern void CCR_getInstructionAndDataErrorCounters(unsigned int*, unsigned int*);

extern rtems_name name_hk_rate_monotonic; // name of the HK rate monotonic
extern rtems_id HK_id; // id of the HK rate monotonic period
extern rtems_name name_avgv_rate_monotonic; // name of the AVGV rate monotonic
extern rtems_id AVGV_id; // id of the AVGV rate monotonic period


// RTEMS TASKS
rtems_task load_task(rtems_task_argument argument);
rtems_task hous_task(rtems_task_argument argument);
rtems_task avgv_task(rtems_task_argument argument);
rtems_task dumb_task(rtems_task_argument unused);
rtems_task scrubbing_task(rtems_task_argument unused);
rtems_task calibration_sweep_task(rtems_task_argument unused);

static inline rtems_status_code send_event_dumb_task(rtems_event_set event)
{
#ifdef DUMB_TASK_ENABLED
    return rtems_event_send(Task_id[TASKID_DUMB], event);
#else
    (void)event;
    return RTEMS_SUCCESSFUL;
#endif
}

extern int sched_yield(void);
extern void rtems_cpu_usage_reset();
extern ring_node* current_ring_node_f3;
extern ring_node* ring_node_to_send_cwf_f3;
extern ring_node waveform_ring_f3[];
extern unsigned short sequenceCounterHK;

extern unsigned char hk_lfr_q_sd_fifo_size_max;
extern unsigned char hk_lfr_q_rv_fifo_size_max;
extern unsigned char hk_lfr_q_p0_fifo_size_max;
extern unsigned char hk_lfr_q_p1_fifo_size_max;
extern unsigned char hk_lfr_q_p2_fifo_size_max;

static inline unsigned char increase_unsigned_char_counter(unsigned char counter)
{
    return (counter + 1) & 0xFF;
}

#endif // FSW_MISC_H_INCLUDED


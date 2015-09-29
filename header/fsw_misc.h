#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>
#include <stdio.h>
#include <grspw.h>
#include <grlib_regs.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"
#include "lfr_cpu_usage_report.h"

enum lfr_reset_cause_t{
    UNKNOWN_CAUSE,
    POWER_ON,
    TC_RESET,
    WATCHDOG,
    ERROR_RESET,
    UNEXP_RESET
};

#define LFR_RESET_CAUSE_UNKNOWN_CAUSE 0

rtems_name name_hk_rate_monotonic;     // name of the HK rate monotonic
rtems_id HK_id;         // id of the HK rate monotonic period

void configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
void timer_start( gptimer_regs_t *gptimer_regs, unsigned char timer );
void timer_stop( gptimer_regs_t *gptimer_regs, unsigned char timer );
void timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider);

// SERIAL LINK
int send_console_outputs_on_apbuart_port( void );
int enable_apbuart_transmitter( void );
void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value);

// RTEMS TASKS
rtems_task stat_task( rtems_task_argument argument );
rtems_task hous_task( rtems_task_argument argument );
rtems_task dumb_task( rtems_task_argument unused );

void init_housekeeping_parameters( void );
void increment_seq_counter(unsigned short *packetSequenceControl);
void getTime( unsigned char *time);
unsigned long long int getTimeAsUnsignedLongLongInt( );
void send_dumb_hk( void );
void get_temperatures( unsigned char *temperatures );
void get_v_e1_e2_f3( unsigned char *spacecraft_potential );
void get_cpu_load( unsigned char *resource_statistics );
void set_hk_lfr_sc_potential_flag( bool state );
void set_hk_lfr_mag_fields_flag( bool state );
void set_hk_lfr_calib_enable( bool state );
void set_hk_lfr_reset_cause( enum lfr_reset_cause_t lfr_reset_cause );

extern int sched_yield( void );
extern void rtems_cpu_usage_reset();
extern ring_node *current_ring_node_f3;
extern ring_node *ring_node_to_send_cwf_f3;
extern ring_node waveform_ring_f3[];
extern unsigned short sequenceCounterHK;

extern unsigned char hk_lfr_q_sd_fifo_size_max;
extern unsigned char hk_lfr_q_rv_fifo_size_max;
extern unsigned char hk_lfr_q_p0_fifo_size_max;
extern unsigned char hk_lfr_q_p1_fifo_size_max;
extern unsigned char hk_lfr_q_p2_fifo_size_max;

#endif // FSW_MISC_H_INCLUDED

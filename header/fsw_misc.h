#ifndef FSW_MISC_H_INCLUDED
#define FSW_MISC_H_INCLUDED

#include <rtems.h>
#include <stdio.h>
#include <grspw.h>
#include <grlib_regs.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"
#include "lfr_cpu_usage_report.h"

#define WATCHDOG_LOOP_PRINTF    10
#define WATCHDOG_LOOP_DEBUG     3

#define NB_RTEMS_EVENTS 32
#define EVENT_12        12
#define EVENT_13        13
#define EVENT_14        14
#define DUMB_MESSAGE_1  "in DUMB *** timecode_irq_handler"
#define DUMB_MESSAGE_12 "WATCHDOG timer"
#define DUMB_MESSAGE_13 "TIMECODE timer"

enum lfr_reset_cause_t{
    UNKNOWN_CAUSE,
    POWER_ON,
    TC_RESET,
    WATCHDOG,
    ERROR_RESET,
    UNEXP_RESET
};

typedef struct{
    unsigned char dpu_spw_parity;
    unsigned char dpu_spw_disconnect;
    unsigned char dpu_spw_escape;
    unsigned char dpu_spw_credit;
    unsigned char dpu_spw_write_sync;
    unsigned char timecode_erroneous;
    unsigned char timecode_missing;
    unsigned char timecode_invalid;
    unsigned char time_timecode_it;
    unsigned char time_not_synchro;
    unsigned char time_timecode_ctr;
    unsigned char ahb_correctable;
} hk_lfr_le_t;

typedef struct{
    unsigned char dpu_spw_early_eop;
    unsigned char dpu_spw_invalid_addr;
    unsigned char dpu_spw_eep;
    unsigned char dpu_spw_rx_too_big;
} hk_lfr_me_t;

#define B00 196
#define B01 196
#define B02 0
#define B10 131
#define B11 -244
#define B12 131
#define B20 161
#define B21 -314
#define B22 161

#define A00 1
#define A01 -925
#define A02 0
#define A10 1
#define A11 -947
#define A12 439
#define A20 1
#define A21 -993
#define A22 486

#define GAIN_B0  12
#define GAIN_B1  11
#define GAIN_B2  10

#define GAIN_A0  10
#define GAIN_A1  9
#define GAIN_A2  9

#define NB_COEFFS   3
#define COEFF0  0
#define COEFF1  1
#define COEFF2  2

typedef struct filter_ctx
{
    int W[NB_COEFFS][NB_COEFFS];
}filter_ctx;

extern gptimer_regs_t *gptimer_regs;
extern void ASR16_get_FPRF_IURF_ErrorCounters( unsigned int*, unsigned int* );
extern void CCR_getInstructionAndDataErrorCounters( unsigned int*, unsigned int* );

extern rtems_name name_hk_rate_monotonic;            // name of the HK rate monotonic
extern rtems_id HK_id;// id of the HK rate monotonic period
extern rtems_name name_avgv_rate_monotonic;            // name of the AVGV rate monotonic
extern rtems_id AVGV_id;// id of the AVGV rate monotonic period

void timer_configure( unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() );
void timer_start( unsigned char timer );
void timer_stop( unsigned char timer );
void timer_set_clock_divider(unsigned char timer, unsigned int clock_divider);

// WATCHDOG
rtems_isr watchdog_isr( rtems_vector_number vector );
void watchdog_configure(void);
void watchdog_stop(void);
void watchdog_reload(void);
void watchdog_start(void);

// SERIAL LINK
int send_console_outputs_on_apbuart_port( void );
int enable_apbuart_transmitter( void );
void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value);

// RTEMS TASKS
rtems_task load_task( rtems_task_argument argument );
rtems_task hous_task( rtems_task_argument argument );
rtems_task avgv_task( rtems_task_argument argument );
rtems_task dumb_task( rtems_task_argument unused );
rtems_task scrubbing_task( rtems_task_argument unused );
rtems_task calibration_sweep_task( rtems_task_argument unused );

void init_housekeeping_parameters( void );
void increment_seq_counter(unsigned short *packetSequenceControl);
void getTime( unsigned char *time);
unsigned long long int getTimeAsUnsignedLongLongInt( );
void get_temperatures( unsigned char *temperatures );
void get_v_e1_e2_f3( unsigned char *spacecraft_potential );
void get_cpu_load( unsigned char *resource_statistics );
void set_hk_lfr_sc_potential_flag( bool state );
void set_sy_lfr_pas_filter_enabled( bool state );
void set_sy_lfr_watchdog_enabled( bool state );
void set_hk_lfr_calib_enable( bool state );
void set_hk_lfr_reset_cause( enum lfr_reset_cause_t lfr_reset_cause );
void hk_lfr_le_me_he_update();
void set_hk_lfr_time_not_synchro();

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

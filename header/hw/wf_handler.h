#ifndef WF_HANDLER_H_INCLUDED
#define WF_HANDLER_H_INCLUDED

#include <rtems.h>

#include <grspw.h>

#include <math.h>
#include <stdio.h>


#include <fsw_params.h>

#include "fsw_init.h"
#include "fsw_params_wf_handler.h"

#define pi             3.14159265359
#define T0_IN_FINETIME (65536. / 24576.)
#define T1_IN_FINETIME (65536. / 4096.)
#define T2_IN_FINETIME (65536. / 256.)
#define T3_IN_FINETIME (65536. / 16.)

#define TICKS_PER_T1 16
#define TICKS_PER_T2 256
#define TICKS_PER_S  65536.
#define MS_PER_S     1000.

#define FREQ_F0 24576.
#define FREQ_F1 4096.
#define FREQ_F2 256.

#define DELTAT_F0 2731 // (2048. / 24576. / 2.) * 65536. = 2730.667;
#define DELTAT_F1 16384 // (2048. / 4096.  / 2.) * 65536. = 16384;
#define DELTAT_F2 262144 // (2048. / 256.   / 2.) * 65536. = 262144;

#define OFFSET_2_BYTES 2

#define ONE_TICK_CORR_INTERVAL_0_MIN 0.5
#define ONE_TICK_CORR_INTERVAL_0_MAX 1.0
#define ONE_TICK_CORR_INTERVAL_1_MIN -1.0
#define ONE_TICK_CORR_INTERVAL_1_MAX -0.5
#define ONE_TICK_CORR                1
#define CORR_MULT                    2

extern int fdSPW;

//*****************
// waveform buffers
extern volatile int wf_buffer_f0[];
extern volatile int wf_buffer_f1[];
extern volatile int wf_buffer_f2[];
extern volatile int wf_buffer_f3[];

extern Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
extern struct param_local_str param_local;

extern unsigned short sequenceCounters_SCIENCE_NORMAL_BURST;
extern unsigned short sequenceCounters_SCIENCE_SBM1_SBM2;


extern unsigned char lfrCurrentMode;

//**********
// RTEMS_ISR
void reset_extractSWF(void);
rtems_isr waveforms_isr(rtems_vector_number vector);

//***********
// RTEMS_TASK
rtems_task wfrm_task(rtems_task_argument argument);
rtems_task cwf3_task(rtems_task_argument argument);
rtems_task cwf2_task(rtems_task_argument argument);
rtems_task cwf1_task(rtems_task_argument argument);
rtems_task swbd_task(rtems_task_argument argument);

//******************
// general functions
void WFP_init_rings(void);

void WFP_reset_current_ring_nodes(void);
//
int init_header_continuous_cwf3_light_table(Header_TM_LFR_SCIENCE_CWF_t* headerCWF);
//
int send_waveform_CWF3_light(
    ring_node* ring_node_to_send, ring_node* ring_node_cwf3_light, rtems_id queue_id);
//
void compute_acquisition_time(unsigned int coarseTime, unsigned int fineTime, unsigned int sid,
    unsigned char pa_lfr_pkt_nr, unsigned char* acquisitionTime);
void build_snapshot_from_ring(ring_node* ring_node_to_send, unsigned char frequencyChannel,
    unsigned long long acquisitionTimeF0_asLong, ring_node* ring_node_swf_extracted,
    int* swf_extracted);
double computeCorrection(const unsigned char * const timePtr);
void applyCorrection(double correction);
void snapshot_resynchronization(unsigned char* timePtr);
//
rtems_id get_pkts_queue_id(void);

//**************
// wfp registers
// RESET
void reset_wfp_burst_enable(void);
void reset_wfp_status(void);
void reset_wfp_buffer_addresses(void);
void reset_waveform_picker_regs(void);
// SET
void set_wfp_data_shaping(void);
void set_wfp_burst_enable_register(unsigned char mode);
void set_wfp_delta_snapshot(void);
void set_wfp_delta_f0_f0_2(void);
void set_wfp_delta_f1(void);
void set_wfp_delta_f2(void);

//*****************
// local parameters
void increment_seq_counter_source_id(unsigned char* packet_sequence_control, unsigned char sid);

#endif // WF_HANDLER_H_INCLUDED

#ifndef WF_HANDLER_H_INCLUDED
#define WF_HANDLER_H_INCLUDED

#include <rtems.h>
#include <grspw.h>
#include <stdio.h>
#include <math.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"
#include "fsw_misc.h"

#define pi 3.1415

extern int fdSPW;
extern volatile int wf_snap_f0[ ];
//
extern volatile int wf_snap_f1[ ];
extern volatile int wf_snap_f1_bis[ ];
extern volatile int wf_snap_f1_norm[ ];
//
extern volatile int wf_snap_f2[ ];
extern volatile int wf_snap_f2_bis[ ];
extern volatile int wf_snap_f2_norm[ ];
//
extern volatile int wf_cont_f3[ ];
extern volatile int wf_cont_f3_bis[ ];
extern char wf_cont_f3_light[ ];
extern waveform_picker_regs_t *waveform_picker_regs;
extern time_management_regs_t *time_management_regs;
extern Packet_TM_LFR_HK_t housekeeping_packet;
extern Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
extern struct param_local_str param_local;

extern unsigned short sequenceCounters_SCIENCE_NORMAL_BURST;
extern unsigned short sequenceCounters_SCIENCE_SBM1_SBM2;

extern rtems_id    Task_id[20];         /* array of task ids */

extern unsigned char lfrCurrentMode;

rtems_isr waveforms_isr( rtems_vector_number vector );
rtems_isr waveforms_simulator_isr( rtems_vector_number vector );
rtems_task wfrm_task( rtems_task_argument argument );
rtems_task cwf3_task( rtems_task_argument argument );
rtems_task cwf2_task( rtems_task_argument argument );
rtems_task cwf1_task( rtems_task_argument argument );

//******************
// general functions
void init_waveforms( void );
//
int init_header_snapshot_wf_table(      unsigned int sid, Header_TM_LFR_SCIENCE_SWF_t *headerSWF );
int init_header_continuous_wf_table(    unsigned int sid, Header_TM_LFR_SCIENCE_CWF_t *headerCWF );
int init_header_continuous_wf3_light_table( Header_TM_LFR_SCIENCE_CWF_t *headerCWF );
//
void reset_waveforms( void );
//
int send_waveform_SWF(  volatile int *waveform, unsigned int sid, Header_TM_LFR_SCIENCE_SWF_t *headerSWF, rtems_id queue_id );
int send_waveform_CWF(  volatile int *waveform, unsigned int sid, Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id );
int send_waveform_CWF3( volatile int *waveform, unsigned int sid, Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id );
int send_waveform_CWF3_light( volatile int *waveform, Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id );
//
rtems_id get_pkts_queue_id( void );

//**************
// wfp registers
void set_wfp_data_shaping();
char set_wfp_delta_snapshot();
void set_wfp_burst_enable_register( unsigned char mode);
void reset_wfp_burst_enable();
void reset_wfp_status();
void reset_waveform_picker_regs();

//*****************
// local parameters
void set_local_sbm1_nb_cwf_max( void );
void set_local_sbm2_nb_cwf_max( void );
void set_local_nb_interrupt_f0_MAX( void );
void reset_local_sbm1_nb_cwf_sent( void );
void reset_local_sbm2_nb_cwf_sent( void );

void increment_seq_counter_source_id( unsigned char *packet_sequence_control, unsigned int sid );

#endif // WF_HANDLER_H_INCLUDED

#ifndef WF_HANDLER_H_INCLUDED
#define WF_HANDLER_H_INCLUDED

#include <rtems.h>
#include <fsw_params.h>
#include <grspw.h>
#include <grlib_regs.h>
#include <ccsds_types.h>
#include <stdio.h>
#include <fsw_init.h>
#include <math.h>

#define pi 3.1415

//#include <sys/ioctl.h>

extern rtems_id Task_id[];         /* array of task ids */
extern int fdSPW;
extern volatile int wf_snap_f0[ ];
extern volatile int wf_snap_f1[ ];
extern volatile int wf_snap_f1_bis[ ];
extern volatile int wf_snap_f2[ ];
extern volatile int wf_snap_f2_bis[ ];
extern volatile int wf_cont_f3[ ];
extern waveform_picker_regs_t *waveform_picker_regs;

rtems_isr waveforms_isr( rtems_vector_number vector );
rtems_isr waveforms_simulator_isr( rtems_vector_number vector );
rtems_task wfrm_task(rtems_task_argument argument);

//******************
// general functions
void init_waveforms( void );
void init_header_snapshot_wf( Header_TM_LFR_SCIENCE_SWF_t *header );
void init_header_continuous_wf( Header_TM_LFR_SCIENCE_CWF_t *header );
void reset_waveforms( void );
void send_waveform_norm(Header_TM_LFR_SCIENCE_SWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send);
void send_waveform_burst(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send);
void send_waveform_sbm1(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send);
void send_waveform_sbm2(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send);
void send_waveform_SWF( Header_TM_LFR_SCIENCE_SWF_t *header, volatile int *waveform,
                        unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send);
void send_waveform_CWF( Header_TM_LFR_SCIENCE_CWF_t *header, volatile int *waveform,
                        unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send);

//**************
// wfp registers
void set_wfp_data_shaping(unsigned char data_shaping);
void set_wfp_delta_snapshot(unsigned int delta_snapshot);
void reset_wfp_burst_enable();
void reset_waveform_picker_regs();
//
int build_value(int value1, int value0);

#endif // WF_HANDLER_H_INCLUDED

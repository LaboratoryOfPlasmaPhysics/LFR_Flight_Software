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
extern volatile int wf_cont_f3[ ];
extern waveform_picker_regs_t *waveform_picker_regs;

rtems_isr waveforms_isr( rtems_vector_number vector );
rtems_isr waveforms_isr_alternative( rtems_vector_number vector );
rtems_isr waveforms_simulator_isr( rtems_vector_number vector );
rtems_task wfrm_task(rtems_task_argument argument);

//******************
// general functions
void init_waveforms( void );
void reset_waveforms( void );
void send_waveform( ExtendedTMHeader_t *header, volatile int *waveform, unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send);
void init_waveform_picker_regs();
void set_data_shaping_parameters(unsigned char parameters);
int build_value(int value1, int value0);

#endif // WF_HANDLER_H_INCLUDED

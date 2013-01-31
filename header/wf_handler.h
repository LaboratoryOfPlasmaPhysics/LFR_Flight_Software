#ifndef WF_HANDLER_H_INCLUDED
#define WF_HANDLER_H_INCLUDED

#include <rtems.h>
#include <fsw_params.h>
#include <grspw.h>
#include <grlib_regs.h>
#include <ccsds_types.h>
#include <stdio.h>
#include <fsw_init.h>

//#include <sys/ioctl.h>

extern rtems_id Task_id[];         /* array of task ids */
extern int fdSPW;

rtems_isr waveforms_isr( rtems_vector_number vector );
rtems_task wfrm_task(rtems_task_argument argument);

#endif // WF_HANDLER_H_INCLUDED

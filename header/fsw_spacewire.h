#ifndef FSW_SPACEWIRE_H_INCLUDED
#define FSW_SPACEWIRE_H_INCLUDED

#include <rtems.h>
#include <grspw.h>

#include <fcntl.h>      // for  O_RDWR
#include <unistd.h>     // for the read call
#include <sys/ioctl.h>  // for the ioctl call
#include <errno.h>

#include "fsw_params.h"
#include "tc_handler.h"

extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;

// RTEMS TASK
rtems_task spiq_task( rtems_task_argument argument );
rtems_task recv_task( rtems_task_argument unused );
rtems_task send_task( rtems_task_argument argument );
rtems_task wtdg_task( rtems_task_argument argument );

int spacewire_open_link( void );
int spacewire_start_link( int fd );
int spacewire_stop_and_start_link( int fd );
int spacewire_configure_link(int fd );
int spacewire_reset_link( void );
void spacewire_set_NP( unsigned char val, unsigned int regAddr ); // No Port force
void spacewire_set_RE( unsigned char val, unsigned int regAddr ); // RMAP Enable
void spacewire_compute_stats_offsets( void );
void spacewire_update_statistics( void );

void init_header_cwf( Header_TM_LFR_SCIENCE_CWF_t *header );
void init_header_swf( Header_TM_LFR_SCIENCE_SWF_t *header );
void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header );
int spw_send_waveform_CWF( ring_node *ring_node_to_send, Header_TM_LFR_SCIENCE_CWF_t *header );
int spw_send_waveform_SWF( ring_node *ring_node_to_send, Header_TM_LFR_SCIENCE_SWF_t *header );
int spw_send_waveform_CWF3_light( ring_node *ring_node_to_send, Header_TM_LFR_SCIENCE_CWF_t *header );
void spw_send_asm( ring_node *ring_node_to_send, Header_TM_LFR_SCIENCE_ASM_t *header );

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc );
rtems_timer_service_routine user_routine( rtems_id timer_id, void *user_data );

void (*grspw_timecode_callback) ( void *pDev, void *regs, int minor, unsigned int tc );

#endif // FSW_SPACEWIRE_H_INCLUDED

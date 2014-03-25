#ifndef TIMEGEN_SPACEWIRE_H_INCLUDED
#define TIMEGEN_SPACEWIRE_H_INCLUDED

#include <rtems.h>
#include <grspw.h>

#include <fcntl.h>      // for  O_RDWR
#include <unistd.h>     // for the read call
#include <sys/ioctl.h>  // for the ioctl call
#include <errno.h>

#include "fsw_params.h"
#include "tc_acceptance.h"
#include "timegen_tc_handler.h"

#define DESTINATION_ID_LFR  0xfe
#define DESTINATION_ID_DPU  0x01
#define SPACEWIRE_LINK_LFR  0x01
#define NODEADDR_TIMEGEN    0xfd

extern rtems_id    Task_id[20];         /* array of task ids */
extern int fdSPW;

extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;

// RTEMS TASK
rtems_task spiq_task( rtems_task_argument argument );
rtems_task recv_task( rtems_task_argument unused );
rtems_task send_task( rtems_task_argument argument );
rtems_task wtdg_task( rtems_task_argument argument );

int spacewire_open_link( void );
int spacewire_start_link( int fd );
int spacewire_stop_start_link( int fd );
int spacewire_configure_link(int fd );
int spacewire_reset_link( void );
void spacewire_set_NP( unsigned char val, unsigned int regAddr ); // No Port force
void spacewire_set_RE( unsigned char val, unsigned int regAddr ); // RMAP Enable
void spacewire_compute_stats_offsets( void );
void spacewire_update_statistics( void );

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc );
rtems_timer_service_routine user_routine( rtems_id timer_id, void *user_data );

void (*grspw_timecode_callback) ( void *pDev, void *regs, int minor, unsigned int tc );

#endif // TIMEGEN_SPACEWIRE_H_INCLUDED

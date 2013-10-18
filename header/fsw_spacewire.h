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
extern Packet_TM_LFR_HK_t housekeeping_packet;
extern rtems_id    Task_id[20];         /* array of task ids */

// RTEMS TASK
rtems_task spiq_task( rtems_task_argument argument );
rtems_task recv_task( rtems_task_argument unused );
rtems_task send_task( rtems_task_argument argument );

int spacewire_configure_link( void );
int spacewire_wait_for_link( void );
void spacewire_set_NP( unsigned char val, unsigned int regAddr ); // No Port force
void spacewire_set_RE( unsigned char val, unsigned int regAddr ); // RMAP Enable
void spacewire_compute_stats_offsets( void );
void spacewire_update_statistics( void );

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc );

void (*grspw_timecode_callback) ( void *pDev, void *regs, int minor, unsigned int tc );

#endif // FSW_SPACEWIRE_H_INCLUDED

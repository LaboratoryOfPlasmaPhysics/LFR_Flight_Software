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
#include "TC_types.h"

extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;
extern rtems_id rtems_task_id_updt;

void resetLocalCoarseTime();
void setLocalCoarseTime( unsigned int value );
unsigned int getLocalCoarseTime();
void incrementLocalCoarseTime();

// TC_LFR_UPDATE_TIME
void initLookUpTableForCRC( void );
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);
unsigned int Crc_opt( unsigned char D, unsigned int Chk);
void updateTimePacket(unsigned int time, Packet_TC_LFR_UPDATE_TIME_WITH_HEADER_t *packet);

// RTEMS TASK
rtems_task spiq_task( rtems_task_argument argument );
rtems_task recv_task( rtems_task_argument unused );
rtems_task send_task( rtems_task_argument argument );
rtems_task wtdg_task( rtems_task_argument argument );
rtems_task updt_task( rtems_task_argument unused );

int spacewire_open_link( void );
int spacewire_start_link( int fd );
int spacewire_stop_and_start_link( int fd );
int spacewire_configure_link(int fd );
int spacewire_reset_link( void );
void spacewire_set_NP( unsigned char val, unsigned int regAddr ); // No Port force
void spacewire_set_RE( unsigned char val, unsigned int regAddr ); // RMAP Enable
void spacewire_compute_stats_offsets( void );
void spacewire_update_statistics( void );

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc );
rtems_timer_service_routine user_routine( rtems_id timer_id, void *user_data );

void (*grspw_timecode_callback) ( void *pDev, void *regs, int minor, unsigned int tc );

#endif // FSW_SPACEWIRE_H_INCLUDED

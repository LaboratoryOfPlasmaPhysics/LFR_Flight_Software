#ifndef TIMEGEN_MISC_H_INCLUDED
#define TIMEGEN_MISC_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "fsw_params.h"
#include "TC_types.h"
#include "tc_acceptance.h"
#include "timegen_init.h"

#define TASK_PRIORITY_UPDT 40

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    Packet_TC_LFR_UPDATE_TIME_t update_time;
} Packet_TC_LFR_UPDATE_TIME_WITH_OVERHEAD_t;

unsigned int coarseTime;

rtems_name rtems_name_updt;
rtems_id   rtems_id_updt;

void timegen_timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc );

void initCoarseTime( void );

rtems_task updt_task( rtems_task_argument unused );

int send_tc_lfr_update_time( rtems_id queue_id );

#endif // TIMEGEN_MISC_H_INCLUDED




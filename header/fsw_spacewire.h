#ifndef FSW_SPACEWIRE_H_INCLUDED
#define FSW_SPACEWIRE_H_INCLUDED

#include <rtems.h>

#include <grspw.h>

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include "fsw_params.h"
#include "ccsds_types.h"
#include "fsw_misc.h"

extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;

// RTEMS TASK
rtems_task spiq_task(rtems_task_argument argument);

int spacewire_configure_link( void );
int spacewire_wait_for_link(void);
void spacewire_set_NP(unsigned char val, unsigned int regAddr); // No Port force
void spacewire_set_RE(unsigned char val, unsigned int regAddr); // RMAP Enable
void spacewire_compute_stats_offsets();

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc);

void (*grspw_timecode_callback) (void *pDev, void *regs, int minor, unsigned int tc);

#endif // FSW_SPACEWIRE_H_INCLUDED

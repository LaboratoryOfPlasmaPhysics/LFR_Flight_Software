#ifndef FSW_SPACEWIRE_H_INCLUDED
#define FSW_SPACEWIRE_H_INCLUDED

#include "fsw_init.h"

extern spw_stats spacewire_stats;
extern spw_stats spacewire_stats_backup;

// RTEMS TASK
rtems_task spiq_task(rtems_task_argument argument);

int spacewire_configure_link( void );
int spacewire_wait_for_link(void);
void spacewire_set_NP(unsigned char val, unsigned int regAddr); // No Port force
void spacewire_set_RE(unsigned char val, unsigned int regAddr); // RMAP Enable
void spacewire_compute_stats_offsets(void);

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc);

void (*grspw_timecode_callback) (void *pDev, void *regs, int minor, unsigned int tc);

#endif // FSW_SPACEWIRE_H_INCLUDED

#ifndef LFR_CPU_USAGE_REPORT_H
#define LFR_CPU_USAGE_REPORT_H

#ifdef HAVE_CONFIG_H
    #include "config.h"
#endif

#include <rtems.h>

#include <assert.h>
#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rtems/bspIo.h>
#include <rtems/cpuuse.h>

#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    #include <rtems/score/timestamp.h>
#endif

#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
extern Timestamp_Control CPU_usage_Uptime_at_last_reset;
#else
extern uint32_t CPU_usage_Ticks_at_last_reset;
#endif

unsigned char lfr_rtems_cpu_usage_report(void);

#define CONST_10   10
#define CONST_100  100
#define CONST_255  255
#define CONST_1000 1000

#endif // LFR_CPU_USAGE_REPORT_H

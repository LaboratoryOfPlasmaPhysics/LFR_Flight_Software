/*
 *  CPU Usage Reporter
 *
 *  COPYRIGHT (c) 1989-2009
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  $Id$
 */

#include "lfr_cpu_usage_report.h"
#include "fsw_globals.h"
#include "fsw_params.h"

unsigned char lfr_rtems_cpu_usage_report(void)
{
    const Thread_Control* the_thread;
    const Objects_Information* information;
    uint32_t ival;
    uint32_t fval;
#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    Timestamp_Control uptime;
    Timestamp_Control total;
    Timestamp_Control ran;

#else
    #error "Can't compute CPU usage using ticks on LFR"
#endif

    unsigned char cpu_load;

    ival = 0;
    cpu_load = 0;

    for (uint32_t api_index = 1; api_index <= OBJECTS_APIS_LAST; api_index++)
    {
        if (!_Objects_Information_table[api_index]) { }
        else
        {
            information = _Objects_Information_table[api_index][1];
            if (information != NULL)
            {
                for (uint32_t information_index = 1; information_index <= information->maximum;
                     information_index++)
                {
                    the_thread = (Thread_Control*)information->local_table[information_index];

                    // Only measure scrubbing task load, CPU load is 100%-Scrubbing
                    if (the_thread != NULL && the_thread->Object.id == Task_id[TASKID_SCRB])
                    {
                        _TOD_Get_uptime(&uptime);
                        _Timestamp_Subtract(&CPU_usage_Uptime_at_last_reset, &uptime, &total);
                        ran = the_thread->cpu_time_used;
                        _Timestamp_Divide(&ran, &total, &ival, &fval);
                        cpu_load = (unsigned char)(CONST_255
                            - ((((ival * CONST_10) + (fval / CONST_100)) * CONST_256)
                                / CONST_1000));
                    }
                }
            }
        }
    }
    return cpu_load;
}

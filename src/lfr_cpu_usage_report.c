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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems.h>

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>

#include <rtems/cpuuse.h>
#include <rtems/bspIo.h>

#include "lfr_cpu_usage_report.h"

#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
  #include <rtems/score/timestamp.h>
#endif

#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
  extern Timestamp_Control  CPU_usage_Uptime_at_last_reset;
#else
  extern uint32_t           CPU_usage_Ticks_at_last_reset;
#endif

/*PAGE
 *
 *  rtems_cpu_usage_report
 */

unsigned char lfr_rtems_cpu_usage_report( void )
{
    uint32_t             api_index;
    Thread_Control      *the_thread;
    Objects_Information *information;
    uint32_t             ival, fval;
#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    Timestamp_Control  uptime, total, ran;
#else
    uint32_t           total_units = 0;
#endif

    unsigned char cpu_load;
    cpu_load = 0;

    /*
     *  When not using nanosecond CPU usage resolution, we have to count
     *  the number of "ticks" we gave credit for to give the user a rough
     *  guideline as to what each number means proportionally.
     */
#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    _TOD_Get_uptime( &uptime );
    _Timestamp_Subtract( &CPU_usage_Uptime_at_last_reset, &uptime, &total );
#else
    for ( api_index = 1 ; api_index <= OBJECTS_APIS_LAST ; api_index++ ) {
        if ( !_Objects_Information_table[ api_index ] )
        {
            continue;
        }
        information = _Objects_Information_table[ api_index ][ 1 ];
        if ( information )
        {
            for ( i=1 ; i <= information->maximum ; i++ ) {
                the_thread = (Thread_Control *)information->local_table[ i ];

                if ( the_thread )
                    total_units += the_thread->cpu_time_used;
            }
        }
    }
#endif

    for ( api_index = 1 ; api_index <= OBJECTS_APIS_LAST ; api_index++ )
    {
        if ( !_Objects_Information_table[ api_index ] )
        {
            continue;
        }
        information = _Objects_Information_table[ api_index ][ 1 ];
        if ( information )
        {
            the_thread = (Thread_Control *)information->local_table[ 1 ];

            if ( !the_thread )
            {
                continue;
            }

#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
            /*
           * If this is the currently executing thread, account for time
           * since the last context switch.
           */
            ran = the_thread->cpu_time_used;
            if ( _Thread_Executing->Object.id == the_thread->Object.id )
            {
                Timestamp_Control used;
                _Timestamp_Subtract(
                            &_Thread_Time_of_last_context_switch, &uptime, &used
                            );
                _Timestamp_Add_to( &ran, &used );
            }
            _Timestamp_Divide( &ran, &total, &ival, &fval );

#else
            if (total_units)
            {
                uint64_t ival_64;

                ival_64 = the_thread->cpu_time_used;
                ival_64 *= 100000;
                ival = ival_64 / total_units;
            }
            else
            {
                ival = 0;
            }

            fval = ival % 1000;
            ival /= 1000;
#endif
        }
    }
    cpu_load = (unsigned char) (100 - ival);

    return cpu_load;
}



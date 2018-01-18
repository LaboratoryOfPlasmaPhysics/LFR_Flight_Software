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

unsigned char lfr_rtems_cpu_usage_report( void )
{
    uint32_t             api_index;
    Thread_Control      *the_thread;
    Objects_Information *information;
    uint32_t             ival;
    uint32_t             fval;
#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    Timestamp_Control  uptime;
    Timestamp_Control  total;
    Timestamp_Control  ran;
#else
    uint32_t           total_units = 0;
#endif

    unsigned char cpu_load;

    ival = 0;
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
        if ( !_Objects_Information_table[ api_index ] ) { }
        else
        {
            information = _Objects_Information_table[ api_index ][ 1 ];
            if ( information != NULL )
            {
                for ( i=1 ; i <= information->maximum ; i++ ) {
                    the_thread = (Thread_Control *)information->local_table[ i ];

                    if ( the_thread != NULL ) {
                        total_units += the_thread->cpu_time_used; }
                }
            }
        }
    }
#endif

    for ( api_index = 1 ; api_index <= OBJECTS_APIS_LAST ; api_index++ )
    {
        if ( !_Objects_Information_table[ api_index ] ) { }
        else
        {
            information = _Objects_Information_table[ api_index ][ 1 ];
            if ( information != NULL )
            {
                the_thread = (Thread_Control *)information->local_table[ 1 ];

                if ( the_thread == NULL ) { }
                else
                {
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
                    if (total_units != 0)
                    {
                        uint64_t ival_64;

                        ival_64 = the_thread->cpu_time_used;
                        ival_64 *= CONST_100000;
                        ival = ival_64 / total_units;
                    }
                    else
                    {
                        ival = 0;
                    }

                    fval = ival % CONST_1000;
                    ival /= CONST_1000;
    #endif
                }
            }
        }
    }
    cpu_load = (unsigned char) (CONST_100 - ival);

    return cpu_load;
}



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
#include "fsw_params.h"

extern rtems_id    Task_id[];

unsigned char lfr_rtems_cpu_usage_report( void )
{
    uint32_t             api_index;
    uint32_t             information_index;
    Thread_Control      *the_thread;
    Objects_Information *information;
    uint32_t             ival;
    uint32_t             fval;
#ifndef __RTEMS_USE_TICKS_FOR_STATISTICS__
    Timestamp_Control  uptime;
    Timestamp_Control  total;
    Timestamp_Control  ran;
    Timestamp_Control  abs_total;
    Timestamp_Control  abs_ran;

    static Timestamp_Control  last_total={0,0};
    static Timestamp_Control  last_ran={0,0};
#else
    #error "Can't compute CPU usage using ticks on LFR"
#endif

    unsigned char cpu_load;

    ival = 0;
    cpu_load = 0;

    _TOD_Get_uptime( &uptime );
    _Timestamp_Subtract( &CPU_usage_Uptime_at_last_reset, &uptime, &abs_total );
    for ( api_index = 1 ; api_index <= OBJECTS_APIS_LAST ; api_index++ )
    {
        if ( !_Objects_Information_table[ api_index ] ) { }
        else
        {
            information = _Objects_Information_table[ api_index ][ 1 ];
            if ( information != NULL )
            {
                for(information_index=1;information_index<=information->maximum;information_index++)
                {
                    the_thread = (Thread_Control *)information->local_table[ information_index ];

                    if ( the_thread == NULL) { }
                    else if(the_thread->Object.id == Task_id[TASKID_SCRB]) // Only measure scrubbing task load, CPU load is 100%-Scrubbing
                    {
                        /*
                        * If this is the currently executing thread, account for time
                        * since the last context switch.
                        */
                        abs_ran = the_thread->cpu_time_used;
                        if ( _Thread_Executing->Object.id == the_thread->Object.id )
                        {
                            Timestamp_Control used;
                            _Timestamp_Subtract(
                                        &_Thread_Time_of_last_context_switch, &uptime, &used
                                        );
                            _Timestamp_Add_to( &abs_ran, &used );
                        }
                        /*
                         * Only consider the time since last call
                         */
                        _Timespec_Subtract(&last_ran, &abs_ran, &ran);
                        _Timespec_Subtract(&last_total, &abs_total, &total);

                        last_ran = abs_ran;
                        last_total = abs_total;

                        _Timestamp_Divide( &ran, &total, &ival, &fval);
                        cpu_load = (unsigned char)(CONST_100 - ival);
                    }
                }
            }
        }
    }
    return cpu_load;
}



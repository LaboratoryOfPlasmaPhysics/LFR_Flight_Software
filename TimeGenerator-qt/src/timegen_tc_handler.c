/** Functions and tasks related to TeleCommand handling.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle TeleCommands:\n
 * action launching\n
 * TC parsing\n
 * ...
 *
 */

#include "timegen_tc_handler.h"

//***********
// RTEMS TASK

rtems_task act__task( rtems_task_argument unused )
{
    /** This RTEMS task is responsible for launching actions upton the reception of valid TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The ACTN task waits for data coming from an RTEMS msesage queue. When data arrives, it launches specific actions depending
     * on the incoming TeleCommand.
     *
     */

    int result;
    rtems_status_code status;       // RTEMS status code
    ccsdsTelecommandPacket_t TC;    // TC sent to the ACTN task
    size_t size;                    // size of the incoming TC packet
    unsigned char subtype;          // subtype of the current TC packet
    unsigned char time[6];
    rtems_id queue_rcv_id;
    rtems_id queue_snd_id;

    status =  get_message_queue_id_recv( &queue_rcv_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR get_message_queue_id_recv %d\n", status)
    }

    status =  get_message_queue_id_send( &queue_snd_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR get_message_queue_id_send %d\n", status)
    }

    result = LFR_SUCCESSFUL;
    subtype = 0;          // subtype of the current TC packet

    BOOT_PRINTF("in ACTN *** \n")

    while(1)
    {
        status = rtems_message_queue_receive( queue_rcv_id, (char*) &TC, &size,
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        getTime( time );    // set time to the current time
        if (status!=RTEMS_SUCCESSFUL)
        {
            PRINTF1("ERR *** in task ACTN *** error receiving a message, code %d \n", status)
        }
        else
        {
            subtype = TC.serviceSubType;
            switch(subtype)
            {
                case TC_SUBTYPE_ENTER:
                    result = timegen_action_enter_mode( &TC, queue_snd_id, time );
                    close_action( &TC, result, queue_snd_id );
                    break;
                case TC_SUBTYPE_UPDT_TIME:
                    result = timegen_action_update_time( &TC );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                default:
                    break;
            }
        }
    }
}

//***********
// TC ACTIONS

int timegen_action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    int ret;

    ret = LFR_SUCCESSFUL;

    return ret;
}

int timegen_action_update_time(ccsdsTelecommandPacket_t *TC)
{
    int ret;

    ret = LFR_SUCCESSFUL;

    return ret;
}

//*******************
// ENTERING THE MODES

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

#include "tc_handler.h"

//***********
// RTEMS TASK

unsigned int incomingTransitionCoarseTime;

void reset_transitionCoarseTime( void )
{
    incomingTransitionCoarseTime = 0xffffffff;
}

void set_transitionCoarseTime( unsigned int value )
{
    incomingTransitionCoarseTime = value;
}

unsigned int get_transitionCoarseTime( void )
{
    return incomingTransitionCoarseTime;
}

rtems_task actn_task( rtems_task_argument unused )
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
                result = action_enter_mode( &TC, queue_snd_id );
                break;
            case TC_SUBTYPE_UPDATE_TIME:
                result = action_update_time( &TC );
                break;
            default:
                break;
            }
        }
    }
}

//***********
// TC ACTIONS

int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    return LFR_SUCCESSFUL;
}

int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    unsigned int *transitionCoarseTime_ptr;
    unsigned int transitionCoarseTime;
    unsigned char * bytePosPtr;

    bytePosPtr = (unsigned char *) &TC->packetID;

    transitionCoarseTime_ptr = (unsigned int *) ( &bytePosPtr[ BYTE_POS_CP_LFR_ENTER_MODE_TIME ] );
    transitionCoarseTime = transitionCoarseTime_ptr[0] & 0x7fffffff;
    printf("local coarse time (without sync bit) = %x, requested transitionCoarseTime = %x\n",
           getLocalCoarseTime(),
           transitionCoarseTime);

    set_transitionCoarseTime( transitionCoarseTime );

    return LFR_SUCCESSFUL;
}

int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    return LFR_SUCCESSFUL;
}

int action_enable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    return LFR_SUCCESSFUL;
}

int action_disable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    return LFR_SUCCESSFUL;
}

int action_update_time(ccsdsTelecommandPacket_t *TC)
{
    unsigned int incomingCoarseTime;
    unsigned int currentLocalCoarseTime;

    incomingCoarseTime = (TC->dataAndCRC[0] << 24)
            + (TC->dataAndCRC[1] << 16)
            + (TC->dataAndCRC[2] << 8)
            + TC->dataAndCRC[3];

    currentLocalCoarseTime = getLocalCoarseTime();
    setLocalCoarseTime( incomingCoarseTime );
    printf( "currentLocalCoarseTime = %x, localCoarseTime set to: %x\n", currentLocalCoarseTime, getLocalCoarseTime() );

    return LFR_SUCCESSFUL;
}

//*******************
// ENTERING THE MODES
int check_mode_value( unsigned char requestedMode )
{
    return LFR_SUCCESSFUL;
}

int check_mode_transition( unsigned char requestedMode )
{
    return LFR_SUCCESSFUL;
}

int check_transition_date( unsigned int transitionCoarseTime )
{
    return LFR_SUCCESSFUL;
}

int stop_current_mode( void )
{
    return LFR_SUCCESSFUL;
}

int enter_mode( unsigned char mode, unsigned int transitionCoarseTime )
{
    return LFR_SUCCESSFUL;
}

int restart_science_tasks(unsigned char lfrRequestedMode )
{
    return LFR_SUCCESSFUL;
}

int suspend_science_tasks()
{
    return LFR_SUCCESSFUL;
}

void launch_waveform_picker( unsigned char mode, unsigned int transitionCoarseTime )
{
}

void launch_spectral_matrix( void )
{
}

void launch_spectral_matrix_simu( void )
{
}

void set_irq_on_new_ready_matrix( unsigned char value )
{
}

void set_run_matrix_spectral( unsigned char value )
{
}

//****************
// CLOSING ACTIONS
void update_last_TC_exe( ccsdsTelecommandPacket_t *TC, unsigned char * time )
{
}

void update_last_TC_rej(ccsdsTelecommandPacket_t *TC, unsigned char * time )
{
}

void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id )
{
}

//***************************
// Interrupt Service Routines
rtems_isr commutation_isr1( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        printf("In commutation_isr1 *** Error sending event to DUMB\n");
    }
}

rtems_isr commutation_isr2( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
        printf("In commutation_isr2 *** Error sending event to DUMB\n");
    }
}

//****************
// OTHER FUNCTIONS
void updateLFRCurrentMode()
{
}


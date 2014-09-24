/** Functions related to the SpaceWire interface.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle SpaceWire transmissions:
 * - configuration of the SpaceWire link
 * - SpaceWire related interruption requests processing
 * - transmission of TeleMetry packets by a dedicated RTEMS task
 * - reception of TeleCommands by a dedicated RTEMS task
 *
 */

#include "timegen_spacewire.h"

rtems_name semq_name;
rtems_id semq_id;

//***********
// RTEMS TASK
rtems_task spiq_task(rtems_task_argument unused)
{
    /** This RTEMS task is awaken by an rtems_event sent by the interruption subroutine of the SpaceWire driver.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     */

    rtems_event_set event_out;
    rtems_status_code status;
    int linkStatus;

    BOOT_PRINTF("in SPIQ *** \n")

    while(true){
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT
        PRINTF("in SPIQ *** got SPW_LINKERR_EVENT\n")

        // [0] SUSPEND RECV AND SEND TASKS
        status = rtems_task_suspend( Task_id[ TASKID_RECV ] );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF("in SPIQ *** ERR suspending RECV Task\n")
        }
        status = rtems_task_suspend( Task_id[ TASKID_SEND ] );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF("in SPIQ *** ERR suspending SEND Task\n")
        }

        // [1] CHECK THE LINK
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);   // get the link status (1)
        if ( linkStatus != 5) {
            PRINTF1("in SPIQ *** linkStatus %d, wait...\n", linkStatus)
            status = rtems_task_wake_after( SY_LFR_DPU_CONNECT_TIMEOUT );        // wait SY_LFR_DPU_CONNECT_TIMEOUT 1000 ms
        }

        // [2] RECHECK THE LINK AFTER SY_LFR_DPU_CONNECT_TIMEOUT
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);    // get the link status (2)
        if ( linkStatus != 5 )  // [2.a] not in run state, reset the link
        {
            spacewire_compute_stats_offsets();
            status = spacewire_reset_link( );
        }
        else                    // [2.b] in run state, start the link
        {
            status = spacewire_stop_start_link( fdSPW ); // start the link
            if ( status != RTEMS_SUCCESSFUL)
            {
                PRINTF1("in SPIQ *** ERR spacewire_start_link %d\n", status)
            }
        }

        // [3] COMPLETE RECOVERY ACTION AFTER SY_LFR_DPU_CONNECT_ATTEMPTS
        if ( status == RTEMS_SUCCESSFUL )   // [3.a] the link is in run state and has been started successfully
        {
            status = rtems_task_restart( Task_id[ TASKID_SEND ], 1 );
            if ( status != RTEMS_SUCCESSFUL ) {
                PRINTF("in SPIQ *** ERR resuming SEND Task\n")
            }
            status = rtems_task_restart( Task_id[ TASKID_RECV ], 1 );
            if ( status != RTEMS_SUCCESSFUL ) {
                PRINTF("in SPIQ *** ERR resuming RECV Task\n")
            }
        }
        else                                // [3.b] the link is not in run state, go in STANDBY mode
        {
            // wake the WTDG task up to wait for the link recovery
            status =  rtems_event_send ( Task_id[TASKID_WTDG], RTEMS_EVENT_0 );
            status = rtems_task_suspend( RTEMS_SELF );
        }
    }
}

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc )
{
//    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_1 );
    struct grgpio_regs_str *grgpio_regs = (struct grgpio_regs_str *) REGS_ADDR_GRGPIO;

    grgpio_regs->io_port_direction_register =
            grgpio_regs->io_port_direction_register | 0x08; // [0001 1000], 0 = output disabled, 1 = output enabled

    if ( (grgpio_regs->io_port_output_register & 0x08) == 0x08 )
    {
        grgpio_regs->io_port_output_register = grgpio_regs->io_port_output_register & 0xf7;
    }
    else
    {
        grgpio_regs->io_port_output_register = grgpio_regs->io_port_output_register | 0x08;
    }

}

rtems_timer_service_routine user_routine( rtems_id timer_id, void *user_data )
{
    int linkStatus;
    rtems_status_code status;

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);   // get the link status

    if ( linkStatus == 5) {
        PRINTF("in spacewire_reset_link *** link is running\n")
        status = RTEMS_SUCCESSFUL;
    }
}

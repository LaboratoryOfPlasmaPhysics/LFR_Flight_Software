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

#include "timegen_misc.h"
#include <stdio.h>

void timegen_timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc )
{
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

    rtems_event_send( rtems_id_updt, RTEMS_EVENT_0 );
}

void initCoarseTime()
{
    coarseTime = 0x00;
}

rtems_task updt_task( rtems_task_argument unused )
{
    rtems_event_set event_out;
    rtems_id send_queue_id;

    get_message_queue_id_send( &send_queue_id );

    BOOT_PRINTF("in UPDT *** waiting for SpaceWire ticks\n")

    while(1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_0,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        // increment the coarse time
        coarseTime = coarseTime + 1;
        PRINTF2("next valid coarseTime = 0x%x *** %d s\n", coarseTime, coarseTime)
        rtems_task_wake_after(70);    // 10 ms * 70 = 700 ms
        send_tc_lfr_update_time( send_queue_id );
    }
}

int send_tc_lfr_update_time(rtems_id queue_id )
{

    rtems_status_code status;
    unsigned char messageSize;

    Packet_TC_LFR_UPDATE_TIME_WITH_OVERHEAD_t packet;
    unsigned char crcAsTwoBytes[2];

    // OVERHEAD
    packet.targetLogicalAddress = DESTINATION_ID_LFR;
    packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    packet.reserved = DEFAULT_RESERVED;
    packet.userApplication = CCSDS_USER_APP;

    // TIME PACKET
    packet.update_time.packetID[0] = (unsigned char) (TC_LFR_PACKET_ID >> 8);
    packet.update_time.packetID[1] = (unsigned char) (TC_LFR_PACKET_ID     );
    packet.update_time.packetSequenceControl[0] = (unsigned char) (TC_LFR_PACKET_SEQUENCE_CONTROL >> 8);
    packet.update_time.packetSequenceControl[1] = (unsigned char) (TC_LFR_PACKET_SEQUENCE_CONTROL     );
    packet.update_time.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_LFR_UPDATE_TIME >> 8);
    packet.update_time.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_LFR_UPDATE_TIME     );

    packet.update_time.ccsdsSecHeaderFlag_pusVersion_ack = 0x19;
    packet.update_time.serviceType = TC_TYPE_LFR_UPDATE_TIME;
    packet.update_time.serviceSubType = TC_SUBTYPE_UPDATE_TIME;
    packet.update_time.sourceID = SID_TC_RPW_INTERNAL;
    packet.update_time.cp_rpw_time[0] = (unsigned char) (coarseTime >> 24);
    packet.update_time.cp_rpw_time[1] = (unsigned char) (coarseTime >> 16);
    packet.update_time.cp_rpw_time[2] = (unsigned char) (coarseTime >> 8);
    packet.update_time.cp_rpw_time[3] = (unsigned char) (coarseTime);
    packet.update_time.cp_rpw_time[4] = 0; // fine time MSB
    packet.update_time.cp_rpw_time[5] = 0; // fine time LSB

    GetCRCAsTwoBytes((unsigned char*) &packet.update_time, crcAsTwoBytes,
                                   PACKET_LENGTH_TC_LFR_UPDATE_TIME + CCSDS_TC_TM_PACKET_OFFSET - 2);
    packet.update_time.crc[0] = crcAsTwoBytes[0];
    packet.update_time.crc[1] = crcAsTwoBytes[1];

    messageSize = PACKET_LENGTH_TC_LFR_UPDATE_TIME + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &packet, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_success *** ERR\n")
    }

    return status;
}

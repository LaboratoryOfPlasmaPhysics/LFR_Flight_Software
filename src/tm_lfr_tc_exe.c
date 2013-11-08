/** Functions to send TM packets related to TC parsing and execution.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to send appropriate TM packets after parsing and execution:
 * - TM_LFR_TC_EXE_SUCCESS
 * - TM_LFR_TC_EXE_INCONSISTENT
 * - TM_LFR_TC_EXE_NOT_EXECUTABLE
 * - TM_LFR_TC_EXE_NOT_IMPLEMENTED
 * - TM_LFR_TC_EXE_ERROR
 * - TM_LFR_TC_EXE_CORRUPTED
 *
 */

#include "tm_lfr_tc_exe.h"

int send_tm_lfr_tc_exe_success( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    /** This function sends a TM_LFR_TC_EXE_SUCCESS packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_SUCCESS_t TM;
    unsigned char messageSize;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_SUCCESS >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_SUCCESS     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_OK;
    TM.destinationID = TC->sourceID;
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];

    messageSize = PACKET_LENGTH_TC_EXE_SUCCESS + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_success *** ERR\n")
    }

    return status;
}

int send_tm_lfr_tc_exe_inconsistent( ccsdsTelecommandPacket_t *TC, rtems_id queue_id,
                                    unsigned char byte_position, unsigned char rcv_value )
{
    /** This function sends a TM_LFR_TC_EXE_INCONSISTENT packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     * @param byte_position is the byte position of the MSB of the parameter that has been seen as inconsistent
     * @param rcv_value  is the value of the LSB of the parameter that has been deteced as inconsistent
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_INCONSISTENT_t TM;
    unsigned char messageSize;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_INCONSISTENT >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_INCONSISTENT     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TC->sourceID;
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (WRONG_APP_DATA >> 8);
    TM.tc_failure_code[1] = (char) (WRONG_APP_DATA     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC
    TM.byte_position = byte_position;
    TM.rcv_value = rcv_value;

    messageSize = PACKET_LENGTH_TC_EXE_INCONSISTENT + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_inconsistent *** ERR\n")
    }

    return status;
}

int send_tm_lfr_tc_exe_not_executable( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    /** This function sends a TM_LFR_TC_EXE_NOT_EXECUTABLE packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_NOT_EXECUTABLE_t TM;
    unsigned char messageSize;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TC->sourceID;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (TC_NOT_EXE >> 8);
    TM.tc_failure_code[1] = (char) (TC_NOT_EXE     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC
    TM.lfr_status_word[0] = housekeeping_packet.lfr_status_word[0];
    TM.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1];

    messageSize = PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_not_executable *** ERR\n")
    }

    return status;
}

int send_tm_lfr_tc_exe_not_implemented( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    /** This function sends a TM_LFR_TC_EXE_NOT_IMPLEMENTED packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_NOT_IMPLEMENTED_t TM;
    unsigned char messageSize;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TC->sourceID;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FUNCT_NOT_IMPL >> 8);
    TM.tc_failure_code[1] = (char) (FUNCT_NOT_IMPL     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC

    messageSize = PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_not_implemented *** ERR\n")
    }

    return status;
}

int send_tm_lfr_tc_exe_error( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    /** This function sends a TM_LFR_TC_EXE_ERROR packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_ERROR_t TM;
    unsigned char messageSize;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_ERROR >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_ERROR     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TC->sourceID;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FAIL_DETECTED >> 8);
    TM.tc_failure_code[1] = (char) (FAIL_DETECTED     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC

    messageSize = PACKET_LENGTH_TC_EXE_ERROR + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_error *** ERR\n")
    }

    return status;
}

int send_tm_lfr_tc_exe_corrupted(ccsdsTelecommandPacket_t *TC, rtems_id queue_id,
                                 unsigned char *computed_CRC, unsigned char *currentTC_LEN_RCV )
{
    /** This function sends a TM_LFR_TC_EXE_CORRUPTED packet in the dedicated RTEMS message queue.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM
     * @param computed_CRC points to a buffer of two bytes containing the CRC computed during the parsing of the TeleCommand
     * @param currentTC_LEN_RCV points to a buffer of two bytes containing a packet size field computed on the received data
     *
     * @return RTEMS directive status code:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_CORRUPTED_t TM;
    unsigned char messageSize;
    unsigned int packetLength;
    unsigned char *packetDataField;

    packetLength = (TC->packetLength[0] * 256) + TC->packetLength[1];   // compute the packet length parameter
    packetDataField = (unsigned char *) &TC->headerFlag_pusVersion_Ack; // get the beginning of the data field

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    increment_seq_counter_destination_id( TM.packetSequenceControl, TC->sourceID );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_CORRUPTED >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_CORRUPTED     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TC->sourceID;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (unsigned char) (CORRUPTED >> 8);
    TM.tc_failure_code[1] = (unsigned char) (CORRUPTED     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC
    TM.pkt_len_rcv_value[0] = TC->packetLength[0];
    TM.pkt_len_rcv_value[1] = TC->packetLength[1];
    TM.pkt_datafieldsize_cnt[0] = currentTC_LEN_RCV[0];
    TM.pkt_datafieldsize_cnt[1] = currentTC_LEN_RCV[1];
    TM.rcv_crc[0] = packetDataField[ packetLength - 1 ];
    TM.rcv_crc[1] = packetDataField[ packetLength ];
    TM.computed_crc[0] = computed_CRC[0];
    TM.computed_crc[1] = computed_CRC[1];

    messageSize = PACKET_LENGTH_TC_EXE_CORRUPTED + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_error *** ERR\n")
    }

    return status;
}

void increment_seq_counter_destination_id( unsigned char *packet_sequence_control, unsigned char destination_id )
{
    unsigned short sequence_cnt;
    unsigned short segmentation_grouping_flag;
    unsigned short new_packet_sequence_control;
    unsigned char i;

    switch (destination_id)
    {
    case SID_TC_GROUND:
        i = GROUND;
        break;
    case SID_TC_MISSION_TIMELINE:
        i = MISSION_TIMELINE;
        break;
    case SID_TC_TC_SEQUENCES:
        i = TC_SEQUENCES;
        break;
    case SID_TC_RECOVERY_ACTION_CMD:
        i = RECOVERY_ACTION_CMD;
        break;
    case SID_TC_BACKUP_MISSION_TIMELINE:
        i = BACKUP_MISSION_TIMELINE;
        break;
    case SID_TC_DIRECT_CMD:
        i = DIRECT_CMD;
        break;
    case SID_TC_SPARE_GRD_SRC1:
        i = SPARE_GRD_SRC1;
        break;
    case SID_TC_SPARE_GRD_SRC2:
        i = SPARE_GRD_SRC2;
        break;
    case SID_TC_OBCP:
        i = OBCP;
        break;
    case SID_TC_SYSTEM_CONTROL:
        i = SYSTEM_CONTROL;
        break;
    case SID_TC_AOCS:
        i = AOCS;
        break;
    case SID_TC_RPW_INTERNAL:
        i = RPW_INTERNAL;
        break;
    default:
        i = UNKNOWN;
        break;
    }

    if (i != UNKNOWN)
    {
        segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << 8;
        sequence_cnt                = sequenceCounters_TC_EXE[ i ] & 0x3fff;

        new_packet_sequence_control = segmentation_grouping_flag | sequence_cnt ;

        packet_sequence_control[0] = (unsigned char) (new_packet_sequence_control >> 8);
        packet_sequence_control[1] = (unsigned char) (new_packet_sequence_control     );

        // increment the seuqence counter for the next packet
        if ( sequenceCounters_TC_EXE[ i ] < SEQ_CNT_MAX)
        {
            sequenceCounters_TC_EXE[ i ] = sequenceCounters_TC_EXE[ i ] + 1;
        }
        else
        {
            sequenceCounters_TC_EXE[ i ] = 0;
        }
    }
    else
    {
        PRINTF1("in increment_seq_counter_destination_id *** ERR destination ID %d not known\n", destination_id)
    }

}

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

#include "fsw_spacewire.h"

char *lstates[6] = {"Error-reset",
                    "Error-wait",
                    "Ready",
                    "Started",
                    "Connecting",
                    "Run"
};

//***********
// RTEMS TASK
rtems_task spiq_task(rtems_task_argument unused)
{
    /** This RTEMS task is dedicated to the handling of interruption requests raised by the SpaceWire driver.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     */

    rtems_event_set event_out;
    rtems_status_code status;
    unsigned char lfrMode;

    while(true){
        BOOT_PRINTF("in SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4; // get the current mode

        status = spacewire_wait_for_link();

        if (status != RTEMS_SUCCESSFUL)
        {
            //****************
            // STOP THE SYSTEM
            spacewire_compute_stats_offsets();
            stop_current_mode();
            if (rtems_task_suspend(Task_id[TASKID_RECV])!=RTEMS_SUCCESSFUL) {   // suspend RECV task
                PRINTF("in SPIQ *** Error suspending RECV Task\n")
            }
            if (rtems_task_suspend(Task_id[TASKID_HOUS])!=RTEMS_SUCCESSFUL) {   // suspend HOUS task
                PRINTF("in SPIQ *** Error suspending HOUS Task\n")
            }

            //***************************
            // RESTART THE SPACEWIRE LINK
            spacewire_configure_link();

            //*******************
            // RESTART THE SYSTEM
            //ioctl(fdSPW, SPACEWIRE_IOCTRL_CLR_STATISTICS);   // clear statistics
            status = rtems_task_restart( Task_id[TASKID_HOUS], 1 );
            if (status != RTEMS_SUCCESSFUL) {
                PRINTF1("in SPIQ *** Error restarting HOUS Task *** code %d\n", status)
            }
            status = rtems_task_restart(Task_id[TASKID_RECV], 1);
            if ( status != RTEMS_SUCCESSFUL) {
                PRINTF("in SPIQ *** Error restarting RECV Task\n")
            }
            enter_mode(lfrMode, NULL); // enter the mode that was running before the SpaceWire interruption
        }
    }
}

rtems_task recv_task( rtems_task_argument unused )
{
    /** This RTEMS task is dedicated to the reception of incoming TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The RECV task blocks on a call to the read system call, waiting for incoming SpaceWire data. When unblocked:
     * 1. It reads the incoming data.
     * 2. Launches the acceptance procedure.
     * 3. If the Telecommand is valid, sends it to a dedicated RTEMS message queue.
     *
     */

    int len;
    ccsdsTelecommandPacket_t currentTC;
    unsigned char computed_CRC[ 2 ];
    unsigned char currentTC_LEN_RCV[ 2 ];
    unsigned int currentTC_LEN_RCV_AsUnsignedInt;
    unsigned int parserCode;
    rtems_status_code status;
    rtems_id queue_recv_id;
    rtems_id queue_send_id;

    initLookUpTableForCRC(); // the table is used to compute Cyclic Redundancy Codes

    status =  rtems_message_queue_ident( misc_name[QUEUE_RECV], 0, &queue_recv_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR getting QUEUE_RECV id, %d\n", status)
    }

    status =  rtems_message_queue_ident( misc_name[QUEUE_SEND], 0, &queue_send_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR getting QUEUE_SEND id, %d\n", status)
    }

    BOOT_PRINTF("in RECV *** \n")

    while(1)
    {
        len = read(fdSPW, (char*) &currentTC, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
        if (len == -1){ // error during the read call
            PRINTF("In RECV *** last read call returned -1\n")
        }
        else {
            if ( (len+1) < CCSDS_TC_PKT_MIN_SIZE ) {
                PRINTF("In RECV *** packet lenght too short\n")
            }
            else {
                currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); // => -3 is for Prot ID, Reserved and User App bytes
                currentTC_LEN_RCV[ 0 ] = (unsigned char) (currentTC_LEN_RCV_AsUnsignedInt >> 8);
                currentTC_LEN_RCV[ 1 ] = (unsigned char) (currentTC_LEN_RCV_AsUnsignedInt     );
                // CHECK THE TC
                parserCode = tc_parser( &currentTC, currentTC_LEN_RCV_AsUnsignedInt, computed_CRC ) ;
                if ( (parserCode == ILLEGAL_APID) || (parserCode == WRONG_LEN_PACKET) || (parserCode == INCOR_CHECKSUM)
                    | (parserCode == ILL_TYPE) || (parserCode == ILL_SUBTYPE) || (parserCode == WRONG_APP_DATA) )
                { // send TM_LFR_TC_EXE_CORRUPTED
                    send_tm_lfr_tc_exe_corrupted( &currentTC, queue_send_id, computed_CRC, currentTC_LEN_RCV );
                }
                else
                { // send valid TC to the action launcher
                    status =  rtems_message_queue_send( queue_recv_id, &currentTC,
                                                        currentTC_LEN_RCV_AsUnsignedInt + CCSDS_TC_TM_PACKET_OFFSET + 3);
                }
            }
        }
    }
}

rtems_task send_task( rtems_task_argument argument)
{
    /** This RTEMS task is dedicated to the transmission of TeleMetry packets.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The SEND task waits for a message to become available in the dedicated RTEMS queue. When a message arrives:
     * - if the first byte is equal to CCSDS_DESTINATION_ID, the message is sent as is using the write system call.
     * - if the first byte is not equal to CCSDS_DESTINATION_ID, the message is handled as a spw_ioctl_pkt_send. After
     * analyzis, the packet is sent either using the write system call or using the ioctl call SPACEWIRE_IOCTRL_SEND, depending on the
     * data it contains.
     *
     */

    rtems_status_code status;               // RTEMS status code
    char incomingData[ACTION_MSG_PKTS_MAX_SIZE];  // incoming data buffer
    spw_ioctl_pkt_send *spw_ioctl_send;
    size_t size;                            // size of the incoming TC packet
    u_int32_t count;
    rtems_id queue_id;

    status =  rtems_message_queue_ident( misc_name[QUEUE_SEND], 0, &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in SEND *** ERR getting queue id, %d\n", status)
    }

    BOOT_PRINTF("in SEND *** \n")

    while(1)
    {
        status = rtems_message_queue_receive( queue_id, incomingData, &size,
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT );

        if (status!=RTEMS_SUCCESSFUL)
        {
            PRINTF1("in SEND *** (1) ERR = %d\n", status)
        }
        else
        {
            if ( incomingData[0] == CCSDS_DESTINATION_ID) // the incoming message is a ccsds packet
            {
                status = write( fdSPW, incomingData, size );
                if (status == -1){
                    PRINTF2("in SEND *** (2.a) ERR = %d, size = %d\n", status, size)
                }
            }
            else // the incoming message is a spw_ioctl_pkt_send structure
            {
                spw_ioctl_send = (spw_ioctl_pkt_send*) incomingData;
                if (spw_ioctl_send->hlen == 0)
                {
                    status = write( fdSPW, spw_ioctl_send->data, spw_ioctl_send->dlen );
                    if (status == -1){
                        PRINTF2("in SEND *** (2.b) ERR = %d, dlen = %d\n", status, spw_ioctl_send->dlen)
                    }
                }
                else
                {
                    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, spw_ioctl_send );
                    if (status == -1){
                        PRINTF2("in SEND *** (2.c) ERR = %d, dlen = %d\n", status, spw_ioctl_send->dlen)
                        PRINTF1("                            hlen = %d\n", spw_ioctl_send->hlen)
                    }
                }
            }
        }

        status = rtems_message_queue_get_number_pending( queue_id, &count );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in SEND *** (3) ERR = %d\n", status)
        }
        else
        {
            if (count > maxCount)
            {
                maxCount = count;
            }
        }
    }
}

//****************
// OTHER FUNCTIONS
int spacewire_configure_link( void )
{
    /** This function configures the SpaceWire link.
     *
     * @return GR-RTEMS-DRIVER directive status codes:
     * - 22  EINVAL - Null pointer or an out of range value was given as the argument.
     * - 16  EBUSY - Only used for SEND. Returned when no descriptors are avialble in non-blocking mode.
     * - 88  ENOSYS - Returned for SET_DESTKEY if RMAP command handler is not available or if a non-implemented call is used.
     * - 116 ETIMEDOUT - REturned for SET_PACKET_SIZE and START if the link could not be brought up.
     * - 12  ENOMEM - Returned for SET_PACKETSIZE if it was unable to allocate the new buffers.
     * - 5   EIO - Error when writing to grswp hardware registers.
     * - 2   ENOENT - No such file or directory
     */

    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    BOOT_PRINTF("OK  *** in configure_spw_link *** try to open "GRSPW_DEVICE_NAME"\n")
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call resets the hardware
    if ( fdSPW<0 ) {
        PRINTF1("ERR *** in configure_spw_link *** Error opening"GRSPW_DEVICE_NAME" with ERR %d\n", errno)
    }

    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, -1) != RTEMS_SUCCESSFUL){
        PRINTF(".")
        fflush( stdout );
        close( fdSPW ); // close the device
        fdSPW = open( GRSPW_DEVICE_NAME, O_RDWR ); // open the device. the open call reset the hardware
        if (fdSPW<0) {
            PRINTF1("ERR *** in configure_spw_link *** Error opening"GRSPW_DEVICE_NAME" with ERR %d\n", errno)
        }
        rtems_task_wake_after(100);
    }

    BOOT_PRINTF("OK  *** In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    spacewire_set_NP(1, REGS_ADDR_GRSPW); // [N]o [P]ort force
    spacewire_set_RE(1, REGS_ADDR_GRSPW); // [R]MAP [E]nable, the dedicated call seems to  break the no port force configuration

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 0);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK, 0);             // transmission blocks
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 0);      // transmission blocks on full
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909); // [Time Rx : Time Tx : Link error : Tick-out IRQ]
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")

    BOOT_PRINTF("OK  *** in configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

int spacewire_wait_for_link( void )
{
    /** This function is executed when an interruption is raised by the SpaceWire driver.
     *
     * @return RTEMS directive status code:
     * - RTEMS_UNSATISFIED is returned is the link is not in the running state after 10 s.
     * - RTEMS_SUCCESSFUL is returned if the link is up before the timeout.
     *
     */

    unsigned int i;
    int linkStatus;
    rtems_status_code status = RTEMS_UNSATISFIED;

    for(i = 0; i< 10; i++){
        PRINTF(".")
        fflush( stdout );
        ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);   // get the link status
        PRINTF1("in spacewire_wait_for_link *** link status is: %s\n", lstates[linkStatus])
        if ( linkStatus == 5) {
            PRINTF("in spacewire_wait_for_link *** link is running\n")
            status = RTEMS_SUCCESSFUL;
            break;
        }
        rtems_task_wake_after(100);
    }

    return status;
}

void spacewire_set_NP( unsigned char val, unsigned int regAddr ) // [N]o [P]ort force
{
    /** This function sets the [N]o [P]ort force bit of the GRSPW control register.
     *
     * @param val is the value, 0 or 1, used to set the value of the NP bit.
     * @param regAddr is the address of the GRSPW control register.
     *
     * NP is the bit 20 of the GRSPW control register.
     *
     */

    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1) {
        *spwptr = *spwptr | 0x00100000; // [NP] set the No port force bit
    }
    if (val== 0) {
        *spwptr = *spwptr & 0xffdfffff;
    }
}

void spacewire_set_RE( unsigned char val, unsigned int regAddr ) // [R]MAP [E]nable
{
    /** This function sets the [R]MAP [E]nable bit of the GRSPW control register.
     *
     * @param val is the value, 0 or 1, used to set the value of the RE bit.
     * @param regAddr is the address of the GRSPW control register.
     *
     * RE is the bit 16 of the GRSPW control register.
     *
     */

    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1)
    {
        *spwptr = *spwptr | 0x00010000; // [RE] set the RMAP Enable bit
    }
    if (val== 0)
    {
        *spwptr = *spwptr & 0xfffdffff;
    }
}

void spacewire_compute_stats_offsets( void )
{
    /** This function computes the SpaceWire statistics offsets in case of a SpaceWire related interruption raising.
     *
     * The offsets keep a record of the statistics in case of a reset of the statistics. They are added to the current statistics
     * to keep the counters consistent even after a reset of the SpaceWire driver (the counter are set to zero by the driver when it
     * during the open systel call).
     *
     */

    spw_stats spacewire_stats_grspw;
    rtems_status_code status;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &spacewire_stats_grspw );

    spacewire_stats_backup.packets_received = spacewire_stats_grspw.packets_received
            + spacewire_stats.packets_received;
    spacewire_stats_backup.packets_sent = spacewire_stats_grspw.packets_sent
            + spacewire_stats.packets_sent;
    spacewire_stats_backup.parity_err = spacewire_stats_grspw.parity_err
            + spacewire_stats.parity_err;
    spacewire_stats_backup.disconnect_err = spacewire_stats_grspw.disconnect_err
            + spacewire_stats.disconnect_err;
    spacewire_stats_backup.escape_err = spacewire_stats_grspw.escape_err
            + spacewire_stats.escape_err;
    spacewire_stats_backup.credit_err = spacewire_stats_grspw.credit_err
            + spacewire_stats.credit_err;
    spacewire_stats_backup.write_sync_err = spacewire_stats_grspw.write_sync_err
            + spacewire_stats.write_sync_err;
    spacewire_stats_backup.rx_rmap_header_crc_err = spacewire_stats_grspw.rx_rmap_header_crc_err
            + spacewire_stats.rx_rmap_header_crc_err;
    spacewire_stats_backup.rx_rmap_data_crc_err = spacewire_stats_grspw.rx_rmap_data_crc_err
            + spacewire_stats.rx_rmap_data_crc_err;
    spacewire_stats_backup.early_ep = spacewire_stats_grspw.early_ep
            + spacewire_stats.early_ep;
    spacewire_stats_backup.invalid_address = spacewire_stats_grspw.invalid_address
            + spacewire_stats.invalid_address;
    spacewire_stats_backup.rx_eep_err = spacewire_stats_grspw.rx_eep_err
            + spacewire_stats.rx_eep_err;
    spacewire_stats_backup.rx_truncated = spacewire_stats_grspw.rx_truncated
            + spacewire_stats.rx_truncated;
}

void spacewire_update_statistics( void )
{
    rtems_status_code status;
    spw_stats spacewire_stats_grspw;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &spacewire_stats_grspw );

    spacewire_stats.packets_received = spacewire_stats_backup.packets_received
            + spacewire_stats_grspw.packets_received;
    spacewire_stats.packets_sent = spacewire_stats_backup.packets_sent
            + spacewire_stats_grspw.packets_sent;
    spacewire_stats.parity_err = spacewire_stats_backup.parity_err
            + spacewire_stats_grspw.parity_err;
    spacewire_stats.disconnect_err = spacewire_stats_backup.disconnect_err
            + spacewire_stats_grspw.disconnect_err;
    spacewire_stats.escape_err = spacewire_stats_backup.escape_err
            + spacewire_stats_grspw.escape_err;
    spacewire_stats.credit_err = spacewire_stats_backup.credit_err
            + spacewire_stats_grspw.credit_err;
    spacewire_stats.write_sync_err = spacewire_stats_backup.write_sync_err
            + spacewire_stats_grspw.write_sync_err;
    spacewire_stats.rx_rmap_header_crc_err = spacewire_stats_backup.rx_rmap_header_crc_err
            + spacewire_stats_grspw.rx_rmap_header_crc_err;
    spacewire_stats.rx_rmap_data_crc_err = spacewire_stats_backup.rx_rmap_data_crc_err
            + spacewire_stats_grspw.rx_rmap_data_crc_err;
    spacewire_stats.early_ep = spacewire_stats_backup.early_ep
            + spacewire_stats_grspw.early_ep;
    spacewire_stats.invalid_address = spacewire_stats_backup.invalid_address
            + spacewire_stats_grspw.invalid_address;
    spacewire_stats.rx_eep_err = spacewire_stats_backup.rx_eep_err
            +  spacewire_stats_grspw.rx_eep_err;
    spacewire_stats.rx_truncated = spacewire_stats_backup.rx_truncated
            + spacewire_stats_grspw.rx_truncated;
    //spacewire_stats.tx_link_err;

    //****************************
    // DPU_SPACEWIRE_IF_STATISTICS
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[0] = (unsigned char) (spacewire_stats.packets_received >> 8);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[1] = (unsigned char) (spacewire_stats.packets_received);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[0] = (unsigned char) (spacewire_stats.packets_sent >> 8);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[1] = (unsigned char) (spacewire_stats.packets_sent);
    //housekeeping_packet.hk_lfr_dpu_spw_tick_out_cnt;
    //housekeeping_packet.hk_lfr_dpu_spw_last_timc;

    //******************************************
    // ERROR COUNTERS / SPACEWIRE / LOW SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_parity = (unsigned char) spacewire_stats.parity_err;
    housekeeping_packet.hk_lfr_dpu_spw_disconnect = (unsigned char) spacewire_stats.disconnect_err;
    housekeeping_packet.hk_lfr_dpu_spw_escape = (unsigned char) spacewire_stats.escape_err;
    housekeeping_packet.hk_lfr_dpu_spw_credit = (unsigned char) spacewire_stats.credit_err;
    housekeeping_packet.hk_lfr_dpu_spw_write_sync = (unsigned char) spacewire_stats.write_sync_err;
    // housekeeping_packet.hk_lfr_dpu_spw_rx_ahb;
    // housekeeping_packet.hk_lfr_dpu_spw_tx_ahb;
    housekeeping_packet.hk_lfr_dpu_spw_header_crc = (unsigned char) spacewire_stats.rx_rmap_header_crc_err;
    housekeeping_packet.hk_lfr_dpu_spw_data_crc = (unsigned char) spacewire_stats.rx_rmap_data_crc_err;

    //*********************************************
    // ERROR COUNTERS / SPACEWIRE / MEDIUM SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_early_eop = (unsigned char) spacewire_stats.early_ep;
    housekeeping_packet.hk_lfr_dpu_spw_invalid_addr = (unsigned char) spacewire_stats.invalid_address;
    housekeeping_packet.hk_lfr_dpu_spw_eep = (unsigned char) spacewire_stats.rx_eep_err;
    housekeeping_packet.hk_lfr_dpu_spw_rx_too_big = (unsigned char) spacewire_stats.rx_truncated;

}

void timecode_irq_handler( void *pDev, void *regs, int minor, unsigned int tc )
{
    //if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_1 ) != RTEMS_SUCCESSFUL) {
    //    printf("In timecode_irq_handler *** Error sending event to DUMB\n");
    //}
}

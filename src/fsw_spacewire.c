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
            status = stop_current_mode();
            if ( status != RTEMS_SUCCESSFUL ) {
                PRINTF1("in SPIQ *** ERR stop_current_mode *** code %d\n", status)
            }
            status = enter_standby_mode();
            if ( status != RTEMS_SUCCESSFUL ) {
                PRINTF1("in SPIQ *** ERR enter_standby_mode *** code %d\n", status)
            }
            // wake the WTDG task up to wait for the link recovery
            status =  rtems_event_send ( Task_id[TASKID_WTDG], RTEMS_EVENT_0 );
            status = rtems_task_suspend( RTEMS_SELF );
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
    unsigned char destinationID;
    unsigned int currentTC_LEN_RCV_AsUnsignedInt;
    unsigned int parserCode;
    unsigned char time[6];
    rtems_status_code status;
    rtems_id queue_recv_id;
    rtems_id queue_send_id;

    initLookUpTableForCRC(); // the table is used to compute Cyclic Redundancy Codes

    status =  get_message_queue_id_recv( &queue_recv_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR get_message_queue_id_recv %d\n", status)
    }

    status =  get_message_queue_id_send( &queue_send_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in RECV *** \n")

    while(1)
    {
        len = read( fdSPW, (char*) &currentTC, CCSDS_TC_PKT_MAX_SIZE ); // the call to read is blocking
        if (len == -1){ // error during the read call
            PRINTF1("in RECV *** last read call returned -1, ERRNO %d\n", errno)
        }
        else {
            if ( (len+1) < CCSDS_TC_PKT_MIN_SIZE ) {
                PRINTF("in RECV *** packet lenght too short\n")
            }
            else {
                currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); // => -3 is for Prot ID, Reserved and User App bytes
                currentTC_LEN_RCV[ 0 ] = (unsigned char) (currentTC_LEN_RCV_AsUnsignedInt >> 8);
                currentTC_LEN_RCV[ 1 ] = (unsigned char) (currentTC_LEN_RCV_AsUnsignedInt     );
                // CHECK THE TC
                parserCode = tc_parser( &currentTC, currentTC_LEN_RCV_AsUnsignedInt, computed_CRC ) ;
                if ( (parserCode == ILLEGAL_APID)       || (parserCode == WRONG_LEN_PKT)
                     || (parserCode == INCOR_CHECKSUM)  || (parserCode == ILL_TYPE)
                     || (parserCode == ILL_SUBTYPE)     || (parserCode == WRONG_APP_DATA)
                     || (parserCode == WRONG_SRC_ID) )
                { // send TM_LFR_TC_EXE_CORRUPTED
                    if ( !( (currentTC.serviceType==TC_TYPE_TIME) && (currentTC.serviceSubType==TC_SUBTYPE_UPDT_TIME) )
                         &&
                         !( (currentTC.serviceType==TC_TYPE_GEN) && (currentTC.serviceSubType==TC_SUBTYPE_UPDT_INFO))
                         )
                    {
                        if ( parserCode == WRONG_SRC_ID )
                        {
                            destinationID = SID_TC_GROUND;
                        }
                        else
                        {
                            destinationID = currentTC.sourceID;
                        }
                        getTime( time );
                        close_action( &currentTC, LFR_DEFAULT, queue_send_id, time);
                        send_tm_lfr_tc_exe_corrupted( &currentTC, queue_send_id,
                                                      computed_CRC, currentTC_LEN_RCV,
                                                      destinationID, time );
                    }
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

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in HOUS *** ERR get_message_queue_id_send %d\n", status)
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
                    PRINTF2("in SEND *** (2.a) ERRNO = %d, size = %d\n", errno, size)
                }
            }
            else // the incoming message is a spw_ioctl_pkt_send structure
            {
                spw_ioctl_send = (spw_ioctl_pkt_send*) incomingData;
                status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, spw_ioctl_send );
                if (status == -1){
                    PRINTF2("in SEND *** (2.b) ERRNO = %d, RTEMS = %d\n", errno, status)
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

rtems_task wtdg_task( rtems_task_argument argument )
{
    rtems_event_set event_out;
    rtems_status_code status;
    int linkStatus;

    BOOT_PRINTF("in WTDG ***\n")

    while(1)
    {
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_0,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        PRINTF("in WTDG *** wait for the link\n")
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);        // get the link status
        while( linkStatus != 5)                                             // wait for the link
        {
            rtems_task_wake_after( 10 );
            status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);    // get the link status
        }

        status = spacewire_stop_start_link( fdSPW );

        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in WTDG *** ERR link not started %d\n", status)
        }
        else
        {
            PRINTF("in WTDG *** OK  link started\n")
        }

        // restart the SPIQ task
        status = rtems_task_restart( Task_id[TASKID_SPIQ], 1 );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF("in SPIQ *** ERR restarting SPIQ Task\n")
        }

        // restart RECV and SEND
        status = rtems_task_restart( Task_id[ TASKID_SEND ], 1 );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF("in SPIQ *** ERR restarting SEND Task\n")
        }
        status = rtems_task_restart( Task_id[ TASKID_RECV ], 1 );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF("in SPIQ *** ERR restarting RECV Task\n")
        }
    }
}

//****************
// OTHER FUNCTIONS
int spacewire_open_link( void )
{
    /** This function opens the SpaceWire link.
     *
     * @return a valid file descriptor in case of success, -1 in case of a failure
     *
     */
    rtems_status_code status;

    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call resets the hardware
    if ( fdSPW < 0 ) {
        PRINTF1("ERR *** in configure_spw_link *** error opening "GRSPW_DEVICE_NAME" with ERR %d\n", errno)
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }

    return status;
}

int spacewire_start_link( int fd )
{
    rtems_status_code status;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
                                                        // -1 default hardcoded driver timeout

    return status;
}

int spacewire_stop_start_link( int fd )
{
    rtems_status_code status;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_STOP);      // start fails if link pDev->running != 0
    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
                                                        // -1 default hardcoded driver timeout

    return status;
}

int spacewire_configure_link( int fd )
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

    spacewire_set_NP(1, REGS_ADDR_GRSPW); // [N]o [P]ort force
    spacewire_set_RE(1, REGS_ADDR_GRSPW); // [R]MAP [E]nable, the dedicated call seems to  break the no port force configuration

    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 0);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK, 0);             // transmission blocks
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK\n")
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 1);      // transmission blocks when no transmission descriptor is available
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909); // [Time Rx : Time Tx : Link error : Tick-out IRQ]
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")

    return status;
}

int spacewire_reset_link( void )
{
    /** This function is executed by the SPIQ rtems_task wehn it has been awaken by an interruption raised by the SpaceWire driver.
     *
     * @return RTEMS directive status code:
     * - RTEMS_UNSATISFIED is returned is the link is not in the running state after 10 s.
     * - RTEMS_SUCCESSFUL is returned if the link is up before the timeout.
     *
     */

    rtems_status_code status_spw;
    int i;

    for ( i=0; i<SY_LFR_DPU_CONNECT_ATTEMPT; i++ )
    {
        PRINTF1("in spacewire_reset_link *** link recovery, try %d\n", i);

        // CLOSING THE DRIVER AT THIS POINT WILL MAKE THE SEND TASK BLOCK THE SYSTEM

        status_spw = spacewire_stop_start_link( fdSPW );
        if (  status_spw != RTEMS_SUCCESSFUL )
        {
            PRINTF1("in spacewire_reset_link *** ERR spacewire_start_link code %d\n",  status_spw)
        }

        if ( status_spw == RTEMS_SUCCESSFUL)
        {
            break;
        }
    }

    return status_spw;
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

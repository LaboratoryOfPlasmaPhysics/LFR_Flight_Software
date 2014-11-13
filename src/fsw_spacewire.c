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

//*****************
// waveform headers
Header_TM_LFR_SCIENCE_CWF_t headerCWF;
Header_TM_LFR_SCIENCE_SWF_t headerSWF;
Header_TM_LFR_SCIENCE_ASM_t headerASM;

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
            status = spacewire_stop_and_start_link( fdSPW ); // start the link
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
            status = enter_mode( LFR_MODE_STANDBY, 0 );
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
    unsigned int estimatedPacketLength;
    unsigned int parserCode;
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
                estimatedPacketLength = (unsigned int) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); // => -3 is for Prot ID, Reserved and User App bytes
                currentTC_LEN_RCV[ 0 ] = (unsigned char) (estimatedPacketLength >> 8);
                currentTC_LEN_RCV[ 1 ] = (unsigned char) (estimatedPacketLength     );
                // CHECK THE TC
                parserCode = tc_parser( &currentTC, estimatedPacketLength, computed_CRC ) ;
                if ( (parserCode == ILLEGAL_APID)       || (parserCode == WRONG_LEN_PKT)
                     || (parserCode == INCOR_CHECKSUM)  || (parserCode == ILL_TYPE)
                     || (parserCode == ILL_SUBTYPE)     || (parserCode == WRONG_APP_DATA)
                     || (parserCode == WRONG_SRC_ID) )
                { // send TM_LFR_TC_EXE_CORRUPTED
                    PRINTF1("TC corrupted received, with code: %d\n", parserCode)
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
                        send_tm_lfr_tc_exe_corrupted( &currentTC, queue_send_id,
                                                      computed_CRC, currentTC_LEN_RCV,
                                                      destinationID );
                    }
                }
                else
                { // send valid TC to the action launcher
                    status =  rtems_message_queue_send( queue_recv_id, &currentTC,
                                                        estimatedPacketLength + CCSDS_TC_TM_PACKET_OFFSET + 3);
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

    rtems_status_code status;                // RTEMS status code
    char incomingData[MSG_QUEUE_SIZE_SEND];  // incoming data buffer
    ring_node *incomingRingNodePtr;
    int ring_node_address;
    char *charPtr;
    spw_ioctl_pkt_send *spw_ioctl_send;
    size_t size;                            // size of the incoming TC packet
    u_int32_t count;
    rtems_id queue_id;
    unsigned char sid;

    incomingRingNodePtr = NULL;
    ring_node_address = 0;
    charPtr = (char *) &ring_node_address;
    sid = 0;

    init_header_cwf( &headerCWF );
    init_header_swf( &headerSWF );
    init_header_asm( &headerASM );

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
            if ( size == sizeof(ring_node*) )
            {
                charPtr[0] = incomingData[0];
                charPtr[1] = incomingData[1];
                charPtr[2] = incomingData[2];
                charPtr[3] = incomingData[3];
                incomingRingNodePtr = (ring_node*) ring_node_address;
                sid = incomingRingNodePtr->sid;
//                printf("sid = %d\n", incomingRingNodePtr->sid);
                if ( (sid==SID_NORM_CWF_LONG_F3)
                     || (sid==SID_BURST_CWF_F2 )
                     || (sid==SID_SBM1_CWF_F1  )
                     || (sid==SID_SBM2_CWF_F2  ))
                {
                    spw_send_waveform_CWF( incomingRingNodePtr, &headerCWF );
                }
                else if ( (sid==SID_NORM_SWF_F0) || (sid== SID_NORM_SWF_F1) || (sid==SID_NORM_SWF_F2) )
                {
                    spw_send_waveform_SWF( incomingRingNodePtr, &headerSWF );
                }
                else if ( (sid==SID_NORM_CWF_F3) )
                {
                    spw_send_waveform_CWF3_light( incomingRingNodePtr, &headerCWF );
                }
                else if ( (sid==SID_NORM_ASM_F0) || (SID_NORM_ASM_F1) || (SID_NORM_ASM_F2) )
                {
                    spw_send_asm( incomingRingNodePtr, &headerASM );
                }
                else
                {
                    printf("unexpected sid = %d\n", sid);
                }
            }
            else if ( incomingData[0] == CCSDS_DESTINATION_ID ) // the incoming message is a ccsds packet
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
                    printf("size = %d, %x, %x, %x, %x, %x\n",
                           size,
                           incomingData[0],
                            incomingData[1],
                            incomingData[2],
                            incomingData[3],
                            incomingData[4]);
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

        status = spacewire_stop_and_start_link( fdSPW );

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
int spacewire_open_link( void )  // by default, the driver resets the core: [SPW_CTRL_WRITE(pDev, SPW_CTRL_RESET);]
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

    status = ioctl( fd, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
                                                        // -1 default hardcoded driver timeout

    return status;
}

int spacewire_stop_and_start_link( int fd )
{
    rtems_status_code status;

    status = ioctl( fd, SPACEWIRE_IOCTRL_STOP);      // start fails if link pDev->running != 0
    status = ioctl( fd, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
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
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 0);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK, 1);              // transmission blocks
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK\n")
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 1);      // transmission blocks when no transmission descriptor is available
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    }
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909); // [Time Rx : Time Tx : Link error : Tick-out IRQ]
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")
    }

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

        status_spw = spacewire_stop_and_start_link( fdSPW );
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
//    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_9 );
    struct grgpio_regs_str *grgpio_regs = (struct grgpio_regs_str *) REGS_ADDR_GRGPIO;

    grgpio_regs->io_port_direction_register =
            grgpio_regs->io_port_direction_register | 0x04; // [0000 0100], 0 = output disabled, 1 = output enabled

    if ( (grgpio_regs->io_port_output_register & 0x04) == 0x04 )
    {
        grgpio_regs->io_port_output_register = grgpio_regs->io_port_output_register & 0xfb; // [1111 1011]
    }
    else
    {
        grgpio_regs->io_port_output_register = grgpio_regs->io_port_output_register | 0x04; // [0000 0100]
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

void init_header_cwf( Header_TM_LFR_SCIENCE_CWF_t *header )
{
    header->targetLogicalAddress        = CCSDS_DESTINATION_ID;
    header->protocolIdentifier          = CCSDS_PROTOCOLE_ID;
    header->reserved        = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetSequenceControl[0]    = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1]    = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_336 >> 8);
    header->packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_336     );
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2    = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType     = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType  = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID   = TM_DESTINATION_ID_GROUND;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    // AUXILIARY DATA HEADER
    header->sid = 0x00;
    header->hkBIA = DEFAULT_HKBIA;
    header->blkNr[0] = (unsigned char) (BLK_NR_CWF >> 8);
    header->blkNr[1] = (unsigned char) (BLK_NR_CWF     );
}

void init_header_swf( Header_TM_LFR_SCIENCE_SWF_t *header )
{
    header->targetLogicalAddress        = CCSDS_DESTINATION_ID;
    header->protocolIdentifier          = CCSDS_PROTOCOLE_ID;
    header->reserved        = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
    header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
    header->packetSequenceControl[0]    = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1]    = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_336 >> 8);
    header->packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_336     );
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2    = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType     = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType  = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID   = TM_DESTINATION_ID_GROUND;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    // AUXILIARY DATA HEADER
    header->sid     = 0x00;
    header->hkBIA   = DEFAULT_HKBIA;
    header->pktCnt  = DEFAULT_PKTCNT;  // PKT_CNT
    header->pktNr   = 0x00;
    header->blkNr[0]        = (unsigned char) (BLK_NR_CWF >> 8);
    header->blkNr[1]        = (unsigned char) (BLK_NR_CWF     );
}

void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header )
{
    header->targetLogicalAddress        = CCSDS_DESTINATION_ID;
    header->protocolIdentifier          = CCSDS_PROTOCOLE_ID;
    header->reserved        = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
    header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
    header->packetSequenceControl[0]    = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1]    = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = 0x00;
    header->packetLength[1] = 0x00;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2    = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType     = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType  = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID   = TM_DESTINATION_ID_GROUND;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    // AUXILIARY DATA HEADER
    header->sid     = 0x00;
    header->biaStatusInfo = 0x00;
    header->pa_lfr_pkt_cnt_asm = 0x00;
    header->pa_lfr_pkt_nr_asm = 0x00;
    header->pa_lfr_asm_blk_nr[0] = 0x00;
    header->pa_lfr_asm_blk_nr[1] = 0x00;
}

int spw_send_waveform_CWF( ring_node *ring_node_to_send,
                      Header_TM_LFR_SCIENCE_CWF_t *header )
{
    /** This function sends CWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerCWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    int *dataPtr;
    unsigned char sid;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;
    sid = (unsigned char) ring_node_to_send->sid;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (int*) ring_node_to_send->buffer_address;

    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF; i++) // send waveform
    {
        spw_ioctl_send_CWF.data = (char*) &dataPtr[ (i * BLK_NR_CWF * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_CWF.hdr  = (char*) header;
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF * NB_BYTES_SWF_BLK;

        // SET PACKET SEQUENCE CONTROL
        increment_seq_counter_source_id( header->packetSequenceControl, sid );

        // SET SID
        header->sid = sid;

        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, sid, i, header->acquisitionTime);
        //
        header->time[0] = header->acquisitionTime[0];
        header->time[1] = header->acquisitionTime[1];
        header->time[2] = header->acquisitionTime[2];
        header->time[3] = header->acquisitionTime[3];
        header->time[4] = header->acquisitionTime[4];
        header->time[5] = header->acquisitionTime[5];

        // SET PACKET ID
        if ( (sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2) )
        {
            header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_SBM1_SBM2 >> 8);
            header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_SBM1_SBM2);
        }
        else
        {
            header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
            header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
        }

        status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_CWF );
        if (status != RTEMS_SUCCESSFUL) {
            printf("%d-%d, ERR %d\n", sid, i, (int) status);
            ret = LFR_DEFAULT;
        }
    }

    return ret;
}

int spw_send_waveform_SWF( ring_node *ring_node_to_send,
                       Header_TM_LFR_SCIENCE_SWF_t *header )
{
    /** This function sends SWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerSWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_SWF;
    int *dataPtr;
    unsigned char sid;

    spw_ioctl_send_SWF.hlen = TM_HEADER_LEN + 4 + 12; // + 4 is for the protocole extra header, + 12 is for the auxiliary header
    spw_ioctl_send_SWF.options = 0;

    ret = LFR_DEFAULT;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (int*) ring_node_to_send->buffer_address;
    sid = ring_node_to_send->sid;

    for (i=0; i<7; i++) // send waveform
    {
        spw_ioctl_send_SWF.data = (char*) &dataPtr[ (i * BLK_NR_304 * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_SWF.hdr = (char*) header;

        // SET PACKET SEQUENCE CONTROL
        increment_seq_counter_source_id( header->packetSequenceControl, sid );

        // SET PACKET LENGTH AND BLKNR
        if (i == 6)
        {
            spw_ioctl_send_SWF.dlen = BLK_NR_224 * NB_BYTES_SWF_BLK;
            header->packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_224 >> 8);
            header->packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_224     );
            header->blkNr[0] = (unsigned char) (BLK_NR_224 >> 8);
            header->blkNr[1] = (unsigned char) (BLK_NR_224     );
        }
        else
        {
            spw_ioctl_send_SWF.dlen = BLK_NR_304 * NB_BYTES_SWF_BLK;
            header->packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_304 >> 8);
            header->packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_304     );
            header->blkNr[0] = (unsigned char) (BLK_NR_304 >> 8);
            header->blkNr[1] = (unsigned char) (BLK_NR_304     );
        }

        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, sid, i, header->acquisitionTime );
        //
        header->time[0] = header->acquisitionTime[0];
        header->time[1] = header->acquisitionTime[1];
        header->time[2] = header->acquisitionTime[2];
        header->time[3] = header->acquisitionTime[3];
        header->time[4] = header->acquisitionTime[4];
        header->time[5] = header->acquisitionTime[5];

        // SET SID
        header->sid = sid;

        // SET PKTNR
        header->pktNr = i+1;    // PKT_NR

        // SEND PACKET
        status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_SWF );
        if (status != RTEMS_SUCCESSFUL) {
            printf("%d-%d, ERR %d\n", sid, i, (int) status);
            ret = LFR_DEFAULT;
        }
    }

    return ret;
}

int spw_send_waveform_CWF3_light( ring_node *ring_node_to_send,
                                  Header_TM_LFR_SCIENCE_CWF_t *header )
{
    /** This function sends CWF_F3 CCSDS packets without the b1, b2 and b3 data.
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param headerCWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * By default, CWF_F3 packet are send without the b1, b2 and b3 data. This function rebuilds a data buffer
     * from the incoming data and sends it in 7 packets, 6 containing 340 blocks and 1 one containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    char *dataPtr;
    unsigned char sid;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;
    sid = ring_node_to_send->sid;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (char*) ring_node_to_send->buffer_address;

    //*********************
    // SEND CWF3_light DATA
    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF_LIGHT; i++) // send waveform
    {
        spw_ioctl_send_CWF.data = (char*) &dataPtr[ (i * BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK) ];
        spw_ioctl_send_CWF.hdr = (char*) header;
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK;

        // SET PACKET SEQUENCE COUNTER
        increment_seq_counter_source_id( header->packetSequenceControl, sid );

        // SET SID
        header->sid = sid;

        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, SID_NORM_CWF_F3, i, header->acquisitionTime );
        //
        header->time[0] = header->acquisitionTime[0];
        header->time[1] = header->acquisitionTime[1];
        header->time[2] = header->acquisitionTime[2];
        header->time[3] = header->acquisitionTime[3];
        header->time[4] = header->acquisitionTime[4];
        header->time[5] = header->acquisitionTime[5];

        // SET PACKET ID
        header->packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
        header->packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);

        // SEND PACKET
        status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_CWF );
        if (status != RTEMS_SUCCESSFUL) {
            printf("%d-%d, ERR %d\n", sid, i, (int) status);
            ret = LFR_DEFAULT;
        }
    }

    return ret;
}

void spw_send_asm( ring_node *ring_node_to_send,
                   Header_TM_LFR_SCIENCE_ASM_t *header )
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;
    unsigned int sid;
    char *spectral_matrix;
    int coarseTime;
    int fineTime;
    spw_ioctl_pkt_send spw_ioctl_send_ASM;

    sid = ring_node_to_send->sid;
    spectral_matrix = (char*) ring_node_to_send->buffer_address;
    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;

    for (i=0; i<2; i++)
    {
        // (1) BUILD THE DATA
        switch(sid)
        {
        case SID_NORM_ASM_F0:
            spw_ioctl_send_ASM.dlen = TOTAL_SIZE_ASM_F0_IN_BYTES / 2;  // 2 packets will be sent
            spw_ioctl_send_ASM.data = &spectral_matrix[
                    ( (ASM_F0_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F0) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F0) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F0);        // BLK_NR LSB
            break;
        case SID_NORM_ASM_F1:
            spw_ioctl_send_ASM.dlen = TOTAL_SIZE_ASM_F1_IN_BYTES / 2;  // 2 packets will be sent
            spw_ioctl_send_ASM.data = &spectral_matrix[
                    ( (ASM_F1_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F1) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F1) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F1);        // BLK_NR LSB
            break;
        case SID_NORM_ASM_F2:
            spw_ioctl_send_ASM.dlen = TOTAL_SIZE_ASM_F2_IN_BYTES / 2;  // 2 packets will be sent
            spw_ioctl_send_ASM.data = &spectral_matrix[
                    ( (ASM_F2_INDICE_START + (i*NB_BINS_PER_PKT_ASM_F2) ) * NB_VALUES_PER_SM ) * 2
                    ];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F2;
            header->pa_lfr_asm_blk_nr[0] = (unsigned char)  ( (NB_BINS_PER_PKT_ASM_F2) >> 8 ); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)    (NB_BINS_PER_PKT_ASM_F2);        // BLK_NR LSB
            break;
        default:
            PRINTF1("ERR *** in spw_send_asm *** unexpected sid %d\n", sid)
            break;
        }
        spw_ioctl_send_ASM.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM + CCSDS_PROTOCOLE_EXTRA_BYTES;
        spw_ioctl_send_ASM.hdr = (char *) header;
        spw_ioctl_send_ASM.options = 0;

        // (2) BUILD THE HEADER
        increment_seq_counter_source_id( header->packetSequenceControl, sid );
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        header->sid = (unsigned char) sid;   // SID
        header->pa_lfr_pkt_cnt_asm = 2;
        header->pa_lfr_pkt_nr_asm = (unsigned char) (i+1);

        // (3) SET PACKET TIME
        header->time[0] = (unsigned char) (coarseTime>>24);
        header->time[1] = (unsigned char) (coarseTime>>16);
        header->time[2] = (unsigned char) (coarseTime>>8);
        header->time[3] = (unsigned char) (coarseTime);
        header->time[4] = (unsigned char) (fineTime>>8);
        header->time[5] = (unsigned char) (fineTime);
        //
        header->acquisitionTime[0] = header->time[0];
        header->acquisitionTime[1] = header->time[1];
        header->acquisitionTime[2] = header->time[2];
        header->acquisitionTime[3] = header->time[3];
        header->acquisitionTime[4] = header->time[4];
        header->acquisitionTime[5] = header->time[5];

        // (4) SEND PACKET
        status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_ASM );
        if (status != RTEMS_SUCCESSFUL) {
            printf("in ASM_send *** ERR %d\n", (int) status);
        }
    }
}

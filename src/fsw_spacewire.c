/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

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
#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_housekeeping.h"
#include "fsw_misc.h"
#include "fsw_watchdog.h"
#include "hw/lfr_regs.h"
#include <errno.h>

rtems_name semq_name = 0;
rtems_id semq_id = RTEMS_ID_NONE;

//*****************
// waveform headers
DISABLE_MISSING_FIELD_INITIALIZER_WARNING
Header_TM_LFR_SCIENCE_CWF_t headerCWF = { 0 };
Header_TM_LFR_SCIENCE_SWF_t headerSWF = { 0 };
Header_TM_LFR_SCIENCE_ASM_t headerASM = { 0 };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

unsigned char previousTimecodeCtr = 0;
unsigned int* grspwPtr = (unsigned int*)(REGS_ADDR_GRSPW + APB_OFFSET_GRSPW_TIME_REGISTER);

//***********
// RTEMS TASK
rtems_task spiq_task(rtems_task_argument unused)
{
    /** This RTEMS task is awaken by an rtems_event sent by the interruption subroutine of the
     * SpaceWire driver.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     */

    IGNORE_UNUSED_PARAMETER(unused);
    rtems_event_set event_out;
    rtems_status_code status;
    int linkStatus;

    event_out = EVENT_SETS_NONE_PENDING;
    linkStatus = 0;

    BOOT_PRINTF("in SPIQ *** \n");

    while (true)
    {
        // wait for an SPW_LINKERR_EVENT
        DEBUG_CHECK_STATUS(
            rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out));
        LFR_PRINTF("in SPIQ *** got SPW_LINKERR_EVENT\n");

        // [0] SUSPEND RECV AND SEND TASKS
        DEBUG_CHECK_STATUS(rtems_task_suspend(Task_id[TASKID_RECV]));
        DEBUG_CHECK_STATUS(rtems_task_suspend(Task_id[TASKID_SEND]));

        // [1] CHECK THE LINK
        // get the link status (1)
        DEBUG_CHECK_STATUS(ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus));
        if (linkStatus != SPW_LINK_OK)
        {
            LFR_PRINTF("in SPIQ *** linkStatus %d, wait...\n", linkStatus);
            // wait SY_LFR_DPU_CONNECT_TIMEOUT 1000 ms
            DEBUG_CHECK_STATUS(rtems_task_wake_after(SY_LFR_DPU_CONNECT_TIMEOUT));
        }

        // [2] RECHECK THE LINK AFTER SY_LFR_DPU_CONNECT_TIMEOUT
        // get the link status (2)
        DEBUG_CHECK_STATUS(ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus));
        if (linkStatus != SPW_LINK_OK) // [2.a] not in run state, reset the link
        {
            spacewire_read_statistics();
            status = spacewire_several_connect_attemps();
            DEBUG_CHECK_STATUS(status);
        }
        else // [2.b] in run state, start the link
        {
            // start the link
            status = spacewire_stop_and_start_link(fdSPW);
            DEBUG_CHECK_STATUS(status);
        }

        // [3] COMPLETE RECOVERY ACTION AFTER SY_LFR_DPU_CONNECT_ATTEMPTS
        // [3.a] the link is in run state and has been started successfully
        if (status == RTEMS_SUCCESSFUL)
        {
            DEBUG_CHECK_STATUS(rtems_task_restart(Task_id[TASKID_SEND], 1));
            DEBUG_CHECK_STATUS(rtems_task_restart(Task_id[TASKID_RECV], 1));
        }
        else // [3.b] the link is not in run state, go in STANDBY mode
        {
            status = enter_mode_standby();
            DEBUG_CHECK_STATUS(status);
            if (status == RTEMS_SUCCESSFUL)
            {
                updateLFRCurrentMode(LFR_MODE_STANDBY);
            }
            // wake the LINK task up to wait for the link recovery
            DEBUG_CHECK_STATUS(rtems_event_send(Task_id[TASKID_LINK], RTEMS_EVENT_0));
            DEBUG_CHECK_STATUS(rtems_task_suspend(RTEMS_SELF));
        }
    }
}

rtems_task recv_task(rtems_task_argument unused)
{
    /** This RTEMS task is dedicated to the reception of incoming TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The RECV task blocks on a call to the read system call, waiting for incoming SpaceWire data.
     * When unblocked:
     * 1. It reads the incoming data.
     * 2. Launches the acceptance procedure.
     * 3. If the Telecommand is valid, sends it to a dedicated RTEMS message queue.
     *
     */

    IGNORE_UNUSED_PARAMETER(unused);
    int len;
    ccsdsTelecommandPacket_t __attribute__((aligned(4))) currentTC;
    unsigned char computed_CRC[BYTES_PER_CRC];
    unsigned char currentTC_LEN_RCV[BYTES_PER_PKT_LEN];
    unsigned char destinationID;
    unsigned int estimatedPacketLength;
    unsigned int parserCode;
    rtems_status_code status;
    rtems_id queue_recv_id;
    rtems_id queue_send_id;

    memset(&currentTC, 0, sizeof(ccsdsTelecommandPacket_t));
    destinationID = 0;
    queue_recv_id = RTEMS_ID_NONE;
    queue_send_id = RTEMS_ID_NONE;

    initLookUpTableForCRC(); // the table is used to compute Cyclic Redundancy Codes

    DEBUG_CHECK_STATUS(get_message_queue_id_recv(&queue_recv_id));
    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_send_id));

    BOOT_PRINTF("in RECV *** \n");

    while (1)
    {
        len = read(fdSPW, (char*)&currentTC, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
        if (len == -1)
        { // error during the read call
            LFR_PRINTF("in RECV *** last read call returned -1, ERRNO %d\n", errno);
        }
        else
        {
            if ((len + 1) < CCSDS_TC_PKT_MIN_SIZE)
            {
                LFR_PRINTF("in RECV *** packet lenght too short\n");
            }
            else
            {
                estimatedPacketLength = (unsigned int)(len - CCSDS_TC_TM_PACKET_OFFSET
                    - PROTID_RES_APP); // => -3 is for Prot ID, Reserved and User App bytes
                LFR_PRINTF("incoming TC with Length (byte): %d\n", len - 3);
                currentTC_LEN_RCV[0] = (unsigned char)(estimatedPacketLength >> SHIFT_1_BYTE);
                currentTC_LEN_RCV[1] = (unsigned char)(estimatedPacketLength);
                // CHECK THE TC
                parserCode = tc_parser(&currentTC, estimatedPacketLength, computed_CRC);
                if ((parserCode == ILLEGAL_APID) || (parserCode == WRONG_LEN_PKT)
                    || (parserCode == INCOR_CHECKSUM) || (parserCode == ILL_TYPE)
                    || (parserCode == ILL_SUBTYPE) || (parserCode == WRONG_APP_DATA)
                    || (parserCode == WRONG_SRC_ID))
                { // send TM_LFR_TC_EXE_CORRUPTED
                    LFR_PRINTF("TC corrupted received, with code: %d\n", parserCode);
                    if (!((currentTC.serviceType == TC_TYPE_TIME)
                            && (currentTC.serviceSubType == TC_SUBTYPE_UPDT_TIME))
                        && !((currentTC.serviceType == TC_TYPE_GEN)
                            && (currentTC.serviceSubType == TC_SUBTYPE_UPDT_INFO)))
                    {
                        if (parserCode == WRONG_SRC_ID)
                        {
                            destinationID = SID_TC_GROUND;
                        }
                        else
                        {
                            destinationID = currentTC.sourceID;
                        }
                        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_corrupted(&currentTC, queue_send_id,
                            computed_CRC, currentTC_LEN_RCV, destinationID));
                    }
                }
                else
                {
                    // send valid TC to the action launcher
                    DEBUG_CHECK_STATUS(rtems_message_queue_send(queue_recv_id, &currentTC,
                        estimatedPacketLength + CCSDS_TC_TM_PACKET_OFFSET + PROTID_RES_APP));
                }
            }
        }

        update_queue_max_count(queue_recv_id, &hk_lfr_q_rv_fifo_size_max);
    }
}

rtems_task send_task(rtems_task_argument argument)
{
    /** This RTEMS task is dedicated to the transmission of TeleMetry packets.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The SEND task waits for a message to become available in the dedicated RTEMS queue. When a
     * message arrives:
     * - if the first byte is equal to CCSDS_DESTINATION_ID, the message is sent as is using the
     * write system call.
     * - if the first byte is not equal to CCSDS_DESTINATION_ID, the message is handled as a
     * spw_ioctl_pkt_send. After analyzis, the packet is sent either using the write system call or
     * using the ioctl call SPACEWIRE_IOCTRL_SEND, depending on the data it contains.
     *
     */
    IGNORE_UNUSED_PARAMETER(argument);
    rtems_status_code status; // RTEMS status code
    char incomingData[MSG_QUEUE_SIZE_SEND]; // incoming data buffer
    ring_node* incomingRingNodePtr = NULL;
    int ring_node_address = 0;
    char* charPtr=(char*)&ring_node_address;
    spw_ioctl_pkt_send* spw_ioctl_send;
    size_t size = 0; // size of the incoming TC packet
    rtems_id queue_send_id= RTEMS_ID_NONE;
    unsigned int sid = 0;
    unsigned char sidAsUnsignedChar = 0;
    unsigned char type;

    init_header_cwf(&headerCWF);
    init_header_swf(&headerSWF);
    init_header_asm(&headerASM);

    DEBUG_CHECK_STATUS(get_message_queue_id_send(&queue_send_id));

    BOOT_PRINTF("in SEND *** \n");

    while (1)
    {
        status = rtems_message_queue_receive(
            queue_send_id, incomingData, &size, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        DEBUG_CHECK_STATUS(status);

        if (status != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("in SEND *** (1) ERR = %d\n", status);
        }
        else
        {
            if (size == sizeof(ring_node*))
            {
                charPtr[0] = incomingData[0];
                charPtr[1] = incomingData[1];
                charPtr[BYTE_2] = incomingData[BYTE_2];
                charPtr[BYTE_3] = incomingData[BYTE_3];
                incomingRingNodePtr = (ring_node*)ring_node_address;
                sid = incomingRingNodePtr->sid;
                if ((sid == SID_NORM_CWF_LONG_F3) || (sid == SID_BURST_CWF_F2)
                    || (sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2))
                {
                    spw_send_waveform_CWF(incomingRingNodePtr, &headerCWF);
                }
                else if ((sid == SID_NORM_SWF_F0) || (sid == SID_NORM_SWF_F1)
                    || (sid == SID_NORM_SWF_F2))
                {
                    spw_send_waveform_SWF(incomingRingNodePtr, &headerSWF);
                }
                else if (sid == SID_NORM_CWF_F3)
                {
                    spw_send_waveform_CWF3_light(incomingRingNodePtr, &headerCWF);
                }
                else if (sid == SID_NORM_ASM_F0)
                {
                    spw_send_asm_f0(incomingRingNodePtr, &headerASM);
                }
                else if (sid == SID_NORM_ASM_F1)
                {
                    spw_send_asm_f1(incomingRingNodePtr, &headerASM);
                }
                else if (sid == SID_NORM_ASM_F2)
                {
                    spw_send_asm_f2(incomingRingNodePtr, &headerASM);
                }
                else if (sid == TM_CODE_K_DUMP)
                {
                    spw_send_k_dump(incomingRingNodePtr);
                }
                else
                {
                    LFR_PRINTF("unexpected sid = %d\n", sid);
                }
            }
            else if (incomingData[0]
                == CCSDS_DESTINATION_ID) // the incoming message is a ccsds packet
            {
                sid = (unsigned char)incomingData[PACKET_POS_PA_LFR_SID_PKT];
                type = (unsigned char)incomingData[PACKET_POS_SERVICE_TYPE];
                if (type == TM_TYPE_LFR_SCIENCE) // this is a BP packet, all other types are handled
                                                 // differently
                // SET THE SEQUENCE_CNT PARAMETER IN CASE OF BP0 OR BP1 PACKETS
                {
                    increment_seq_counter_source_id(
                        (unsigned char*)&incomingData[PACKET_POS_SEQUENCE_CNT], sid);
                }

                status = write(fdSPW, incomingData, size);
                if ((int)status == -1)
                {
                    LFR_PRINTF("in SEND *** (2.a) ERRNO = %d, size = %d\n", errno, size);
                }
            }
            else // the incoming message is a spw_ioctl_pkt_send structure
            {
                spw_ioctl_send = (spw_ioctl_pkt_send*)incomingData;
                status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, spw_ioctl_send);
                if ((int)status == -1)
                {
                    LFR_PRINTF("in SEND *** (2.b) ERRNO = %d, RTEMS = %d\n", errno, status);
                }
            }
        }

        update_queue_max_count(queue_send_id, &hk_lfr_q_sd_fifo_size_max);
    }
}

rtems_task link_task(rtems_task_argument argument)
{
    IGNORE_UNUSED_PARAMETER(argument);
    rtems_event_set event_out;
    rtems_status_code status;
    int linkStatus;

    event_out = EVENT_SETS_NONE_PENDING;
    linkStatus = 0;

    BOOT_PRINTF("in LINK ***\n");

    while (1)
    {
        // wait for an RTEMS_EVENT
        status = rtems_event_receive(
            RTEMS_EVENT_0, RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        DEBUG_CHECK_STATUS(status);
        LFR_PRINTF("in LINK *** wait for the link\n");
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus); // get the link status
        DEBUG_CHECK_STATUS(status);
        while (linkStatus != SPW_LINK_OK) // wait for the link
        {
            status = rtems_task_wake_after(SPW_LINK_WAIT); // monitor the link each 100ms
            DEBUG_CHECK_STATUS(status);
            status = ioctl(
                fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus); // get the link status
            watchdog_reload();
        }

        spacewire_read_statistics();
        status = spacewire_stop_and_start_link(fdSPW);
        DEBUG_CHECK_STATUS(status);

        // restart the SPIQ task
        status = rtems_task_restart(Task_id[TASKID_SPIQ], 1);
        DEBUG_CHECK_STATUS(status);

        // restart RECV and SEND
        status = rtems_task_restart(Task_id[TASKID_SEND], 1);
        DEBUG_CHECK_STATUS(status);

        status = rtems_task_restart(Task_id[TASKID_RECV], 1);
        DEBUG_CHECK_STATUS(status);
    }
}

//****************
// OTHER FUNCTIONS
int spacewire_open_link(
    void) // by default, the driver resets the core: [SPW_CTRL_WRITE(pDev, SPW_CTRL_RESET);]
{
    /** This function opens the SpaceWire link.
     *
     * @return a valid file descriptor in case of success, -1 in case of a failure
     *
     */
    rtems_status_code status;

    status = -1;

    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call resets the hardware
    if (fdSPW < 0)
    {
        LFR_PRINTF("ERR *** in configure_spw_link *** error opening " GRSPW_DEVICE_NAME
                   " with ERR %d\n",
            errno);
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }

    return status;
}

int spacewire_start_link(int fd)
{
    return ioctl(fd, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
                                                  // -1 default hardcoded driver timeout
}

int spacewire_stop_and_start_link(int fd)
{
    rtems_status_code status = RTEMS_SUCCESSFUL;

    status = ioctl(fd, SPACEWIRE_IOCTRL_STOP); // start fails if link pDev->running != 0
    status |= ioctl(fd, SPACEWIRE_IOCTRL_START, -1); // returns successfuly if the link is started
                                                     // -1 default hardcoded driver timeout

    return status;
}

int spacewire_configure_link(int fd)
{
    /** This function configures the SpaceWire link.
     *
     * @return GR-RTEMS-DRIVER directive status codes:
     * - 22  EINVAL - Null pointer or an out of range value was given as the argument.
     * - 16  EBUSY - Only used for SEND. Returned when no descriptors are avialble in non-blocking
     * mode.
     * - 88  ENOSYS - Returned for SET_DESTKEY if RMAP command handler is not available or if a
     * non-implemented call is used.
     * - 116 ETIMEDOUT - REturned for SET_PACKET_SIZE and START if the link could not be brought up.
     * - 12  ENOMEM - Returned for SET_PACKETSIZE if it was unable to allocate the new buffers.
     * - 5   EIO - Error when writing to grswp hardware registers.
     * - 2   ENOENT - No such file or directory
     */

    rtems_status_code status;

    spacewire_set_NP(1, REGS_ADDR_GRSPW); // [N]o [P]ort force
    spacewire_set_RE(1, REGS_ADDR_GRSPW); // [R]MAP [E]nable, the dedicated call seems to  break the
                                          // no port force configuration
    spw_ioctl_packetsize packetsize;

    packetsize.rxsize = SPW_RXSIZE;
    packetsize.txdsize = SPW_TXDSIZE;
    packetsize.txhsize = SPW_TXHSIZE;

    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1); // sets the blocking mode for reception
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_EVENT_ID,
        Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_DISABLE_ERR,
        0); // automatic link-disabling due to link-error interrupts
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1); // sets the link-error interrupt bit
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK, 1); // transmission blocks
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL,
        1); // transmission blocks when no transmission descriptor is available
    DEBUG_CHECK_STATUS(status);
    //
    status = ioctl(fd, SPACEWIRE_IOCTRL_SET_TCODE_CTRL,
        CONF_TCODE_CTRL); // [Time Rx : Time Tx : Link error : Tick-out IRQ]
    DEBUG_CHECK_STATUS(status);
    //
    status
        = ioctl(fd, SPACEWIRE_IOCTRL_SET_PACKETSIZE, packetsize); // set rxsize, txdsize and txhsize
    DEBUG_CHECK_STATUS(status);

    return status;
}

int spacewire_several_connect_attemps(void)
{
    /** This function is executed by the SPIQ rtems_task wehn it has been awaken by an interruption
     * raised by the SpaceWire driver.
     *
     * @return RTEMS directive status code:
     * - RTEMS_UNSATISFIED is returned is the link is not in the running state after 10 s.
     * - RTEMS_SUCCESSFUL is returned if the link is up before the timeout.
     *
     */

    rtems_status_code status_spw;
    rtems_status_code status;
    int i;

    status_spw = RTEMS_SUCCESSFUL;

    i = 0;
    while (i < SY_LFR_DPU_CONNECT_ATTEMPT)
    {
        LFR_PRINTF("in spacewire_reset_link *** link recovery, try %d\n", i);

        // CLOSING THE DRIVER AT THIS POINT WILL MAKE THE SEND TASK BLOCK THE SYSTEM

        status = rtems_task_wake_after(
            SY_LFR_DPU_CONNECT_TIMEOUT); // wait SY_LFR_DPU_CONNECT_TIMEOUT 1000 ms
        DEBUG_CHECK_STATUS(status);

        status_spw = spacewire_stop_and_start_link(fdSPW);

        if (status_spw != RTEMS_SUCCESSFUL)
        {
            i = i + 1;
            LFR_PRINTF(
                "in spacewire_reset_link *** ERR spacewire_start_link code %d\n", status_spw);
        }
        else
        {
            i = SY_LFR_DPU_CONNECT_ATTEMPT;
        }
    }

    return status_spw;
}

void spacewire_set_NP(unsigned char val, unsigned int regAddr) // [N]o [P]ort force
{
    /** This function sets the [N]o [P]ort force bit of the GRSPW control register.
     *
     * @param val is the value, 0 or 1, used to set the value of the NP bit.
     * @param regAddr is the address of the GRSPW control register.
     *
     * NP is the bit 20 of the GRSPW control register.
     *
     */

    unsigned int* spwptr = (unsigned int*)regAddr;

    if (val == 1)
    {
        *spwptr = *spwptr | SPW_BIT_NP; // [NP] set the No port force bit
    }
    if (val == 0)
    {
        *spwptr = *spwptr & SPW_BIT_NP_MASK;
    }
}

void spacewire_set_RE(unsigned char val, unsigned int regAddr) // [R]MAP [E]nable
{
    /** This function sets the [R]MAP [E]nable bit of the GRSPW control register.
     *
     * @param val is the value, 0 or 1, used to set the value of the RE bit.
     * @param regAddr is the address of the GRSPW control register.
     *
     * RE is the bit 16 of the GRSPW control register.
     *
     */

    unsigned int* spwptr = (unsigned int*)regAddr;

    if (val == 1)
    {
        *spwptr = *spwptr | SPW_BIT_RE; // [RE] set the RMAP Enable bit
    }
    if (val == 0)
    {
        *spwptr = *spwptr & SPW_BIT_RE_MASK;
    }
}

void spacewire_read_statistics(void)
{
    /** This function reads the SpaceWire statistics from the grspw RTEMS driver.
     *
     * @param void
     *
     * @return void
     *
     * Once they are read, the counters are stored in a global variable used during the building of
     * the HK packets.
     *
     */

    rtems_status_code status;
    spw_stats current;

    memset(&current, 0, sizeof(spw_stats));

    spacewire_get_last_error();

    // read the current statistics
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &current);
    DEBUG_CHECK_STATUS(status);

    // clear the counters
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_CLR_STATISTICS);
    DEBUG_CHECK_STATUS(status);

    // rx_eep_err
    grspw_stats.rx_eep_err = grspw_stats.rx_eep_err + current.rx_eep_err;
    // rx_truncated
    grspw_stats.rx_truncated = grspw_stats.rx_truncated + current.rx_truncated;
    // parity_err
    grspw_stats.parity_err = grspw_stats.parity_err + current.parity_err;
    // escape_err
    grspw_stats.escape_err = grspw_stats.escape_err + current.escape_err;
    // credit_err
    grspw_stats.credit_err = grspw_stats.credit_err + current.credit_err;
    // write_sync_err
    grspw_stats.write_sync_err = grspw_stats.write_sync_err + current.write_sync_err;
    // disconnect_err
    grspw_stats.disconnect_err = grspw_stats.disconnect_err + current.disconnect_err;
    // early_ep
    grspw_stats.early_ep = grspw_stats.early_ep + current.early_ep;
    // invalid_address
    grspw_stats.invalid_address = grspw_stats.invalid_address + current.invalid_address;
    // packets_sent
    grspw_stats.packets_sent = grspw_stats.packets_sent + current.packets_sent;
    // packets_received
    grspw_stats.packets_received = grspw_stats.packets_received + current.packets_received;
}

void spacewire_get_last_error(void)
{
    static spw_stats previous = { 0 };
    spw_stats current;
    rtems_status_code status;

    unsigned int hk_lfr_last_er_rid;
    unsigned char hk_lfr_last_er_code;
    int coarseTime;
    int fineTime;
    unsigned char update_hk_lfr_last_er;

    memset(&current, 0, sizeof(spw_stats));
    hk_lfr_last_er_rid = INIT_CHAR;
    hk_lfr_last_er_code = INIT_CHAR;
    update_hk_lfr_last_er = INIT_CHAR;

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &current);
    DEBUG_CHECK_STATUS(status);

    // get current time
    coarseTime = time_management_regs->coarse_time;
    fineTime = time_management_regs->fine_time;

    // tx_link_err *** no code associated to this field
    // rx_rmap_header_crc_err ***  LE *** in HK
    if (previous.rx_rmap_header_crc_err != current.rx_rmap_header_crc_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_HEADER_CRC;
        update_hk_lfr_last_er = 1;
    }
    // rx_rmap_data_crc_err ***  LE *** NOT IN HK
    if (previous.rx_rmap_data_crc_err != current.rx_rmap_data_crc_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_DATA_CRC;
        update_hk_lfr_last_er = 1;
    }
    // rx_eep_err
    if (previous.rx_eep_err != current.rx_eep_err)
    {
        hk_lfr_last_er_rid = RID_ME_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_EEP;
        update_hk_lfr_last_er = 1;
    }
    // rx_truncated
    if (previous.rx_truncated != current.rx_truncated)
    {
        hk_lfr_last_er_rid = RID_ME_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_RX_TOO_BIG;
        update_hk_lfr_last_er = 1;
    }
    // parity_err
    if (previous.parity_err != current.parity_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_PARITY;
        update_hk_lfr_last_er = 1;
    }
    // escape_err
    if (previous.parity_err != current.parity_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_ESCAPE;
        update_hk_lfr_last_er = 1;
    }
    // credit_err
    if (previous.credit_err != current.credit_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_CREDIT;
        update_hk_lfr_last_er = 1;
    }
    // write_sync_err
    if (previous.write_sync_err != current.write_sync_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_WRITE_SYNC;
        update_hk_lfr_last_er = 1;
    }
    // disconnect_err
    if (previous.disconnect_err != current.disconnect_err)
    {
        hk_lfr_last_er_rid = RID_LE_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_DISCONNECT;
        update_hk_lfr_last_er = 1;
    }
    // early_ep
    if (previous.early_ep != current.early_ep)
    {
        hk_lfr_last_er_rid = RID_ME_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_EARLY_EOP_EEP;
        update_hk_lfr_last_er = 1;
    }
    // invalid_address
    if (previous.invalid_address != current.invalid_address)
    {
        hk_lfr_last_er_rid = RID_ME_LFR_DPU_SPW;
        hk_lfr_last_er_code = CODE_INVALID_ADDRESS;
        update_hk_lfr_last_er = 1;
    }

    // if a field has changed, update the hk_last_er fields
    if (update_hk_lfr_last_er == 1)
    {
        update_hk_lfr_last_er_fields(hk_lfr_last_er_rid, hk_lfr_last_er_code);
    }

    previous = current;
}

void update_hk_with_grspw_stats(void)
{
    //****************************
    // DPU_SPACEWIRE_IF_STATISTICS
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[0]
        = (unsigned char)(grspw_stats.packets_received >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[1]
        = (unsigned char)(grspw_stats.packets_received);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[0]
        = (unsigned char)(grspw_stats.packets_sent >> SHIFT_1_BYTE);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[1] = (unsigned char)(grspw_stats.packets_sent);

    //******************************************
    // ERROR COUNTERS / SPACEWIRE / LOW SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_parity = (unsigned char)grspw_stats.parity_err;
    housekeeping_packet.hk_lfr_dpu_spw_disconnect = (unsigned char)grspw_stats.disconnect_err;
    housekeeping_packet.hk_lfr_dpu_spw_escape = (unsigned char)grspw_stats.escape_err;
    housekeeping_packet.hk_lfr_dpu_spw_credit = (unsigned char)grspw_stats.credit_err;
    housekeeping_packet.hk_lfr_dpu_spw_write_sync = (unsigned char)grspw_stats.write_sync_err;

    //*********************************************
    // ERROR COUNTERS / SPACEWIRE / MEDIUM SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_early_eop = (unsigned char)grspw_stats.early_ep;
    housekeeping_packet.hk_lfr_dpu_spw_invalid_addr = (unsigned char)grspw_stats.invalid_address;
    housekeeping_packet.hk_lfr_dpu_spw_eep = (unsigned char)grspw_stats.rx_eep_err;
    housekeeping_packet.hk_lfr_dpu_spw_rx_too_big = (unsigned char)grspw_stats.rx_truncated;
}

void spacewire_update_hk_lfr_link_state(unsigned char* hk_lfr_status_word_0)
{
    unsigned int* statusRegisterPtr;
    unsigned char linkState;

    statusRegisterPtr = (unsigned int*)(REGS_ADDR_GRSPW + APB_OFFSET_GRSPW_STATUS_REGISTER);
    linkState = (unsigned char)(((*statusRegisterPtr) >> SPW_LINK_STAT_POS)
        & STATUS_WORD_LINK_STATE_BITS); // [0000 0111]

    *hk_lfr_status_word_0
        = *hk_lfr_status_word_0 & STATUS_WORD_LINK_STATE_MASK; // [1111 1000] set link state to 0

    *hk_lfr_status_word_0 = *hk_lfr_status_word_0 | linkState; // update hk_lfr_dpu_spw_link_state
}

unsigned int check_timecode_and_previous_timecode_coherency(unsigned char currentTimecodeCtr)
{
    /** This function checks the coherency between the incoming timecode and the last valid
     * timecode.
     *
     * @param currentTimecodeCtr is the incoming timecode
     *
     * @return returned codes::
     * - LFR_DEFAULT
     * - LFR_SUCCESSFUL
     *
     */

    static unsigned char firstTickout = 1;
    unsigned char ret;

    ret = LFR_DEFAULT;

    if (firstTickout == 0)
    {
        if (currentTimecodeCtr == 0)
        {
            if (previousTimecodeCtr == SPW_TIMECODE_MAX)
            {
                ret = LFR_SUCCESSFUL;
            }
            else
            {
                ret = LFR_DEFAULT;
            }
        }
        else
        {
            if (currentTimecodeCtr == (previousTimecodeCtr + 1))
            {
                ret = LFR_SUCCESSFUL;
            }
            else
            {
                ret = LFR_DEFAULT;
            }
        }
    }
    else
    {
        firstTickout = 0;
        ret = LFR_SUCCESSFUL;
    }

    return ret;
}

unsigned int check_timecode_and_internal_time_coherency(
    unsigned char timecode, unsigned char internalTime)
{
    unsigned int ret;

    ret = LFR_DEFAULT;

    if (timecode == internalTime)
    {
        ret = LFR_SUCCESSFUL;
    }
    else
    {
        ret = LFR_DEFAULT;
    }

    return ret;
}

void timecode_irq_handler(void* pDev, void* regs, int minor, unsigned int tc)
{
    // a tickout has been emitted, perform actions on the incoming timecode

    IGNORE_UNUSED_PARAMETER(pDev);
    IGNORE_UNUSED_PARAMETER(regs);
    IGNORE_UNUSED_PARAMETER(minor);
    IGNORE_UNUSED_PARAMETER(tc);

    unsigned char incomingTimecode;
    unsigned char updateTime;
    unsigned char internalTime;
    rtems_status_code status;

    incomingTimecode = (unsigned char)(grspwPtr[0] & TIMECODE_MASK);
    updateTime = time_management_regs->coarse_time_load & TIMECODE_MASK;
    internalTime = time_management_regs->coarse_time & TIMECODE_MASK;

    housekeeping_packet.hk_lfr_dpu_spw_last_timc = incomingTimecode;

    // update the number of tickout that have been generated
    housekeeping_packet.hk_lfr_dpu_spw_tick_out_cnt
        = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_dpu_spw_tick_out_cnt);

    //**************************
    // HK_LFR_TIMECODE_ERRONEOUS
    // MISSING and INVALID are handled by the timecode_timer_routine service routine
    if (check_timecode_and_previous_timecode_coherency(incomingTimecode) == LFR_DEFAULT)
    {
        // this is unexpected but a tickout could have been raised despite of the timecode being
        // erroneous
        housekeeping_packet.hk_lfr_timecode_erroneous
            = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_timecode_erroneous);
        update_hk_lfr_last_er_fields(RID_LE_LFR_TIMEC, CODE_ERRONEOUS);
    }

    //************************
    // HK_LFR_TIME_TIMECODE_IT
    // check the coherency between the SpaceWire timecode and the Internal Time
    if (check_timecode_and_internal_time_coherency(incomingTimecode, internalTime) == LFR_DEFAULT)
    {
        housekeeping_packet.hk_lfr_time_timecode_it
            = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_time_timecode_it);
        update_hk_lfr_last_er_fields(RID_LE_LFR_TIME, CODE_TIMECODE_IT);
    }

    //********************
    // HK_LFR_TIMECODE_CTR
    // check the value of the timecode with respect to the last TC_LFR_UPDATE_TIME => SSS-CP-FS-370
    if (oneTcLfrUpdateTimeReceived == 1)
    {
        if (incomingTimecode != updateTime)
        {
            housekeeping_packet.hk_lfr_time_timecode_ctr
                = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_time_timecode_ctr);
            update_hk_lfr_last_er_fields(RID_LE_LFR_TIME, CODE_TIMECODE_CTR);
        }
    }

    // launch the timecode timer to detect missing or invalid timecodes
    previousTimecodeCtr = incomingTimecode; // update the previousTimecodeCtr value
    status = rtems_timer_fire_after(
        timecode_timer_id, TIMECODE_TIMER_TIMEOUT, timecode_timer_routine, NULL);
    if (status != RTEMS_SUCCESSFUL)
    {
        send_event_dumb_task(RTEMS_EVENT_14);
    }
}

rtems_timer_service_routine timecode_timer_routine(rtems_id timer_id, void* user_data)
{
    IGNORE_UNUSED_PARAMETER(timer_id);
    IGNORE_UNUSED_PARAMETER(user_data);

    rtems_status_code status;

    static unsigned char initStep = 1;

    unsigned char currentTimecodeCtr;

    currentTimecodeCtr = (unsigned char)(grspwPtr[0] & TIMECODE_MASK);

    if (initStep == 1)
    {
        if (currentTimecodeCtr == previousTimecodeCtr)
        {
            //************************
            // HK_LFR_TIMECODE_MISSING
            // the timecode value has not changed, no valid timecode has been received, the timecode
            // is MISSING
            housekeeping_packet.hk_lfr_timecode_missing
                = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_timecode_missing);
            update_hk_lfr_last_er_fields(RID_LE_LFR_TIMEC, CODE_MISSING);
        }
        else if (currentTimecodeCtr == (previousTimecodeCtr + 1))
        {
            // the timecode value has changed and the value is valid, this is unexpected because
            // the timer should not have fired, the timecode_irq_handler should have been raised
        }
        else
        {
            //************************
            // HK_LFR_TIMECODE_INVALID
            // the timecode value has changed and the value is not valid, no tickout has been
            // generated this is why the timer has fired
            housekeeping_packet.hk_lfr_timecode_invalid
                = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_timecode_invalid);
            update_hk_lfr_last_er_fields(RID_LE_LFR_TIMEC, CODE_INVALID);
        }
    }
    else
    {
        initStep = 1;
        //************************
        // HK_LFR_TIMECODE_MISSING
        housekeeping_packet.hk_lfr_timecode_missing
            = increase_unsigned_char_counter(housekeeping_packet.hk_lfr_timecode_missing);
        update_hk_lfr_last_er_fields(RID_LE_LFR_TIMEC, CODE_MISSING);
    }

    status = send_event_dumb_task(RTEMS_EVENT_13);
    DEBUG_CHECK_STATUS(status);
}

void init_header_cwf(Header_TM_LFR_SCIENCE_CWF_t* header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = INIT_CHAR;
    header->packetLength[1] = INIT_CHAR;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    header->time[BYTE_0] = INIT_CHAR;
    header->time[BYTE_1] = INIT_CHAR;
    header->time[BYTE_2] = INIT_CHAR;
    header->time[BYTE_3] = INIT_CHAR;
    header->time[BYTE_4] = INIT_CHAR;
    header->time[BYTE_5] = INIT_CHAR;
    // AUXILIARY DATA HEADER
    header->sid = INIT_CHAR;
    header->pa_bia_status_info = DEFAULT_HKBIA;
    header->blkNr[0] = INIT_CHAR;
    header->blkNr[1] = INIT_CHAR;
}

void init_header_swf(Header_TM_LFR_SCIENCE_SWF_t* header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST >> SHIFT_1_BYTE);
    header->packetID[1] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST);
    header->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = (unsigned char)(TM_LEN_SCI_CWF_336 >> SHIFT_1_BYTE);
    header->packetLength[1] = (unsigned char)(TM_LEN_SCI_CWF_336);
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    header->time[BYTE_0] = INIT_CHAR;
    header->time[BYTE_1] = INIT_CHAR;
    header->time[BYTE_2] = INIT_CHAR;
    header->time[BYTE_3] = INIT_CHAR;
    header->time[BYTE_4] = INIT_CHAR;
    header->time[BYTE_5] = INIT_CHAR;
    // AUXILIARY DATA HEADER
    header->sid = INIT_CHAR;
    header->pa_bia_status_info = DEFAULT_HKBIA;
    header->pktCnt = PKTCNT_SWF; // PKT_CNT
    header->pktNr = INIT_CHAR;
    header->blkNr[0] = (unsigned char)(BLK_NR_CWF >> SHIFT_1_BYTE);
    header->blkNr[1] = (unsigned char)(BLK_NR_CWF);
}

void init_header_asm(Header_TM_LFR_SCIENCE_ASM_t* header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = DEFAULT_RESERVED;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST >> SHIFT_1_BYTE);
    header->packetID[1] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST);
    header->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    header->packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    header->packetLength[0] = INIT_CHAR;
    header->packetLength[1] = INIT_CHAR;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_3; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    header->time[BYTE_0] = INIT_CHAR;
    header->time[BYTE_1] = INIT_CHAR;
    header->time[BYTE_2] = INIT_CHAR;
    header->time[BYTE_3] = INIT_CHAR;
    header->time[BYTE_4] = INIT_CHAR;
    header->time[BYTE_5] = INIT_CHAR;
    // AUXILIARY DATA HEADER
    header->sid = INIT_CHAR;
    header->pa_bia_status_info = INIT_CHAR;
    header->pa_lfr_pkt_cnt_asm = INIT_CHAR;
    header->pa_lfr_pkt_nr_asm = INIT_CHAR;
    header->pa_lfr_asm_blk_nr[0] = INIT_CHAR;
    header->pa_lfr_asm_blk_nr[1] = INIT_CHAR;
}

int spw_send_waveform_CWF(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_CWF_t* header)
{
    /** This function sends CWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerCWF points to a table of headers that have been prepared for the data
     * transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be
     * send. The structures contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks
     * and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    int* dataPtr;
    unsigned char sid;

    spw_ioctl_send_CWF.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_CWF;
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;
    sid = (unsigned char)ring_node_to_send->sid;

    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;
    dataPtr = (int*)ring_node_to_send->buffer_address;

    header->packetLength[0] = (unsigned char)(TM_LEN_SCI_CWF_336 >> SHIFT_1_BYTE);
    header->packetLength[1] = (unsigned char)(TM_LEN_SCI_CWF_336);
    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
    header->blkNr[0] = (unsigned char)(BLK_NR_CWF >> SHIFT_1_BYTE);
    header->blkNr[1] = (unsigned char)(BLK_NR_CWF);

    for (i = 0; i < NB_PACKETS_PER_GROUP_OF_CWF; i++) // send waveform
    {
        spw_ioctl_send_CWF.data = (char*)&dataPtr[(i * BLK_NR_CWF * NB_WORDS_SWF_BLK)];
        spw_ioctl_send_CWF.hdr = (char*)header;
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF * NB_BYTES_SWF_BLK;

        // SET PACKET SEQUENCE CONTROL
        increment_seq_counter_source_id(header->packetSequenceControl, sid);

        // SET SID
        header->sid = sid;

        // SET PACKET TIME
        compute_acquisition_time(coarseTime, fineTime, sid, i, header->acquisitionTime);
        //
        header->time[0] = header->acquisitionTime[0];
        header->time[1] = header->acquisitionTime[1];
        header->time[BYTE_2] = header->acquisitionTime[BYTE_2];
        header->time[BYTE_3] = header->acquisitionTime[BYTE_3];
        header->time[BYTE_4] = header->acquisitionTime[BYTE_4];
        header->time[BYTE_5] = header->acquisitionTime[BYTE_5];

        // SET PACKET ID
        if ((sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2))
        {
            header->packetID[0] = (unsigned char)(APID_TM_SCIENCE_SBM1_SBM2 >> SHIFT_1_BYTE);
            header->packetID[1] = (unsigned char)(APID_TM_SCIENCE_SBM1_SBM2);
        }
        else
        {
            header->packetID[0] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST >> SHIFT_1_BYTE);
            header->packetID[1] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST);
        }

        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_CWF);
        DEBUG_CHECK_STATUS(status);
    }

    return ret;
}

int spw_send_waveform_SWF(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_SWF_t* header)
{
    /** This function sends SWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerSWF points to a table of headers that have been prepared for the data
     * transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be
     * send. The structures contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks
     * and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_SWF;
    int* dataPtr;
    unsigned char sid;

    spw_ioctl_send_SWF.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_SWF;
    spw_ioctl_send_SWF.options = 0;

    ret = LFR_DEFAULT;

    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;
    dataPtr = (int*)ring_node_to_send->buffer_address;
    sid = ring_node_to_send->sid;

    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;

    for (i = 0; i < PKTCNT_SWF; i++) // send waveform
    {
        spw_ioctl_send_SWF.data = (char*)&dataPtr[(i * BLK_NR_304 * NB_WORDS_SWF_BLK)];
        spw_ioctl_send_SWF.hdr = (char*)header;

        // SET PACKET SEQUENCE CONTROL
        increment_seq_counter_source_id(header->packetSequenceControl, sid);

        // SET PACKET LENGTH AND BLKNR
        if (i == (PKTCNT_SWF - 1))
        {
            spw_ioctl_send_SWF.dlen = BLK_NR_224 * NB_BYTES_SWF_BLK;
            header->packetLength[0] = (unsigned char)(TM_LEN_SCI_SWF_224 >> SHIFT_1_BYTE);
            header->packetLength[1] = (unsigned char)(TM_LEN_SCI_SWF_224);
            header->blkNr[0] = (unsigned char)(BLK_NR_224 >> SHIFT_1_BYTE);
            header->blkNr[1] = (unsigned char)(BLK_NR_224);
        }
        else
        {
            spw_ioctl_send_SWF.dlen = BLK_NR_304 * NB_BYTES_SWF_BLK;
            header->packetLength[0] = (unsigned char)(TM_LEN_SCI_SWF_304 >> SHIFT_1_BYTE);
            header->packetLength[1] = (unsigned char)(TM_LEN_SCI_SWF_304);
            header->blkNr[0] = (unsigned char)(BLK_NR_304 >> SHIFT_1_BYTE);
            header->blkNr[1] = (unsigned char)(BLK_NR_304);
        }

        // SET PACKET TIME
        compute_acquisition_time(coarseTime, fineTime, sid, i, header->acquisitionTime);
        //
        header->time[BYTE_0] = header->acquisitionTime[BYTE_0];
        header->time[BYTE_1] = header->acquisitionTime[BYTE_1];
        header->time[BYTE_2] = header->acquisitionTime[BYTE_2];
        header->time[BYTE_3] = header->acquisitionTime[BYTE_3];
        header->time[BYTE_4] = header->acquisitionTime[BYTE_4];
        header->time[BYTE_5] = header->acquisitionTime[BYTE_5];

        // SET SID
        header->sid = sid;

        // SET PKTNR
        header->pktNr = i + 1; // PKT_NR

        // SEND PACKET
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_SWF);
        DEBUG_CHECK_STATUS(status);
    }

    return ret;
}

int spw_send_waveform_CWF3_light(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_CWF_t* header)
{
    /** This function sends CWF_F3 CCSDS packets without the b1, b2 and b3 data.
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param headerCWF points to a table of headers that have been prepared for the data
     * transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be
     * send. The structures contain information to setup the transmission of the data packets.
     *
     * By default, CWF_F3 packet are send without the b1, b2 and b3 data. This function rebuilds a
     * data buffer from the incoming data and sends it in 7 packets, 6 containing 340 blocks and 1
     * one containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    char* dataPtr;
    unsigned char sid;

    spw_ioctl_send_CWF.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_CWF;
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;
    sid = ring_node_to_send->sid;

    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;
    dataPtr = (char*)ring_node_to_send->buffer_address;

    header->packetLength[0] = (unsigned char)(TM_LEN_SCI_CWF_672 >> SHIFT_1_BYTE);
    header->packetLength[1] = (unsigned char)(TM_LEN_SCI_CWF_672);
    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;
    header->blkNr[0] = (unsigned char)(BLK_NR_CWF_SHORT_F3 >> SHIFT_1_BYTE);
    header->blkNr[1] = (unsigned char)(BLK_NR_CWF_SHORT_F3);

    //*********************
    // SEND CWF3_light DATA
    for (i = 0; i < NB_PACKETS_PER_GROUP_OF_CWF_LIGHT; i++) // send waveform
    {
        spw_ioctl_send_CWF.data
            = (char*)&dataPtr[(i * BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK)];
        spw_ioctl_send_CWF.hdr = (char*)header;
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK;

        // SET PACKET SEQUENCE COUNTER
        increment_seq_counter_source_id(header->packetSequenceControl, sid);

        // SET SID
        header->sid = sid;

        // SET PACKET TIME
        compute_acquisition_time(coarseTime, fineTime, SID_NORM_CWF_F3, i, header->acquisitionTime);
        //
        header->time[BYTE_0] = header->acquisitionTime[BYTE_0];
        header->time[BYTE_1] = header->acquisitionTime[BYTE_1];
        header->time[BYTE_2] = header->acquisitionTime[BYTE_2];
        header->time[BYTE_3] = header->acquisitionTime[BYTE_3];
        header->time[BYTE_4] = header->acquisitionTime[BYTE_4];
        header->time[BYTE_5] = header->acquisitionTime[BYTE_5];

        // SET PACKET ID
        header->packetID[0] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST >> SHIFT_1_BYTE);
        header->packetID[1] = (unsigned char)(APID_TM_SCIENCE_NORMAL_BURST);

        // SEND PACKET
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_CWF);
        DEBUG_CHECK_STATUS(status);
    }

    return ret;
}

void spw_send_asm_f0(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_ASM_t* header)
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;
    unsigned int sid;
    float* spectral_matrix;
    int coarseTime;
    int fineTime;
    spw_ioctl_pkt_send spw_ioctl_send_ASM;

    sid = ring_node_to_send->sid;
    spectral_matrix = (float*)ring_node_to_send->buffer_address;
    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;

    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;

    for (i = 0; i < PKTCNT_ASM; i++)
    {
        if ((i == 0) || (i == 1))
        {
            spw_ioctl_send_ASM.dlen = DLEN_ASM_F0_PKT_1;
            spw_ioctl_send_ASM.data = (char*)&spectral_matrix[(
                (ASM_F0_INDICE_START + (i * NB_BINS_PER_PKT_ASM_F0_1)) * NB_FLOATS_PER_SM)];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_1;
            header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6;
            header->pa_lfr_asm_blk_nr[0]
                = (unsigned char)((NB_BINS_PER_PKT_ASM_F0_1) >> SHIFT_1_BYTE); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)(NB_BINS_PER_PKT_ASM_F0_1); // BLK_NR LSB
        }
        else
        {
            spw_ioctl_send_ASM.dlen = DLEN_ASM_F0_PKT_2;
            spw_ioctl_send_ASM.data = (char*)&spectral_matrix[(
                (ASM_F0_INDICE_START + (i * NB_BINS_PER_PKT_ASM_F0_1)) * NB_FLOATS_PER_SM)];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_2;
            header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6;
            header->pa_lfr_asm_blk_nr[0]
                = (unsigned char)((NB_BINS_PER_PKT_ASM_F0_2) >> SHIFT_1_BYTE); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)(NB_BINS_PER_PKT_ASM_F0_2); // BLK_NR LSB
        }

        spw_ioctl_send_ASM.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM;
        spw_ioctl_send_ASM.hdr = (char*)header;
        spw_ioctl_send_ASM.options = 0;

        // (2) BUILD THE HEADER
        increment_seq_counter_source_id(header->packetSequenceControl, sid);
        header->packetLength[0] = (unsigned char)(length >> SHIFT_1_BYTE);
        header->packetLength[1] = (unsigned char)(length);
        header->sid = (unsigned char)sid; // SID
        header->pa_lfr_pkt_cnt_asm = PKTCNT_ASM;
        header->pa_lfr_pkt_nr_asm = (unsigned char)(i + 1);

        // (3) SET PACKET TIME
        header->time[BYTE_0] = (unsigned char)(coarseTime >> SHIFT_3_BYTES);
        header->time[BYTE_1] = (unsigned char)(coarseTime >> SHIFT_2_BYTES);
        header->time[BYTE_2] = (unsigned char)(coarseTime >> SHIFT_1_BYTE);
        header->time[BYTE_3] = (unsigned char)(coarseTime);
        header->time[BYTE_4] = (unsigned char)(fineTime >> SHIFT_1_BYTE);
        header->time[BYTE_5] = (unsigned char)(fineTime);
        //
        header->acquisitionTime[BYTE_0] = header->time[BYTE_0];
        header->acquisitionTime[BYTE_1] = header->time[BYTE_1];
        header->acquisitionTime[BYTE_2] = header->time[BYTE_2];
        header->acquisitionTime[BYTE_3] = header->time[BYTE_3];
        header->acquisitionTime[BYTE_4] = header->time[BYTE_4];
        header->acquisitionTime[BYTE_5] = header->time[BYTE_5];

        // (4) SEND PACKET
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_ASM);
        DEBUG_CHECK_STATUS(status);
    }
}

void spw_send_asm_f1(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_ASM_t* header)
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;
    unsigned int sid;
    float* spectral_matrix;
    int coarseTime;
    int fineTime;
    spw_ioctl_pkt_send spw_ioctl_send_ASM;

    sid = ring_node_to_send->sid;
    spectral_matrix = (float*)ring_node_to_send->buffer_address;
    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;

    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;

    for (i = 0; i < PKTCNT_ASM; i++)
    {
        if ((i == 0) || (i == 1))
        {
            spw_ioctl_send_ASM.dlen = DLEN_ASM_F1_PKT_1;
            spw_ioctl_send_ASM.data = (char*)&spectral_matrix[(
                (ASM_F1_INDICE_START + (i * NB_BINS_PER_PKT_ASM_F1_1)) * NB_FLOATS_PER_SM)];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_1;
            header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6;
            header->pa_lfr_asm_blk_nr[0]
                = (unsigned char)((NB_BINS_PER_PKT_ASM_F1_1) >> SHIFT_1_BYTE); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)(NB_BINS_PER_PKT_ASM_F1_1); // BLK_NR LSB
        }
        else
        {
            spw_ioctl_send_ASM.dlen = DLEN_ASM_F1_PKT_2;
            spw_ioctl_send_ASM.data = (char*)&spectral_matrix[(
                (ASM_F1_INDICE_START + (i * NB_BINS_PER_PKT_ASM_F1_1)) * NB_FLOATS_PER_SM)];
            length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_2;
            header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_6;
            header->pa_lfr_asm_blk_nr[0]
                = (unsigned char)((NB_BINS_PER_PKT_ASM_F1_2) >> SHIFT_1_BYTE); // BLK_NR MSB
            header->pa_lfr_asm_blk_nr[1] = (unsigned char)(NB_BINS_PER_PKT_ASM_F1_2); // BLK_NR LSB
        }

        spw_ioctl_send_ASM.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM;
        spw_ioctl_send_ASM.hdr = (char*)header;
        spw_ioctl_send_ASM.options = 0;

        // (2) BUILD THE HEADER
        increment_seq_counter_source_id(header->packetSequenceControl, sid);
        header->packetLength[0] = (unsigned char)(length >> SHIFT_1_BYTE);
        header->packetLength[1] = (unsigned char)(length);
        header->sid = (unsigned char)sid; // SID
        header->pa_lfr_pkt_cnt_asm = PKTCNT_ASM;
        header->pa_lfr_pkt_nr_asm = (unsigned char)(i + 1);

        // (3) SET PACKET TIME
        header->time[BYTE_0] = (unsigned char)(coarseTime >> SHIFT_3_BYTES);
        header->time[BYTE_1] = (unsigned char)(coarseTime >> SHIFT_2_BYTES);
        header->time[BYTE_2] = (unsigned char)(coarseTime >> SHIFT_1_BYTE);
        header->time[BYTE_3] = (unsigned char)(coarseTime);
        header->time[BYTE_4] = (unsigned char)(fineTime >> SHIFT_1_BYTE);
        header->time[BYTE_5] = (unsigned char)(fineTime);
        //
        header->acquisitionTime[BYTE_0] = header->time[BYTE_0];
        header->acquisitionTime[BYTE_1] = header->time[BYTE_1];
        header->acquisitionTime[BYTE_2] = header->time[BYTE_2];
        header->acquisitionTime[BYTE_3] = header->time[BYTE_3];
        header->acquisitionTime[BYTE_4] = header->time[BYTE_4];
        header->acquisitionTime[BYTE_5] = header->time[BYTE_5];

        // (4) SEND PACKET
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_ASM);
        DEBUG_CHECK_STATUS(status);
    }
}

/**
 * @brief spw_send_asm_f2 Sends an ASM packet at F2 over spacewire
 * @param ring_node_to_send node pointing to the actual buffer to send
 * @param header
 */
void spw_send_asm_f2(ring_node* ring_node_to_send, Header_TM_LFR_SCIENCE_ASM_t* header)
{
    unsigned int i;
    unsigned int length = 0;
    rtems_status_code status;
    unsigned int sid;
    float* spectral_matrix;
    int coarseTime;
    int fineTime;
    spw_ioctl_pkt_send spw_ioctl_send_ASM;

    sid = ring_node_to_send->sid;
    spectral_matrix = (float*)ring_node_to_send->buffer_address;
    coarseTime = ring_node_to_send->coarseTime;
    fineTime = ring_node_to_send->fineTime;

    header->pa_bia_status_info = pa_bia_status_info;
    header->sy_lfr_common_parameters = parameter_dump_packet.sy_lfr_common_parameters;

    for (i = 0; i < PKTCNT_ASM; i++)
    {

        spw_ioctl_send_ASM.dlen = DLEN_ASM_F2_PKT;
        spw_ioctl_send_ASM.data = (char*)&spectral_matrix[(
            (ASM_F2_INDICE_START + (i * NB_BINS_PER_PKT_ASM_F2)) * NB_FLOATS_PER_SM)];
        length = PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F2;
        header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE_3;
        header->pa_lfr_asm_blk_nr[0]
            = (unsigned char)((NB_BINS_PER_PKT_ASM_F2) >> SHIFT_1_BYTE); // BLK_NR MSB
        header->pa_lfr_asm_blk_nr[1] = (unsigned char)(NB_BINS_PER_PKT_ASM_F2); // BLK_NR LSB

        spw_ioctl_send_ASM.hlen = HEADER_LENGTH_TM_LFR_SCIENCE_ASM;
        spw_ioctl_send_ASM.hdr = (char*)header;
        spw_ioctl_send_ASM.options = 0;

        // (2) BUILD THE HEADER
        increment_seq_counter_source_id(header->packetSequenceControl, sid);
        header->packetLength[0] = (unsigned char)(length >> SHIFT_1_BYTE);
        header->packetLength[1] = (unsigned char)(length);
        header->sid = (unsigned char)sid; // SID
        header->pa_lfr_pkt_cnt_asm = PKTCNT_ASM;
        header->pa_lfr_pkt_nr_asm = (unsigned char)(i + 1);

        // (3) SET PACKET TIME
        header->time[BYTE_0] = (unsigned char)(coarseTime >> SHIFT_3_BYTES);
        header->time[BYTE_1] = (unsigned char)(coarseTime >> SHIFT_2_BYTES);
        header->time[BYTE_2] = (unsigned char)(coarseTime >> SHIFT_1_BYTE);
        header->time[BYTE_3] = (unsigned char)(coarseTime);
        header->time[BYTE_4] = (unsigned char)(fineTime >> SHIFT_1_BYTE);
        header->time[BYTE_5] = (unsigned char)(fineTime);
        //
        header->acquisitionTime[BYTE_0] = header->time[BYTE_0];
        header->acquisitionTime[BYTE_1] = header->time[BYTE_1];
        header->acquisitionTime[BYTE_2] = header->time[BYTE_2];
        header->acquisitionTime[BYTE_3] = header->time[BYTE_3];
        header->acquisitionTime[BYTE_4] = header->time[BYTE_4];
        header->acquisitionTime[BYTE_5] = header->time[BYTE_5];

        // (4) SEND PACKET
        status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send_ASM);
        DEBUG_CHECK_STATUS(status);
    }
}

/**
 * @brief spw_send_k_dump Sends k coefficients dump packet over spacewire
 * @param ring_node_to_send node pointing to the actual buffer to send
 */
void spw_send_k_dump(ring_node* ring_node_to_send)
{
    rtems_status_code status;
    Packet_TM_LFR_KCOEFFICIENTS_DUMP_t* kcoefficients_dump;
    unsigned int packetLength;
    unsigned int size;

    LFR_PRINTF("spw_send_k_dump\n");

    kcoefficients_dump = (Packet_TM_LFR_KCOEFFICIENTS_DUMP_t*)ring_node_to_send->buffer_address;

    packetLength
        = (kcoefficients_dump->packetLength[0] * CONST_256) + kcoefficients_dump->packetLength[1];

    size = packetLength + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    LFR_PRINTF("packetLength %d, size %d\n", packetLength, size);

    status = write(fdSPW, (char*)ring_node_to_send->buffer_address, size);

    if ((int)status == -1)
    {
        LFR_PRINTF("in SEND *** (2.a) ERRNO = %d, size = %d\n", errno, size);
    }

    ring_node_to_send->status = INIT_CHAR;
}

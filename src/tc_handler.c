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

char *DumbMessages[6] = {"in DUMB *** default",                                             // RTEMS_EVENT_0
                    "in DUMB *** timecode_irq_handler",                                     // RTEMS_EVENT_1
                    "in DUMB *** waveforms_isr",                                            // RTEMS_EVENT_2
                    "in DUMB *** in SMIQ *** Error sending event to AVF0",                  // RTEMS_EVENT_3
                    "in DUMB *** spectral_matrices_isr *** Error sending event to SMIQ",    // RTEMS_EVENT_4
                    "in DUMB *** waveforms_simulator_isr"                                   // RTEMS_EVENT_5
};

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
unsigned char currentTC_processedFlag;

unsigned int lookUpTableForCRC[256];

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt( unsigned char D, unsigned int Chk)
{
    /** This function generate the CRC for one byte and returns the value of the new syndrome.
     *
     * @param D is the current byte of data.
     * @param Chk is the current syndrom value.
     * @return the value of the new syndrome on two bytes.
     *
     */

    return(((Chk << 8) & 0xff00)^lookUpTableForCRC [(((Chk >> 8)^D) & 0x00ff)]);
}

void initLookUpTableForCRC( void )
{
    /** This function is used to initiates the look-up table for fast CRC computation.
     *
     * The global table lookUpTableForCRC[256] is initiated.
     *
     */

    unsigned int i;
    unsigned int tmp;

    for (i=0; i<256; i++)
    {
        tmp = 0;
        if((i & 1) != 0) {
            tmp = tmp ^ 0x1021;
        }
        if((i & 2) != 0) {
            tmp = tmp ^ 0x2042;
        }
        if((i & 4) != 0) {
            tmp = tmp ^ 0x4084;
        }
        if((i & 8) != 0) {
            tmp = tmp ^ 0x8108;
        }
        if((i & 16) != 0) {
            tmp = tmp ^ 0x1231;
        }
        if((i & 32) != 0) {
            tmp = tmp ^ 0x2462;
        }
        if((i & 64) != 0) {
            tmp = tmp ^ 0x48c4;
        }
        if((i & 128) != 0) {
            tmp = tmp ^ 0x9188;
        }
        lookUpTableForCRC[i] = tmp;
    }
}

void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData)
{
    /** This function calculates a two bytes Cyclic Redundancy Code.
     *
     * @param data points to a buffer containing the data on which to compute the CRC.
     * @param crcAsTwoBytes points points to a two bytes buffer in which the CRC is stored.
     * @param sizeOfData is the number of bytes of *data* used to compute the CRC.
     *
     * The specification of the Cyclic Redundancy Code is described in the following document: ECSS-E-70-41-A.
     *
     */

    unsigned int Chk;
    int j;
    Chk = 0xffff; // reset the syndrom to all ones
    for (j=0; j<sizeOfData; j++) {
        Chk = Crc_opt(data[j], Chk);
    }
    crcAsTwoBytes[0] = (unsigned char) (Chk >> 8);
    crcAsTwoBytes[1] = (unsigned char) (Chk & 0x00ff);
}

void updateLFRCurrentMode()
{
    /** This function updates the value of the global variable lfrCurrentMode.
     *
     * lfrCurrentMode is a parameter used by several functions to know in which mode LFR is running.
     *
     */
    // update the local value of lfrCurrentMode with the value contained in the housekeeping_packet structure
    lfrCurrentMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;
}

//*********************
// ACCEPTANCE FUNCTIONS
int tc_acceptance(ccsdsTelecommandPacket_t *TC, unsigned int tc_len_recv, rtems_id queue_queu_id, rtems_id queue_pkts_id)
{
    /** This function executes the TeleCommand acceptance steps.
     *
     * @param TC points to the TeleCommand packet which is under investigation.
     * @param tc_len_recv contains the length of the packet that has been received.
     * @param queue_queu_id is the id of the rtems queue in which messages are written if the acceptance is not successful
     * @param queue_pkts_id is the id of the rtems queue in which messages are written if the acceptance is successful
     * @return status code
     *
     * The acceptance steps can result in two different actions.
     * 1. If the acceptance is successful, the TC is sent in the receiving queue for processing.
     * 2. If the acceptance fails, a TM packet is transmitted to report the error.
     *
     */

    int ret = 0;
    rtems_status_code status;
    unsigned int parserCode = 0;
    unsigned char computed_CRC[2];

    GetCRCAsTwoBytes( (unsigned char*) TC->packetID, computed_CRC, tc_len_recv + 5 );
    parserCode = tc_parser( TC, tc_len_recv ) ;
    if ( (parserCode == ILLEGAL_APID) | (parserCode == WRONG_LEN_PACKET) | (parserCode == INCOR_CHECKSUM)
        | (parserCode == ILL_TYPE) | (parserCode == ILL_SUBTYPE) | (parserCode == WRONG_APP_DATA) )
    { // send TM_LFR_TC_EXE_CORRUPTED
        send_tm_lfr_tc_exe_corrupted( TC, queue_queu_id, computed_CRC, currentTC_LEN_RCV );
    }
    else { // send valid TC to the action launcher
        status =  rtems_message_queue_send( queue_queu_id, TC, tc_len_recv + CCSDS_TC_TM_PACKET_OFFSET + 3);
        ret = LFR_SUCCESSFUL;
    }
    return ret;
}

int tc_parser(ccsdsTelecommandPacket_t * TCPacket, unsigned int TC_LEN_RCV)
{
    /** This function parses TeleCommands.
     *
     * @param TC points to the TeleCommand that will be parsed.
     * @param TC_LEN_RCV is the received packet length.
     * @return Status code of the parsing.
     *
     * The parsing checks:
     * - process id
     * - category
     * - length: a global check is performed and a per subtype check also
     * - type
     * - subtype
     * - crc
     *
     */

    int status;
    unsigned char pid;
    unsigned char category;
    unsigned int length;
    unsigned char packetType;
    unsigned char packetSubtype;

    status = CCSDS_TM_VALID;

    // APID check *** APID on 2 bytes
    pid = ((TCPacket->packetID[0] & 0x07)<<4) + ( (TCPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TCPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    length = (TCPacket->packetLength[0] * 256) + TCPacket->packetLength[1];
    packetType = TCPacket->serviceType;
    packetSubtype = TCPacket->serviceSubType;

    if ( pid != CCSDS_PROCESS_ID )  // CHECK THE PROCESS ID
    {
        status = ILLEGAL_APID;
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE CATEGORY
    {
        if ( category != CCSDS_PACKET_CATEGORY )
        {
            status = ILLEGAL_APID;
        }
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE PACKET LENGTH FIELD AND THE ACTUAL LENGTH COMPLIANCE
    {
        if (length != TC_LEN_RCV ) {
            status = WRONG_LEN_PACKET;
        }
    }
    if (status == CCSDS_TM_VALID)   // CHECK THAT THE PACKET DOES NOT EXCEED THE MAX SIZE
    {
        if ( length >= CCSDS_TC_PKT_MAX_SIZE ) {
            status = WRONG_LEN_PACKET;
        }
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE TYPE
    {
        status = tc_check_type( packetType );
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE SUBTYPE
    {
        status = tc_check_subtype( packetSubtype );
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE SUBTYPE AND LENGTH COMPLIANCE
    {
        status = tc_check_length( packetSubtype, length );
    }
    if (status == CCSDS_TM_VALID )  // CHECK CRC
    {
        status = tc_check_crc( TCPacket, length );
    }

    return status;
}

int tc_check_type( unsigned char packetType )
{
    /** This function checks that the type of a TeleCommand is valid.
     *
     * @param packetType is the type to check.
     * @return Status code CCSDS_TM_VALID or ILL_TYPE.
     *
     */

    int status;

    if ( (packetType == TC_TYPE_GEN) || (packetType == TC_TYPE_TIME))
    {
        status = CCSDS_TM_VALID;
    }
    else
    {
        status = ILL_TYPE;
    }

    return status;
}

int tc_check_subtype( unsigned char packetSubType )
{
    /** This function checks that the subtype of a TeleCommand is valid.
     *
     * @param packetSubType is the subtype to check.
     * @return Status code CCSDS_TM_VALID or ILL_SUBTYPE.
     *
     */

    int status;

    if ( (packetSubType == TC_SUBTYPE_RESET)
         || (packetSubType == TC_SUBTYPE_LOAD_COMM)
         || (packetSubType == TC_SUBTYPE_LOAD_NORM) || (packetSubType == TC_SUBTYPE_LOAD_BURST)
         || (packetSubType == TC_SUBTYPE_LOAD_SBM1) || (packetSubType == TC_SUBTYPE_LOAD_SBM2)
         || (packetSubType == TC_SUBTYPE_DUMP)
         || (packetSubType == TC_SUBTYPE_ENTER)
         || (packetSubType == TC_SUBTYPE_UPDT_INFO) || (packetSubType == TC_SUBTYPE_UPDT_TIME)
         || (packetSubType == TC_SUBTYPE_EN_CAL)    || (packetSubType == TC_SUBTYPE_DIS_CAL) )
    {
        status = CCSDS_TM_VALID;
    }
    else
    {
        status = ILL_TYPE;
    }

    return status;
}

int tc_check_length( unsigned char packetSubType, unsigned int length )
{
    /** This function checks that the subtype and the length are compliant.
     *
     * @param packetSubType is the subtype to check.
     * @param length is the length to check.
     * @return Status code CCSDS_TM_VALID or ILL_TYPE.
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    switch(packetSubType)
    {
    case TC_SUBTYPE_RESET:
        if (length!=(TC_LEN_RESET-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_COMM:
        if (length!=(TC_LEN_LOAD_COMM-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_NORM:
        if (length!=(TC_LEN_LOAD_NORM-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_BURST:
        if (length!=(TC_LEN_LOAD_BURST-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_SBM1:
        if (length!=(TC_LEN_LOAD_SBM1-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_SBM2:
        if (length!=(TC_LEN_LOAD_SBM2-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_DUMP:
        if (length!=(TC_LEN_DUMP-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_ENTER:
        if (length!=(TC_LEN_ENTER-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_UPDT_INFO:
        if (length!=(TC_LEN_UPDT_INFO-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_EN_CAL:
        if (length!=(TC_LEN_EN_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_DIS_CAL:
        if (length!=(TC_LEN_DIS_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_UPDT_TIME:
        if (length!=(TC_LEN_UPDT_TIME-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PACKET;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    default: // if the subtype is not a legal value, return ILL_SUBTYPE
        status = ILL_SUBTYPE;
        break    ;
    }

    return status;
}

int tc_check_crc( ccsdsTelecommandPacket_t * TCPacket, unsigned int length )
{
    /** This function checks the CRC validity of the corresponding TeleCommand packet.
     *
     * @param TCPacket points to the TeleCommand packet to check.
     * @param length is the length of the TC packet.
     * @return Status code CCSDS_TM_VALID or INCOR_CHECKSUM.
     *
     */

    int status;
    unsigned char * CCSDSContent;
    unsigned char currentTC_COMPUTED_CRC[2];

    CCSDSContent = (unsigned char*) TCPacket->packetID;
    GetCRCAsTwoBytes(CCSDSContent, currentTC_COMPUTED_CRC, length + CCSDS_TC_TM_PACKET_OFFSET - 2); // 2 CRC bytes removed from the calculation of the CRC
    if (currentTC_COMPUTED_CRC[0] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -2]) {
        status = INCOR_CHECKSUM;
    }
    else if (currentTC_COMPUTED_CRC[1] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -1]) {
        status = INCOR_CHECKSUM;
    }
    else {
        status = CCSDS_TM_VALID;
    }

    return status;
}

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused )
{
    /** This RTEMS task is dedicated to the reception of incoming TeleCommands.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The RECV task blocks on a call to the read system call, waiting for incoming SpaceWire data. When unblocked:
     * 1. It reads the incoming data.
     * 2. Launches the acceptance procedure.
     * 3. If the Telecommand is valid, sends it to the ACTN task using an RTEMS message queue.
     *
     */

    int len = 0;
    unsigned int i = 0;
    ccsdsTelecommandPacket_t currentTC;
    char data[100];
    rtems_status_code status;
    rtems_id queue_queu_id;
    rtems_id queue_pkts_id;

    for(i=0; i<100; i++) data[i] = 0;

    initLookUpTableForCRC(); // the table is used to compute Cyclic Redundancy Codes

    status =  rtems_message_queue_ident( misc_name[QUEUE_QUEU], 0, &queue_queu_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR getting queue_queu id, %d\n", status)
    }

    status =  rtems_message_queue_ident( misc_name[QUEUE_PKTS], 0, &queue_pkts_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in RECV *** ERR getting queue_pkts id, %d\n", status)
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
                currentTC_LEN_RCV[0] = 0x00;
                currentTC_LEN_RCV[1] = (unsigned char) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); //  build the corresponding packet size field
                currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); // => -3 is for Prot ID, Reserved and User App bytes
                // CHECK THE TC
                tc_acceptance(&currentTC, currentTC_LEN_RCV_AsUnsignedInt, queue_queu_id, queue_pkts_id);
            }
        }
    }
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
    rtems_id queue_rcv_id;
    rtems_id queue_snd_id;

    status =  rtems_message_queue_ident( misc_name[QUEUE_QUEU], 0, &queue_rcv_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR getting queue_rcv_id %d\n", status)
    }

    status =  rtems_message_queue_ident( misc_name[QUEUE_PKTS], 0, &queue_snd_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in ACTN *** ERR getting queue_snd_id %d\n", status)
    }

    result = LFR_SUCCESSFUL;
    subtype = 0;          // subtype of the current TC packet

    BOOT_PRINTF("in ACTN *** \n")

    while(1)
    {
        status = rtems_message_queue_receive( queue_rcv_id, (char*) &TC, &size,
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        if (status!=RTEMS_SUCCESSFUL) PRINTF1("ERR *** in task ACTN *** error receiving a message, code %d \n", status)
        else
        {
            subtype = TC.serviceSubType;
            switch(subtype)
            {
                case TC_SUBTYPE_RESET:
                    result = action_reset( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_COMM:
                    result = action_load_common_par( &TC );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_NORM:
                    result = action_load_normal_par( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_BURST:
                    result = action_load_burst_par( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM1:
                    result = action_load_sbm1_par( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM2:
                    result = action_load_sbm2_par( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_DUMP:
                    result = action_dump_par( queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_ENTER:
                    result = action_enter_mode( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_UPDT_INFO:
                    result = action_update_info( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_EN_CAL:
                    result = action_enable_calibration( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_DIS_CAL:
                    result = action_disable_calibration( &TC, queue_snd_id );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                case TC_SUBTYPE_UPDT_TIME:
                    result = action_update_time( &TC );
                    close_action( &TC, result, queue_snd_id );
                    break;
                    //
                default:
                    break;
            }
        }
    }
}

rtems_task dumb_task( rtems_task_argument unused )
{
    /** This RTEMS taks is used to print messages without affecting the general behaviour of the software.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The DUMB taks waits for RTEMS events and print messages depending on the incoming events.
     *
     */

    unsigned int i;
    unsigned int intEventOut;
    unsigned int coarse_time = 0;
    unsigned int fine_time = 0;
    rtems_event_set event_out;

    BOOT_PRINTF("in DUMB *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3 | RTEMS_EVENT_4 | RTEMS_EVENT_5,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut =  (unsigned int) event_out;
        for ( i=0; i<32; i++)
        {
            if ( ((intEventOut >> i) & 0x0001) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                printf("in DUMB *** time = coarse: %x, fine: %x, %s\n", coarse_time, fine_time, DumbMessages[i]);
            }
        }
    }
}

//***********
// TC ACTIONS

int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_RESET TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    send_tm_lfr_tc_exe_not_implemented( TC, queue_id );
    return LFR_DEFAULT;
}

int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_ENTER_MODE TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    rtems_status_code status;
    unsigned char requestedMode;

    requestedMode = TC->dataAndCRC[1];

    if ( (requestedMode != LFR_MODE_STANDBY)
         && (requestedMode != LFR_MODE_NORMAL) && (requestedMode != LFR_MODE_BURST)
         && (requestedMode != LFR_MODE_SBM1) && (requestedMode != LFR_MODE_SBM2) )
    {
        status = RTEMS_UNSATISFIED;
        send_tm_lfr_tc_exe_inconsistent( TC, queue_id, BYTE_POS_CP_LFR_MODE, requestedMode );
    }
    else
    {
        printf("try to enter mode %d\n", requestedMode);

#ifdef PRINT_TASK_STATISTICS
        if (requestedMode != LFR_MODE_STANDBY)
        {
            rtems_cpu_usage_reset();
            maxCount = 0;
        }
#endif

        status = transition_validation(requestedMode);

        if ( status == LFR_SUCCESSFUL ) {
            if ( lfrCurrentMode != LFR_MODE_STANDBY)
            {
                status = stop_current_mode();
            }
            if (status != RTEMS_SUCCESSFUL)
            {
                PRINTF("ERR *** in action_enter *** stop_current_mode\n")
            }
            status = enter_mode(requestedMode, TC);
        }
        else
        {
            PRINTF("ERR *** in action_enter *** transition rejected\n")
            send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        }
    }

    return status;
}

int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_INFO TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    unsigned int val;
    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( (lfrMode == LFR_MODE_STANDBY) ) {
        send_tm_lfr_tc_exe_not_implemented( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        val = housekeeping_packet.hk_lfr_update_info_tc_cnt[0] * 256
                + housekeeping_packet.hk_lfr_update_info_tc_cnt[1];
        val++;
        housekeeping_packet.hk_lfr_update_info_tc_cnt[0] = (unsigned char) (val >> 8);
        housekeeping_packet.hk_lfr_update_info_tc_cnt[1] = (unsigned char) (val);
        result = LFR_SUCCESSFUL;
    }

    return result;
}

int action_enable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_ENABLE_CALIBRATION TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( (lfrMode == LFR_MODE_STANDBY) | (lfrMode == LFR_MODE_BURST) | (lfrMode == LFR_MODE_SBM2) ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        send_tm_lfr_tc_exe_not_implemented( TC, queue_id );
        result = LFR_DEFAULT;
    }
    return result;
}

int action_disable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    /** This function executes specific actions when a TC_LFR_DISABLE_CALIBRATION TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( (lfrMode == LFR_MODE_STANDBY) | (lfrMode == LFR_MODE_BURST) | (lfrMode == LFR_MODE_SBM2) ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        send_tm_lfr_tc_exe_not_implemented( TC, queue_id );
        result = LFR_DEFAULT;
    }
    return result;
}

int action_update_time(ccsdsTelecommandPacket_t *TC)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_TIME TeleCommand has been received.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM transmission by the SpaceWire driver
     *
     */

    unsigned int val;

    time_management_regs->coarse_time_load = (TC->dataAndCRC[0] << 24)
                                                + (TC->dataAndCRC[1] << 16)
                                                + (TC->dataAndCRC[2] << 8)
                                                + TC->dataAndCRC[3];
    val = housekeeping_packet.hk_lfr_update_time_tc_cnt[0] * 256
            + housekeeping_packet.hk_lfr_update_time_tc_cnt[1];
    val++;
    housekeeping_packet.hk_lfr_update_time_tc_cnt[0] = (unsigned char) (val >> 8);
    housekeeping_packet.hk_lfr_update_time_tc_cnt[1] = (unsigned char) (val);
    time_management_regs->ctrl = time_management_regs->ctrl | 1;

    return LFR_SUCCESSFUL;
}

//*******************
// ENTERING THE MODES

int transition_validation(unsigned char requestedMode)
{
    int status;

    switch (requestedMode)
    {
    case LFR_MODE_STANDBY:
        if ( lfrCurrentMode == LFR_MODE_STANDBY ) {
            status = LFR_DEFAULT;
        }
        else
        {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_NORMAL:
        if ( lfrCurrentMode == LFR_MODE_NORMAL ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_BURST:
        if ( lfrCurrentMode == LFR_MODE_BURST ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_SBM1:
        if ( lfrCurrentMode == LFR_MODE_SBM1 ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    case LFR_MODE_SBM2:
        if ( lfrCurrentMode == LFR_MODE_SBM2 ) {
            status = LFR_DEFAULT;
        }
        else {
            status = LFR_SUCCESSFUL;
        }
        break;
    default:
        status = LFR_DEFAULT;
        break;
    }

    return status;
}

int stop_current_mode()
{
    /** This function stops the current mode by masking interrupt lines and suspending science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_ALREADY_SUSPENDED - task already suspended
     *
     */

    rtems_status_code status;

    status = RTEMS_SUCCESSFUL;

    // mask all IRQ lines related to signal processing
    LEON_Mask_interrupt( IRQ_SM );                  // mask spectral matrices interrupt (coming from the timer VHDL IP)
    LEON_Clear_interrupt( IRQ_SM );                 // clear spectral matrices interrupt (coming from the timer VHDL IP)

#ifdef GSA
    LEON_Mask_interrupt( IRQ_WF );                  // mask waveform interrupt (coming from the timer VHDL IP)
    LEON_Clear_interrupt( IRQ_WF );                  // clear waveform interrupt (coming from the timer VHDL IP)
    timer_stop( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR );
#else
    LEON_Mask_interrupt( IRQ_WAVEFORM_PICKER );     // mask waveform picker interrupt
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );    // clear waveform picker interrupt
    LEON_Mask_interrupt( IRQ_SPECTRAL_MATRIX );     // mask spectral matrix interrupt
    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX );    // clear spectral matrix interrupt
    LEON_Mask_interrupt( IRQ_SM );                  // for SM simulation
    LEON_Clear_interrupt( IRQ_SM );                 // for SM simulation
#endif
    //**********************
    // suspend several tasks
    if (lfrCurrentMode != LFR_MODE_STANDBY) {
        status = suspend_science_tasks();
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in stop_current_mode *** in suspend_science_tasks *** ERR code: %d\n", status)
    }

    //*************************
    // initialize the registers
#ifdef GSA
#else
    reset_wfp_burst_enable();   // reset burst and enable bits
    reset_wfp_status();         // reset all the status bits
#endif

    return status;
}

int enter_mode(unsigned char mode, ccsdsTelecommandPacket_t *TC )
{
    rtems_status_code status;

    status = RTEMS_UNSATISFIED;

    housekeeping_packet.lfr_status_word[0] = (unsigned char) ((mode << 4) + 0x0d);
    lfrCurrentMode = mode;

    switch(mode){
    case LFR_MODE_STANDBY:
        status = enter_standby_mode( TC );
        break;
    case LFR_MODE_NORMAL:
        status = enter_normal_mode( TC );
        break;
    case LFR_MODE_BURST:
        status = enter_burst_mode( TC );
        break;
    case LFR_MODE_SBM1:
        status = enter_sbm1_mode( TC );
        break;
    case LFR_MODE_SBM2:
        status = enter_sbm2_mode( TC );
        break;
    default:
        status = RTEMS_UNSATISFIED;
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF("in enter_mode *** ERR\n")
        status = RTEMS_UNSATISFIED;
    }

    return status;
}

int enter_standby_mode()
{
    reset_waveform_picker_regs();

    PRINTF1("maxCount = %d\n", maxCount)

#ifdef PRINT_TASK_STATISTICS
    rtems_cpu_usage_report();
#endif

#ifdef PRINT_STACK_REPORT
    rtems_stack_checker_report_usage();
#endif

    return LFR_SUCCESSFUL;
}

int enter_normal_mode()
{
    rtems_status_code status;

    status = restart_science_tasks();

#ifdef GSA
    timer_start( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_WF_SIMULATOR );
    timer_start( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR );
    LEON_Clear_interrupt( IRQ_WF );
    LEON_Unmask_interrupt( IRQ_WF );
    //
    set_local_nb_interrupt_f0_MAX();
    LEON_Clear_interrupt( IRQ_SM ); // the IRQ_SM seems to be incompatible with the IRQ_WF on the xilinx board
    LEON_Unmask_interrupt( IRQ_SM );
#else
    //****************
    // waveform picker
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    reset_waveform_picker_regs();
    set_wfp_burst_enable_register(LFR_MODE_NORMAL);
    //****************
    // spectral matrix
//    set_local_nb_interrupt_f0_MAX();
//    LEON_Clear_interrupt( IRQ_SPECTRAL_MATRIX ); // the IRQ_SM seems to be incompatible with the IRQ_WF on the xilinx board
//    LEON_Unmask_interrupt( IRQ_SPECTRAL_MATRIX );
//    spectral_matrix_regs->config = 0x01;
//    spectral_matrix_regs->status = 0x00;
#endif

    return status;
}

int enter_burst_mode()
{
    rtems_status_code status;

    status = restart_science_tasks();

#ifdef GSA
    LEON_Unmask_interrupt( IRQ_SM );
#else
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    reset_waveform_picker_regs();
    set_wfp_burst_enable_register(LFR_MODE_BURST);
#endif

    return status;
}

int enter_sbm1_mode()
{
    rtems_status_code status;

    status = restart_science_tasks();

    set_local_sbm1_nb_cwf_max();

    reset_local_sbm1_nb_cwf_sent();

#ifdef GSA
    LEON_Unmask_interrupt( IRQ_SM );
#else
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    reset_waveform_picker_regs();
    set_wfp_burst_enable_register(LFR_MODE_SBM1);
    // SM simulation
//    timer_start( (gptimer_regs_t*) REGS_ADDR_GPTIMER, TIMER_SM_SIMULATOR );
//    LEON_Clear_interrupt( IRQ_SM ); // the IRQ_SM seems to be incompatible with the IRQ_WF on the xilinx board
//    LEON_Unmask_interrupt( IRQ_SM );
 #endif

    return status;
}

int enter_sbm2_mode()
{
    rtems_status_code status;

    status = restart_science_tasks();

    set_local_sbm2_nb_cwf_max();

    reset_local_sbm2_nb_cwf_sent();

#ifdef GSA
    LEON_Unmask_interrupt( IRQ_SM );
#else
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    reset_waveform_picker_regs();
    set_wfp_burst_enable_register(LFR_MODE_SBM2);
#endif

    return status;
}

int restart_science_tasks()
{
    rtems_status_code status[6];
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status[0] = rtems_task_restart( Task_id[TASKID_AVF0], 1 );
    if (status[0] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 0 ERR %d\n", status[0])
    }

    status[1] = rtems_task_restart( Task_id[TASKID_BPF0],1 );
    if (status[1] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 1 ERR %d\n", status[1])
    }

    status[2] = rtems_task_restart( Task_id[TASKID_WFRM],1 );
    if (status[2] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 2 ERR %d\n", status[2])
    }

    status[3] = rtems_task_restart( Task_id[TASKID_CWF3],1 );
    if (status[3] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 3 ERR %d\n", status[3])
    }

    status[4] = rtems_task_restart( Task_id[TASKID_CWF2],1 );
    if (status[4] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 4 ERR %d\n", status[4])
    }

    status[5] = rtems_task_restart( Task_id[TASKID_CWF1],1 );
    if (status[5] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in restart_science_task *** 5 ERR %d\n", status[5])
    }

    if ( (status[0] != RTEMS_SUCCESSFUL) || (status[1] != RTEMS_SUCCESSFUL) || (status[2] != RTEMS_SUCCESSFUL) ||
         (status[3] != RTEMS_SUCCESSFUL) || (status[4] != RTEMS_SUCCESSFUL) || (status[5] != RTEMS_SUCCESSFUL) )
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

int suspend_science_tasks()
{
    /** This function suspends the science tasks.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task restarted successfully
     * - RTEMS_INVALID_ID - task id invalid
     * - RTEMS_ALREADY_SUSPENDED - task already suspended
     *
     */

    rtems_status_code status;

    status = rtems_task_suspend( Task_id[TASKID_AVF0] );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** AVF0 ERR %d\n", status)
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend BPF0
    {
        status = rtems_task_suspend( Task_id[TASKID_BPF0] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** BPF0 ERR %d\n", status)
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend WFRM
    {
        status = rtems_task_suspend( Task_id[TASKID_WFRM] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** WFRM ERR %d\n", status)
        }
    }

    if (status == RTEMS_SUCCESSFUL)        // suspend CWF3
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF3] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF3 ERR %d\n", status)
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend CWF2
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF2] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF2 ERR %d\n", status)
        }
    }
    if (status == RTEMS_SUCCESSFUL)        // suspend CWF1
    {
        status = rtems_task_suspend( Task_id[TASKID_CWF1] );
        if (status != RTEMS_SUCCESSFUL)
        {
            PRINTF1("in suspend_science_task *** CWF1 ERR %d\n", status)
        }
    }

    return status;
}

//****************
// CLOSING ACTIONS
void update_last_TC_exe(ccsdsTelecommandPacket_t *TC)
{
    housekeeping_packet.hk_lfr_last_exe_tc_id[0] = TC->packetID[0];
    housekeeping_packet.hk_lfr_last_exe_tc_id[1] = TC->packetID[1];
    housekeeping_packet.hk_lfr_last_exe_tc_type[0] = 0x00;
    housekeeping_packet.hk_lfr_last_exe_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[0] = 0x00;
    housekeeping_packet.hk_lfr_last_exe_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_exe_tc_time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    housekeeping_packet.hk_lfr_last_exe_tc_time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    housekeeping_packet.hk_lfr_last_exe_tc_time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    housekeeping_packet.hk_lfr_last_exe_tc_time[3] = (unsigned char) (time_management_regs->coarse_time);
    housekeeping_packet.hk_lfr_last_exe_tc_time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    housekeeping_packet.hk_lfr_last_exe_tc_time[5] = (unsigned char) (time_management_regs->fine_time);
}

void update_last_TC_rej(ccsdsTelecommandPacket_t *TC)
{
    housekeeping_packet.hk_lfr_last_rej_tc_id[0] = TC->packetID[0];
    housekeeping_packet.hk_lfr_last_rej_tc_id[1] = TC->packetID[1];
    housekeeping_packet.hk_lfr_last_rej_tc_type[0] = 0x00;
    housekeeping_packet.hk_lfr_last_rej_tc_type[1] = TC->serviceType;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[0] = 0x00;
    housekeeping_packet.hk_lfr_last_rej_tc_subtype[1] = TC->serviceSubType;
    housekeeping_packet.hk_lfr_last_rej_tc_time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    housekeeping_packet.hk_lfr_last_rej_tc_time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    housekeeping_packet.hk_lfr_last_rej_tc_time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    housekeeping_packet.hk_lfr_last_rej_tc_time[3] = (unsigned char) (time_management_regs->coarse_time);
    housekeeping_packet.hk_lfr_last_rej_tc_time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    housekeeping_packet.hk_lfr_last_rej_tc_time[5] = (unsigned char) (time_management_regs->fine_time);
}

void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id)
{
    unsigned int val = 0;
    if (result == LFR_SUCCESSFUL)
    {
        if ( !( (TC->serviceType==TC_TYPE_TIME) && (TC->serviceSubType==TC_SUBTYPE_UPDT_TIME) ) )
        {
            send_tm_lfr_tc_exe_success( TC, queue_id );
        }
        update_last_TC_exe( TC );
        val = housekeeping_packet.hk_dpu_exe_tc_lfr_cnt[0] * 256 + housekeeping_packet.hk_dpu_exe_tc_lfr_cnt[1];
        val++;
        housekeeping_packet.hk_dpu_exe_tc_lfr_cnt[0] = (unsigned char) (val >> 8);
        housekeeping_packet.hk_dpu_exe_tc_lfr_cnt[1] = (unsigned char) (val);
    }
    else
    {
        update_last_TC_rej( TC );
        val = housekeeping_packet.hk_dpu_rej_tc_lfr_cnt[0] * 256 + housekeeping_packet.hk_dpu_rej_tc_lfr_cnt[1];
        val++;
        housekeeping_packet.hk_dpu_rej_tc_lfr_cnt[0] = (unsigned char) (val >> 8);
        housekeeping_packet.hk_dpu_rej_tc_lfr_cnt[1] = (unsigned char) (val);
    }
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





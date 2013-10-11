#include <tc_handler.h>
#include <fsw_params.h>

char *DumbMessages[6] = {"in DUMB *** default",                                             // RTEMS_EVENT_0
                    "in DUMB *** timecode_irq_handler",                                     // RTEMS_EVENT_1
                    "in DUMB *** waveforms_isr",                                            // RTEMS_EVENT_2
                    "in DUMB *** in SMIQ *** Error sending event to AVF0",                  // RTEMS_EVENT_3
                    "in DUMB *** spectral_matrices_isr *** Error sending event to SMIQ",    // RTEMS_EVENT_4
                    "in DUMB *** waveforms_simulator_isr"                                   // RTEMS_EVENT_5
};

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned char currentTC_COMPUTED_CRC[2];
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
unsigned char currentTC_processedFlag;

unsigned int lookUpTableForCRC[256];

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt( unsigned char D, unsigned int Chk)
{
    return(((Chk << 8) & 0xff00)^lookUpTableForCRC [(((Chk >> 8)^D) & 0x00ff)]);
}

void initLookUpTableForCRC( void )
{
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
    lfrCurrentMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;
}

//*********************
// ACCEPTANCE FUNCTIONS
int TC_acceptance(ccsdsTelecommandPacket_t *TC, unsigned int tc_len_recv, rtems_id queue_queu_id, rtems_id queue_pkts_id)
{
    int ret = 0;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_CORRUPTED_t packet;
    unsigned int parserCode = 0;
    unsigned char computed_CRC[2];
    unsigned int packetLength;
    unsigned char *packetDataField;

    GetCRCAsTwoBytes( (unsigned char*) TC->packetID, computed_CRC, tc_len_recv + 5 );
    parserCode = TC_parser( TC, tc_len_recv ) ;
    if ( (parserCode == ILLEGAL_APID) | (parserCode == WRONG_LEN_PACKET) | (parserCode == INCOR_CHECKSUM)
        | (parserCode == ILL_TYPE) | (parserCode == ILL_SUBTYPE) | (parserCode == WRONG_APP_DATA) )
    { // generate TM_LFR_TC_EXE_CORRUPTED
        packetDataField = (unsigned char *) &TC->headerFlag_pusVersion_Ack;     // get the beginning of the data field
        packetLength    = (TC->packetLength[0] * 256) + TC->packetLength[1];    // compute the packet length
        //
        packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
        packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
        packet.reserved = DEFAULT_RESERVED;
        packet.userApplication = CCSDS_USER_APP;
        // PACKET HEADER
        packet.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
        packet.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
        packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
        packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
        packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_CORRUPTED >> 8);
        packet.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_CORRUPTED     );
        // DATA FIELD HEADER
        packet.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
        packet.serviceType = TM_TYPE_TC_EXE;
        packet.serviceSubType = TM_SUBTYPE_EXE_NOK;
        packet.destinationID = TM_DESTINATION_ID_GROUND;
        packet.time[0] = (time_management_regs->coarse_time>>24 );
        packet.time[1] = (time_management_regs->coarse_time>>16 );
        packet.time[2] = (time_management_regs->coarse_time>>8  );
        packet.time[3] = (time_management_regs->coarse_time     );
        packet.time[4] = (time_management_regs->fine_time>>8    );
        packet.time[5] = (time_management_regs->fine_time       );
        //
        packet.tc_failure_code[0] = (unsigned char) (FAILURE_CODE_CORRUPTED >> 8);
        packet.tc_failure_code[1] = (unsigned char) (FAILURE_CODE_CORRUPTED     );
        packet.telecommand_pkt_id[0] = TC->packetID[0];
        packet.telecommand_pkt_id[1] = TC->packetID[1];
        packet.pkt_seq_control[0] = TC->packetSequenceControl[0];
        packet.pkt_seq_control[0] = TC->packetSequenceControl[1];
        packet.tc_service = TC->serviceType;
        packet.tc_subtype = TC->serviceSubType;
        packet.pkt_len_rcv_value[0] = TC->packetLength[0];
        packet.pkt_len_rcv_value[1] = TC->packetLength[1];
        packet.pkt_datafieldsize_cnt[0] = currentTC_LEN_RCV[0];
        packet.pkt_datafieldsize_cnt[1] = currentTC_LEN_RCV[1];
        packet.rcv_crc[0] = packetDataField[ packetLength - 1 ];
        packet.rcv_crc[1] = packetDataField[ packetLength     ];
        packet.computed_crc[0] = computed_CRC[0];
        packet.computed_crc[1] = computed_CRC[1];
        // SEND DATA
        status =  rtems_message_queue_urgent( queue_pkts_id, &packet, PACKET_LENGTH_TC_EXE_CORRUPTED + CCSDS_TC_TM_PACKET_OFFSET + 4);
        if (status != RTEMS_SUCCESSFUL) {
            PRINTF("in TC_acceptance *** ERR rtems_message_queue_urgent\n")
            ret = LFR_DEFAULT;
        }
    }
    else { // send valid TC to the action launcher
        status =  rtems_message_queue_send( queue_queu_id, TC, tc_len_recv + CCSDS_TC_TM_PACKET_OFFSET + 3);
        ret = LFR_SUCCESSFUL;
    }
    return ret;
}

unsigned char TC_parser(ccsdsTelecommandPacket_t * TCPacket, unsigned int TC_LEN_RCV)
{
    unsigned char ret = 0;
    unsigned char pid = 0;
    unsigned char category = 0;
    unsigned int length = 0;
    unsigned char packetType = 0;
    unsigned char packetSubtype = 0;
    unsigned char * CCSDSContent = NULL;

    // APID check *** APID on 2 bytes
    pid = ((TCPacket->packetID[0] & 0x07)<<4) + ( (TCPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TCPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    length = (TCPacket->packetLength[0] * 256) + TCPacket->packetLength[1];
    packetType = TCPacket->serviceType;
    packetSubtype = TCPacket->serviceSubType;

    if ( pid != CCSDS_PROCESS_ID ) {
        ret = ILLEGAL_APID;
    }
    else if ( category != CCSDS_PACKET_CATEGORY ) {
        ret = ILLEGAL_APID;
    }
    else if (length != TC_LEN_RCV ) {   // packet length check
        ret = WRONG_LEN_PACKET; // LEN RCV != SIZE FIELD
    }
    else if ( length >= CCSDS_TC_PKT_MAX_SIZE ) {
        ret = WRONG_LEN_PACKET; // check that the packet does not exceed the MAX size
    }
    else if ( packetType == TC_TYPE_GEN ){ // service type, subtype and packet length check
        switch(packetSubtype){ //subtype, autorized values are 3, 20, 21, 24, 27, 28, 30, 40, 50, 60, 61
            case TC_SUBTYPE_RESET:
                if (length!=(TC_LEN_RESET-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_LOAD_COMM:
                if (length!=(TC_LEN_LOAD_COMM-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_LOAD_NORM:
                if (length!=(TC_LEN_LOAD_NORM-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_LOAD_BURST:
                if (length!=(TC_LEN_LOAD_BURST-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_LOAD_SBM1:
                if (length!=(TC_LEN_LOAD_SBM1-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_LOAD_SBM2:
                if (length!=(TC_LEN_LOAD_SBM2-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_DUMP:
                if (length!=(TC_LEN_DUMP-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_ENTER:
                if (length!=(TC_LEN_ENTER-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_UPDT_INFO:
                if (length!=(TC_LEN_UPDT_INFO-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_EN_CAL:
                if (length!=(TC_LEN_EN_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            case TC_SUBTYPE_DIS_CAL:
                if (length!=(TC_LEN_DIS_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
                    ret = WRONG_LEN_PACKET;
                }
                else {
                    ret = CCSDS_TM_VALID;
                }
                break;
            default:
                ret = ILL_SUBTYPE;
                break;
        }
    }
    else if ( packetType == TC_TYPE_TIME ){
        if (packetSubtype!=TC_SUBTYPE_UPDT_TIME) {
            ret = ILL_SUBTYPE;
        }
        else if (length!=(TC_LEN_UPDT_TIME-CCSDS_TC_TM_PACKET_OFFSET)) {
            ret = WRONG_LEN_PACKET;
        }
        else {
            ret = CCSDS_TM_VALID;
        }
    }
    else {
        ret = ILL_TYPE;
    }

    // source ID check // Source ID not documented in the ICD

    // packet error control, CRC check
    if ( ret == CCSDS_TM_VALID ) {
        CCSDSContent = (unsigned char*) TCPacket->packetID;
        GetCRCAsTwoBytes(CCSDSContent, currentTC_COMPUTED_CRC, length + CCSDS_TC_TM_PACKET_OFFSET - 2); // 2 CRC bytes removed from the calculation of the CRC
        if (currentTC_COMPUTED_CRC[0] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -2]) {
            ret = INCOR_CHECKSUM;
        }
        else if (currentTC_COMPUTED_CRC[1] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -1]) {
            ret = INCOR_CHECKSUM;
        }
        else {
            ret = CCSDS_TM_VALID;
        }
    }

    return ret;
}

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused )
{
    int len = 0;
    unsigned int i = 0;
    ccsdsTelecommandPacket_t currentTC;
    char data[100];
    rtems_status_code status;
    rtems_id queue_queu_id;
    rtems_id queue_pkts_id;

    for(i=0; i<100; i++) data[i] = 0;

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
                TC_acceptance(&currentTC, currentTC_LEN_RCV_AsUnsignedInt, queue_queu_id, queue_pkts_id);
            }
        }
    }
}

rtems_task actn_task( rtems_task_argument unused )
{
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
                    result = action_dump_par( &TC );
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
    send_tm_lfr_tc_exe_not_implemented( TC, queue_id );
    return LFR_DEFAULT;
}

int action_load_common_par(ccsdsTelecommandPacket_t *TC)
{
    parameter_dump_packet.unused0 =  TC->dataAndCRC[0];
    parameter_dump_packet.bw_sp0_sp1_r0_r1 = TC->dataAndCRC[1];

    set_wfp_data_shaping(parameter_dump_packet.bw_sp0_sp1_r0_r1);

    return LFR_SUCCESSFUL;
}

int action_load_normal_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int result;
    unsigned int tmp;

    result = LFR_SUCCESSFUL;

    if ( lfrCurrentMode == LFR_MODE_NORMAL ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        // sy_lfr_n_swf_l
        parameter_dump_packet.sy_lfr_n_swf_l[0] = TC->dataAndCRC[0];
        parameter_dump_packet.sy_lfr_n_swf_l[1] = TC->dataAndCRC[1];

        // sy_lfr_n_swf_p
        tmp = ( unsigned int ) floor(
                    (TC->dataAndCRC[2] * 256
                    + TC->dataAndCRC[3])/8
                ) * 8;
        if ( (tmp < 16) || (tmp>65528) )
        {
            send_tm_lfr_tc_exe_inconsistent( TC, queue_id, 12, TC->dataAndCRC[2]);
            send_tm_lfr_tc_exe_inconsistent( TC, queue_id, 13, TC->dataAndCRC[3]);
            result = LFR_DEFAULT;
        }
        else
        {
            parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char) (tmp >> 8);
            parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char) (tmp     );
        }

        // sy_lfr_n_asm_p
        parameter_dump_packet.sy_lfr_n_asm_p[0] = TC->dataAndCRC[4];
        parameter_dump_packet.sy_lfr_n_asm_p[1] = TC->dataAndCRC[5];

        // sy_lfr_n_bp_p0
        parameter_dump_packet.sy_lfr_n_bp_p0 = TC->dataAndCRC[6];

        // sy_lfr_n_bp_p1
        parameter_dump_packet.sy_lfr_n_bp_p1 = TC->dataAndCRC[7];
    }

    return result;
}

int action_load_burst_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( lfrMode == LFR_MODE_BURST ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        parameter_dump_packet.sy_lfr_b_bp_p0 = TC->dataAndCRC[0];
        parameter_dump_packet.sy_lfr_b_bp_p1 = TC->dataAndCRC[1];

        result = LFR_SUCCESSFUL;
    }

    return result;
}

int action_load_sbm1_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( lfrMode == LFR_MODE_SBM1 ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        parameter_dump_packet.sy_lfr_s1_bp_p0 = TC->dataAndCRC[0];
        parameter_dump_packet.sy_lfr_s1_bp_p1 = TC->dataAndCRC[1];

        result = LFR_SUCCESSFUL;
    }

    return result;
}

int action_load_sbm2_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int result;
    unsigned char lfrMode;

    result = LFR_DEFAULT;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    if ( lfrMode == LFR_MODE_SBM2 ) {
        send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        result = LFR_DEFAULT;
    }
    else {
        parameter_dump_packet.sy_lfr_s2_bp_p0 = TC->dataAndCRC[0];
        parameter_dump_packet.sy_lfr_s2_bp_p1 = TC->dataAndCRC[1];

        result = LFR_SUCCESSFUL;
    }

    return result;
}

int action_dump_par(ccsdsTelecommandPacket_t *TC)
{
    int status;
    // send parameter dump packet
    status = write(fdSPW, (char *) &parameter_dump_packet,
                   PACKET_LENGTH_PARAMETER_DUMP + CCSDS_TC_TM_PACKET_OFFSET + 4);
    if (status == -1)
    {
        PRINTF1("in action_dump *** ERR sending packet, code %d", status)
        status = RTEMS_UNSATISFIED;
    }
    else
    {
        status = RTEMS_SUCCESSFUL;
    }

    return status;
}

int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
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

int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id) {
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
        suspend_science_tasks();
    }

    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF("ERR *** in stop_current_mode *** suspending tasks\n")
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
    rtems_status_code status[6];
    rtems_status_code ret;

    ret = RTEMS_SUCCESSFUL;

    status[0] = rtems_task_suspend( Task_id[TASKID_AVF0] );
    if (status[0] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 0 ERR %d\n", status[0])
    }

    status[1] = rtems_task_suspend( Task_id[TASKID_BPF0] );
    if (status[1] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 1 ERR %d\n", status[1])
    }

    status[2] = rtems_task_suspend( Task_id[TASKID_WFRM] );
    if (status[2] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 2 ERR %d\n", status[2])
    }

    status[3] = rtems_task_suspend( Task_id[TASKID_CWF3] );
    if (status[3] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 3 ERR %d\n", status[3])
    }

    status[4] = rtems_task_suspend( Task_id[TASKID_CWF2] );
    if (status[4] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 4 ERR %d\n", status[4])
    }

    status[5] = rtems_task_suspend( Task_id[TASKID_CWF1] );
    if (status[5] != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in suspend_science_task *** 5 ERR %d\n", status[5])
    }

    if ( (status[0] != RTEMS_SUCCESSFUL) || (status[1] != RTEMS_SUCCESSFUL) || (status[2] != RTEMS_SUCCESSFUL) ||
         (status[3] != RTEMS_SUCCESSFUL) || (status[4] != RTEMS_SUCCESSFUL) || (status[5] != RTEMS_SUCCESSFUL) )
    {
        ret = RTEMS_UNSATISFIED;
    }

    return ret;
}

//****************
// CLOSING ACTIONS

int send_tm_lfr_tc_exe_success(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int ret;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_SUCCESS_t TM;
    unsigned char messageSize;

    ret = LFR_SUCCESSFUL;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    TM.packetSequenceControl[0] = (TM_PACKET_SEQ_CTRL_STANDALONE >> 8);
    TM.packetSequenceControl[1] = (TM_PACKET_SEQ_CTRL_STANDALONE     );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_SUCCESS >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_SUCCESS     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_OK;
    TM.destinationID = TM_DESTINATION_ID_GROUND;    // default destination id
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
    status =  rtems_message_queue_urgent( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_success *** ERR\n")
        ret = LFR_DEFAULT;
    }

    return LFR_SUCCESSFUL;
}

int send_tm_lfr_tc_exe_inconsistent(ccsdsTelecommandPacket_t *TC, rtems_id queue_id,
                                    unsigned char byte_position, unsigned char rcv_value)
{
    int ret;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_INCONSISTENT_t TM;
    unsigned char messageSize;

    ret = LFR_SUCCESSFUL;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    TM.packetSequenceControl[0] = (TM_PACKET_SEQ_CTRL_STANDALONE >> 8);
    TM.packetSequenceControl[1] = (TM_PACKET_SEQ_CTRL_STANDALONE     );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_INCONSISTENT >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_INCONSISTENT     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TM_DESTINATION_ID_GROUND;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FAILURE_CODE_INCONSISTENT >> 8);
    TM.tc_failure_code[1] = (char) (FAILURE_CODE_INCONSISTENT     );
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
    status =  rtems_message_queue_urgent( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_inconsistent *** ERR\n")
        ret = LFR_DEFAULT;
    }

    return LFR_SUCCESSFUL;
}

int send_tm_lfr_tc_exe_not_executable(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int ret;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_NOT_EXECUTABLE_t TM;
    unsigned char messageSize;

    ret = LFR_SUCCESSFUL;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    TM.packetSequenceControl[0] = (TM_PACKET_SEQ_CTRL_STANDALONE >> 8);
    TM.packetSequenceControl[1] = (TM_PACKET_SEQ_CTRL_STANDALONE     );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TM_DESTINATION_ID_GROUND;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FAILURE_CODE_NOT_EXECUTABLE >> 8);
    TM.tc_failure_code[1] = (char) (FAILURE_CODE_NOT_EXECUTABLE     );
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
    status =  rtems_message_queue_urgent( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_not_executable *** ERR\n")
        ret = LFR_DEFAULT;
    }

    return LFR_SUCCESSFUL;
}

int send_tm_lfr_tc_exe_not_implemented(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int ret;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_NOT_IMPLEMENTED_t TM;
    unsigned char messageSize;

    ret = LFR_SUCCESSFUL;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    TM.packetSequenceControl[0] = (TM_PACKET_SEQ_CTRL_STANDALONE >> 8);
    TM.packetSequenceControl[1] = (TM_PACKET_SEQ_CTRL_STANDALONE     );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TM_DESTINATION_ID_GROUND;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FAILURE_CODE_NOT_IMPLEMENTED >> 8);
    TM.tc_failure_code[1] = (char) (FAILURE_CODE_NOT_IMPLEMENTED     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC

    messageSize = PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_urgent( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_not_implemented *** ERR\n")
        ret = LFR_DEFAULT;
    }

    return LFR_SUCCESSFUL;
}

int send_tm_lfr_tc_exe_error(ccsdsTelecommandPacket_t *TC, rtems_id queue_id)
{
    int ret;
    rtems_status_code status;
    Packet_TM_LFR_TC_EXE_ERROR_t TM;
    unsigned char messageSize;

    ret = LFR_SUCCESSFUL;

    TM.targetLogicalAddress = CCSDS_DESTINATION_ID;
    TM.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    TM.reserved = DEFAULT_RESERVED;
    TM.userApplication = CCSDS_USER_APP;
    // PACKET HEADER
    TM.packetID[0] = (unsigned char) (TM_PACKET_ID_TC_EXE >> 8);
    TM.packetID[1] = (unsigned char) (TM_PACKET_ID_TC_EXE     );
    TM.packetSequenceControl[0] = (TM_PACKET_SEQ_CTRL_STANDALONE >> 8);
    TM.packetSequenceControl[1] = (TM_PACKET_SEQ_CTRL_STANDALONE     );
    TM.packetLength[0] = (unsigned char) (PACKET_LENGTH_TC_EXE_ERROR >> 8);
    TM.packetLength[1] = (unsigned char) (PACKET_LENGTH_TC_EXE_ERROR     );
    // DATA FIELD HEADER
    TM.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    TM.serviceType = TM_TYPE_TC_EXE;
    TM.serviceSubType = TM_SUBTYPE_EXE_NOK;
    TM.destinationID = TM_DESTINATION_ID_GROUND;    // default destination id
    TM.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    TM.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    TM.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    TM.time[3] = (unsigned char) (time_management_regs->coarse_time);
    TM.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    TM.time[5] = (unsigned char) (time_management_regs->fine_time);
    //
    TM.tc_failure_code[0] = (char) (FAILURE_CODE_ERROR >> 8);
    TM.tc_failure_code[1] = (char) (FAILURE_CODE_ERROR     );
    TM.telecommand_pkt_id[0] = TC->packetID[0];
    TM.telecommand_pkt_id[1] = TC->packetID[1];
    TM.pkt_seq_control[0] = TC->packetSequenceControl[0];
    TM.pkt_seq_control[1] = TC->packetSequenceControl[1];
    TM.tc_service = TC->serviceType;      // type of the rejected TC
    TM.tc_subtype = TC->serviceSubType;   // subtype of the rejected TC

    messageSize = PACKET_LENGTH_TC_EXE_ERROR + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES;

    // SEND DATA
    status =  rtems_message_queue_urgent( queue_id, &TM, messageSize);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF("in send_tm_lfr_tc_exe_error *** ERR\n")
        ret = LFR_DEFAULT;
    }

    return LFR_SUCCESSFUL;
}

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





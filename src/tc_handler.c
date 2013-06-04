#include <tc_handler.h>
#include <fsw_params.h>

char *DumbMessages[5] = {"default",
                    "timecode_irq_handler",
                    "waveforms_isr",
                    "",
                    ""
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

//*********************
// ACCEPTANCE FUNCTIONS
int TC_checker(ccsdsTelecommandPacket_t *TC, unsigned int tc_len_recv)
{
    int ret = 0;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send;
    TMHeader_t TM_header;
    unsigned int code = 0;
    unsigned char computed_CRC[2];
    char data[ TM_LEN_EXE_CORR + CCSDS_TC_TM_PACKET_OFFSET - TM_HEADER_LEN ];

    GetCRCAsTwoBytes( (unsigned char*) TC->packetID, computed_CRC, tc_len_recv + 5 );
    code = acceptTM( TC, tc_len_recv ) ;
    if ( (code == ILLEGAL_APID) | (code == WRONG_LEN_PACKET) | (code == INCOR_CHECKSUM)
        | (code == ILL_TYPE) | (code == ILL_SUBTYPE) | (code == WRONG_APP_DATA) )
    { // generate TM_LFR_TC_EXE_CORRUPTED
        // BUILD HEADER
        TM_build_header( TM_LFR_TC_EXE_ERR, TM_LEN_EXE_CORR, 0, 0, &TM_header);
        // BUILD DATA
        TM_build_data( TC, data, SID_EXE_CORR, computed_CRC);
        // PREPARE TM SENDING
        spw_ioctl_send.hlen = TM_HEADER_LEN + 4; // + 4 is for the protocole extra header
        spw_ioctl_send.hdr = (char*) &TM_header;
        spw_ioctl_send.dlen = 16;
        spw_ioctl_send.data = data;
        // SEND PACKET
        write_spw(&spw_ioctl_send);
    }
    else { // send valid TC to the action launcher
        status =  rtems_message_queue_send( misc_id[0], TC, tc_len_recv + CCSDS_TC_TM_PACKET_OFFSET + 3);
        ret = -1;
    }
    return ret;
}

unsigned char acceptTM(ccsdsTelecommandPacket_t * TMPacket, unsigned int TC_LEN_RCV)
{
    unsigned char ret = 0;
    unsigned char pid = 0;
    unsigned char category = 0;
    unsigned int length = 0;
    unsigned char packetType = 0;
    unsigned char packetSubtype = 0;
    unsigned char * CCSDSContent = NULL;

    // APID check *** APID on 2 bytes
    pid = ((TMPacket->packetID[0] & 0x07)<<4) + ( (TMPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TMPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    length = (TMPacket->packetLength[0] * 256) + TMPacket->packetLength[1];
    packetType = TMPacket->dataFieldHeader[1];
    packetSubtype = TMPacket->dataFieldHeader[2];

    if (pid!=CCSDS_PROCESS_ID) {
        ret = ILLEGAL_APID;
    }
    else if (category!=CCSDS_PACKET_CATEGORY) {
        ret = ILLEGAL_APID;
    }
    else if (length != TC_LEN_RCV ) {   // packet length check
        ret = WRONG_LEN_PACKET; // LEN RCV != SIZE FIELD
    }
    else if (length >= CCSDS_TC_PKT_MAX_SIZE) {
        ret = WRONG_LEN_PACKET; // check that the packet does not exceed the MAX size
    }
    else if (packetType == TC_TYPE_GEN){ // service type, subtype and packet length check
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
    else if (packetType == TC_TYPE_TIME){
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
        CCSDSContent = (unsigned char*) TMPacket->packetID;
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

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int packetLength,
                              unsigned int coarseTime, unsigned int fineTime, TMHeader_t *TMHeader)
{
    TMHeader->targetLogicalAddress = CCSDS_DESTINATION_ID;
    TMHeader->protocolIdentifier = 0x02;
    TMHeader->reserved = 0x00;
    TMHeader->userApplication = 0x00;
    TMHeader->packetID[0] = 0x0c;
    TMHeader->packetSequenceControl[0] = 0xc0;
    TMHeader->packetSequenceControl[1] = 0x00;
    TMHeader->packetLength[0] = (unsigned char) (packetLength>>8);
    TMHeader->packetLength[1] = (unsigned char) packetLength;
    TMHeader->dataFieldHeader[0] = 0x10;
    TMHeader->dataFieldHeader[3] = CCSDS_DESTINATION_ID;
    switch (tm_type){
        case(TM_LFR_TC_EXE_OK):
            TMHeader->packetID[1] = 0xc1;
            TMHeader->dataFieldHeader[1] = 1; // type
            TMHeader->dataFieldHeader[2] = 7; // subtype
            break;
        case(TM_LFR_TC_EXE_ERR):
            TMHeader->packetID[1] = 0xc1;
            TMHeader->dataFieldHeader[1] = 1; // type
            TMHeader->dataFieldHeader[2] = 8; // subtype
            break;
        case(TM_LFR_HK):
            TMHeader->packetID[1] = 0xc4;
            TMHeader->dataFieldHeader[1] = 3;  // type
            TMHeader->dataFieldHeader[2] = 25; // subtype
            break;
        case(TM_LFR_SCI):
            TMHeader->packetID[1] = 0xcc;
            TMHeader->dataFieldHeader[1] = 21;  // type
            TMHeader->dataFieldHeader[2] = 3; // subtype
            break;
        case(TM_LFR_SCI_SBM):
            TMHeader->packetID[1] = 0xfc;
            TMHeader->dataFieldHeader[1] = 21;  // type
            TMHeader->dataFieldHeader[2] = 3; // subtype
            break;
        case(TM_LFR_PAR_DUMP):
            TMHeader->packetID[1] = 0xc9;
            TMHeader->dataFieldHeader[1] = 181;  // type
            TMHeader->dataFieldHeader[2] = 31; // subtype
            break;
        default:
            return 0;
    }
    TMHeader->dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
    TMHeader->dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
    TMHeader->dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
    TMHeader->dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
    TMHeader->dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
    TMHeader->dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
    return 1;
}

unsigned char TM_build_header_bis( enum TM_TYPE tm_type, unsigned int packetLength,
                              unsigned int coarseTime, unsigned int fineTime, Packet_TM_LFR_TC_EXE_t *packet)
{

    packet->targetLogicalAddress = CCSDS_DESTINATION_ID;
    packet->protocolIdentifier = 0x02;
    packet->reserved = 0x00;
    packet->userApplication = 0x00;
    packet->packetID[0] = 0x0c;
    packet->packetSequenceControl[0] = 0xc0;
    packet->packetSequenceControl[1] = 0x00;
    packet->packetLength[0] = (unsigned char) (packetLength>>8);
    packet->packetLength[1] = (unsigned char) packetLength;
    packet->dataFieldHeader[0] = 0x10;
    packet->dataFieldHeader[3] = CCSDS_DESTINATION_ID;
    switch (tm_type){
        case(TM_LFR_TC_EXE_OK):
            packet->packetID[1] = 0xc1;
            packet->dataFieldHeader[1] = 1; // type
            packet->dataFieldHeader[2] = 7; // subtype
            break;
        case(TM_LFR_TC_EXE_ERR):
            packet->packetID[1] = 0xc1;
            packet->dataFieldHeader[1] = 1; // type
            packet->dataFieldHeader[2] = 8; // subtype
            break;
        case(TM_LFR_HK):
            packet->packetID[1] = 0xc4;
            packet->dataFieldHeader[1] = 3;  // type
            packet->dataFieldHeader[2] = 25; // subtype
            break;
        case(TM_LFR_SCI):
            packet->packetID[1] = 0xcc;
            packet->dataFieldHeader[1] = 21;  // type
            packet->dataFieldHeader[2] = 3; // subtype
            break;
        case(TM_LFR_SCI_SBM):
            packet->packetID[1] = 0xfc;
            packet->dataFieldHeader[1] = 21;  // type
            packet->dataFieldHeader[2] = 3; // subtype
            break;
        case(TM_LFR_PAR_DUMP):
            packet->packetID[1] = 0xc9;
            packet->dataFieldHeader[1] = 181;  // type
            packet->dataFieldHeader[2] = 31; // subtype
            break;
        default:
            return 0;
    }
    packet->dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
    packet->dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
    packet->dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
    packet->dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
    packet->dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
    packet->dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
    return 1;
}

unsigned char TM_build_data(ccsdsTelecommandPacket_t *TC, char* data, unsigned int SID, unsigned char *computed_CRC)
{
    unsigned int packetLength;
    packetLength = (TC->packetLength[0] * 256) + TC->packetLength[1];
    switch (SID){
        case (SID_NOT_EXE):
            break;
        case (SID_NOT_IMP):
            data[0] = 0x9c;
            data[1] = 0x42;
            data[2] = TC->packetID[0];
            data[3] = TC->packetID[1];
            data[4] = TC->packetSequenceControl[0];
            data[5] = TC->packetSequenceControl[1];
            data[6] = TC->dataFieldHeader[1]; // type
            data[7] = TC->dataFieldHeader[2]; // subtype
            break;
        case (SID_EXE_ERR):
            break;
        case (SID_EXE_CORR):
            data[0] = 0x9c;
            data[1] = 0x45;
            data[2] = TC->packetID[0];
            data[3] = TC->packetID[1];
            data[4] = TC->packetSequenceControl[0];
            data[5] = TC->packetSequenceControl[1];
            data[6] = TC->dataFieldHeader[1]; // type
            data[7] = TC->dataFieldHeader[2]; // subtype
            data[8] = currentTC_LEN_RCV[0];
            data[9] = currentTC_LEN_RCV[1];
            data[10] = TC->packetLength[0];
            data[11] = TC->packetLength[1];
            data[12] = TC->dataAndCRC[packetLength];
            data[13] = TC->dataAndCRC[packetLength+1];
            data[14] = computed_CRC[0];
            data[15] = computed_CRC[1];
            break;
        default:
            return 0;
    }
    return 1;
}

int create_message_queue( void )
{
    rtems_status_code status;
    misc_name[0] = rtems_build_name( 'Q', 'U', 'E', 'U' );
    status = rtems_message_queue_create( misc_name[0], ACTION_MSG_QUEUE_COUNT, CCSDS_TC_PKT_MAX_SIZE,
                                                 RTEMS_FIFO | RTEMS_LOCAL, &misc_id[0] );
    if (status!=RTEMS_SUCCESSFUL) {
        PRINTF("in create_message_queue *** error creating message queue\n")
    }

    return 0;
}

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused )
{
    int len = 0;
    unsigned int i = 0;
    unsigned int data_length = 0;
    ccsdsTelecommandPacket_t currentTC;
    char data[100];

    for(i=0; i<100; i++) data[i] = 0;

    PRINTF("in RECV *** \n")

    while(1)
    {
        len = read(fdSPW, (char*) &currentTC, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
        if (len == -1){ // error during the read call
            PRINTF("In RECV *** last read call returned -1\n")
            if (rtems_event_send( Task_id[TASKID_SPIQ], SPW_LINKERR_EVENT ) != RTEMS_SUCCESSFUL) {
                PRINTF("IN RECV *** Error: rtems_event_send SPW_LINKERR_EVENT\n")
            }
            if (rtems_task_suspend(RTEMS_SELF) != RTEMS_SUCCESSFUL) {
                PRINTF("In RECV *** Error: rtems_task_suspend(RTEMS_SELF)\n")
            }
        }
        else {
            PRINTF1("Got pck of length %d\n", len+1)
            if ( (len+1) < CCSDS_TC_PKT_MIN_SIZE ) {
                PRINTF("In RECV *** packet lenght too short\n")
            }
            else {
                currentTC_LEN_RCV[0] = 0x00;
                currentTC_LEN_RCV[1] = (unsigned char) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); //  build the corresponding packet size field
                currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) (len - CCSDS_TC_TM_PACKET_OFFSET - 3); // => -3 is for Prot ID, Reserved and User App bytes
                // CHECK THE TC AND BUILD THE APPROPRIATE TM
                data_length = TC_checker(&currentTC, currentTC_LEN_RCV_AsUnsignedInt);
                if (data_length!=-1)
                {
                }
            }
        }
    }
}

rtems_task actn_task( rtems_task_argument unused )
{
    int result = 0;
    rtems_status_code status;           // RTEMS status code
    ccsdsTelecommandPacket_t TC;        // TC sent to the ACTN task
    size_t size;                        // size of the incoming TC packet
    unsigned char subtype = 0;          // subtype of the current TC packet

    PRINTF("in ACTN *** \n")

    while(1)
    {
        status = rtems_message_queue_receive(misc_id[0], (char*) &TC, &size,
                                             RTEMS_WAIT, RTEMS_NO_TIMEOUT);
        if (status!=RTEMS_SUCCESSFUL) PRINTF1("in task ACTN *** error receiving a message, code %d \n", status)
        else
        {
            subtype = TC.dataFieldHeader[2];
            switch(subtype)
            {
                case TC_SUBTYPE_RESET:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_LOAD_COMM:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_LOAD_NORM:
                    result = action_load_norm( &TC );
                    send_tm_lfr_tc_exe_success( &TC );
                    break;
                    //
                case TC_SUBTYPE_LOAD_BURST:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM1:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_LOAD_SBM2:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_DUMP:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_ENTER:
                    result = action_enter( &TC );
                    send_tm_lfr_tc_exe_success( &TC );
                    break;
                    //
                case TC_SUBTYPE_UPDT_INFO:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_EN_CAL:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_DIS_CAL:
                    result = action_default( &TC );
                    break;
                    //
                case TC_SUBTYPE_UPDT_TIME:
                    result = action_updt_time( &TC );
                    send_tm_lfr_tc_exe_success( &TC );
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
    unsigned int coarse_time = 0;
    unsigned int fine_time = 0;
    unsigned int indice = 0;
    unsigned int shiftedIndice = 0;
    rtems_event_set event_out;

    PRINTF("in DUMB *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_1, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        indice = 0;
        shiftedIndice = (unsigned int) event_out;
        while(!(shiftedIndice & 0x0001))
        {
            shiftedIndice = shiftedIndice >> 1;
            indice++;
        }
        coarse_time = time_management_regs->coarse_time;
        fine_time = time_management_regs->fine_time;
        printf("in DUMB *** time = coarse: %x, fine: %x, %s\n", coarse_time, fine_time, DumbMessages[indice]);
    }
}

//***********
// TC ACTIONS

int action_default(ccsdsTelecommandPacket_t *TC)
{
    Packet_TM_LFR_TC_EXE_t packet;

    TM_build_header_bis( TM_LFR_TC_EXE_ERR, TM_LEN_NOT_IMP,
                        time_management_regs->coarse_time, time_management_regs->fine_time, &packet);

    packet.data[0] = 0x9c;
    packet.data[1] = 0x42;
    packet.data[2] = TC->packetID[0];
    packet.data[3] = TC->packetID[1];
    packet.data[4] = TC->packetSequenceControl[0];
    packet.data[5] = TC->packetSequenceControl[1];
    packet.data[6] = TC->dataFieldHeader[1]; // type
    packet.data[7] = TC->dataFieldHeader[2]; // subtype

    // SEND DATA
    if (write ( fdSPW, &packet, LEN_TM_LFR_TC_EXE_NOT_IMP)==-1) {
        PRINTF("ERR *** in action_default *** send TM packet\n");
    }

    return 0;
}

int action_enter(ccsdsTelecommandPacket_t *TC)
{
    unsigned char lfr_mode = TC->dataAndCRC[1];
    printf("enter mode %d\n", lfr_mode);
    switch(lfr_mode)
    {
        //********
        // STANDBY
        case(LFR_MODE_STANDBY):
            stop_current_mode();
            break;

        //******
        // NORMAL
        case(LFR_MODE_NORMAL):
            stop_current_mode();
            enter_normal_mode();
            break;

        //******
        // BURST
        case(LFR_MODE_BURST):
            stop_current_mode();
            break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
            stop_current_mode();
            enter_sbm1_mode();
            break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
            stop_current_mode();
            break;

        //********
        // DEFAULT
        default:
            stop_current_mode();
            break;
    }
    return 0;
}

int stop_current_mode()
{
    // mask all IRQ lines related to signal processing
    LEON_Mask_interrupt( IRQ_WF );                  // mask waveform interrupt (coming from the timer VHDL IP)
    LEON_Mask_interrupt( IRQ_SM );                  // mask spectral matrices interrupt (coming from the timer VHDL IP)
    LEON_Mask_interrupt( IRQ_WAVEFORM_PICKER );     // mask waveform picker interrupt

    // clear all pending interruptions related to signal processing
    LEON_Clear_interrupt( IRQ_WF );                  // clear waveform interrupt (coming from the timer VHDL IP)
    LEON_Clear_interrupt( IRQ_SM );                  // clear spectral matrices interrupt (coming from the timer VHDL IP)
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );     // clear waveform picker interrupt

    // suspend several tasks
    suspend_if_needed( Task_id[TASKID_AVF0] );
    suspend_if_needed( Task_id[TASKID_BPF0] );
    suspend_if_needed( Task_id[TASKID_WFRM] );

    // initialize the registers
    waveform_picker_regs->burst_enable = 0x00;      // initialize

    return 0;
}

int enter_normal_mode()
{

    restart_if_needed( Task_id[TASKID_AVF0] );
    restart_if_needed( Task_id[TASKID_BPF0] );
    restart_if_needed( Task_id[TASKID_WFRM] );

#ifdef GSA
    LEON_Unmask_interrupt( IRQ_WF );
#else
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    waveform_picker_regs->burst_enable = 0x07;
    waveform_picker_regs->status = 0x00;
#endif
    LEON_Unmask_interrupt( IRQ_SM );
    return 0;
}

int enter_sbm1_mode()
{
    restart_if_needed( Task_id[TASKID_AVF0] );
    restart_if_needed( Task_id[TASKID_BPF0] );
    restart_if_needed( Task_id[TASKID_WFRM] );

#ifdef GSA
#else
    LEON_Clear_interrupt( IRQ_WAVEFORM_PICKER );
    LEON_Unmask_interrupt( IRQ_WAVEFORM_PICKER );
    waveform_picker_regs->burst_enable = 0x20;  // [0010 0000] burst f2, f1, f0 enable f3 f2 f1 f0
    waveform_picker_regs->burst_enable =  waveform_picker_regs->burst_enable | 0x02;
    waveform_picker_regs->status = 0x00;
#endif
    //LEON_Unmask_interrupt( IRQ_SM );
    return 0;
}

int action_load_norm(ccsdsTelecommandPacket_t *TC)
{
    param_norm.sy_lfr_n_swf_l = (TC->dataAndCRC[0] * 256) + TC->dataAndCRC[1];
    param_norm.sy_lfr_n_swf_p = (TC->dataAndCRC[2] * 256) + TC->dataAndCRC[3];
    param_norm.sy_lfr_n_asm_p = (TC->dataAndCRC[4] * 256) + TC->dataAndCRC[5];
    param_norm.sy_lfr_n_bp_p0 = TC->dataAndCRC[6];
    param_norm.sy_lfr_n_bp_p1 = TC->dataAndCRC[7];
    /*printf("sy_lfr_n_ => swf_l %d, swf_p %d, asm_p %d, bsp_p0 %d, bsp_p1 %d\n",
           param_norm.sy_lfr_n_swf_l, param_norm.sy_lfr_n_swf_p,
           param_norm.sy_lfr_n_asm_p, param_norm.sy_lfr_n_bp_p0, param_norm.sy_lfr_n_bp_p1);*/
    return 0;
}

int action_updt_time(ccsdsTelecommandPacket_t *TC)
{
    time_management_regs->coarse_time_load = (TC->dataAndCRC[0] << 24)
                                                + (TC->dataAndCRC[1] << 16)
                                                + (TC->dataAndCRC[2] << 8)
                                                + TC->dataAndCRC[3];
    //time_management_regs->ctrl = time_management_regs->ctrl | 1;
    return 0;
}

int send_tm_lfr_tc_exe_success(ccsdsTelecommandPacket_t *TC)
{
    TMHeader_t TM_header;
    char data[4];
    spw_ioctl_pkt_send spw_ioctl_send;

    TM_build_header( TM_LFR_TC_EXE_OK, TM_LEN_EXE,
                              time_management_regs->coarse_time, time_management_regs->fine_time, &TM_header);

    data[0] = TC->packetID[0];
    data[1] = TC->packetID[1];
    data[2] = TC->packetSequenceControl[0];
    data[3] = TC->packetSequenceControl[1];

    // filling the structure for the spacewire transmission
    spw_ioctl_send.hlen = TM_HEADER_LEN + 3; // + 4 is for the protocole extra header
    spw_ioctl_send.hdr = (char*) &TM_header;
    spw_ioctl_send.dlen = 3;
    spw_ioctl_send.data = data;

    // SEND DATA
    write_spw(&spw_ioctl_send);

    return 0;
}

rtems_status_code restart_if_needed(rtems_id id)
{
    rtems_status_code status;

    status = rtems_task_is_suspended( id );

    if (status==RTEMS_SUCCESSFUL) {
        status = rtems_task_restart( id, 0 );
        if (status!=RTEMS_SUCCESSFUL) {
            PRINTF1("in restart_if_needed *** Error restarting with id %d\n", (int) id)
        }
    }

    return status;
}

rtems_status_code suspend_if_needed(rtems_id id)
{
    rtems_status_code status;

    status = rtems_task_is_suspended( id );

    if (status!=RTEMS_SUCCESSFUL) {
        status = rtems_task_suspend( id );
        if (status!=RTEMS_SUCCESSFUL) {
            PRINTF1("in suspend_if_needed *** Error suspending task with id %d\n", (int) id)
        }
    }

    return status;
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





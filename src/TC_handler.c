#include <TC_handler.h>
#include <FSW-config.h>

char *errorCCSDSMsg[8] = { "ILLEGAL_APID 0",
                            "WRONG_LEN_PACKET 1",
                            "INCOR_CHECKSUM 2",
                            "ILL_TYPE 3",
                            "ILL_SUBTYPE 4",
                            "WRONG_APP_DATA 5",
                            "WRONG_CMD_CODE 6",
                            "CCSDS_TM_VALID 7"
};

unsigned int Crc_opt( unsigned char D, unsigned int Chk)
{
    return(((Chk << 8) & 0xff00)^lookUpTableForCRC [(((Chk >> 8)^D) & 0x00ff)]);
}

void InitLookUpTableForCRC()
{
    unsigned int i, tmp;
    for (i=0; i<256; i++)
    {
        tmp = 0;
        if((i & 1) != 0) tmp = tmp ^ 0x1021;
        if((i & 2) != 0) tmp = tmp ^ 0x2042;
        if((i & 4) != 0) tmp = tmp ^ 0x4084;
        if((i & 8) != 0) tmp = tmp ^ 0x8108;
        if((i & 16) != 0) tmp = tmp ^ 0x1231;
        if((i & 32) != 0) tmp = tmp ^ 0x2462;
        if((i & 64) != 0) tmp = tmp ^ 0x48c4;
        if((i & 128) != 0) tmp = tmp ^ 0x9188;
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

unsigned int TC_checker(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV,
                         TMHeader_t *TM_Header, unsigned int *hlen, char *data)
{
    unsigned int code = 0;
    unsigned int data_length = 0;
    unsigned char computed_CRC[2];
    unsigned char subtype = 0;

    subtype = TC->dataFieldHeader[2];
    GetCRCAsTwoBytes( (unsigned char*) TC->packetID, computed_CRC, TC_LEN_RCV + 5 );
    code = acceptTM( TC, TC_LEN_RCV ) ;
    //PRINTF1("in TC_checker *** %s\n", errorCCSDSMsg[code]);
    if ( (code == 0) | (code == 1) | (code == 2)
        | (code == 3) | (code == 4) | (code == 5) )
    { // generate TM_LFR_TC_EXE_CORRUPTED
        // BUILD HEADER
        TM_build_header( TM_LFR_TC_EXE_ERR, SID_EXE_CORR, TM_LEN_EXE_CORR, 0, 0, TM_Header);
        // BUILD DATA
        TM_build_data( TC, data, SID_EXE_CORR, computed_CRC);
        data_length = TM_LEN_EXE_CORR + CCSDS_TC_TM_PACKET_OFFSET - TM_HEADER_LEN;
    }
    if (subtype == SID_TC_UPDT_TIME){
        // BUILD HEADER
        TM_build_header( TM_LFR_TC_EXE_OK, SID_DEFAULT, TM_LEN_EXE, 0, 0, TM_Header);
        // BUILD DATA
        TM_build_data( TC, data, SID_DEFAULT, computed_CRC);
        data_length = TM_LEN_NOT_IMP + CCSDS_TC_TM_PACKET_OFFSET - TM_HEADER_LEN;
    }
    else { // TM_LFR_TC_EXE_NOT_IMPLEMENTED
        // BUILD HEADER
        TM_build_header( TM_LFR_TC_EXE_ERR, SID_NOT_IMP, TM_LEN_NOT_IMP, 0, 0, TM_Header);
        // BUILD DATA
        TM_build_data( TC, data, SID_NOT_IMP, computed_CRC);
        data_length = TM_LEN_NOT_IMP + CCSDS_TC_TM_PACKET_OFFSET - TM_HEADER_LEN;
    }

    return data_length;
}

unsigned char acceptTM(ccsdsTelecommandPacket_t * TMPacket, unsigned int TC_LEN_RCV)
{
    unsigned char pid = 0;
    unsigned char category = 0;
    unsigned int length = 0;
    unsigned char packetType = 0;
    unsigned char packetSubtype = 0;
    unsigned char * CCSDSContent;

    // APID check *** APID on 2 bytes
    pid = ((TMPacket->packetID[0] & 0x07)<<4) + ( (TMPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TMPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    if (pid!=CCSDS_PROCESS_ID) return ILLEGAL_APID;
    if (category!=CCSDS_PACKET_CATEGORY) return ILLEGAL_APID;

    // packet length check
    length = TMPacket->packetLength[0] * 256 + TMPacket->packetLength[1];
    if (length != TC_LEN_RCV ) return WRONG_LEN_PACKET; // LEN RCV != SIZE FIELD
    if (length >= CCSDS_TC_PKT_MAX_SIZE) return WRONG_LEN_PACKET; // check that the packet does not exceed the MAX size

    packetType = TMPacket->dataFieldHeader[1];
    packetSubtype = TMPacket->dataFieldHeader[2];
    // service type, subtype and packet length check
    if (packetType == 181){
        switch(packetSubtype){ //subtype, autorized values are 3, 20, 21, 24, 27, 28, 30, 40, 50, 60, 61
            case 3:
                if (length!=(12-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 20:
                if (length!=(14-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 21:
                if (length!=(20-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 24:
                if (length!=(14-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 27:
                if (length!=(14-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 28:
                if (length!=(14-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 30:
                if (length!=(12-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 40:
                if (length!=(20-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 50:
                if (length!=(50-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 60:
                if (length!=(12-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            case 61:
                if (length!=(12-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
                break;
            default:
                return ILL_SUBTYPE;
                break;
            }
    }
    else if (packetType == 9){
        if (packetSubtype!=129) return ILL_SUBTYPE;
        if (length!=(18-CCSDS_TC_TM_PACKET_OFFSET)) return WRONG_LEN_PACKET;
    }
    else return ILL_TYPE;

    // source ID check // Source ID not documented in the ICD

    // packet error control, CRC check
    CCSDSContent = (unsigned char*) TMPacket->packetID;
    GetCRCAsTwoBytes(CCSDSContent, currentTC_COMPUTED_CRC, length + CCSDS_TC_TM_PACKET_OFFSET - 2); // 2 CRC bytes removed from the calculation of the CRC
    if (currentTC_COMPUTED_CRC[0] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -2]) return INCOR_CHECKSUM;
    if (currentTC_COMPUTED_CRC[1] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -1]) return INCOR_CHECKSUM;

    return CCSDS_TM_VALID;
}

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int SID, unsigned int packetLength,
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
    /*TMHeader->dataFieldHeader[4] = (unsigned char) (coarseTime>>24);
    TMHeader->dataFieldHeader[5] = (unsigned char) (coarseTime>>16);
    TMHeader->dataFieldHeader[6] = (unsigned char) (coarseTime>>8);
    TMHeader->dataFieldHeader[7] = (unsigned char) (coarseTime);
    TMHeader->dataFieldHeader[8] = (unsigned char) (fineTime>>8);
    TMHeader->dataFieldHeader[9] = (unsigned char) (fineTime);*/
    TMHeader->dataFieldHeader[4] = 0xaa;
    TMHeader->dataFieldHeader[5] = 0xbb;
    TMHeader->dataFieldHeader[6] = 0xcc;
    TMHeader->dataFieldHeader[7] = 0xdd;
    TMHeader->dataFieldHeader[8] = 0xee;
    TMHeader->dataFieldHeader[9] = 0xff;
    return 1;
}

unsigned char TM_build_data(ccsdsTelecommandPacket_t *TC, char* data, unsigned int SID, unsigned char *computed_CRC)
{
    unsigned int packetLength;
    packetLength = TC->packetLength[0] * 256 + TC->packetLength[1];
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





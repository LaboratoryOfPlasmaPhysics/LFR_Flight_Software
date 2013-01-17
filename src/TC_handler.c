#include <TC_handler.h>

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

void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData){
    unsigned int Chk;
    int j;
    Chk = 0xffff; // reset the syndrom to all ones
    for (j=0; j<sizeOfData; j++) {
        Chk = Crc_opt(data[j], Chk);
    }
    crcAsTwoBytes[0] = (unsigned char) (Chk >> 8);
    crcAsTwoBytes[1] = (unsigned char) (Chk & 0x00ff);
}

unsigned char TM_checker(ccsdsTelecommandPacket_t * TMPacket)
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
    if (length != (currentTC_LEN_RCV[0] * 256 + currentTC_LEN_RCV[1]) ) return WRONG_LEN_PACKET; // LEN RCV != SIZE FIELD
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

unsigned char TM_acceptance_generator(ccsdsTelecommandPacket_t * TCPacket, unsigned int code, ccsdsTelemetryPacket_t * TMPacket)
{
    unsigned int packetLength = 0;
    packetLength = TMPacket->packetLength[0] * 256 + TMPacket->packetLength[1];
    if ( (code == 0) | (code == 1) | (code == 2)
        | (code == 3) | (code == 4) | (code == 5) )
        { // generated TM_LFR_TC_EXE_CORRUPTED
            TMPacket->targetLogicalAddress = CCSDS_DESTINATION_ID;
            TMPacket->protocolIdentifier = 0x02;
            TMPacket->reserved = 0x00;
            TMPacket->userApplication = 0x00;
            //
            TMPacket->packetID[0] = 0x0c;
            TMPacket->packetID[1] = 0xc1;
            TMPacket->packetSequenceControl[0] = 0xc0;
            TMPacket->packetSequenceControl[1] = 0x00;
            TMPacket->packetLength[0] = 0x00;
            TMPacket->packetLength[1] = SIZE_TM_LFR_TC_EXE_CORRUPTED-CCSDS_TC_TM_PACKET_OFFSET;
            //
            TMPacket->dataFieldHeader[0] = 0x10;
            TMPacket->dataFieldHeader[1] = 0x01; // service type
            TMPacket->dataFieldHeader[2] = 0x08; // service subtype
            TMPacket->dataFieldHeader[3] = CCSDS_DESTINATION_ID;
            TMPacket->dataFieldHeader[4] = 0x00; // time
            TMPacket->dataFieldHeader[5] = 0x00; // time
            TMPacket->dataFieldHeader[6] = 0x00; // time
            TMPacket->dataFieldHeader[7] = 0x00; // time
            TMPacket->dataFieldHeader[8] = 0x00; // time
            TMPacket->dataFieldHeader[9] = 0x00; // time
            //
            TMPacket->data[0] = 0x9c; // failure code
            TMPacket->data[1] = 0x45; // failure code
            TMPacket->data[2] = TCPacket->packetID[0];
            TMPacket->data[3] = TCPacket->packetID[1];
            TMPacket->data[4] = TCPacket->packetSequenceControl[0];
            TMPacket->data[5] = TCPacket->packetSequenceControl[1];
            TMPacket->data[6] = TCPacket->dataFieldHeader[1]; // type
            TMPacket->data[7] = TCPacket->dataFieldHeader[2]; // subtype
            TMPacket->data[8] = currentTC_LEN_RCV[0];
            TMPacket->data[9] = currentTC_LEN_RCV[1];
            TMPacket->data[10] = TCPacket->packetLength[0];
            TMPacket->data[11] = TCPacket->packetLength[1];
            TMPacket->data[12] = TCPacket->dataAndCRC[packetLength];
            TMPacket->data[13] = TCPacket->dataAndCRC[packetLength+1];
            TMPacket->data[14] = currentTC_COMPUTED_CRC[0];
            TMPacket->data[15] = currentTC_COMPUTED_CRC[1];
            //
            currentTM_length = SIZE_TM_LFR_TC_EXE_CORRUPTED-CCSDS_TC_TM_PACKET_OFFSET;
        }
    else return 0;
    return 1;
}

unsigned char TM_not_implemented_generator(ccsdsTelecommandPacket_t * TCPacket, ccsdsTelemetryPacket_t * TMPacket)
{
    TMPacket->targetLogicalAddress = CCSDS_DESTINATION_ID;
    TMPacket->protocolIdentifier = 0x02;
    TMPacket->reserved = 0x00;
    TMPacket->userApplication = 0x00;
    //
    TMPacket->packetID[0] = 0x0c;
    TMPacket->packetID[1] = 0xc1;
    TMPacket->packetSequenceControl[0] = 0xc0;
    TMPacket->packetSequenceControl[1] = 0x00;
    TMPacket->packetLength[0] = 0x00;
    TMPacket->packetLength[1] = SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED-CCSDS_TC_TM_PACKET_OFFSET;
    //
    TMPacket->dataFieldHeader[0] = 0x10;
    TMPacket->dataFieldHeader[1] = 0x01; // service type
    TMPacket->dataFieldHeader[2] = 0x08; // service subtype
    TMPacket->dataFieldHeader[3] = CCSDS_DESTINATION_ID;
    TMPacket->dataFieldHeader[4] = 0x00; // time
    TMPacket->dataFieldHeader[5] = 0x00; // time
    TMPacket->dataFieldHeader[6] = 0x00; // time
    TMPacket->dataFieldHeader[7] = 0x00; // time
    TMPacket->dataFieldHeader[8] = 0x00; // time
    TMPacket->dataFieldHeader[9] = 0x00; // time
    //
    TMPacket->data[0] = 0x9c; // failure code
    TMPacket->data[1] = 0x42; // failure code
    TMPacket->data[2] = TCPacket->packetID[0];
    TMPacket->data[3] = TCPacket->packetID[1];
    TMPacket->data[4] = TCPacket->packetSequenceControl[0];
    TMPacket->data[5] = TCPacket->packetSequenceControl[1];
    TMPacket->data[6] = TCPacket->dataFieldHeader[1]; // type
    TMPacket->data[7] = TCPacket->dataFieldHeader[2]; // subtype
    //
    currentTM_length = SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED-CCSDS_TC_TM_PACKET_OFFSET;

    return 1;
}

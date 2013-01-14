#include <..\header\TC_handler.h>

char TM_checker(ccsdsTelecommandSourcePacketHeader_t * TMPacket)
{
    unsigned char pid = 0;
    unsigned char category = 0;
    unsigned int length = 0;

    // APID check *** APID on 2 bytes
    pid = ((TMPacket->packetID[0] & 0x07)<<4) + ( (TMPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TMPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    if (pid!=CCSDS_PROCESS_ID) return CCSDS_ERR_PID;
    if (category!=CCSDS_PACKET_CATEGORY) return CCSDS_ERR_CAT;

    // packet length check
    length = TMPacket->packetLength[1] + TMPacket->packetLength[0] * 256;
    if (category<=CCSDS_TELECOMMAND_MAX_PACKET_LENGTH) return CCSDS_ERR_LENGTH; // check that the packet does not exceed the MAX size

    // service type, subtype and packet length check
    switch(TMPacket->dataFieldHeader[1]) { // service type, authorized values are 181 and 9
        case 181:
            switch(TMPacket->dataFieldHeader[2]){ //subtype, autorized values are 3, 20, 21, 24, 27, 28, 30, 40, 50, 60, 61
                case 3:
                    if (length!=12) return CCSDS_ERR_LENGTH;
                    break;
                case 20:
                    if (length!=14) return CCSDS_ERR_LENGTH;
                    break;
                case 21:
                    if (length!=20) return CCSDS_ERR_LENGTH;
                    break;
                case 24:
                    if (length!=14) return CCSDS_ERR_LENGTH;
                    break;
                case 27:
                    if (length!=14) return CCSDS_ERR_LENGTH;
                    break;
                case 28:
                    if (length!=14) return CCSDS_ERR_LENGTH;
                    break;
                case 30:
                    if (length!=12) return CCSDS_ERR_LENGTH;
                    break;
                case 40:
                    if (length!=20) return CCSDS_ERR_LENGTH;
                    break;
                case 50:
                    if (length!=50) return CCSDS_ERR_LENGTH;
                    break;
                case 60:
                    if (length!=12) return CCSDS_ERR_LENGTH;
                    break;
                case 61:
                    if (length!=12) return CCSDS_ERR_LENGTH;
                    break;
                default:
                    return CCSDS_ERR_SUBTYPE;
                    break;
            }
        case 9:
            if (TMPacket->dataFieldHeader[2]==129) return CCSDS_ERR_SUBTYPE;
            if (length!=18) return CCSDS_ERR_LENGTH;
            break;
        default:
            return CCSDS_ERR_TYPE;
            break;
    }

    // source ID check // Source ID not documented in the ICD

    // packet error control, CRC check

    return CCSDS_TM_VALID;
}

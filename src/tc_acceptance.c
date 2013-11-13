/** Functions related to TeleCommand acceptance.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle TeleCommands parsing.\n
 *
 */

#include "tc_acceptance.h"

unsigned int lookUpTableForCRC[256];

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt( unsigned char D, unsigned int Chk)
{
    /** This function generate the CRC for one byte and returns the value of the new syndrome.
     *
     * @param D is the current byte of data.
     * @param Chk is the current syndrom value.
     *
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

//*********************
// ACCEPTANCE FUNCTIONS
int tc_parser(ccsdsTelecommandPacket_t * TCPacket, unsigned int TC_LEN_RCV, unsigned char *computed_CRC)
{
    /** This function parses TeleCommands.
     *
     * @param TC points to the TeleCommand that will be parsed.
     * @param TC_LEN_RCV is the received packet length.
     *
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
    int status_crc;
    unsigned char pid;
    unsigned char category;
    unsigned int length;
    unsigned char packetType;
    unsigned char packetSubtype;
    unsigned char sid;

    status = CCSDS_TM_VALID;

    // APID check *** APID on 2 bytes
    pid = ((TCPacket->packetID[0] & 0x07)<<4) + ( (TCPacket->packetID[1]>>4) & 0x0f );   // PID = 11 *** 7 bits xxxxx210 7654xxxx
    category = (TCPacket->packetID[1] & 0x0f);         // PACKET_CATEGORY = 12 *** 4 bits xxxxxxxx xxxx3210
    length = (TCPacket->packetLength[0] * 256) + TCPacket->packetLength[1];
    packetType = TCPacket->serviceType;
    packetSubtype = TCPacket->serviceSubType;
    sid = TCPacket->sourceID;

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
            status = WRONG_LEN_PKT;
        }
    }
    if (status == CCSDS_TM_VALID)   // CHECK THAT THE PACKET DOES NOT EXCEED THE MAX SIZE
    {
        if ( length >= CCSDS_TC_PKT_MAX_SIZE ) {
            status = WRONG_LEN_PKT;
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
    if (status == CCSDS_TM_VALID)   // CHECK THE SID
    {
        status = tc_check_sid( sid );
    }
    if (status == CCSDS_TM_VALID)   // CHECK THE SUBTYPE AND LENGTH COMPLIANCE
    {
        status = tc_check_length( packetSubtype, length );
    }
    status_crc = tc_check_crc( TCPacket, length, computed_CRC );
    if (status == CCSDS_TM_VALID )  // CHECK CRC
    {
        status = status_crc;
    }

    return status;
}

int tc_check_type( unsigned char packetType )
{
    /** This function checks that the type of a TeleCommand is valid.
     *
     * @param packetType is the type to check.
     *
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
     *
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
        status = ILL_SUBTYPE;
    }

    return status;
}

int tc_check_sid( unsigned char sid )
{
    /** This function checks that the sid of a TeleCommand is valid.
     *
     * @param sid is the sid to check.
     *
     * @return Status code CCSDS_TM_VALID or CORRUPTED.
     *
     */

    int status;

    if ( (sid == SID_TC_GROUND)
         || (sid == SID_TC_MISSION_TIMELINE) || (sid == SID_TC_TC_SEQUENCES)   || (sid == SID_TC_RECOVERY_ACTION_CMD)
         || (sid == SID_TC_BACKUP_MISSION_TIMELINE)
         || (sid == SID_TC_DIRECT_CMD)       || (sid == SID_TC_SPARE_GRD_SRC1) || (sid == SID_TC_SPARE_GRD_SRC2)
         || (sid == SID_TC_OBCP)             || (sid == SID_TC_SYSTEM_CONTROL) || (sid == SID_TC_AOCS)
         || (sid == SID_TC_RPW_INTERNAL))
    {
        status = CCSDS_TM_VALID;
    }
    else
    {
        status = WRONG_SRC_ID;
    }

    return status;
}

int tc_check_length( unsigned char packetSubType, unsigned int length )
{
    /** This function checks that the subtype and the length are compliant.
     *
     * @param packetSubType is the subtype to check.
     * @param length is the length to check.
     *
     * @return Status code CCSDS_TM_VALID or ILL_TYPE.
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    switch(packetSubType)
    {
    case TC_SUBTYPE_RESET:
        if (length!=(TC_LEN_RESET-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_COMM:
        if (length!=(TC_LEN_LOAD_COMM-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_NORM:
        if (length!=(TC_LEN_LOAD_NORM-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_BURST:
        if (length!=(TC_LEN_LOAD_BURST-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_SBM1:
        if (length!=(TC_LEN_LOAD_SBM1-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_LOAD_SBM2:
        if (length!=(TC_LEN_LOAD_SBM2-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_DUMP:
        if (length!=(TC_LEN_DUMP-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_ENTER:
        if (length!=(TC_LEN_ENTER-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_UPDT_INFO:
        if (length!=(TC_LEN_UPDT_INFO-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_EN_CAL:
        if (length!=(TC_LEN_EN_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_DIS_CAL:
        if (length!=(TC_LEN_DIS_CAL-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
        }
        else {
            status = CCSDS_TM_VALID;
        }
        break;
    case TC_SUBTYPE_UPDT_TIME:
        if (length!=(TC_LEN_UPDT_TIME-CCSDS_TC_TM_PACKET_OFFSET)) {
            status = WRONG_LEN_PKT;
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

int tc_check_crc( ccsdsTelecommandPacket_t * TCPacket, unsigned int length, unsigned char *computed_CRC )
{
    /** This function checks the CRC validity of the corresponding TeleCommand packet.
     *
     * @param TCPacket points to the TeleCommand packet to check.
     * @param length is the length of the TC packet.
     *
     * @return Status code CCSDS_TM_VALID or INCOR_CHECKSUM.
     *
     */

    int status;
    unsigned char * CCSDSContent;

    CCSDSContent = (unsigned char*) TCPacket->packetID;
    GetCRCAsTwoBytes(CCSDSContent, computed_CRC, length + CCSDS_TC_TM_PACKET_OFFSET - 2); // 2 CRC bytes removed from the calculation of the CRC
    if (computed_CRC[0] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -2]) {
        status = INCOR_CHECKSUM;
    }
    else if (computed_CRC[1] != CCSDSContent[length + CCSDS_TC_TM_PACKET_OFFSET -1]) {
        status = INCOR_CHECKSUM;
    }
    else {
        status = CCSDS_TM_VALID;
    }

    return status;
}




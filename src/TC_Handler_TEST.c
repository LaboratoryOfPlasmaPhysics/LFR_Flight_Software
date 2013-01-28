#include <stdio.h>
#include <stdlib.h>
#include <TC_handler.h>

char *errorCCSDSMsg[8] = { "ILLEGAL_APID 0",
                            "WRONG_LEN_PACKET 1",
                            "INCOR_CHECKSUM 2",
                            "ILL_TYPE 3",
                            "ILL_SUBTYPE 4",
                            "WRONG_APP_DATA 5",
                            "WRONG_CMD_CODE 6",
                            "CCSDS_TM_VALID 7"
};

char *tmGeneratorMsg[2] = { "NOTHING_TO_DO",
                            "TM_GENERATED"
};

void test_TC_handler(){
    ccsdsTelecommandPacket_t TCPacketToCheck;
    unsigned char result;
    unsigned char CRCAsTwoBytes[2] = {0, 0};
    unsigned char *ccsdsContent;
    unsigned int i = 0;
    unsigned int length = 0;
    unsigned int packetLengthField = 0;

    printf("\n*** start of test_TC_handler()\n");

    //*******************
    // TC_LFR_DUMP_PAR TC
    printf("\n     =============> TEST TC_LFR_DUMP_PAR_TC <============\n");
    length = 12;
    packetLengthField = length - 6 - 1; // -6 for the header and -1 to meet the definition of PACKET_LENGTH
    currentTC_LEN_RCV[0] = 0x00;
    currentTC_LEN_RCV[1] = packetLengthField;
    //
    //TCPacketToCheck.targetLogicalAddress = 0xfe;
    TCPacketToCheck.protocolIdentifier = 0x02;
    TCPacketToCheck.reserved = 0x00;
    TCPacketToCheck.userApplication = 0x00;
    //
    TCPacketToCheck.packetID[0] = 0x1c;
    TCPacketToCheck.packetID[1] = 0xcc;
    TCPacketToCheck.packetSequenceControl[0] = 0xc0;
    TCPacketToCheck.packetSequenceControl[1] = 0x00;
    TCPacketToCheck.packetLength[0] = 0x00;
    TCPacketToCheck.packetLength[1] = packetLengthField;
    TCPacketToCheck.dataFieldHeader[0] = 0x19; //
    TCPacketToCheck.dataFieldHeader[1] = 0xb5; // type
    TCPacketToCheck.dataFieldHeader[2] = 0x1e; // subtype
    TCPacketToCheck.dataFieldHeader[3] = 0x00; // source ID
    // no data, the CRC comes immediately after the packet header
    GetCRCAsTwoBytes((unsigned char*)TCPacketToCheck.packetID, CRCAsTwoBytes, packetLengthField + 5);
    TCPacketToCheck.dataAndCRC[length-10-2] = CRCAsTwoBytes[0];
    TCPacketToCheck.dataAndCRC[length-10-1] = CRCAsTwoBytes[1];

    ccsdsContent = (unsigned char*)TCPacketToCheck.packetID;
    // TEST 1
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_DUMP_PAR                       %s\n", errorCCSDSMsg[result]);
    // TEST 2
    TCPacketToCheck.dataFieldHeader[2] = 0x04; // subtype
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_DUMP_PAR (with wrong subtype)  %s\n", errorCCSDSMsg[result]);
    // TEST 3
    TCPacketToCheck.dataFieldHeader[2] = 0x1e; // subtype
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_DUMP_PAR                       %s\n", errorCCSDSMsg[result]);
    // TEST 4
    TCPacketToCheck.packetLength[1] = 49; // length
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_DUMP_PAR (with wrong length)   %s\n", errorCCSDSMsg[result]);
    // TEST 5
    TCPacketToCheck.packetLength[1] = packetLengthField; // length
    TCPacketToCheck.dataAndCRC[0] = 0;
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_DUMP_PAR (with wrong CRC)      %s\n", errorCCSDSMsg[result]);

    //*******************
    // TC_LFR_UPDATE_INFO
    printf("\n     =============> TEST TC_LFR_UPDATE_INFO <============\n");
    length = 50;
    packetLengthField = length - 6 - 1; // -6 for the header and -1 to meet the definition of PACKET_LENGTH
    currentTC_LEN_RCV[0] = 0x00;
    currentTC_LEN_RCV[1] = packetLengthField;
    //
    //TCPacketToCheck.targetLogicalAddress = 0xfe;
    TCPacketToCheck.protocolIdentifier = 0x02;
    TCPacketToCheck.reserved = 0x00;
    TCPacketToCheck.userApplication = 0x00;
    //
    TCPacketToCheck.packetID[0] = 0x1c;
    TCPacketToCheck.packetID[1] = 0xcc;
    TCPacketToCheck.packetSequenceControl[0] = 0xc0;
    TCPacketToCheck.packetSequenceControl[1] = 0x00;
    TCPacketToCheck.packetLength[0] = 0x00;
    TCPacketToCheck.packetLength[1] = packetLengthField;
    TCPacketToCheck.dataFieldHeader[0] = 0x19; //
    TCPacketToCheck.dataFieldHeader[1] = 0xb5; // type
    TCPacketToCheck.dataFieldHeader[2] = 0x32; // subtype
    TCPacketToCheck.dataFieldHeader[3] = 0x00; // source ID
    // no data, the CRC comes immediately after the packet header
    for (i=0; i<length-12; i++) TCPacketToCheck.dataAndCRC[i] = 0;
    GetCRCAsTwoBytes((unsigned char*)TCPacketToCheck.packetID, CRCAsTwoBytes, packetLengthField + 5);
    TCPacketToCheck.dataAndCRC[packetLengthField-3-2] = CRCAsTwoBytes[0];
    TCPacketToCheck.dataAndCRC[packetLengthField-3-1] = CRCAsTwoBytes[1];

    ccsdsContent = (unsigned char*)TCPacketToCheck.packetID;
    // TEST 1
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO                       %s\n", errorCCSDSMsg[result]);
    // TEST 2
    TCPacketToCheck.dataFieldHeader[2] = 0x05; // subtype
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO (with wrong subtype)  %s\n", errorCCSDSMsg[result]);
    // TEST 3
    TCPacketToCheck.dataFieldHeader[2] = 0x32; // subtype
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO                       %s\n", errorCCSDSMsg[result]);
    // TEST 4
    TCPacketToCheck.packetLength[1] = 49; // length
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO (invalid length)      %s\n", errorCCSDSMsg[result]);
    // TEST 5
    TCPacketToCheck.packetLength[1] = packetLengthField; // length reset to the right value
    TCPacketToCheck.dataAndCRC[0] = TCPacketToCheck.dataAndCRC[0]+1; // introduce error in the CRC
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO (wrong CRC)           %s\n", errorCCSDSMsg[result]);
    // TEST 6
    currentTC_LEN_RCV[1] = 0; // introduce error in the receive packet length
    GetCRCAsTwoBytes((unsigned char*)TCPacketToCheck.packetID, CRCAsTwoBytes, packetLengthField + 5);
    TCPacketToCheck.dataAndCRC[packetLengthField-3-2] = CRCAsTwoBytes[0];
    TCPacketToCheck.dataAndCRC[packetLengthField-3-1] = CRCAsTwoBytes[1];
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO (LEN RCV != SIZE)     %s\n", errorCCSDSMsg[result]);
    // TEST 6
    currentTC_LEN_RCV[1] = packetLengthField; // introduce error in the receive packet length
    result = TM_checker(&TCPacketToCheck);
    printf("     => test TC_LFR_UPDATE_INFO                       %s\n", errorCCSDSMsg[result]);

    printf("*** end of test_TC_handler()\n");
}

void test_GetCRC(){
    unsigned char indata[2];
    unsigned char indata0[3];
    unsigned char indata1[4];
    unsigned char indata2[6];
    unsigned char indata3[10];
    unsigned char crcAsTwoBytes[2];

    printf("\n*** start of test_GetCRC()\n");

    indata[0] = 0x00;
    indata[1] = 0x00;
    GetCRCAsTwoBytes(indata, crcAsTwoBytes, 2);
    printf("crc is %x %x, should be 0x1d 0x0f\n", crcAsTwoBytes[0], crcAsTwoBytes[1]);

    indata0[0] = 0x00;
    indata0[1] = 0x00;
    indata0[2] = 0x00;
    GetCRCAsTwoBytes(indata0, crcAsTwoBytes, 3);
    printf("crc is %x %x, should be 0xcc 0x9c\n", crcAsTwoBytes[0], crcAsTwoBytes[1]);

    indata1[0] = 0xab;
    indata1[1] = 0xcd;
    indata1[2] = 0xef;
    indata1[3] = 0x01;
    GetCRCAsTwoBytes(indata1, crcAsTwoBytes, 4);
    printf("crc is %x %x, should be 0x04 0xa2\n", crcAsTwoBytes[0], crcAsTwoBytes[1]);

    indata2[0] = 0x14;
    indata2[1] = 0x56;
    indata2[2] = 0xf8;
    indata2[3] = 0x9a;
    indata2[4] = 0x00;
    indata2[5] = 0x01;
    GetCRCAsTwoBytes(indata2, crcAsTwoBytes, 6);
    printf("crc is %x %x, should be 0x7f 0xd5\n", crcAsTwoBytes[0], crcAsTwoBytes[1]);

    indata3[0] = 0x1c;
    indata3[1] = 0xcc;
    indata3[2] = 0xc0;
    indata3[3] = 0x00;
    indata3[4] = 0x00;
    indata3[5] = 0x0c;
    indata3[6] = 0x19; //
    indata3[7] = 0xb5; // type
    indata3[8] = 0x1e; // subtype
    indata3[9] = 0x00; // source ID
    GetCRCAsTwoBytes(indata3, crcAsTwoBytes, 10);
    printf("crc is %x %x\n", crcAsTwoBytes[0], crcAsTwoBytes[1]);

    printf("*** end of test_GetCRC()\n");
}

void test_TM_acceptance_generator()
{
    unsigned char result = 0;
    unsigned char code = 0;
    unsigned char CRCAsTwoBytes[2] = {0,0};
    unsigned char * ccsdsContent;
    unsigned int length = 0;
    unsigned int packetLengthField = 0;
    ccsdsTelemetryPacket_t TMPacket;
    ccsdsTelecommandPacket_t TCPacket;

    //*******************
    // TC_LFR_DUMP_PAR TC
    length = 12;
    packetLengthField = length - 6 - 1; // -6 for the header and -1 to meet the definition of PACKET_LENGTH
    currentTC_LEN_RCV[0] = 0x00;
    currentTC_LEN_RCV[01] = packetLengthField;
    //
    //TCPacket.targetLogicalAddress = 0xfe;
    TCPacket.protocolIdentifier = 0x02;
    TCPacket.reserved = 0x00;
    TCPacket.userApplication = 0x00;
    //
    TCPacket.packetID[0] = 0x1c;
    TCPacket.packetID[1] = 0xcc;
    TCPacket.packetSequenceControl[0] = 0xc0;
    TCPacket.packetSequenceControl[1] = 0x00;
    TCPacket.packetLength[0] = 0x00;
    TCPacket.packetLength[1] = packetLengthField;
    TCPacket.dataFieldHeader[0] = 0x19; //
    TCPacket.dataFieldHeader[1] = 0xb5; // type
    TCPacket.dataFieldHeader[2] = 0x1e; // subtype
    TCPacket.dataFieldHeader[3] = 0x00; // source ID
    // no data, the CRC comes immediately after the packet header
    GetCRCAsTwoBytes((unsigned char*)TCPacket.packetID, CRCAsTwoBytes, packetLengthField + 5);
    TCPacket.dataAndCRC[length-10-2] = CRCAsTwoBytes[0];
    TCPacket.dataAndCRC[length-10-1] = CRCAsTwoBytes[1];

    ccsdsContent = (unsigned char*)TCPacket.packetID;
    // TEST 1
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 1\n");
    printf("     => test TC_LFR_DUMP_PAR                           %s\n", errorCCSDSMsg[code]);
    printf("     => result                                         %s\n", tmGeneratorMsg[result]);
    // TEST 2
    TCPacket.dataFieldHeader[2] = 0x04; // subtype
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 2\n");
    printf("     => test TC_LFR_DUMP_PAR (with wrong subtype)      %s\n", errorCCSDSMsg[code]);
    printf("     => result                                         %s\n", tmGeneratorMsg[result]);
    // TEST 3
    TCPacket.dataFieldHeader[2] = 0x1e; // subtype
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 2\n");
    printf("     => test TC_LFR_DUMP_PAR                           %s\n", errorCCSDSMsg[code]);
    printf("     => result                                         %s\n", tmGeneratorMsg[result]);
    // TEST 4
    TCPacket.packetLength[1] = 49; // length
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 4\n");
    printf("     => test TC_LFR_DUMP_PAR (invalid length)          %s\n", errorCCSDSMsg[code]);
    printf("     => result                                         %s\n", tmGeneratorMsg[result]);
    // TEST 5
    TCPacket.packetLength[1] = packetLengthField; // length
    TCPacket.dataAndCRC[0] = 0;
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 5\n");
    printf("     => test TC_LFR_DUMP_PAR (with wrong CRC)          %s\n", errorCCSDSMsg[code]);
    printf("     => result                                         %s\n", tmGeneratorMsg[result]);
    printf("     => received CRC: [%x, %x], should be [%x, %x]\n",
           TCPacket.dataAndCRC[0], TCPacket.dataAndCRC[1],
           currentTC_COMPUTED_CRC[0], TCPacket.dataAndCRC[1]);
    // TEST 6
    GetCRCAsTwoBytes((unsigned char*)TCPacket.packetID, CRCAsTwoBytes, packetLengthField + 5);
    TCPacket.dataAndCRC[length-10-2] = CRCAsTwoBytes[0]; // restore the CRC
    currentTC_LEN_RCV[1] = 1; // put error in the received packet length
    code = TM_checker(&TCPacket);
    result = TM_acceptance_generator(&TCPacket, code, &TMPacket);
    printf("     => TEST 6\n");
    printf("     => test TC_LFR_DUMP_PAR (RCV LEN != SIZE FIELD)   %s\n", errorCCSDSMsg[code]);
    printf("     => result:                                        %s\n", tmGeneratorMsg[result]);
    printf("     => RCV LEN: [%x, %x], should be [%x, %x]\n",
           currentTC_LEN_RCV[0], currentTC_LEN_RCV[1],
           TCPacket.packetLength[0], TCPacket.packetLength[1]);
}

int main()
{
    InitLookUpTableForCRC();

    //test_GetCRC();
    //test_TC_handler();
    test_TM_acceptance_generator();

    return 0;
}

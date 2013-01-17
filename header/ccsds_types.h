#ifndef CCSDS_H_INCLUDED
#define CCSDS_H_INCLUDED

#define CCSDS_TELEMETRY_HEADER_LENGTH 16+4
#define CCSDS_TM_PKT_MAX_SIZE 4412
#define CCSDS_TELECOMMAND_HEADER_LENGTH 10+4
#define CCSDS_TC_PKT_MAX_SIZE 248
#define CCSDS_TC_TM_PACKET_OFFSET 7
#define CCSDS_PROCESS_ID 76
#define CCSDS_PACKET_CATEGORY 12
#define CCSDS_DESTINATION_ID 0x21

#define SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED 24
#define SIZE_TM_LFR_TC_EXE_CORRUPTED 32

#define ILLEGAL_APID 0
#define WRONG_LEN_PACKET 1
#define INCOR_CHECKSUM 2
#define ILL_TYPE 3
#define ILL_SUBTYPE 4
#define WRONG_APP_DATA 5
#define WRONG_CMD_CODE 6
#define CCSDS_TM_VALID 7
//
#define TC_NOT_EXE 40000
#define WRONG_SRC_ID 40001
#define FUNCT_NOT_IMPL 40002
#define FAIL_DETECTED 40003
#define NOT_ALLOWED 40004
#define CORRUPTED 40005
//
#define CCSDS_ERR_SRC 8
#define CCSDS_ERR_CAT 9

struct ccsdsTelemetryPacket_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
    volatile unsigned char data[CCSDS_TM_PKT_MAX_SIZE-16];
};
typedef struct ccsdsTelemetryPacket_str ccsdsTelemetryPacket_t;

struct ccsdsTelecommandPacket_str
{
    //unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[4];
    volatile unsigned char dataAndCRC[CCSDS_TC_PKT_MAX_SIZE-10];
};
typedef struct ccsdsTelecommandPacket_str ccsdsTelecommandPacket_t;

#endif // CCSDS_H_INCLUDED

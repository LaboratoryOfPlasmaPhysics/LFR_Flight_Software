#ifndef CCSDS_H_INCLUDED
#define CCSDS_H_INCLUDED

#define CCSDS_TELEMETRY_HEADER_LENGTH 16+4
#define CCSDS_TELECOMMAND_HEADER_LENGTH 10+4
#define CCSDS_TELECOMMAND_MAX_PACKET_LENGTH 248
#define CCSDS_PROCESS_ID 11
#define CCSDS_PACKET_CATEGORY 12

#define CCSDS_ERR_PID -1
#define CCSDS_ERR_CAT -2
#define CCSDS_ERR_LENGTH -3
#define CCSDS_ERR_TYPE -4
#define CCSDS_ERR_SUBTYPE -5
#define CCSDS_ERR_SRC -6
#define CCSDS_ERR_CRC -7
#define CCSDS_TM_VALID 1

struct ccsdsTelemetrySourcePacketHeader_str
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    unsigned char dataFieldHeader[10];
};
typedef struct ccsdsTelemetrySourcePacketHeader_str ccsdsTelemetrySourcePacketHeader_t;

struct ccsdsTelecommandSourcePacketHeader_str
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    unsigned char dataFieldHeader[4];
};
typedef struct ccsdsTelemetrySourcePacketHeader_str ccsdsTelecommandSourcePacketHeader_t;

// initialize the ccsds telemetry header
ccsdsTelemetrySourcePacketHeader_t ccsdsTelemetryHeader;
#define INIT_CCSDS_TELEMETRY_HEADER ccsdsTelemetryHeader.targetLogicalAddress = 0x21; \
ccsdsTelemetryHeader.protocolIdentifier = 0x02; \
ccsdsTelemetryHeader.reserved = 0x00; \
ccsdsTelemetryHeader.userApplication = 0x00; \
ccsdsTelemetryHeader.packetID[0] = 0x08; \
ccsdsTelemetryHeader.packetID[1] = 0xbc; \
ccsdsTelemetryHeader.packetSequenceControl[0] = 0xc0; \
ccsdsTelemetryHeader.packetSequenceControl[1] = 0x00; \
ccsdsTelemetryHeader.packetLength[0] = 0x00; \
ccsdsTelemetryHeader.packetLength[1] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[0] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[1] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[2] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[3] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[4] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[5] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[6] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[7] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[8] = 0x00; \
ccsdsTelemetryHeader.dataFieldHeader[9] = 0x00;

#endif // CCSDS_H_INCLUDED

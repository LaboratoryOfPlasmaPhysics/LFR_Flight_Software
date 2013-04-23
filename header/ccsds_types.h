#ifndef CCSDS_H_INCLUDED
#define CCSDS_H_INCLUDED

#define CCSDS_TELEMETRY_HEADER_LENGTH 16+4
#define CCSDS_TM_PKT_MAX_SIZE 4412
#define CCSDS_TELECOMMAND_HEADER_LENGTH 10+4
#define CCSDS_TC_PKT_MAX_SIZE 50 // size of the TC_LFR_UPDATE_INFO packet
#define CCSDS_TC_TM_PACKET_OFFSET 7
#define CCSDS_PROCESS_ID 76
#define CCSDS_PACKET_CATEGORY 12
#define CCSDS_NODE_ADDRESS 0xfe
//
#define CCSDS_DESTINATION_ID_GROUND 0x00
#define CCSDS_DESTINATION_ID 0x01
#define CCSDS_DESTINATION_ID_DPU 0x01
//
#define CCSDS_PROTOCOLE_ID 0x02
#define CCSDS_USER_APP 0x00

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

// // TC SUBTYPES
#define TC_SUBTYPE_RESET 3
#define TC_SUBTYPE_LOAD_COMM 20
#define TC_SUBTYPE_LOAD_NORM 21
#define TC_SUBTYPE_LOAD_BURST 24
#define TC_SUBTYPE_LOAD_SBM1 27
#define TC_SUBTYPE_LOAD_SBM2 28
#define TC_SUBTYPE_DUMP 30
#define TC_SUBTYPE_ENTER 40
#define TC_SUBTYPE_UPDT_INFO 50
#define TC_SUBTYPE_EN_CAL 60
#define TC_SUBTYPE_DIS_CAL 61
#define TC_SUBTYPE_UPDT_TIME 129

#define SID_DEFAULT 0
#define SID_HK 1
#define SID_EXE_INC 5
#define SID_NOT_EXE 40000
#define SID_NOT_IMP 40002
#define SID_EXE_ERR 40003
#define SID_EXE_CORR 40005

#define SID_NORM_SWF_F0 3
#define SID_NORM_SWF_F1 4
#define SID_NORM_SWF_F2 5
#define SID_NORM_CWF_F3 1
#define SID_BURST_CWF_F2 2
#define SID_SBM1_CWF_F1 24
#define SID_SBM2_CWF_F2 25
#define SID_NORM_ASM_F0 11
#define SID_NORM_ASM_F1 12
#define SID_NORM_ASM_F2 13
#define SID_NORM_BP1_F0 14
#define SID_NORM_BP1_F1 15
#define SID_NORM_BP1_F2 16
#define SID_NORM_BP2_F0 19
#define SID_NORM_BP2_F1 20
#define SID_NORM_BP2_F2 21
#define SID_BURST_BP1_F0 17
#define SID_BURST_BP2_F0 22
#define SID_BURST_BP1_F1 18
#define SID_BURST_BP2_F1 23
#define SID_SBM1_BP1_F0 28
#define SID_SBM1_BP2_F0 31
#define SID_SBM2_BP1_F0 29
#define SID_SBM2_BP2_F0 32
#define SID_SBM1_BP1_F1 30
#define SID_SBM1_BP2_F1 33

// LENGTH (BYTES)
#define LENGTH_TM_LFR_HK 126
#define LENGTH_TM_LFR_TC_EXE_MAX 32
#define LENGTH_TM_LFR_SCIENCE_NORMAL_WF_MAX 4102
//
#define TM_LEN_EXE 20 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_LEN_NOT_EXE 26 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_LEN_NOT_IMP 24 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_LEN_EXE_ERR 24 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_LEN_EXE_CORR 32 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_HEADER_LEN 16

#define LEN_TM_LFR_HK 126 + 4
#define LEN_TM_LFR_TC_EXE_NOT_IMP 24 +4

#define TM_LEN_SCI_NORM_SWF_340 340 * 12 + 6 + 10 - 1
#define TM_LEN_SCI_NORM_SWF_8 8 * 12 + 6 + 10 - 1

enum TM_TYPE{
    TM_LFR_TC_EXE_OK,
    TM_LFR_TC_EXE_ERR,
    TM_LFR_HK,
    TM_LFR_SCI,
    TM_LFR_SCI_SBM,
    TM_LFR_PAR_DUMP
};

struct TMHeader_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
};
typedef struct TMHeader_str TMHeader_t;

struct Packet_TM_LFR_HK_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
    volatile unsigned char data[LENGTH_TM_LFR_HK - 10 + 1];
};
typedef struct Packet_TM_LFR_HK_str Packet_TM_LFR_HK_t;

struct Packet_TM_LFR_TC_EXE_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
    volatile unsigned char data[LENGTH_TM_LFR_TC_EXE_MAX - 10 + 1];
};
typedef struct Packet_TM_LFR_TC_EXE_str Packet_TM_LFR_TC_EXE_t;

struct Packet_TM_LFR_SCIENCE_NORMAL_WF_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
    volatile unsigned char auxiliaryHeader[6];
    volatile unsigned char data[LENGTH_TM_LFR_SCIENCE_NORMAL_WF_MAX - 10 + 1];
};
typedef struct Packet_TM_LFR_SCIENCE_NORMAL_WF_str Packet_TM_LFR_SCIENCE_NORMAL_WF_t;

struct ExtendedTMHeader_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    volatile unsigned char dataFieldHeader[10];
    volatile unsigned char auxiliaryHeader[6];
};
typedef struct ExtendedTMHeader_str ExtendedTMHeader_t;

struct ccsdsTelecommandPacket_str
{
    //unsigned char targetLogicalAddress; // removed by the grspw module
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

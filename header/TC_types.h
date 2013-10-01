#ifndef TC_TYPES_H
#define TC_TYPES_H

#include <ccsds_types.h>

#define PROTOCOLE_IDENTIFIER 0x02

// PACKET ID
#define TC_LFR_PACKET_ID                0x1ccc // PID 76 CAT 12

#define PACKET_LENGTH_TC_LFR_RESET                  (12 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_LOAD_COMMON_PAR        (14 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_LOAD_NORMAL_PAR        (20 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_LOAD_BURST_PAR         (14 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_LOAD_SBM1_PAR          (14 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_LOAD_SBM2_PAR          (14 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_DUMP_PAR               (12 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_ENTER_MODE             (20 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_UPDATE_INFO            (48 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_ENABLE_CALIBRATION     (12 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_DISABLE_CALIBRATION    (12 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_LFR_UPDATE_TIME            (18 - CCSDS_TC_TM_PACKET_OFFSET)

// TC TYPES
#define TC_TYPE_DEFAULT         181
#define TC_TYPE_LFR_UPDATE_TIME 9

// TC SUBTYPES
#define TC_SUBTYPE_RESET                1
#define TC_SUBTYPE_LOAD_COMMON_PAR      11
#define TC_SUBTYPE_LOAD_NORMAL_PAR      13
#define TC_SUBTYPE_LOAD_BURST_PAR       19
#define TC_SUBTYPE_LOAD_SBM1_PAR        25
#define TC_SUBTYPE_LOAD_SBM2_PAR        27
#define TC_SUBTYPE_DUMP_PAR             31
#define TC_SUBTYPE_ENTER_MODE           41
#define TC_SUBTYPE_UPDATE_INFO          51
#define TC_SUBTYPE_ENABLE_CALIBRATION   61
#define TC_SUBTYPE_DISABLE_CALIBRATION  63
#define TC_SUBTYPE_UPDATE_TIME          129

// OTHER CONSTANTS
#define TC_LFR_PACKET_SEQUENCE_CONTROL  0xc000 // PID 76 CAT 12
#define TC_LFR_DATA_FIELD_HEADER0       0x19
#define TC_LFR_LOAD_COMMON_PAR_SPARE    0x00

struct Packet_TC_LFR_RESET_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_RESET_str Packet_TC_LFR_RESET_t;

struct Packet_TC_LFR_ENTER_MODE_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char spare;
    unsigned char mode;
    unsigned char enterModeTime[6];
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_ENTER_MODE_str Packet_TC_LFR_ENTER_MODE_t;

struct Packet_TC_LFR_UPDATE_INFO_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char set1;
    unsigned char set2;
    unsigned char set3_bias_setting_set1[6];
    unsigned char set3_bias_setting_set2[6];
    unsigned char set3_bias_voltage[4];
    unsigned char set4[8];
    unsigned char set5;
    unsigned char set6;
    unsigned char set7[8];
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_UPDATE_INFO_str Packet_TC_LFR_UPDATE_INFO_t;

struct Packet_TC_LFR_DUMP_PAR_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char crc[2];

};
typedef struct Packet_TC_LFR_DUMP_PAR_str Packet_TC_LFR_DUMP_PAR_t;

struct Packet_TC_LFR_LOAD_COMMON_PAR_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char spare;
    unsigned char bw_sp0_sp1_r0_r1;
    unsigned char crc[2];

};
typedef struct Packet_TC_LFR_LOAD_COMMON_PAR_str Packet_TC_LFR_LOAD_COMMON_PAR_t;

struct Packet_TC_LFR_LOAD_NORMAL_PAR_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char sy_lfr_n_swf_l[2];
    unsigned char sy_lfr_n_swf_p[2];
    unsigned char sy_lfr_n_asm_p[2];
    unsigned char sy_lfr_n_bp_p0;
    unsigned char sy_lfr_n_bp_p1;
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_LOAD_NORMAL_PAR_str Packet_TC_LFR_LOAD_NORMAL_PAR_t;

struct Packet_TC_LFR_LOAD_BURST_SBM1_SBM2_PAR_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char sy_lfr_bp_p0;
    unsigned char sy_lfr_bp_p1;
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_LOAD_BURST_SBM1_SBM2_PAR_str Packet_TC_LFR_LOAD_BURST_SBM1_SBM2_PAR_t;

struct Packet_TC_LFR_ENABLE_DISABLE_CALIBRATION_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_ENABLE_DISABLE_CALIBRATION_str Packet_TC_LFR_ENABLE_DISABLE_CALIBRATION_t;

struct Packet_TC_LFR_UPDATE_TIME_str
{   // the CCSDS header is added by LPPMON
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char ccsdsSecHeaderFlag_pusVersion_ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char cp_rpw_time[6];
    unsigned char crc[2];
};
typedef struct Packet_TC_LFR_UPDATE_TIME_str Packet_TC_LFR_UPDATE_TIME_t;

#endif // TC_TYPES_H

#ifndef CCSDS_TYPES_H_INCLUDED
#define CCSDS_TYPES_H_INCLUDED

#define CCSDS_PROTOCOLE_EXTRA_BYTES 4
#define CCSDS_TC_TM_PACKET_OFFSET 7
#define CCSDS_TELEMETRY_HEADER_LENGTH 16+4
#define CCSDS_TM_PKT_MAX_SIZE 4412
#define CCSDS_TELECOMMAND_HEADER_LENGTH 10+4
#define CCSDS_TC_PKT_MAX_SIZE 256
#define CCSDS_TC_PKT_MIN_SIZE 16
#define CCSDS_PROCESS_ID 76
#define CCSDS_PACKET_CATEGORY 12
#define CCSDS_NODE_ADDRESS 0xfe
#define CCSDS_USER_APP 0x00

#define DEFAULT_SPARE1_PUSVERSION_SPARE2 0x10
#define DEFAULT_RESERVED 0x00
#define DEFAULT_HKBIA 0x1e  // 0001 1110

// PACKET ID
#define TM_PACKET_PID_DEFAULT           76
#define TM_PACKET_PID_BURST_SBM1_SBM2   79
#define APID_TM_TC_EXE                  0x0cc1 // PID 76 CAT 1
#define APID_TM_HK                      0x0cc4 // PID 76 CAT 4
#define APID_TM_SCIENCE_NORMAL_BURST    0x0ccc // PID 76 CAT 12
#define APID_TM_SCIENCE_SBM1_SBM2       0x0cfc // PID 79 CAT 12
#define APID_TM_PARAMETER_DUMP          0x0cc6 // PID 76 CAT 6
#define APID_TM_KCOEFFICIENTS_DUMP      0x0cc6 // PID 76 CAT 6
// PACKET CAT
#define TM_PACKET_CAT_TC_EXE    1
#define TM_PACKET_CAT_HK        4
#define TM_PACKET_CAT_SCIENCE   12
#define TM_PACKET_CAT_DUMP      6

// PACKET SEQUENCE CONTROL
#define TM_PACKET_SEQ_CTRL_CONTINUATION 0x00    // [0000 0000]
#define TM_PACKET_SEQ_CTRL_FIRST        0x40    // [0100 0000]
#define TM_PACKET_SEQ_CTRL_LAST         0x80    // [1000 0000]
#define TM_PACKET_SEQ_CTRL_STANDALONE   0xc0    // [1100 0000]
#define TM_PACKET_SEQ_CNT_DEFAULT       0x00    // [0000 0000]

// DESTINATION ID
#define TM_DESTINATION_ID_GROUND                    0
#define TM_DESTINATION_ID_MISSION_TIMELINE          110
#define TM_DESTINATION_ID_TC_SEQUENCES              111
#define TM_DESTINATION_ID_RECOVERY_ACTION_COMMAND   112
#define TM_DESTINATION_ID_BACKUP_MISSION_TIMELINE   113
#define TM_DESTINATION_ID_DIRECT_CMD                120
#define TM_DESTINATION_ID_SPARE_GRD_SRC1            121
#define TM_DESTINATION_ID_SPARE_GRD_SRC2            122
#define TM_DESTINATION_ID_OBCP                      15
#define TM_DESTINATION_ID_SYSTEM_CONTROL            14
#define TM_DESTINATION_ID_AOCS                      11

//*********************************************************
//*** /!\ change CCSDS_DESTINATION_ID before flight /!\ ***
//*********************************************************
#ifdef LPP_DPU_DESTID
#define CCSDS_DESTINATION_ID 32
#else
#define CCSDS_DESTINATION_ID 0x01
#endif
#define CCSDS_PROTOCOLE_ID 0x02
#define CCSDS_RESERVED 0x00
#define CCSDS_USER_APP 0x00

#define SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED 24
#define SIZE_TM_LFR_TC_EXE_CORRUPTED 32
#define SIZE_HK_PARAMETERS 112

// TC TYPES
#define TC_TYPE_GEN     181
#define TC_TYPE_TIME    9

// TC SUBTYPES
#define TC_SUBTYPE_RESET                1
#define TC_SUBTYPE_LOAD_COMM            11
#define TC_SUBTYPE_LOAD_NORM            13
#define TC_SUBTYPE_LOAD_BURST           19
#define TC_SUBTYPE_LOAD_SBM1            25
#define TC_SUBTYPE_LOAD_SBM2            27
#define TC_SUBTYPE_DUMP                 31
#define TC_SUBTYPE_ENTER                41
#define TC_SUBTYPE_UPDT_INFO            51
#define TC_SUBTYPE_EN_CAL               61
#define TC_SUBTYPE_DIS_CAL              63
#define TC_SUBTYPE_LOAD_K               93
#define TC_SUBTYPE_DUMP_K               95
#define TC_SUBTYPE_LOAD_FBINS           91
#define TC_SUBTYPE_LOAD_FILTER_PAR      97
#define TC_SUBTYPE_UPDT_TIME            129

// TC LEN
#define TC_LEN_RESET                12
#define TC_LEN_LOAD_COMM            14
#define TC_LEN_LOAD_NORM            22
#define TC_LEN_LOAD_BURST           14
#define TC_LEN_LOAD_SBM1            14
#define TC_LEN_LOAD_SBM2            14
#define TC_LEN_DUMP                 12
#define TC_LEN_ENTER                20
#define TC_LEN_UPDT_INFO            110
#define TC_LEN_EN_CAL               12
#define TC_LEN_DIS_CAL              12
#define TC_LEN_LOAD_K               142
#define TC_LEN_DUMP_K               12
#define TC_LEN_LOAD_FBINS           60
#define TC_LEN_LOAD_FILTER_PAR      28
#define TC_LEN_UPDT_TIME            18

// PACKET CODES
#define TM_CODE_K_DUMP  0xb5600b00 // 181 (0xb5) ** 96 (0x60) ** 11 (0x0b) ** 0 (0x00)

// TM TYPES
#define TM_TYPE_TC_EXE              1
#define TM_TYPE_HK                  3
#define TM_TYPE_LFR_SCIENCE         21
#define TM_TYPE_PARAMETER_DUMP      181
#define TM_TYPE_K_DUMP              181

// TM SUBTYPES
#define TM_SUBTYPE_EXE_OK           7
#define TM_SUBTYPE_EXE_NOK          8
#define TM_SUBTYPE_HK               25
#define TM_SUBTYPE_LFR_SCIENCE_3    3   // TM packets with fixed size
#define TM_SUBTYPE_LFR_SCIENCE_6    6   // TM packets with variable size
#define TM_SUBTYPE_PARAMETER_DUMP   32
#define TM_SUBTYPE_K_DUMP           96

// FAILURE CODES
#define ILLEGAL_APID        0
#define WRONG_LEN_PKT       1
#define INCOR_CHECKSUM      2
#define ILL_TYPE            3
#define ILL_SUBTYPE         4
#define WRONG_APP_DATA      5       // 0x00 0x05
#define TC_NOT_EXE          42000   // 0xa4 0x10
#define WRONG_SRC_ID        42001   // 0xa4 0x11
#define FUNCT_NOT_IMPL      42002   // 0xa4 0x12
#define FAIL_DETECTED       42003   // 0xa4 0x13
#define NOT_ALLOWED         42004   // 0xa4 0x14
#define CORRUPTED           42005   // 0xa4 0x15
#define CCSDS_TM_VALID      7

// HK_LFR_LAST_ER_RID
#define RID_LE_LFR_TIME     42119
#define RID_LE_LFR_DPU_SPW  42128
#define RID_LE_LFR_TIMEC    42129
#define RID_ME_LFR_DPU_SPW  42338
// HK_LFR_LAST_ER_CODE
#define CODE_PARITY             1
#define CODE_DISCONNECT         2
#define CODE_ESCAPE             3
#define CODE_CREDIT             4
#define CODE_WRITE_SYNC         5
#define CODE_EARLY_EOP_EEP      6
#define CODE_INVALID_ADDRESS    7
#define CODE_EEP                8
#define CODE_RX_TOO_BIG         9
#define CODE_HEADER_CRC         16
#define CODE_DATA_CRC           17
#define CODE_ERRONEOUS          20
#define CODE_MISSING            21
#define CODE_INVALID            22
#define CODE_TIMECODE_IT        24
#define CODE_NOT_SYNCHRO        25
#define CODE_TIMECODE_CTR       26

// TC SID
#define SID_TC_GROUND                   0
#define SID_TC_MISSION_TIMELINE         110
#define SID_TC_TC_SEQUENCES             111
#define SID_TC_RECOVERY_ACTION_CMD      112
#define SID_TC_BACKUP_MISSION_TIMELINE  113
#define SID_TC_DIRECT_CMD               120
#define SID_TC_SPARE_GRD_SRC1           121
#define SID_TC_SPARE_GRD_SRC2           122
#define SID_TC_OBCP                     15
#define SID_TC_SYSTEM_CONTROL           14
#define SID_TC_AOCS                     11
#define SID_TC_RPW_INTERNAL             254

enum apid_destid{
    GROUND,
    MISSION_TIMELINE,
    TC_SEQUENCES,
    RECOVERY_ACTION_CMD,
    BACKUP_MISSION_TIMELINE,
    DIRECT_CMD,
    SPARE_GRD_SRC1,
    SPARE_GRD_SRC2,
    OBCP,
    SYSTEM_CONTROL,
    AOCS,
    RPW_INTERNAL
};
// SEQUENCE COUNTERS
#define SEQ_CNT_MAX 16383
#define SEQ_CNT_NB_DEST_ID 12

// TM SID
#define SID_HK 1

#define SID_NORM_SWF_F0     3
#define SID_NORM_SWF_F1     4
#define SID_NORM_SWF_F2     5
#define SID_NORM_CWF_F3     1
#define SID_BURST_CWF_F2    2
#define SID_SBM1_CWF_F1     24
#define SID_SBM2_CWF_F2     25
#define SID_NORM_ASM_F0     11
#define SID_NORM_ASM_F1     12
#define SID_NORM_ASM_F2     13
#define SID_NORM_BP1_F0     14
#define SID_NORM_BP1_F1     15
#define SID_NORM_BP1_F2     16
#define SID_NORM_BP2_F0     19
#define SID_NORM_BP2_F1     20
#define SID_NORM_BP2_F2     21
#define SID_BURST_BP1_F0    17
#define SID_BURST_BP2_F0    22
#define SID_BURST_BP1_F1    18
#define SID_BURST_BP2_F1    23
#define SID_SBM1_BP1_F0     28
#define SID_SBM1_BP2_F0     31
#define SID_SBM2_BP1_F0     29
#define SID_SBM2_BP2_F0     32
#define SID_SBM2_BP1_F1     30
#define SID_SBM2_BP2_F1     33
#define SID_NORM_CWF_LONG_F3 34

#define SID_PARAMETER_DUMP  10
#define SID_K_DUMP          11

// HEADER_LENGTH
//#define TM_HEADER_LEN 16
#define HEADER_LENGTH_TM_LFR_SCIENCE_CWF 32
#define HEADER_LENGTH_TM_LFR_SCIENCE_SWF 34
#define HEADER_LENGTH_TM_LFR_SCIENCE_ASM 34
// PACKET_LENGTH
#define PACKET_LENGTH_TC_EXE_SUCCESS                (20   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_INCONSISTENT           (26   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE         (26   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED        (24   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_ERROR                  (24   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_CORRUPTED              (32   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_HK                            (136  - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_PARAMETER_DUMP                (148  - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_K_DUMP                        (3920 - CCSDS_TC_TM_PACKET_OFFSET)
// SCIENCE ASM
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_1       (3230 - CCSDS_TC_TM_PACKET_OFFSET)  // 32 * 25 * 4 + 30 => 32 bins  (32 + 32 + 24 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_2       (2430 - CCSDS_TC_TM_PACKET_OFFSET)  // 24 * 25 * 4 + 30 => 24 bins  (32 + 32 + 24 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_1       (3630 - CCSDS_TC_TM_PACKET_OFFSET)  // 36 * 25 * 4 + 30 => 36 bins  (36 + 36 + 32 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_2       (3230 - CCSDS_TC_TM_PACKET_OFFSET)  // 32 * 25 * 4 + 30 => 32 bins  (36 + 36 + 32 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F2         (3230 - CCSDS_TC_TM_PACKET_OFFSET)  // 32 * 25 * 4 + 30 => 96 bins  (32 + 32 + 32 ), 3 packets
// SCIENCE NORM
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0    (150  - CCSDS_TC_TM_PACKET_OFFSET)  // 11 * 11     + 29 (1 spare byte in the header)
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F1    (172  - CCSDS_TC_TM_PACKET_OFFSET)  // 13 * 11     + 29 (1 spare byte in the header)
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F2    (160  - CCSDS_TC_TM_PACKET_OFFSET)  // 12 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0    (358  - CCSDS_TC_TM_PACKET_OFFSET)  // 11 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F1    (418  - CCSDS_TC_TM_PACKET_OFFSET)  // 13 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F2    (388  - CCSDS_TC_TM_PACKET_OFFSET)  // 12 * 30     + 28
// SCIENCE SBM
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0     (270  - CCSDS_TC_TM_PACKET_OFFSET)  // 22 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0     (688  - CCSDS_TC_TM_PACKET_OFFSET)  // 22 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F1     (314  - CCSDS_TC_TM_PACKET_OFFSET)  // 26 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F1     (808  - CCSDS_TC_TM_PACKET_OFFSET)  // 26 * 30     + 28

#define PACKET_LENGTH_DELTA                         11                                  // 7 + 4

#define SPARE1_PUSVERSION_SPARE2 0x10

// R3
// one snapshot          = 2048 samples = 6 packets * 304 + 224
#define TM_LEN_SCI_SWF_304  (3678 - CCSDS_TC_TM_PACKET_OFFSET)    // 304 * 12 + 30
#define TM_LEN_SCI_SWF_224  (2718 - CCSDS_TC_TM_PACKET_OFFSET)    // 224 * 12 + 30
// one continuous buffer = 2688 samples = 8 packets * 336
#define TM_LEN_SCI_CWF_336  (4060 - CCSDS_TC_TM_PACKET_OFFSET)    // 336 * 12 + 28
#define TM_LEN_SCI_CWF_672  (4060 - CCSDS_TC_TM_PACKET_OFFSET)    // 672 * 6  + 28
//
#define DEFAULT_PKTCNT      0x07
#define BLK_NR_304          0x0130
#define BLK_NR_224          0x00e0
#define BLK_NR_CWF          0x0150  // 336
#define BLK_NR_CWF_SHORT_F3 0x02a0  // 672

enum TM_TYPE{
    TM_LFR_TC_EXE_OK,
    TM_LFR_TC_EXE_ERR,
    TM_LFR_HK,
    TM_LFR_SCI,
    TM_LFR_SCI_SBM,
    TM_LFR_PAR_DUMP
};

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
} Packet_TM_LFR_TC_EXE_SUCCESS_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
    unsigned char tc_failure_code[2];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char byte_position;
    unsigned char rcv_value;
} Packet_TM_LFR_TC_EXE_INCONSISTENT_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
    unsigned char tc_failure_code[2];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char lfr_status_word[2];
} Packet_TM_LFR_TC_EXE_NOT_EXECUTABLE_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
    unsigned char tc_failure_code[2];
    unsigned char tc_service;
    unsigned char tc_subtype;
} Packet_TM_LFR_TC_EXE_NOT_IMPLEMENTED_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
    unsigned char tc_failure_code[2];
    unsigned char tc_service;
    unsigned char tc_subtype;
} Packet_TM_LFR_TC_EXE_ERROR_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    //
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
    unsigned char tc_failure_code[2];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char pkt_len_rcv_value[2];
    unsigned char pkt_datafieldsize_cnt[2];
    unsigned char rcv_crc[2];
    unsigned char computed_crc[2];
} Packet_TM_LFR_TC_EXE_CORRUPTED_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char pktCnt;
    unsigned char pktNr;
    unsigned char acquisitionTime[6];
    unsigned char blkNr[2];
} Header_TM_LFR_SCIENCE_SWF_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    // AUXILIARY DATA HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[6];
    unsigned char blkNr[2];
} Header_TM_LFR_SCIENCE_CWF_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char pa_lfr_pkt_cnt_asm;
    unsigned char pa_lfr_pkt_nr_asm;
    unsigned char acquisitionTime[6];
    unsigned char pa_lfr_asm_blk_nr[2];
} Header_TM_LFR_SCIENCE_ASM_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[6];
    unsigned char source_data_spare;
    unsigned char pa_lfr_bp_blk_nr[2];
} Header_TM_LFR_SCIENCE_BP_with_spare_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[6];
    unsigned char pa_lfr_bp_blk_nr[2];
} Header_TM_LFR_SCIENCE_BP_t;

typedef struct {
    //targetLogicalAddress is removed by the grspw module
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char headerFlag_pusVersion_Ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char dataAndCRC[CCSDS_TC_PKT_MAX_SIZE-10];
} ccsdsTelecommandPacket_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    unsigned char sid;

    //**************
    // HK PARAMETERS
    unsigned char lfr_status_word[2];
    unsigned char lfr_sw_version[4];
    unsigned char lfr_fpga_version[3];
    // ressource statistics
    unsigned char hk_lfr_cpu_load;
    unsigned char hk_lfr_cpu_load_max;
    unsigned char hk_lfr_cpu_load_aver;
    unsigned char hk_lfr_q_sd_fifo_size_max;
    unsigned char hk_lfr_q_sd_fifo_size;
    unsigned char hk_lfr_q_rv_fifo_size_max;
    unsigned char hk_lfr_q_rv_fifo_size;
    unsigned char hk_lfr_q_p0_fifo_size_max;
    unsigned char hk_lfr_q_p0_fifo_size;
    unsigned char hk_lfr_q_p1_fifo_size_max;
    unsigned char hk_lfr_q_p1_fifo_size;
    unsigned char hk_lfr_q_p2_fifo_size_max;
    unsigned char hk_lfr_q_p2_fifo_size;
    // tc statistics
    unsigned char hk_lfr_update_info_tc_cnt[2];
    unsigned char hk_lfr_update_time_tc_cnt[2];
    unsigned char hk_lfr_exe_tc_cnt[2];
    unsigned char hk_lfr_rej_tc_cnt[2];
    unsigned char hk_lfr_last_exe_tc_id[2];
    unsigned char hk_lfr_last_exe_tc_type[2];
    unsigned char hk_lfr_last_exe_tc_subtype[2];
    unsigned char hk_lfr_last_exe_tc_time[6];
    unsigned char hk_lfr_last_rej_tc_id[2];
    unsigned char hk_lfr_last_rej_tc_type[2];
    unsigned char hk_lfr_last_rej_tc_subtype[2];
    unsigned char hk_lfr_last_rej_tc_time[6];
    // anomaly statistics
    unsigned char hk_lfr_le_cnt[2];
    unsigned char hk_lfr_me_cnt[2];
    unsigned char hk_lfr_he_cnt[2];
    unsigned char hk_lfr_last_er_rid[2];
    unsigned char hk_lfr_last_er_code;
    unsigned char hk_lfr_last_er_time[6];
    // vhdl_blk_status
    unsigned char hk_lfr_vhdl_aa_sm;
    unsigned char hk_lfr_vhdl_fft_sr;
    unsigned char hk_lfr_vhdl_cic_hk;
    unsigned char hk_lfr_vhdl_iir_cal;
    // spacewire_if_statistics
    unsigned char hk_lfr_dpu_spw_pkt_rcv_cnt[2];
    unsigned char hk_lfr_dpu_spw_pkt_sent_cnt[2];
    unsigned char hk_lfr_dpu_spw_tick_out_cnt;
    unsigned char hk_lfr_dpu_spw_last_timc;
    // ahb error statistics
    unsigned char hk_lfr_last_fail_addr[4];
    // temperatures
    unsigned char hk_lfr_temp_scm[2];
    unsigned char hk_lfr_temp_pcb[2];
    unsigned char hk_lfr_temp_fpga[2];
    // spacecraft potential
    unsigned char hk_lfr_sc_v_f3[2];
    unsigned char hk_lfr_sc_e1_f3[2];
    unsigned char hk_lfr_sc_e2_f3[2];
    // lfr common parameters
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    // error counters
    unsigned char hk_lfr_dpu_spw_parity;
    unsigned char hk_lfr_dpu_spw_disconnect;
    unsigned char hk_lfr_dpu_spw_escape;
    unsigned char hk_lfr_dpu_spw_credit;
    unsigned char hk_lfr_dpu_spw_write_sync;
    unsigned char hk_lfr_dpu_spw_rx_ahb;
    unsigned char hk_lfr_dpu_spw_tx_ahb;
    unsigned char hk_lfr_dpu_spw_early_eop;
    unsigned char hk_lfr_dpu_spw_invalid_addr;
    unsigned char hk_lfr_dpu_spw_eep;
    unsigned char hk_lfr_dpu_spw_rx_too_big;
    // timecode
    unsigned char hk_lfr_timecode_erroneous;
    unsigned char hk_lfr_timecode_missing;
    unsigned char hk_lfr_timecode_invalid;
    // time
    unsigned char hk_lfr_time_timecode_it;
    unsigned char hk_lfr_time_not_synchro;
    unsigned char hk_lfr_time_timecode_ctr;
    // hk_lfr_buffer_dpu_
    unsigned char hk_lfr_buffer_dpu_tc_fifo;
    unsigned char hk_lfr_buffer_dpu_tm_fifo;
    // hk_lfr_ahb_
    unsigned char hk_lfr_ahb_correctable;
    unsigned char hk_lfr_ahb_uncorrectable;
    // reaction wheel frequency
    unsigned char hk_lfr_sc_rw_f_flags;
} Packet_TM_LFR_HK_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    unsigned char sid;

    //******************
    // COMMON PARAMETERS
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;

    //******************
    // NORMAL PARAMETERS
    unsigned char sy_lfr_n_swf_l[2];
    unsigned char sy_lfr_n_swf_p[2];
    unsigned char sy_lfr_n_asm_p[2];
    unsigned char sy_lfr_n_bp_p0;
    unsigned char sy_lfr_n_bp_p1;
    unsigned char sy_lfr_n_cwf_long_f3;
    unsigned char lfr_normal_parameters_spare;

    //*****************
    // BURST PARAMETERS
    unsigned char sy_lfr_b_bp_p0;
    unsigned char sy_lfr_b_bp_p1;

    //****************
    // SBM1 PARAMETERS
    unsigned char sy_lfr_s1_bp_p0;
    unsigned char sy_lfr_s1_bp_p1;

    //****************
    // SBM2 PARAMETERS
    unsigned char sy_lfr_s2_bp_p0;
    unsigned char sy_lfr_s2_bp_p1;

    // mask F0
    unsigned char sy_lfr_fbins_f0_word1[4];
    unsigned char sy_lfr_fbins_f0_word2[4];
    unsigned char sy_lfr_fbins_f0_word3[4];
    unsigned char sy_lfr_fbins_f0_word4[4];
    // mask F1
    unsigned char sy_lfr_fbins_f1_word1[4];
    unsigned char sy_lfr_fbins_f1_word2[4];
    unsigned char sy_lfr_fbins_f1_word3[4];
    unsigned char sy_lfr_fbins_f1_word4[4];
    // mask F2
    unsigned char sy_lfr_fbins_f2_word1[4];
    unsigned char sy_lfr_fbins_f2_word2[4];
    unsigned char sy_lfr_fbins_f2_word3[4];
    unsigned char sy_lfr_fbins_f2_word4[4];

    // PAS FILTER PARAMETERS
    unsigned char spare_sy_lfr_pas_filter_enabled;
    unsigned char sy_lfr_pas_filter_modulus;
    unsigned char sy_lfr_pas_filter_tbad[4];
    unsigned char sy_lfr_pas_filter_offset;
    unsigned char sy_lfr_pas_filter_shift[4];
    unsigned char sy_lfr_sc_rw_delta_f[4];

    // LFR_RW_MASK
    unsigned char sy_lfr_rw_mask_f0_word1[4];
    unsigned char sy_lfr_rw_mask_f0_word2[4];
    unsigned char sy_lfr_rw_mask_f0_word3[4];
    unsigned char sy_lfr_rw_mask_f0_word4[4];
    unsigned char sy_lfr_rw_mask_f1_word1[4];
    unsigned char sy_lfr_rw_mask_f1_word2[4];
    unsigned char sy_lfr_rw_mask_f1_word3[4];
    unsigned char sy_lfr_rw_mask_f1_word4[4];
    unsigned char sy_lfr_rw_mask_f2_word1[4];
    unsigned char sy_lfr_rw_mask_f2_word2[4];
    unsigned char sy_lfr_rw_mask_f2_word3[4];
    unsigned char sy_lfr_rw_mask_f2_word4[4];

    // SPARE
    unsigned char source_data_spare;
} Packet_TM_LFR_PARAMETER_DUMP_t;

typedef struct {
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[2];
    unsigned char packetSequenceControl[2];
    unsigned char packetLength[2];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[6];
    unsigned char sid;
    unsigned char pkt_cnt;
    unsigned char pkt_nr;
    unsigned char blk_nr;

    //******************
    // SOURCE DATA repeated N times with N in [0 .. PA_LFR_KCOEFF_BLK_NR]
    unsigned char kcoeff_blks[3900]; // one blk is 2 + 4 * 32 = 130 bytes, 30 blks max in one packet (30 * 130 = 3900)

} Packet_TM_LFR_KCOEFFICIENTS_DUMP_t;

#endif // CCSDS_TYPES_H_INCLUDED

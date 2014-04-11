#ifndef CCSDS_TYPES_H_INCLUDED
#define CCSDS_TYPES_H_INCLUDED

#include "fsw_params_processing.h"

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
#define APID_TM_TC_EXE                  0x0cc1 // PID 76 CAT 1
#define APID_TM_HK                      0x0cc4 // PID 76 CAT 4
#define APID_TM_PARAMETER_DUMP          0x0cc9 // PID 76 CAT 9
#define APID_TM_SCIENCE_NORMAL_BURST    0x0ccc // PID 76 CAT 12
#define APID_TM_SCIENCE_SBM1_SBM2       0x0cfc // PID 79 CAT 12
#define TM_PACKET_PID_DEFAULT           76
#define TM_PACKET_PID_BURST_SBM1_SBM2   79
#define TM_PACKET_CAT_TC_EXE            1
#define TM_PACKET_CAT_HK                4
#define TM_PACKET_CAT_PARAMETER_DUMP    9
#define TM_PACKET_CAT_SCIENCE           12
#define TC_PACKET_CAT                   12

// PACKET SEQUENCE CONTROL
#define TM_PACKET_SEQ_CTRL_CONTINUATION 0x00    // [0000 0000]
#define TM_PACKET_SEQ_CTRL_FIRST        0x40    // [0100 0000]
#define TM_PACKET_SEQ_CTRL_LAST         0x80    // [1000 0000]
#define TM_PACKET_SEQ_CTRL_STANDALONE   0xc0    // [1100 0000]
#define TM_PACKET_SEQ_CNT_DEFAULT       0x00    // [0000 0000]

// DESTINATION ID
#define TM_DESTINATION_ID_GROUND 0
#define TM_DESTINATION_ID_MISSION_TIMELINE 110
#define TM_DESTINATION_ID_TC_SEQUENCES 111
#define TM_DESTINATION_ID_RECOVERY_ACTION_COMMAND 112
#define TM_DESTINATION_ID_BACKUP_MISSION_TIMELINE 113
#define TM_DESTINATION_ID_DIRECT_CMD 120
#define TM_DESTINATION_ID_SPARE_GRD_SRC1 121
#define TM_DESTINATION_ID_SPARE_GRD_SRC2 122
#define TM_DESTINATION_ID_OBCP 15
#define TM_DESTINATION_ID_SYSTEM_CONTROL 14
#define TM_DESTINATION_ID_AOCS 11

#define CCSDS_DESTINATION_ID 0x01
#define CCSDS_PROTOCOLE_ID 0x02
#define CCSDS_RESERVED 0x00
#define CCSDS_USER_APP 0x00

#define SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED 24
#define SIZE_TM_LFR_TC_EXE_CORRUPTED 32
#define SIZE_HK_PARAMETERS 112

// TC TYPES
#define TC_TYPE_GEN 181
#define TC_TYPE_TIME 9

// TC SUBTYPES
#define TC_SUBTYPE_RESET 1
#define TC_SUBTYPE_LOAD_COMM 11
#define TC_SUBTYPE_LOAD_NORM 13
#define TC_SUBTYPE_LOAD_BURST 19
#define TC_SUBTYPE_LOAD_SBM1 25
#define TC_SUBTYPE_LOAD_SBM2 27
#define TC_SUBTYPE_DUMP 31
#define TC_SUBTYPE_ENTER 41
#define TC_SUBTYPE_UPDT_INFO 51
#define TC_SUBTYPE_EN_CAL 61
#define TC_SUBTYPE_DIS_CAL 63
#define TC_SUBTYPE_UPDT_TIME 129

// TC LEN
#define TC_LEN_RESET 12
#define TC_LEN_LOAD_COMM 14
#define TC_LEN_LOAD_NORM 22
#define TC_LEN_LOAD_BURST 14
#define TC_LEN_LOAD_SBM1 14
#define TC_LEN_LOAD_SBM2 14
#define TC_LEN_DUMP 12
#define TC_LEN_ENTER 20
#define TC_LEN_UPDT_INFO 46
#define TC_LEN_EN_CAL 12
#define TC_LEN_DIS_CAL 12
#define TC_LEN_UPDT_TIME 18

// TM TYPES
#define TM_TYPE_TC_EXE 1
#define TM_TYPE_HK 3
#define TM_TYPE_PARAMETER_DUMP 3
#define TM_TYPE_LFR_SCIENCE 21

// TM SUBTYPES
#define TM_SUBTYPE_EXE_OK 7
#define TM_SUBTYPE_EXE_NOK 8
#define TM_SUBTYPE_HK 25
#define TM_SUBTYPE_PARAMETER_DUMP 25
#define TM_SUBTYPE_SCIENCE 3
#define TM_SUBTYPE_LFR_SCIENCE 3

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
#define SID_PARAMETER_DUMP 10

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
#define SID_SBM2_BP1_F1 30
#define SID_SBM2_BP2_F1 33
#define SID_NORM_CWF_LONG_F3 34

// LENGTH (BYTES)
#define LENGTH_TM_LFR_TC_EXE_MAX 32
#define LENGTH_TM_LFR_HK 126

// HEADER_LENGTH
#define TM_HEADER_LEN 16
#define HEADER_LENGTH_TM_LFR_SCIENCE_ASM 28
// PACKET_LENGTH
#define PACKET_LENGTH_TC_EXE_SUCCESS                (20   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_INCONSISTENT           (26   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE         (26   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED        (24   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_ERROR                  (24   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_CORRUPTED              (32   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_HK                            (124  - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_PARAMETER_DUMP                (36   - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0         (2228 - CCSDS_TC_TM_PACKET_OFFSET)  // 44 * 25 * 2 + 28 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1         (2628 - CCSDS_TC_TM_PACKET_OFFSET)  // 52 * 25 * 2 + 28 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F2         (2428 - CCSDS_TC_TM_PACKET_OFFSET)  // 48 * 25 * 2 + 28 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0    (126  - CCSDS_TC_TM_PACKET_OFFSET)  // 11 * 9      + 27 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0    (356  - CCSDS_TC_TM_PACKET_OFFSET)  // 11 * 30     + 25 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_BURST_BP2_F1   (806  - CCSDS_TC_TM_PACKET_OFFSET)  // 26 * 30     + 26 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0    (224  - CCSDS_TC_TM_PACKET_OFFSET)  // 22 * 9      + 26 - 7
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0    (686  - CCSDS_TC_TM_PACKET_OFFSET)  // 22 * 30     + 26 - 7

#define PACKET_LENGTH_DELTA                         11                                  // 7 + 4

#define SPARE1_PUSVERSION_SPARE2 0x10

// R1
#define TM_LEN_SCI_SWF_340          4101    // 340 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_SWF_8            117     //   8 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_CWF_340          4099    // 340 * 12 + 10 + 10 - 1
#define TM_LEN_SCI_CWF_8            115     //   8 * 12 + 10 + 10 - 1
#define TM_LEN_SCI_CWF3_LIGHT_340   2059    // 340 * 6  + 10 + 10 - 1
#define TM_LEN_SCI_CWF3_LIGHT_8     67      //   8 * 6  + 10 + 10 - 1
// R2
#define TM_LEN_SCI_SWF_304          3669    // 304 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_SWF_224          2709    // 224 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_CWF_336          4051    // 336 * 12 + 10 + 10 - 1
#define TM_LEN_SCI_CWF_672          4051    // 672 * 6 + 10 + 10 - 1
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
    unsigned char tc_failure_code[2];
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
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
    unsigned char tc_failure_code[2];
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
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
    unsigned char tc_failure_code[2];
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
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
    unsigned char tc_failure_code[2];
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
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
    unsigned char tc_failure_code[2];
    unsigned char telecommand_pkt_id[2];
    unsigned char pkt_seq_control[2];
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
    unsigned char hkBIA;
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
    unsigned char hkBIA;
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
    unsigned char biaStatusInfo;
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
    unsigned char biaStatusInfo;
    unsigned char acquisitionTime[6];
    unsigned char source_data_spare[2];
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
    unsigned char biaStatusInfo;
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
    unsigned char hk_lfr_load_max;
    unsigned char hk_lfr_load_aver;
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
    unsigned int hk_lfr_last_fail_addr;
    // temperatures
    unsigned char hk_lfr_temp_scm[2];
    unsigned char hk_lfr_temp_pcb[2];
    unsigned char hk_lfr_temp_fpga[2];
    // spacecraft potential
    unsigned char hk_lfr_sc_v_f3[2];
    unsigned char hk_lfr_sc_e1_f3[2];
    unsigned char hk_lfr_sc_e2_f3[2];
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
    // spare
    unsigned char parameters_spare;
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
    unsigned char unused0;
    unsigned char bw_sp0_sp1_r0_r1;

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

    // SPARE
    unsigned char source_data_spare;
} Packet_TM_LFR_PARAMETER_DUMP_t;


#endif // CCSDS_TYPES_H_INCLUDED

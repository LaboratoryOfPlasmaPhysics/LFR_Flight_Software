#ifndef CCSDS_H_INCLUDED
#define CCSDS_H_INCLUDED

#define CCSDS_TELEMETRY_HEADER_LENGTH 16+4
#define CCSDS_TM_PKT_MAX_SIZE 4412
#define CCSDS_TELECOMMAND_HEADER_LENGTH 10+4
#define CCSDS_TC_PKT_MAX_SIZE 256
#define CCSDS_TC_PKT_MIN_SIZE 16
#define CCSDS_TC_TM_PACKET_OFFSET 7
#define CCSDS_PROCESS_ID 76
#define CCSDS_PACKET_CATEGORY 12
#define CCSDS_NODE_ADDRESS 0xfe

// PACKET IDs
#define TM_PACKET_ID_TC_EXE                  0x0cc1 // PID 76 CAT 1
#define TM_PACKET_ID_HK                      0x0cc4 // PID 76 CAT 4
#define TM_PACKET_ID_PARAMETER_DUMP          0x0cc9 // PID 76 CAT 9
#define TM_PACKET_ID_SCIENCE_NORMAL          0x0ccc // PID 76 CAT 12
#define TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2 0x0cfc // PID 79 CAT 12

// FAILURE CODES
#define FAILURE_CODE_INCONSISTENT       5       // 0x00 0x05
#define FAILURE_CODE_NOT_EXECUTABLE     40000   // 0x9c 0x40
#define FAILURE_CODE_NOT_IMPLEMENTED    40002   // 0x9c 0x42
#define FAILURE_CODE_ERROR              40003   // 0x9c 0x43
#define FAILURE_CODE_CORRUPTED          40005   // 0x9c 0x45
//
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
#define CCSDS_USER_APP 0x00

#define SIZE_TM_LFR_TC_EXE_NOT_IMPLEMENTED 24
#define SIZE_TM_LFR_TC_EXE_CORRUPTED 32
#define SIZE_HK_PARAMETERS 112

#define ILLEGAL_APID 0
#define WRONG_LEN_PACKET 1
#define INCOR_CHECKSUM 2
#define ILL_TYPE 3
#define ILL_SUBTYPE 4
#define WRONG_APP_DATA 5
#define WRONG_CMD_CODE 6
#define CCSDS_TM_VALID 7

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
#define TC_LEN_LOAD_NORM 20
#define TC_LEN_LOAD_BURST 14
#define TC_LEN_LOAD_SBM1 14
#define TC_LEN_LOAD_SBM2 14
#define TC_LEN_DUMP 12
#define TC_LEN_ENTER 20
#define TC_LEN_UPDT_INFO 48
#define TC_LEN_EN_CAL 12
#define TC_LEN_DIS_CAL 12
#define TC_LEN_UPDT_TIME 18

// TM TYPES
#define TM_TYPE_TC_EXE 1
#define TM_TYPE_HK 3
#define TM_TYPE_LFR_SCIENCE 21

// TM SUBTYPES
#define TM_SUBTYPE_EXE_OK 7
#define TM_SUBTYPE_EXE_NOK 8
#define TM_SUBTYPE_HK 25
#define TM_SUBTYPE_SCIENCE 3
#define TM_SUBTYPE_LFR_SCIENCE 3

// TM SID
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
#define LENGTH_TM_LFR_TC_EXE_MAX 32
#define LENGTH_TM_LFR_HK 126
// PACKET_LENGTH
#define PACKET_LENGTH_TC_EXE_SUCCESS            20 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_TC_EXE_INCONSISTENT       26 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE     26 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED    24 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_TC_EXE_ERROR              24 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_TC_EXE_CORRUPTED          32 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_HK                        126 - CCSDS_TC_TM_PACKET_OFFSET
#define PACKET_LENGTH_PARAMETER_DUMP            28 - CCSDS_TC_TM_PACKET_OFFSET
#define TM_HEADER_LEN 16

#define LEN_TM_LFR_HK 126 + 4
#define LEN_TM_LFR_TC_EXE_NOT_IMP 24 +4

#define TM_LEN_SCI_SWF_340 340 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_SWF_8 8 * 12 + 10 + 12 - 1
#define TM_LEN_SCI_CWF_340 340 * 12 + 10 + 10 - 1
#define TM_LEN_SCI_CWF_8 8 * 12 + 10 + 10 - 1

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
    // DATA FIELD HEADER
    volatile unsigned char spare1_pusVersion_spare2;
    volatile unsigned char serviceType;
    volatile unsigned char serviceSubType;
    volatile unsigned char destinationID;
    volatile unsigned char time[6];
};
typedef struct TMHeader_str TMHeader_t;

struct Packet_TM_LFR_TC_EXE_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    // DATA FIELD HEADER
    volatile unsigned char spare1_pusVersion_spare2;
    volatile unsigned char serviceType;
    volatile unsigned char serviceSubType;
    volatile unsigned char destinationID;
    volatile unsigned char time[6];
    volatile unsigned char data[LENGTH_TM_LFR_TC_EXE_MAX - 10 + 1];
};
typedef struct Packet_TM_LFR_TC_EXE_str Packet_TM_LFR_TC_EXE_t;

struct Header_TM_LFR_SCIENCE_SWF_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    // DATA FIELD HEADER
    volatile unsigned char spare1_pusVersion_spare2;
    volatile unsigned char serviceType;
    volatile unsigned char serviceSubType;
    volatile unsigned char destinationID;
    volatile unsigned char time[6];
    // AUXILIARY HEADER
    volatile unsigned char sid;
    volatile unsigned char hkBIA;
    volatile unsigned char pktCnt;
    volatile unsigned char pktNr;
    volatile unsigned char acquisitionTime[6];
    volatile unsigned char blkNr[2];
};
typedef struct Header_TM_LFR_SCIENCE_SWF_str Header_TM_LFR_SCIENCE_SWF_t;

struct Header_TM_LFR_SCIENCE_CWF_str
{
    volatile unsigned char targetLogicalAddress;
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    // DATA FIELD HEADER
    volatile unsigned char spare1_pusVersion_spare2;
    volatile unsigned char serviceType;
    volatile unsigned char serviceSubType;
    volatile unsigned char destinationID;
    volatile unsigned char time[6];
    // AUXILIARY DATA HEADER
    volatile unsigned char sid;
    volatile unsigned char hkBIA;
    volatile unsigned char acquisitionTime[6];
    volatile unsigned char blkNr[2];
};
typedef struct Header_TM_LFR_SCIENCE_CWF_str Header_TM_LFR_SCIENCE_CWF_t;

struct ccsdsTelecommandPacket_str
{
    //unsigned char targetLogicalAddress; // removed by the grspw module
    volatile unsigned char protocolIdentifier;
    volatile unsigned char reserved;
    volatile unsigned char userApplication;
    volatile unsigned char packetID[2];
    volatile unsigned char packetSequenceControl[2];
    volatile unsigned char packetLength[2];
    // DATA FIELD HEADER
    volatile unsigned char headerFlag_pusVersion_Ack;
    volatile unsigned char serviceType;
    volatile unsigned char serviceSubType;
    volatile unsigned char sourceID;
    volatile unsigned char dataAndCRC[CCSDS_TC_PKT_MAX_SIZE-10];
};
typedef struct ccsdsTelecommandPacket_str ccsdsTelecommandPacket_t;

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
    volatile unsigned char sid;

    //**************
    // HK PARAMETERS
    unsigned char lfr_status_word[2];
    unsigned char lfr_sw_version[4];
    // tc statistics
    unsigned char hk_lfr_update_info_tc_cnt[2];
    unsigned char hk_lfr_update_time_tc_cnt[2];
    unsigned char hk_dpu_exe_tc_lfr_cnt[2];
    unsigned char hk_dpu_rej_tc_lfr_cnt[2];
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
    // error counters
    unsigned char hk_lfr_dpu_spw_parity;
    unsigned char hk_lfr_dpu_spw_disconnect;
    unsigned char hk_lfr_dpu_spw_escape;
    unsigned char hk_lfr_dpu_spw_credit;
    unsigned char hk_lfr_dpu_spw_write_sync;
    unsigned char hk_lfr_dpu_spw_rx_ahb;
    unsigned char hk_lfr_dpu_spw_tx_ahb;
    unsigned char hk_lfr_dpu_spw_header_crc;
    unsigned char hk_lfr_dpu_spw_data_crc;
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
    unsigned char hk_lfr_ahb_fails_trans;
    // hk_lfr_adc_
    unsigned char hk_lfr_adc_failure;
    unsigned char hk_lfr_adc_timeout;
    unsigned char hk_lfr_toomany_err;
    // hk_lfr_cpu_
    unsigned char hk_lfr_cpu_write_err;
    unsigned char hk_lfr_cpu_ins_access_err;
    unsigned char hk_lfr_cpu_illegal_ins;
    unsigned char hk_lfr_cpu_privilegied_ins;
    unsigned char hk_lfr_cpu_register_hw;
    unsigned char hk_lfr_cpu_not_aligned;
    unsigned char hk_lfr_cpu_data_exception;
    unsigned char hk_lfr_cpu_div_exception;
    unsigned char hk_lfr_cpu_arith_overflow;
};
typedef struct Packet_TM_LFR_HK_str Packet_TM_LFR_HK_t;

#endif // CCSDS_H_INCLUDED

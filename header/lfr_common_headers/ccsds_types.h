/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

#ifndef CCSDS_TYPES_H_INCLUDED
#define CCSDS_TYPES_H_INCLUDED

#define TXBDCNT    50
#define RXBDCNT    10
#define TXDATASIZE 4096
#define TXHDRSIZE  34
#define RXPKTSIZE  200

#define SPW_RXSIZE  228
#define SPW_TXDSIZE 4096
#define SPW_TXHSIZE 64

#define BITS_PID_0 0x07
#define BITS_PID_1 0x0f
#define BITS_CAT   0x0f

#define CCSDS_PROTOCOLE_EXTRA_BYTES 4
#define CCSDS_TC_TM_PACKET_OFFSET   7
#define PROTID_RES_APP              3
#define CCSDS_TC_HEADER_LENGTH      10
#define CCSDS_TC_PKT_MAX_SIZE                                                                      \
    232 // (228+3) with 3 for Prot ID, Reserved and User App bytes, SHALL BE A
        // MULTIPLE OF 4
#define CCSDS_TC_PKT_MIN_SIZE 16
#define CCSDS_PROCESS_ID      76
#define CCSDS_PACKET_CATEGORY 12
#define CCSDS_USER_APP        0x00

#define DEFAULT_SPARE1_PUSVERSION_SPARE2 0x10
#define DEFAULT_RESERVED                 0x00
#define DEFAULT_HKBIA                    0x1e // 0001 1110

// PACKET ID
#define APID_TM_TC_EXE               0x0cc1 // PID 76 CAT 1
#define APID_TM_HK                   0x0cc4 // PID 76 CAT 4
#define APID_TM_SCIENCE_NORMAL_BURST 0x0ccc // PID 76 CAT 12
#define APID_TM_SCIENCE_SBM1_SBM2    0x0cfc // PID 79 CAT 12
#define APID_TM_PARAMETER_DUMP       0x0cc6 // PID 76 CAT 6

// PACKET SEQUENCE CONTROL
#define TM_PACKET_SEQ_CTRL_STANDALONE 0xc0 // [1100 0000]
#define TM_PACKET_SEQ_CNT_DEFAULT     0x00 // [0000 0000]
#define TM_PACKET_SEQ_SHIFT           8
#define SEQ_CNT_MAX                   16383
#define SEQ_CNT_NB_DEST_ID            12
#define SEQ_CNT_MASK                  0x3fff // [0011 1111 1111 1111]

// DESTINATION ID
#define TM_DESTINATION_ID_GROUND 0

//*********************************************************
//*** /!\ change CCSDS_DESTINATION_ID before flight /!\ ***
//*********************************************************
#ifdef LPP_DPU_DESTID
    #define CCSDS_DESTINATION_ID 32
#else
    #define CCSDS_DESTINATION_ID 0x01
#endif
#define CCSDS_PROTOCOLE_ID 0x02
#define CCSDS_RESERVED     0x00
#define CCSDS_USER_APP     0x00

// TC TYPES
#define TC_TYPE_GEN  181
#define TC_TYPE_TIME 9

// TC SUBTYPES
#define TC_SUBTYPE_RESET           1
#define TC_SUBTYPE_LOAD_COMM       11
#define TC_SUBTYPE_LOAD_NORM       13
#define TC_SUBTYPE_LOAD_BURST      19
#define TC_SUBTYPE_LOAD_SBM1       25
#define TC_SUBTYPE_LOAD_SBM2       27
#define TC_SUBTYPE_DUMP            31
#define TC_SUBTYPE_ENTER           41
#define TC_SUBTYPE_UPDT_INFO       51
#define TC_SUBTYPE_EN_CAL          61
#define TC_SUBTYPE_DIS_CAL         63
#define TC_SUBTYPE_LOAD_K          93
#define TC_SUBTYPE_DUMP_K          95
#define TC_SUBTYPE_LOAD_FBINS      91
#define TC_SUBTYPE_LOAD_FILTER_PAR 97
#define TC_SUBTYPE_UPDT_TIME       129

// TC LEN
#define TC_LEN_RESET           12
#define TC_LEN_LOAD_COMM       14
#define TC_LEN_LOAD_NORM       22
#define TC_LEN_LOAD_BURST      14
#define TC_LEN_LOAD_SBM1       14
#define TC_LEN_LOAD_SBM2       14
#define TC_LEN_DUMP            12
#define TC_LEN_ENTER           20
#define TC_LEN_UPDT_INFO       110
#define TC_LEN_EN_CAL          12
#define TC_LEN_DIS_CAL         12
#define TC_LEN_LOAD_K          142
#define TC_LEN_DUMP_K          12
#define TC_LEN_LOAD_FBINS      60
#define TC_LEN_LOAD_FILTER_PAR 92
#define TC_LEN_UPDT_TIME       18

// PACKET CODES
// For others packets ring nodes are identified by SID but K_DUMP has the same SID than SID_NORM_ASM_F0
// let's use 64 since it's unused
#define TM_K_DUMP_PKT_ID 64

// TM TYPES
#define TM_TYPE_TC_EXE         1
#define TM_TYPE_HK             3
#define TM_TYPE_LFR_SCIENCE    21
#define TM_TYPE_PARAMETER_DUMP 181
#define TM_TYPE_K_DUMP         181

// TM SUBTYPES
#define TM_SUBTYPE_EXE_OK         7
#define TM_SUBTYPE_EXE_NOK        8
#define TM_SUBTYPE_HK             25
#define TM_SUBTYPE_LFR_SCIENCE_3  3 // TM packets with fixed size
#define TM_SUBTYPE_LFR_SCIENCE_6  6 // TM packets with variable size
#define TM_SUBTYPE_PARAMETER_DUMP 32
#define TM_SUBTYPE_K_DUMP         96

// FAILURE CODES
#define ILLEGAL_APID   0
#define WRONG_LEN_PKT  1
#define INCOR_CHECKSUM 2
#define ILL_TYPE       3
#define ILL_SUBTYPE    4
#define WRONG_APP_DATA 5 // 0x00 0x05
#define TC_NOT_EXE     42000 // 0xa4 0x10
#define WRONG_SRC_ID   42001 // 0xa4 0x11
#define FUNCT_NOT_IMPL 42002 // 0xa4 0x12
#define FAIL_DETECTED  42003 // 0xa4 0x13
#define CORRUPTED      42005 // 0xa4 0x15
#define CCSDS_TM_VALID 7

// HK_LFR_LAST_ER_RID
#define RID_LE_LFR_TIME    42119
#define RID_LE_LFR_DPU_SPW 42128
#define RID_LE_LFR_TIMEC   42129
#define RID_ME_LFR_DPU_SPW 42338
// HK_LFR_LAST_ER_CODE
#define CODE_PARITY          1
#define CODE_DISCONNECT      2
#define CODE_ESCAPE          3
#define CODE_CREDIT          4
#define CODE_WRITE_SYNC      5
#define CODE_EARLY_EOP_EEP   6
#define CODE_INVALID_ADDRESS 7
#define CODE_EEP             8
#define CODE_RX_TOO_BIG      9
#define CODE_HEADER_CRC      16
#define CODE_DATA_CRC        17
#define CODE_ERRONEOUS       20
#define CODE_MISSING         21
#define CODE_INVALID         22
#define CODE_TIMECODE_IT     24
#define CODE_NOT_SYNCHRO     25
#define CODE_TIMECODE_CTR    26

// TC SID
#define SID_TC_GROUND                  0
#define SID_TC_MISSION_TIMELINE        110
#define SID_TC_TC_SEQUENCES            111
#define SID_TC_RECOVERY_ACTION_CMD     112
#define SID_TC_BACKUP_MISSION_TIMELINE 113
#define SID_TC_DIRECT_CMD              120
#define SID_TC_SPARE_GRD_SRC1          121
#define SID_TC_SPARE_GRD_SRC2          122
#define SID_TC_OBCP                    15
#define SID_TC_SYSTEM_CONTROL          14
#define SID_TC_AOCS                    11
#define SID_TC_RPW_INTERNAL            254

enum apid_destid
{
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

// TM SID
#define SID_HK 1

#define SID_NORM_SWF_F0      3
#define SID_NORM_SWF_F1      4
#define SID_NORM_SWF_F2      5
#define SID_NORM_CWF_F3      1
#define SID_BURST_CWF_F2     2
#define SID_SBM1_CWF_F1      24
#define SID_SBM2_CWF_F2      25
#define SID_NORM_ASM_F0      11
#define SID_NORM_ASM_F1      12
#define SID_NORM_ASM_F2      13
#define SID_NORM_BP1_F0      14
#define SID_NORM_BP1_F1      15
#define SID_NORM_BP1_F2      16
#define SID_NORM_BP2_F0      19
#define SID_NORM_BP2_F1      20
#define SID_NORM_BP2_F2      21
#define SID_BURST_BP1_F0     17
#define SID_BURST_BP2_F0     22
#define SID_BURST_BP1_F1     18
#define SID_BURST_BP2_F1     23
#define SID_SBM1_BP1_F0      28
#define SID_SBM1_BP2_F0      31
#define SID_SBM2_BP1_F0      29
#define SID_SBM2_BP2_F0      32
#define SID_SBM2_BP1_F1      30
#define SID_SBM2_BP2_F1      33
#define SID_NORM_CWF_LONG_F3 34

#define SID_PARAMETER_DUMP 10
#define SID_K_DUMP         11

// HEADER_LENGTH
#define HEADER_LENGTH_TM_LFR_SCIENCE_CWF 32
#define HEADER_LENGTH_TM_LFR_SCIENCE_SWF 34
#define HEADER_LENGTH_TM_LFR_SCIENCE_ASM 34
// PACKET_LENGTH
#define PACKET_LENGTH_TC_EXE_SUCCESS         (20 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_INCONSISTENT    (26 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_EXECUTABLE  (26 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_NOT_IMPLEMENTED (24 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_ERROR           (24 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_TC_EXE_CORRUPTED       (32 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_HK                     (136 - CCSDS_TC_TM_PACKET_OFFSET)
#define PACKET_LENGTH_PARAMETER_DUMP         (212 - CCSDS_TC_TM_PACKET_OFFSET)
// SCIENCE ASM
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_1                                                      \
    (3230 - CCSDS_TC_TM_PACKET_OFFSET) // 32 * 25 * 4 + 30 => 32 bins  (32 + 32 +
                                       // 24 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F0_2                                                      \
    (2430 - CCSDS_TC_TM_PACKET_OFFSET) // 24 * 25 * 4 + 30 => 24 bins  (32 + 32 +
                                       // 24 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_1                                                      \
    (3630 - CCSDS_TC_TM_PACKET_OFFSET) // 36 * 25 * 4 + 30 => 36 bins  (36 + 36 +
                                       // 32 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F1_2                                                      \
    (3230 - CCSDS_TC_TM_PACKET_OFFSET) // 32 * 25 * 4 + 30 => 32 bins  (36 + 36 +
                                       // 32 ), 3 packets
#define PACKET_LENGTH_TM_LFR_SCIENCE_ASM_F2                                                        \
    (3230 - CCSDS_TC_TM_PACKET_OFFSET) // 32 * 25 * 4 + 30 => 96 bins  (32 + 32 +
                                       // 32 ), 3 packets
// SCIENCE NORM
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F0                                                   \
    (150 - CCSDS_TC_TM_PACKET_OFFSET) // 11 * 11     + 29 (1 spare byte in the header)
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F1                                                   \
    (172 - CCSDS_TC_TM_PACKET_OFFSET) // 13 * 11     + 29 (1 spare byte in the header)
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP1_F2                                                   \
    (160 - CCSDS_TC_TM_PACKET_OFFSET) // 12 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F0                                                   \
    (358 - CCSDS_TC_TM_PACKET_OFFSET) // 11 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F1                                                   \
    (418 - CCSDS_TC_TM_PACKET_OFFSET) // 13 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_NORM_BP2_F2                                                   \
    (388 - CCSDS_TC_TM_PACKET_OFFSET) // 12 * 30     + 28
// SCIENCE SBM
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F0                                                    \
    (270 - CCSDS_TC_TM_PACKET_OFFSET) // 22 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F0                                                    \
    (688 - CCSDS_TC_TM_PACKET_OFFSET) // 22 * 30     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP1_F1                                                    \
    (314 - CCSDS_TC_TM_PACKET_OFFSET) // 26 * 11     + 28
#define PACKET_LENGTH_TM_LFR_SCIENCE_SBM_BP2_F1                                                    \
    (808 - CCSDS_TC_TM_PACKET_OFFSET) // 26 * 30     + 28

#define PACKET_LENGTH_DELTA 11 // 7 + 4

#define SPARE1_PUSVERSION_SPARE2 0x10

// R3
// one snapshot          = 2048 samples = 6 packets * 304 + 224
#define TM_LEN_SCI_SWF_304 (3678 - CCSDS_TC_TM_PACKET_OFFSET) // 304 * 12 + 30
#define TM_LEN_SCI_SWF_224 (2718 - CCSDS_TC_TM_PACKET_OFFSET) // 224 * 12 + 30
// one continuous buffer = 2688 samples = 8 packets * 336
#define TM_LEN_SCI_CWF_336 (4060 - CCSDS_TC_TM_PACKET_OFFSET) // 336 * 12 + 28
#define TM_LEN_SCI_CWF_672 (4060 - CCSDS_TC_TM_PACKET_OFFSET) // 672 * 6  + 28
//
#define PKTCNT_SWF          0x07
#define PKTCNT_ASM          3
#define BLK_NR_304          0x0130
#define BLK_NR_224          0x00e0
#define BLK_NR_CWF          0x0150 // 336
#define BLK_NR_CWF_SHORT_F3 0x02a0 // 672

enum TM_TYPE
{
    TM_LFR_TC_EXE_OK,
    TM_LFR_TC_EXE_ERR,
    TM_LFR_HK,
    TM_LFR_SCI,
    TM_LFR_SCI_SBM,
    TM_LFR_PAR_DUMP
};

#define BYTES_PER_PACKETID  2
#define BYTES_PER_SEQ_CTRL  2
#define BYTES_PER_PKT_LEN   2
#define BYTES_PER_TIME      6
#define BYTES_PER_ERR_CODE  2
#define BYTES_PER_STA_WRD   2
#define BYTES_PER_CRC       2
#define BYTES_PER_BLKNR     2
#define BYTES_PER_SW_VER    4
#define BYTES_PER_VHD_VER   3
#define COUNTER_2_BYTES     2
#define BYTES_PER_TYPE      2
#define BYTES_PER_SUBTYPE   2
#define BYTES_PER_ADDR      4
#define BYTES_PER_TEMP      2
#define BYTES_PER_V         2
#define BYTES_PER_WORD      4
#define BYTES_PER_MASK      16
#define BYTES_PER_MASKS_SET 48 // 4 * 4 * 3

#define COUNTER_2_BYTES 2
#define PARAM_2_BYTES   2
#define PARAM_4_BYTES   4

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
} Packet_TM_LFR_TC_EXE_SUCCESS_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
    unsigned char tc_failure_code[BYTES_PER_ERR_CODE];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char byte_position;
    unsigned char rcv_value;
} Packet_TM_LFR_TC_EXE_INCONSISTENT_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
    unsigned char tc_failure_code[BYTES_PER_ERR_CODE];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char lfr_status_word[2];
} Packet_TM_LFR_TC_EXE_NOT_EXECUTABLE_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
    unsigned char tc_failure_code[BYTES_PER_ERR_CODE];
    unsigned char tc_service;
    unsigned char tc_subtype;
} Packet_TM_LFR_TC_EXE_NOT_IMPLEMENTED_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
    unsigned char tc_failure_code[BYTES_PER_ERR_CODE];
    unsigned char tc_service;
    unsigned char tc_subtype;
} Packet_TM_LFR_TC_EXE_ERROR_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    // PACKET HEADER
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    //
    unsigned char telecommand_pkt_id[BYTES_PER_PACKETID];
    unsigned char pkt_seq_control[BYTES_PER_SEQ_CTRL];
    unsigned char tc_failure_code[BYTES_PER_ERR_CODE];
    unsigned char tc_service;
    unsigned char tc_subtype;
    unsigned char pkt_len_rcv_value[BYTES_PER_PKT_LEN];
    unsigned char pkt_datafieldsize_cnt[BYTES_PER_PKT_LEN];
    unsigned char rcv_crc[BYTES_PER_CRC];
    unsigned char computed_crc[BYTES_PER_CRC];
} Packet_TM_LFR_TC_EXE_CORRUPTED_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char pktCnt;
    unsigned char pktNr;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char blkNr[BYTES_PER_BLKNR];
} Header_TM_LFR_SCIENCE_SWF_t;

//*******************
// TM_LFR_SCIENCE_CWF

#define CWF_BLK_SIZE 6

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY DATA HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char blkNr[BYTES_PER_BLKNR];
} Header_TM_LFR_SCIENCE_CWF_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char pa_lfr_pkt_cnt_asm;
    unsigned char pa_lfr_pkt_nr_asm;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char pa_lfr_asm_blk_nr[BYTES_PER_BLKNR];
} Header_TM_LFR_SCIENCE_ASM_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char source_data_spare;
    unsigned char pa_lfr_bp_blk_nr[BYTES_PER_BLKNR];
} Header_TM_LFR_SCIENCE_BP_with_spare_t;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char pa_lfr_bp_blk_nr[BYTES_PER_BLKNR];
} Header_TM_LFR_SCIENCE_BP_t;

typedef struct
{
    // TARGET LOGICAL ADDRESS (targetLogicalAddress) IS REMOVED BY THE GRSPW
    // MODULE
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char headerFlag_pusVersion_Ack;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char sourceID;
    unsigned char dataAndCRC[CCSDS_TC_PKT_MAX_SIZE - CCSDS_TC_HEADER_LENGTH];
} ccsdsTelecommandPacket_t;

//**********
//**********
// TM_LFR_HK

#define STATUS_WORD_SC_POTENTIAL_FLAG_BIT   0x40 // [0100 0000]
#define STATUS_WORD_SC_POTENTIAL_FLAG_MASK  0xbf // [1011 1111]
#define STATUS_WORD_PAS_FILTER_ENABLED_BIT  0x20 // [0010 0000]
#define STATUS_WORD_PAS_FILTER_ENABLED_MASK 0xdf // [1101 1111]
#define STATUS_WORD_WATCHDOG_BIT            0x10 // [0001 0000]
#define STATUS_WORD_WATCHDOG_MASK           0xef // [1110 1111]
#define STATUS_WORD_CALIB_BIT               0x08 // [0000 1000]
#define STATUS_WORD_CALIB_MASK              0xf7 // [1111 0111]
#define STATUS_WORD_RESET_CAUSE_BITS        0x07 // [0000 0111]
#define STATUS_WORD_RESET_CAUSE_MASK        0xf8 // [1111 1000]
#define STATUS_WORD_LINK_STATE_BITS         0x07 // [0000 0111]
#define STATUS_WORD_LINK_STATE_MASK         0xf8 // [1111 1000]
#define STATUS_WORD_LFR_MODE_SHIFT          4
#define STATUS_WORD_LFR_MODE_MASK           0x0f // [0000 1111]

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    unsigned char sid;

    //**************
    // HK PARAMETERS
    unsigned char lfr_status_word[BYTES_PER_STA_WRD];
    unsigned char lfr_sw_version[BYTES_PER_SW_VER];
    unsigned char lfr_fpga_version[BYTES_PER_VHD_VER];
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
    unsigned char hk_lfr_update_info_tc_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_update_time_tc_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_exe_tc_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_rej_tc_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_last_exe_tc_id[BYTES_PER_PACKETID];
    unsigned char hk_lfr_last_exe_tc_type[BYTES_PER_TYPE];
    unsigned char hk_lfr_last_exe_tc_subtype[BYTES_PER_SUBTYPE];
    unsigned char hk_lfr_last_exe_tc_time[BYTES_PER_TIME];
    unsigned char hk_lfr_last_rej_tc_id[BYTES_PER_PACKETID];
    unsigned char hk_lfr_last_rej_tc_type[BYTES_PER_TYPE];
    unsigned char hk_lfr_last_rej_tc_subtype[BYTES_PER_SUBTYPE];
    unsigned char hk_lfr_last_rej_tc_time[BYTES_PER_TIME];
    // anomaly statistics
    unsigned char hk_lfr_le_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_me_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_he_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_last_er_rid[COUNTER_2_BYTES];
    unsigned char hk_lfr_last_er_code;
    unsigned char hk_lfr_last_er_time[BYTES_PER_TIME];
    // vhdl_blk_status
    unsigned char hk_lfr_vhdl_aa_sm;
    unsigned char hk_lfr_vhdl_fft_sr;
    unsigned char hk_lfr_vhdl_cic_hk;
    unsigned char hk_lfr_vhdl_iir_cal;
    // spacewire_if_statistics
    unsigned char hk_lfr_dpu_spw_pkt_rcv_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_dpu_spw_pkt_sent_cnt[COUNTER_2_BYTES];
    unsigned char hk_lfr_dpu_spw_tick_out_cnt;
    unsigned char hk_lfr_dpu_spw_last_timc;
    // ahb error statistics
    unsigned char hk_lfr_last_fail_addr[BYTES_PER_ADDR];
    // temperatures
    unsigned char hk_lfr_temp_scm[BYTES_PER_TEMP];
    unsigned char hk_lfr_temp_pcb[BYTES_PER_TEMP];
    unsigned char hk_lfr_temp_fpga[BYTES_PER_TEMP];
    // spacecraft potential
    unsigned char hk_lfr_sc_v_f3[BYTES_PER_V];
    unsigned char hk_lfr_sc_e1_f3[BYTES_PER_V];
    unsigned char hk_lfr_sc_e2_f3[BYTES_PER_V];
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
    // reaction wheel frequency
    unsigned char hk_lfr_sc_rw1_rw2_f_flags;
    unsigned char hk_lfr_sc_rw3_rw4_f_flags;
} Packet_TM_LFR_HK_t;

//***************
//***************
// PARAMETER_DUMP

#define BIT_PAS_FILTER_ENABLED 0x01
#define BIT_CWF_LONG_F3        0x01

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    unsigned char sid;

    //******************
    // COMMON PARAMETERS
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;

    //******************
    // NORMAL PARAMETERS
    unsigned char sy_lfr_n_swf_l[PARAM_2_BYTES];
    unsigned char sy_lfr_n_swf_p[PARAM_2_BYTES];
    unsigned char sy_lfr_n_asm_p[PARAM_2_BYTES];
    unsigned char sy_lfr_n_bp_p0;
    unsigned char sy_lfr_n_bp_p1;
    unsigned char sy_lfr_n_cwf_long_f3;
    unsigned char pa_rpw_spare8_1;

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
    unsigned char sy_lfr_fbins_f0_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f0_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f0_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f0_word4[BYTES_PER_WORD];
    // mask F1
    unsigned char sy_lfr_fbins_f1_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f1_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f1_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f1_word4[BYTES_PER_WORD];
    // mask F2
    unsigned char sy_lfr_fbins_f2_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f2_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f2_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_fbins_f2_word4[BYTES_PER_WORD];

    // PAS FILTER PARAMETERS
    unsigned char pa_rpw_spare8_2;
    unsigned char spare_sy_lfr_pas_filter_enabled;
    unsigned char sy_lfr_pas_filter_modulus;
    unsigned char sy_lfr_pas_filter_tbad[PARAM_4_BYTES];
    unsigned char sy_lfr_pas_filter_offset;
    unsigned char sy_lfr_pas_filter_shift[PARAM_4_BYTES];
    unsigned char sy_lfr_sc_rw_delta_f[PARAM_4_BYTES];

    // SY_LFR_RWi_Kj REACTION WHEELS K COEFFICIENTS
    // RW1_K
    unsigned char sy_lfr_rw1_k1[4];
    unsigned char sy_lfr_rw1_k2[4];
    unsigned char sy_lfr_rw1_k3[4];
    unsigned char sy_lfr_rw1_k4[4];
    // RW2_K
    unsigned char sy_lfr_rw2_k1[4];
    unsigned char sy_lfr_rw2_k2[4];
    unsigned char sy_lfr_rw2_k3[4];
    unsigned char sy_lfr_rw2_k4[4];
    // RW3_K
    unsigned char sy_lfr_rw3_k1[4];
    unsigned char sy_lfr_rw3_k2[4];
    unsigned char sy_lfr_rw3_k3[4];
    unsigned char sy_lfr_rw3_k4[4];
    // RW4_K
    unsigned char sy_lfr_rw4_k1[4];
    unsigned char sy_lfr_rw4_k2[4];
    unsigned char sy_lfr_rw4_k3[4];
    unsigned char sy_lfr_rw4_k4[4];

    // LFR_RW_MASK REACTION WHEELS MASKS
    unsigned char sy_lfr_rw_mask_f0_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f0_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f0_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f0_word4[BYTES_PER_WORD];
    // mask F1
    unsigned char sy_lfr_rw_mask_f1_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f1_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f1_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f1_word4[BYTES_PER_WORD];
    // mask F2
    unsigned char sy_lfr_rw_mask_f2_word1[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f2_word2[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f2_word3[BYTES_PER_WORD];
    unsigned char sy_lfr_rw_mask_f2_word4[BYTES_PER_WORD];

    // SPARE
    unsigned char pa_rpw_spare8_3;
} Packet_TM_LFR_PARAMETER_DUMP_t;

//**************************
//**************************
// TM_LFR_KCOEFFICIENTS_DUMP


#define KCOEFF_BLK_MAX_SZ  3900
#define KCOEFF_BLK_NR_PKT1 26
#define KCOEFF_BLK_NR_PKT2 13
#define KCOEFF_PKTCNT      2
#define PKTNR_1            1
#define PKTNR_2            2

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    unsigned char sid;
    unsigned char pkt_cnt;
    unsigned char pkt_nr;
    unsigned char blk_nr;

    //******************
    // SOURCE DATA repeated N times with N in [0 .. PA_LFR_KCOEFF_BLK_NR]
    // one blk is 2 + 4 * 32 = 130 bytes, 30 blks max in one packet (30 * 130 =
    // 3900)
    unsigned char kcoeff_blks[KCOEFF_BLK_MAX_SZ];

} Packet_TM_LFR_KCOEFFICIENTS_DUMP_t;

#define MAX_SRC_DATA            780 // MAX size is 26 bins * 30 Bytes [TM_LFR_SCIENCE_BURST_BP2_F1]
#define MAX_SRC_DATA_WITH_SPARE 143 // 13 bins  * 11 Bytes

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char pa_lfr_bp_blk_nr[BYTES_PER_BLKNR];
    // SOURCE DATA
    unsigned char data[MAX_SRC_DATA]; // MAX size is 26 bins * 30 Bytes
                                      // [TM_LFR_SCIENCE_BURST_BP2_F1]
} bp_packet;

typedef struct
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
    unsigned char packetID[BYTES_PER_PACKETID];
    unsigned char packetSequenceControl[BYTES_PER_SEQ_CTRL];
    unsigned char packetLength[BYTES_PER_PKT_LEN];
    // DATA FIELD HEADER
    unsigned char spare1_pusVersion_spare2;
    unsigned char serviceType;
    unsigned char serviceSubType;
    unsigned char destinationID;
    unsigned char time[BYTES_PER_TIME];
    // AUXILIARY HEADER
    unsigned char sid;
    unsigned char pa_bia_status_info;
    unsigned char sy_lfr_common_parameters_spare;
    unsigned char sy_lfr_common_parameters;
    unsigned char acquisitionTime[BYTES_PER_TIME];
    unsigned char source_data_spare;
    unsigned char pa_lfr_bp_blk_nr[BYTES_PER_BLKNR];
    // SOURCE DATA
    unsigned char data[MAX_SRC_DATA_WITH_SPARE]; // 13 bins  * 11 Bytes
} bp_packet_with_spare; // only for TM_LFR_SCIENCE_NORMAL_BP1_F0 and F1

#endif // CCSDS_TYPES_H_INCLUDED

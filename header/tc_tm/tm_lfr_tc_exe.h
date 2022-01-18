#ifndef TM_LFR_TC_EXE_H_INCLUDED
#define TM_LFR_TC_EXE_H_INCLUDED

#include <ccsds_types.h>
#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"

extern unsigned short sequenceCounters_TC_EXE[];

int send_tm_lfr_tc_exe_success(const ccsdsTelecommandPacket_t * const TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_inconsistent(const ccsdsTelecommandPacket_t * const TC, rtems_id queue_id,
    unsigned char byte_position, unsigned char rcv_value);
int send_tm_lfr_tc_exe_not_executable(const ccsdsTelecommandPacket_t * const TC, rtems_id queue_id);
#ifdef ENABLE_DEAD_CODE
int send_tm_lfr_tc_exe_not_implemented(
    ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
#endif
int send_tm_lfr_tc_exe_error(const ccsdsTelecommandPacket_t * const TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_corrupted(const ccsdsTelecommandPacket_t * const TC, rtems_id queue_id,
    const unsigned char * const computed_CRC, const unsigned char * const currentTC_LEN_RCV, unsigned char destinationID);

void increment_seq_counter_destination_id(
    unsigned char* packet_sequence_control, unsigned char destination_id);

#endif // TM_LFR_TC_EXE_H_INCLUDED

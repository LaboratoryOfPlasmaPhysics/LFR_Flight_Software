#ifndef TM_LFR_TC_EXE_H_INCLUDED
#define TM_LFR_TC_EXE_H_INCLUDED

#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"

extern time_management_regs_t *time_management_regs;
extern Packet_TM_LFR_HK_t housekeeping_packet;

int send_tm_lfr_tc_exe_success(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_inconsistent(ccsdsTelecommandPacket_t *TC, rtems_id queue_id,
                                    unsigned char byte_position, unsigned char rcv_value);
int send_tm_lfr_tc_exe_not_executable(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_not_implemented(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_error(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_corrupted(ccsdsTelecommandPacket_t *TC, rtems_id queue_id,
                                 unsigned char *computed_CRC, unsigned char *currentTC_LEN_RCV);

#endif // TM_LFR_TC_EXE_H_INCLUDED




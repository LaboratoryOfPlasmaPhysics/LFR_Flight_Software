#ifndef TC_LOAD_DUMP_PARAMETERS_H
#define TC_LOAD_DUMP_PARAMETERS_H

#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"
#include "wf_handler.h"
#include "tm_lfr_tc_exe.h"

extern int fdSPW;
extern unsigned char lfrCurrentMode;
extern Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
extern Packet_TM_LFR_HK_t housekeeping_packet;

int action_load_common_par( ccsdsTelecommandPacket_t *TC );
int action_load_normal_par( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int action_load_burst_par( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int action_load_sbm1_par( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int action_load_sbm2_par( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int action_dump_par(rtems_id queue_id );

int set_sy_lfr_n_swf_l( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int set_sy_lfr_n_swf_p( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int set_sy_lfr_n_asm_p( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int set_sy_lfr_n_bp_p0( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int set_sy_lfr_n_bp_p1( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );

#endif // TC_LOAD_DUMP_PARAMETERS_H

#ifndef TC_LOAD_DUMP_PARAMETERS_H
#define TC_LOAD_DUMP_PARAMETERS_H

#include "tc_handler.h"

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
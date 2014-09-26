#ifndef TC_LOAD_DUMP_PARAMETERS_H
#define TC_LOAD_DUMP_PARAMETERS_H

#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"
#include "wf_handler.h"
#include "tm_lfr_tc_exe.h"
#include "fsw_misc.h"

#define FLOAT_EQUAL_ZERO 0.001

extern unsigned short sequenceCounterParameterDump;

int action_load_common_par( ccsdsTelecommandPacket_t *TC );
int action_load_normal_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_burst_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_sbm1_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_sbm2_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_dump_par(rtems_id queue_id );

// NORMAL
int check_common_par_consistency( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int set_sy_lfr_n_swf_l( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_n_swf_p( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_n_asm_p( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_n_bp_p0( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_n_bp_p1( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_n_cwf_long_f3( ccsdsTelecommandPacket_t *TC );

// BURST
int set_sy_lfr_b_bp_p0( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_b_bp_p1( ccsdsTelecommandPacket_t *TC );

// SBM1
int set_sy_lfr_s1_bp_p0( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_s1_bp_p1( ccsdsTelecommandPacket_t *TC );

// SBM2
int set_sy_lfr_s2_bp_p0( ccsdsTelecommandPacket_t *TC );
int set_sy_lfr_s2_bp_p1( ccsdsTelecommandPacket_t *TC );

// TC_LFR_UPDATE_INFO
unsigned int check_update_info_hk_lfr_mode( unsigned char mode );
unsigned int check_update_info_hk_tds_mode( unsigned char mode );
unsigned int check_update_info_hk_thr_mode( unsigned char mode );

void init_parameter_dump( void );

#endif // TC_LOAD_DUMP_PARAMETERS_H

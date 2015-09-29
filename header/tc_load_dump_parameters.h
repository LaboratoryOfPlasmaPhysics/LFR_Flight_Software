#ifndef TC_LOAD_DUMP_PARAMETERS_H
#define TC_LOAD_DUMP_PARAMETERS_H

#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"
#include "wf_handler.h"
#include "tm_lfr_tc_exe.h"
#include "fsw_misc.h"
#include "basic_parameters_params.h"
#include "avf0_prc0.h"

#define FLOAT_EQUAL_ZERO 0.001

extern unsigned short sequenceCounterParameterDump;
extern unsigned short sequenceCounters_TM_DUMP[];
extern float k_coeff_intercalib_f0_norm[ ];
extern float k_coeff_intercalib_f0_sbm[ ];
extern float k_coeff_intercalib_f1_norm[ ];
extern float k_coeff_intercalib_f1_sbm[ ];
extern float k_coeff_intercalib_f2[ ];

int action_load_common_par( ccsdsTelecommandPacket_t *TC );
int action_load_normal_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_burst_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_sbm1_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_sbm2_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id , unsigned char *time);
int action_load_kcoefficients(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_load_fbins_mask(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_dump_kcoefficients(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_dump_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id );

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

// FBINS_MASK
int set_sy_lfr_fbins( ccsdsTelecommandPacket_t *TC );

// KCOEFFICIENTS
int set_sy_lfr_kcoeff(ccsdsTelecommandPacket_t *TC , rtems_id queue_id);
void copyFloatByChar( unsigned char *destination, unsigned char *source );

void init_parameter_dump( void );
void init_kcoefficients_dump( void );
void init_kcoefficients_dump_packet( Packet_TM_LFR_KCOEFFICIENTS_DUMP_t *kcoefficients_dump, unsigned char pkt_nr, unsigned char blk_nr );
void increment_seq_counter_destination_id_dump( unsigned char *packet_sequence_control, unsigned char destination_id );

#endif // TC_LOAD_DUMP_PARAMETERS_H

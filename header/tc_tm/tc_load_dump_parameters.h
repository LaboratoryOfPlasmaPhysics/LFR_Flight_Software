#ifndef TC_LOAD_DUMP_PARAMETERS_H
#define TC_LOAD_DUMP_PARAMETERS_H

#include <rtems.h>
#include <stdio.h>

#include "avf0_prc0.h"
#include "basic_parameters_params.h"
#include "fsw_misc.h"
#include "fsw_params.h"
#include "tm_lfr_tc_exe.h"

#define FLOAT_EQUAL_ZERO  0.001
#define NB_BINS_TO_REMOVE 3
#define FI_INTERVAL_COEFF 0.285
#define BIN_MIN           0
#define BIN_MAX           127
#define DELTAF_F0         96.
#define DELTAF_F1         16.
#define DELTAF_F2         1.

#define WHEEL_1                1
#define WHEEL_2                2
#define WHEEL_3                3
#define WHEEL_4                4
#define FREQ_1                 1
#define FREQ_2                 2
#define FREQ_3                 3
#define FREQ_4                 4
#define FLAG_OFFSET_WHEELS_1_3 8
#define FLAG_OFFSET_WHEELS_2_4 4

#define FLAG_NAN 0 // Not A NUMBER
#define FLAG_IAN 1 // Is A Number

#define SBM_KCOEFF_PER_NORM_KCOEFF 2

extern unsigned short sequenceCounterParameterDump;
extern unsigned short sequenceCounters_TM_DUMP[];

extern fbins_masks_t fbins_masks;

int action_load_common_par(ccsdsTelecommandPacket_t* TC);
int action_load_normal_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_burst_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_sbm1_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_sbm2_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_kcoefficients(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_fbins_mask(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_load_filter_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id);
int action_dump_kcoefficients(ccsdsTelecommandPacket_t* TC, rtems_id queue_id, unsigned char* time);
int action_dump_par(ccsdsTelecommandPacket_t* TC, rtems_id queue_id);

// NORMAL
int check_normal_par_consistency(ccsdsTelecommandPacket_t* TC, rtems_id queue_id);
int set_sy_lfr_n_swf_l(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_n_swf_p(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_n_asm_p(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_n_bp_p0(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_n_bp_p1(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_n_cwf_long_f3(ccsdsTelecommandPacket_t* TC);

// BURST
int set_sy_lfr_b_bp_p0(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_b_bp_p1(ccsdsTelecommandPacket_t* TC);

// SBM1
int set_sy_lfr_s1_bp_p0(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_s1_bp_p1(ccsdsTelecommandPacket_t* TC);

// SBM2
int set_sy_lfr_s2_bp_p0(ccsdsTelecommandPacket_t* TC);
int set_sy_lfr_s2_bp_p1(ccsdsTelecommandPacket_t* TC);

// TC_LFR_UPDATE_INFO
unsigned int check_update_info_hk_lfr_mode(unsigned char mode);
unsigned int check_update_info_hk_tds_mode(unsigned char mode);
unsigned int check_update_info_hk_thr_mode(unsigned char mode);
void set_hk_lfr_sc_rw_f_flag(unsigned char wheel, unsigned char freq, float value);
void set_hk_lfr_sc_rw_f_flags(void);
int check_sy_lfr_rw_f(ccsdsTelecommandPacket_t* TC, int offset, int* pos, float* value);
int check_all_sy_lfr_rw_f(ccsdsTelecommandPacket_t* TC, int* pos, float* value);
void getReactionWheelsFrequencies(ccsdsTelecommandPacket_t* TC);
void setFBinMask(unsigned char* fbins_mask, float rw_f, unsigned char deltaFreq, float sy_lfr_rw_k);
void build_sy_lfr_rw_mask(unsigned int channel);
void build_sy_lfr_rw_masks();
void merge_fbins_masks(void);

// FBINS_MASK
int set_sy_lfr_fbins(ccsdsTelecommandPacket_t* TC);

// TC_LFR_LOAD_PARS_FILTER_PAR
int check_sy_lfr_rw_k(ccsdsTelecommandPacket_t* TC, int offset, int* pos, float* value);
int check_all_sy_lfr_rw_k(ccsdsTelecommandPacket_t* TC, int* pos, float* value);
int check_sy_lfr_filter_parameters(ccsdsTelecommandPacket_t* TC, rtems_id queue_id);

// KCOEFFICIENTS
int set_sy_lfr_kcoeff(ccsdsTelecommandPacket_t* TC, rtems_id queue_id);
void copyFloatByChar(unsigned char* destination, unsigned char* source);
void copyInt32ByChar(unsigned char* destination, unsigned char* source);
void copyInt16ByChar(unsigned char* destination, unsigned char* source);
void floatToChar(float value, unsigned char* ptr);

void init_parameter_dump(void);
void init_kcoefficients_dump(void);
void init_kcoefficients_dump_packet(Packet_TM_LFR_KCOEFFICIENTS_DUMP_t* kcoefficients_dump,
    unsigned char pkt_nr, unsigned char blk_nr);
void increment_seq_counter_destination_id_dump(
    unsigned char* packet_sequence_control, unsigned char destination_id);

#endif // TC_LOAD_DUMP_PARAMETERS_H

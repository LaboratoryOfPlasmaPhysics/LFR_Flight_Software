#include <fsw_processing.h>
#include <rtems.h>
#include <grspw.h>
#include <ccsds_types.h>

// RTEMS GLOBAL VARIABLES
rtems_name misc_name[5];
rtems_name misc_id[5];
rtems_id   Task_id[15];         /* array of task ids */
rtems_name Task_name[15];       /* array of task names */
int fdSPW = 0;
int fdUART = 0;

spectral_matrices_regs_t *spectral_matrices_regs = NULL;

// APB CONFIGURATION REGISTERS
time_management_regs_t *time_management_regs = (time_management_regs_t*) REGS_ADDR_TIME_MANAGEMENT;
waveform_picker_regs_t *waveform_picker_regs = (waveform_picker_regs_t*) REGS_ADDR_WAVEFORM_PICKER;
gptimer_regs_t         *gptimer_regs         = (gptimer_regs_t *)        REGS_ADDR_GPTIMER;

// WAVEFORMS GLOBAL VARIABLES // 2048 * 3 * 4 + 2 * 4 = 24576 + 8 bytes
volatile int wf_snap_f0[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f1[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f1_bis[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f2[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f2_bis[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_cont_f3[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];

// SPECTRAL MATRICES GLOBAL VARIABLES
volatile int spec_mat_f0_a[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_b[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_c[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_d[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_e[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_f[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_g[ TOTAL_SIZE_SPEC_MAT ];
volatile int spec_mat_f0_h[ TOTAL_SIZE_SPEC_MAT ];
//
float averaged_spec_mat_f0[ TOTAL_SIZE_SPEC_MAT ];
float compressed_spec_mat_f0[ TOTAL_SIZE_COMPRESSED_MATRIX_f0 ];

// MODE PARAMETERS
struct param_local_str param_local;
struct param_common_str param_common;
struct param_norm_str param_norm;
struct param_burst_str param_burst;
struct param_sbm1_str param_sbm1;
struct param_sbm2_str param_sbm2;

// HK PACKETS
Packet_TM_LFR_HK_t housekeeping_packet;
// sequence counters are incremented by APID (PID + CAT) and destination ID
unsigned short sequenceCounters[SEQ_CNT_NB_PID][SEQ_CNT_NB_CAT][SEQ_CNT_NB_DEST_ID];
spw_stats spacewire_stats;
spw_stats spacewire_stats_backup;

// BASIC PARAMETERS GLOBAL VARIABLES
unsigned char LFR_BP1_F0[ NB_BINS_COMPRESSED_MATRIX_f0 * 9 ];
BP1_t data_BP1[ NB_BINS_COMPRESSED_MATRIX_f0 ];


//#include <fsw_processing.h>
#include <rtems.h>
#include <grspw.h>
#include <ccsds_types.h>
#include <grlib_regs.h>
#include <fsw_params.h>

// RTEMS GLOBAL VARIABLES
rtems_name misc_name[5];
rtems_name misc_id[5];
rtems_id   Task_id[15];         /* array of task ids */
rtems_name Task_name[15];       /* array of task names */
int fdSPW = 0;
int fdUART = 0;

// APB CONFIGURATION REGISTERS
time_management_regs_t *time_management_regs = (time_management_regs_t*) REGS_ADDR_TIME_MANAGEMENT;
gptimer_regs_t         *gptimer_regs         = (gptimer_regs_t *)        REGS_ADDR_GPTIMER;
#ifdef GSA
#else
    waveform_picker_regs_t *waveform_picker_regs = (waveform_picker_regs_t*) REGS_ADDR_WAVEFORM_PICKER;
#endif
spectral_matrix_regs_t *spectral_matrix_regs = (spectral_matrix_regs_t*) REGS_ADDR_SPECTRAL_MATRIX;

// WAVEFORMS GLOBAL VARIABLES // 2048 * 3 * 4 + 2 * 4 = 24576 + 8 bytes
volatile int wf_snap_f0[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f1[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f1_bis[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f2[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_snap_f2_bis[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];
volatile int wf_cont_f3[ NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET];

// SPECTRAL MATRICES GLOBAL VARIABLES
volatile int spec_mat_f0_a[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_b[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_c[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_d[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_e[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_f[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_g[ TOTAL_SIZE_SM ];
volatile int spec_mat_f0_h[ TOTAL_SIZE_SM ];
//
volatile int spec_mat_f1_a[ TOTAL_SIZE_SM ];
volatile int spec_mat_f2_a[ TOTAL_SIZE_SM ];
//
volatile int spec_mat_f0_a_bis[ TOTAL_SIZE_SM ];
volatile int spec_mat_f1_a_bis[ TOTAL_SIZE_SM ];
volatile int spec_mat_f2_a_bis[ TOTAL_SIZE_SM ];

// MODE PARAMETERS
Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
struct param_local_str param_local;

// HK PACKETS
Packet_TM_LFR_HK_t housekeeping_packet;
// sequence counters are incremented by APID (PID + CAT) and destination ID
unsigned short sequenceCounters[SEQ_CNT_NB_PID][SEQ_CNT_NB_CAT][SEQ_CNT_NB_DEST_ID];
spw_stats spacewire_stats;
spw_stats spacewire_stats_backup;



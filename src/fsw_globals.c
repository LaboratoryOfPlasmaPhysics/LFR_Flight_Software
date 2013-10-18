/** Global variables of the LFR flight software.
 *
 * @file
 * @author P. LEROY
 *
 * Among global variables, there are:
 * - RTEMS names and id.
 * - APB configuration registers.
 * - waveforms global buffers, used by the waveform picker hardware module to store data.
 * - spectral  matrices buffesr, used by the hardware module to store data.
 * - variable related to LFR modes parameters.
 * - the global HK packet buffer.
 * - the global dump parameter buffer.
 *
 */

#include <rtems.h>
#include <grspw.h>

#include "ccsds_types.h"
#include "grlib_regs.h"
#include "fsw_params.h"

// RTEMS GLOBAL VARIABLES
rtems_name  misc_name[5];
rtems_id    misc_id[5];
rtems_name  Task_name[20];       /* array of task names */
rtems_id    Task_id[20];         /* array of task ids */
unsigned int maxCount;
int fdSPW = 0;
int fdUART = 0;
unsigned char lfrCurrentMode;

// APB CONFIGURATION REGISTERS
time_management_regs_t *time_management_regs = (time_management_regs_t*) REGS_ADDR_TIME_MANAGEMENT;
gptimer_regs_t         *gptimer_regs         = (gptimer_regs_t *)        REGS_ADDR_GPTIMER;
#ifdef GSA
#else
    waveform_picker_regs_t *waveform_picker_regs = (waveform_picker_regs_t*) REGS_ADDR_WAVEFORM_PICKER;
#endif
spectral_matrix_regs_t *spectral_matrix_regs = (spectral_matrix_regs_t*) REGS_ADDR_SPECTRAL_MATRIX;

// WAVEFORMS GLOBAL VARIABLES // 2048 * 3 * 4 + 2 * 4 = 24576 + 8 bytes
volatile int wf_snap_f0[       NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
//
volatile int wf_snap_f1[       NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
volatile int wf_snap_f1_bis[   NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
volatile int wf_snap_f1_norm[  NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
//
volatile int wf_snap_f2[       NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
volatile int wf_snap_f2_bis[   NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
volatile int wf_snap_f2_norm[  NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
//
volatile int wf_cont_f3[       NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
volatile int wf_cont_f3_bis[   NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK + TIME_OFFSET ];
char         wf_cont_f3_light[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_CWF3_LIGHT_BLK        ];

// SPECTRAL MATRICES GLOBAL VARIABLES
volatile int spec_mat_f0_0[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_1[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_a[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_b[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_c[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_d[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_e[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_f[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_g[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_h[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_0_bis[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f0_1_bis[ SM_HEADER + TOTAL_SIZE_SM ];
//
volatile int spec_mat_f1[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f1_bis[ SM_HEADER + TOTAL_SIZE_SM ];
//
volatile int spec_mat_f2[ SM_HEADER + TOTAL_SIZE_SM ];
volatile int spec_mat_f2_bis[ SM_HEADER + TOTAL_SIZE_SM ];

// MODE PARAMETERS
Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
struct param_local_str param_local;

// HK PACKETS
Packet_TM_LFR_HK_t housekeeping_packet;
// sequence counters are incremented by APID (PID + CAT) and destination ID
unsigned short sequenceCounters[SEQ_CNT_NB_PID][SEQ_CNT_NB_CAT][SEQ_CNT_NB_DEST_ID];
spw_stats spacewire_stats;
spw_stats spacewire_stats_backup;



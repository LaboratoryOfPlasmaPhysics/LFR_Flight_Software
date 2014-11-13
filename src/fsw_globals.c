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
#include "fsw_params_wf_handler.h"

// RTEMS GLOBAL VARIABLES
rtems_name  misc_name[5];
rtems_id    misc_id[5];
rtems_name  Task_name[20];       /* array of task names */
rtems_id    Task_id[20];         /* array of task ids */
unsigned int maxCount;
int fdSPW = 0;
int fdUART = 0;
unsigned char lfrCurrentMode;

// WAVEFORMS GLOBAL VARIABLES   // 2048 * 3 * 4 + 2 * 4 = 24576 + 8 bytes = 24584
                                //  97 * 256 = 24832 => delta = 248 bytes = 62 words
// WAVEFORMS GLOBAL VARIABLES   // 2688 * 3 * 4 + 2 * 4 = 32256 + 8 bytes = 32264
                                // 127 * 256 = 32512 => delta = 248 bytes = 62 words
// F0 F1 F2 F3
volatile int wf_buffer_f0[ NB_RING_NODES_F0 * WFRM_BUFFER ] __attribute__((aligned(0x100)));
volatile int wf_buffer_f1[ NB_RING_NODES_F1 * WFRM_BUFFER ] __attribute__((aligned(0x100)));
volatile int wf_buffer_f2[ NB_RING_NODES_F2 * WFRM_BUFFER ] __attribute__((aligned(0x100)));
volatile int wf_buffer_f3[ NB_RING_NODES_F3 * WFRM_BUFFER ] __attribute__((aligned(0x100)));
char         wf_cont_f3_light[ (NB_SAMPLES_PER_SNAPSHOT) * NB_BYTES_CWF3_LIGHT_BLK + TIME_OFFSET_IN_BYTES ] __attribute__((aligned(0x100)));

//***********************************
// SPECTRAL MATRICES GLOBAL VARIABLES

// alignment constraints for the spectral matrices buffers => the first data after the time (8 bytes) shall be aligned on 0x00
volatile int sm_f0[ NB_RING_NODES_SM_F0 * TOTAL_SIZE_SM ] __attribute__((aligned(0x100)));
volatile int sm_f1[ NB_RING_NODES_SM_F1 * TOTAL_SIZE_SM ] __attribute__((aligned(0x100)));
volatile int sm_f2[ NB_RING_NODES_SM_F2 * TOTAL_SIZE_SM ] __attribute__((aligned(0x100)));

// APB CONFIGURATION REGISTERS
time_management_regs_t          *time_management_regs   = (time_management_regs_t*)         REGS_ADDR_TIME_MANAGEMENT;
gptimer_regs_t                  *gptimer_regs           = (gptimer_regs_t *)                REGS_ADDR_GPTIMER;
waveform_picker_regs_0_1_18_t   *waveform_picker_regs   = (waveform_picker_regs_0_1_18_t*)  REGS_ADDR_WAVEFORM_PICKER;
spectral_matrix_regs_t          *spectral_matrix_regs   = (spectral_matrix_regs_t*)         REGS_ADDR_SPECTRAL_MATRIX;

// MODE PARAMETERS
Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;
struct param_local_str param_local;

// HK PACKETS
Packet_TM_LFR_HK_t housekeeping_packet;
// sequence counters are incremented by APID (PID + CAT) and destination ID
unsigned short sequenceCounters_SCIENCE_NORMAL_BURST;
unsigned short sequenceCounters_SCIENCE_SBM1_SBM2;
unsigned short sequenceCounters_TC_EXE[SEQ_CNT_NB_DEST_ID];
unsigned short sequenceCounterHK;
unsigned short sequenceCounterParameterDump;
spw_stats spacewire_stats;
spw_stats spacewire_stats_backup;



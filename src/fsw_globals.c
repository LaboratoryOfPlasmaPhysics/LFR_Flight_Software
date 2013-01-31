#include <fsw_processing.h>
#include <rtems.h>

// RTEMS GLOBAL VARIABLES
rtems_name misc_names[5];

// WAVEFORMS GLOBAL VARIABLES
volatile int wf_snap_f0[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int wf_snap_f1[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int wf_snap_f2[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int wf_cont_f3[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes

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

// BASIC PARAMETERS GLOBAL VAIRABLES
unsigned char LFR_BP1_F0[ NB_BINS_COMPRESSED_MATRIX_f0 * 9 ];

BP1_t data_BP1[ NB_BINS_COMPRESSED_MATRIX_f0 ];

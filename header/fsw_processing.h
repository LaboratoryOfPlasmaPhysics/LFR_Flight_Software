#ifndef FSW_PROCESSING_H_INCLUDED
#define FSW_PROCESSING_H_INCLUDED

#include <rtems.h>
#include <grspw.h>
#include <math.h>
#include <stdlib.h> // abs() is in the stdlib
#include <stdio.h>  // printf()
#include <math.h>

#include "fsw_params.h"
#include "fsw_spacewire.h"

extern volatile int spec_mat_f0_0[ ];
extern volatile int spec_mat_f0_1[ ];
extern volatile int spec_mat_f0_a[ ];
extern volatile int spec_mat_f0_b[ ];
extern volatile int spec_mat_f0_c[ ];
extern volatile int spec_mat_f0_d[ ];
extern volatile int spec_mat_f0_e[ ];
extern volatile int spec_mat_f0_f[ ];
extern volatile int spec_mat_f0_g[ ];
extern volatile int spec_mat_f0_h[ ];

extern volatile int spec_mat_f1[ ];
extern volatile int spec_mat_f2[ ];

extern volatile int spec_mat_f1_bis[ ];
extern volatile int spec_mat_f2_bis[ ];
extern volatile int spec_mat_f0_0_bis[ ];
extern volatile int spec_mat_f0_1_bis[ ];

// parameters
extern struct param_local_str param_local;

// registers
extern time_management_regs_t *time_management_regs;
extern spectral_matrix_regs_t *spectral_matrix_regs;

extern rtems_name  misc_name[5];
extern rtems_id    Task_id[20];         /* array of task ids */

// ISR
rtems_isr spectral_matrices_isr( rtems_vector_number vector );
rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector );

// RTEMS TASKS
rtems_task spw_bppr_task(rtems_task_argument argument);
rtems_task avf0_task(rtems_task_argument argument);
rtems_task bpf0_task(rtems_task_argument argument);
rtems_task smiq_task(rtems_task_argument argument); // added to test the spectral matrix simulator
rtems_task matr_task(rtems_task_argument argument);

void matrix_compression(volatile float *averaged_spec_mat, unsigned char fChannel, float *compressed_spec_mat);
void matrix_reset(volatile float *averaged_spec_mat);
void BP1_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1);
void BP2_set(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat);
//
void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header);
void send_spectral_matrix(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id);
void convert_averaged_spectral_matrix(volatile float *input_matrix, char *output_matrix);
void fill_averaged_spectral_matrix( void );
void reset_spectral_matrix_regs();

#endif // FSW_PROCESSING_H_INCLUDED

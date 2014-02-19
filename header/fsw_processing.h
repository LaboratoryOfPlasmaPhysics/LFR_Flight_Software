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
#include "basic_parameters.h"

extern volatile int sm_f0[ ][ SM_HEADER + TOTAL_SIZE_SM ];
extern volatile int sm_f1[ ][ SM_HEADER + TOTAL_SIZE_SM ];
extern volatile int sm_f2[ ][ SM_HEADER + TOTAL_SIZE_SM ];

// parameters
extern struct param_local_str param_local;

// registers
extern time_management_regs_t *time_management_regs;
extern spectral_matrix_regs_t *spectral_matrix_regs;

extern rtems_name  misc_name[5];
extern rtems_id    Task_id[20];         /* array of task ids */

void init_sm_rings( void );
void reset_current_sm_ring_nodes( void );

// ISR
void reset_nb_sm_f0( void );
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
void BP1_set_old(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat, unsigned char * LFR_BP1);
void BP2_set_old(float * compressed_spec_mat, unsigned char nb_bins_compressed_spec_mat);
//
void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header);
void send_spectral_matrix(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id);
void convert_averaged_spectral_matrix(volatile float *input_matrix, char *output_matrix);
void fill_averaged_spectral_matrix( void );
void reset_spectral_matrix_regs();

#endif // FSW_PROCESSING_H_INCLUDED

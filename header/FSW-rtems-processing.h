#ifndef FSW_RTEMS_PROCESSING_H_INCLUDED
#define FSW_RTEMS_PROCESSING_H_INCLUDED

#include <rtems.h>

#define ADDRESS_APB_SPECTRAL_MATRICES 0x80000700
#define LPP_SPECTRAL_MATRIX_CTRL 0x80000700
#define LPP_SPECTRAL_MATRIX_1 0x80000704
#define LPP_SPECTRAL_MATRIX_2 0x80000708
#define IRQ_SPECTRAL_MATRICES 12

#define NB_BINS_SPECTRAL_MATRIX 128
#define NB_VALUES_PER_SPECTRAL_MATRIX 25
#define TOTAL_SIZE_SPECTRAL_MATRIX NB_BINS_SPECTRAL_MATRIX * NB_VALUES_PER_SPECTRAL_MATRIX
#define NB_BINS_COMPRESSED_MATRIX_f0 11
#define SIZE_COMPRESSED_SPECTRAL_MATRIX_f1 13
#define SIZE_COMPRESSED_SPECTRAL_MATRIX_f2 12
#define TOTAL_SIZE_COMPRESSED_MATRIX_f0 NB_BINS_COMPRESSED_MATRIX_f0 * NB_VALUES_PER_SPECTRAL_MATRIX
#define NB_AVERAGE_NORMAL_f0 96*4

rtems_isr spectral_matrices_isr( rtems_vector_number vector );
rtems_task spw_bppr_task(rtems_task_argument argument);
rtems_task spw_avf0_task(rtems_task_argument argument);
rtems_task spw_bpf0_task(rtems_task_argument argument);
rtems_task spw_bppr_task_rate_monotonic(rtems_task_argument argument);
void matrix_average(volatile int *spectral_matrix, float *averaged_spectral_matrix);
void matrix_compression(float *averaged_spectral_matrix, unsigned char fChannel, float *compressed_spectral_matrix);
void matrix_reset(float *averaged_spectral_matrix);
void BP1_set(float * compressed_spectral_matrix, unsigned char nb_bins_compressed_spectral_matrix, unsigned char * LFR_BP1);
void BP2_set(float * compressed_spectral_matrix, unsigned char nb_bins_compressed_spectral_matrix);

#endif // FSW_RTEMS_PROCESSING_H_INCLUDED

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

typedef struct ring_node_sm
{
    struct ring_node_sm *previous;
    int buffer_address;
    struct ring_node_sm *next;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
} ring_node_sm;

typedef struct ring_node_bp
{
    struct ring_node_bp *previous;
    struct ring_node_bp *next;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
    Header_TM_LFR_SCIENCE_BP_t header;
    unsigned char data[ 30 * 22 ];   // MAX size is 22 * 30 TM_LFR_SCIENCE_BURST_BP2_F1
} ring_node_bp;

typedef struct ring_node_bp_with_spare
{
    struct ring_node_bp_with_spare *previous;
    struct ring_node_bp_with_spare *next;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
    Header_TM_LFR_SCIENCE_BP_with_spare_t header;
    unsigned char data[ 9 * 22 ];
} ring_node_bp_with_spare;

extern volatile int sm_f0[ ];
extern volatile int sm_f1[ ];
extern volatile int sm_f2[ ];

// parameters
extern struct param_local_str param_local;

// registers
extern time_management_regs_t *time_management_regs;
extern spectral_matrix_regs_t *spectral_matrix_regs;

extern rtems_name  misc_name[5];
extern rtems_id    Task_id[20];         /* array of task ids */

void init_sm_rings( void );
void reset_current_sm_ring_nodes( void );
void reset_current_bp_ring_nodes( void );

// ISR
void reset_nb_sm_f0( void );
rtems_isr spectral_matrices_isr( rtems_vector_number vector );
rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector );

// RTEMS TASKS
rtems_task avf0_task(rtems_task_argument argument);
rtems_task smiq_task(rtems_task_argument argument); // added to test the spectral matrix simulator
rtems_task matr_task(rtems_task_argument argument);

//*****************************
// Spectral matrices processing

void SM_average(float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int firstTimeF0, unsigned int firstTimeF1 );
void ASM_reorganize_and_divide(float *averaged_spec_mat, float *averaged_spec_mat_reorganized,
                               float divider );
void ASM_compress_reorganize_and_divide(float *averaged_spec_mat, float *compressed_spec_mat,
                                  float divider,
                                  unsigned char nbBinsCompressedMatrix, unsigned char nbBinsToAverage , unsigned char ASMIndexStart);
void ASM_convert(volatile float *input_matrix, char *output_matrix);
void ASM_send(Header_TM_LFR_SCIENCE_ASM_t *header, char *spectral_matrix,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send, rtems_id queue_id);

void BP_send(char *data,
               rtems_id queue_id ,
               unsigned int nbBytesToSend );

void init_header_asm( Header_TM_LFR_SCIENCE_ASM_t *header);
void init_bp_ring_sbm1_bp1( void );
void init_bp_ring_sbm1_bp2( void );
void init_headers_bp_ring_sbm1_bp1();
void init_header_bp(Header_TM_LFR_SCIENCE_BP_t *header,
                    unsigned int apid, unsigned char sid,
                    unsigned int packetLength , unsigned char blkNr);
void init_header_bp_with_spare(Header_TM_LFR_SCIENCE_BP_with_spare_t *header,
                               unsigned int apid, unsigned char sid,
                               unsigned int packetLength, unsigned char blkNr );

void reset_spectral_matrix_regs( void );

void set_time( unsigned char *time, unsigned int coarseTime, unsigned int fineTime );

#endif // FSW_PROCESSING_H_INCLUDED

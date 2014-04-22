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
    struct ring_node_sm *next;
    int buffer_address;
    unsigned int status;
    unsigned int coarseTime;
    unsigned int fineTime;
} ring_node_sm;

typedef struct ring_node_asm
{
    struct ring_node_asm *previous;
    struct ring_node_asm *next;
    float  matrix[ TOTAL_SIZE_SM ];
    unsigned int status;
} ring_node_asm;

typedef struct bp_packet
{
    Header_TM_LFR_SCIENCE_BP_t header;
    unsigned char data[ 30 * 22 ];   // MAX size is 22 * 30 [TM_LFR_SCIENCE_BURST_BP2_F1]
} bp_packet;

typedef struct bp_packet_with_spare
{
    Header_TM_LFR_SCIENCE_BP_with_spare_t header;
    unsigned char data[ 9 * 13 ];   // only for TM_LFR_SCIENCE_NORMAL_BP1_F0 and F1
} bp_packet_with_spare;

typedef struct asm_msg
{
    ring_node_asm *norm_f0;
    ring_node_asm *burst_sbmf0;
    rtems_event_set event;
    unsigned int coarseTime;
    unsigned int fineTime;
} asm_msg;

extern nb_sm_t nb_sm;
extern nb_sm_before_bp_t nb_sm_before_bp;

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

// ISR
void reset_nb_sm_f0( unsigned char lfrMode );
rtems_isr spectral_matrices_isr( rtems_vector_number vector );
rtems_isr spectral_matrices_isr_simu( rtems_vector_number vector );

// RTEMS TASKS
rtems_task smiq_task( rtems_task_argument argument ); // added to test the spectral matrix simulator
rtems_task avf0_task( rtems_task_argument lfrRequestedMode );
rtems_task matr_task( rtems_task_argument lfrRequestedMode );

//******************
// Spectral Matrices
void SM_init_rings( void );
void ASM_init_rings( void );
void SM_reset_current_ring_nodes( void );
void ASM_reset_current_ring_node( void );
void ASM_init_header( Header_TM_LFR_SCIENCE_ASM_t *header);
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

//*****************
// Basic Parameters

void BP_reset_current_ring_nodes( void );
void BP_init_header(Header_TM_LFR_SCIENCE_BP_t *header,
                    unsigned int apid, unsigned char sid,
                    unsigned int packetLength , unsigned char blkNr);
void BP_init_header_with_spare(Header_TM_LFR_SCIENCE_BP_with_spare_t *header,
                               unsigned int apid, unsigned char sid,
                               unsigned int packetLength, unsigned char blkNr );
void BP_send(char *data,
               rtems_id queue_id ,
               unsigned int nbBytesToSend );

//******************
// general functions
void reset_spectral_matrix_regs( void );
void set_time(unsigned char *time, unsigned char *timeInBuffer );

extern rtems_status_code get_message_queue_id_matr( rtems_id *queue_id );

#endif // FSW_PROCESSING_H_INCLUDED

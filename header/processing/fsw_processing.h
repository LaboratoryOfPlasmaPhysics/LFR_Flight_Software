#ifndef FSW_PROCESSING_H_INCLUDED
#define FSW_PROCESSING_H_INCLUDED

#include <rtems.h>

#include <grlib_regs.h>
#include <grspw.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h> // abs() is in the stdlib

#include "ASM/spectralmatrices.h"
#include "fsw_params.h"

#define SBM_COEFF_PER_NORM_COEFF 2

extern unsigned char thisIsAnASMRestart;

extern volatile int sm_f0[];
extern volatile int sm_f1[];
extern volatile int sm_f2[];
extern unsigned int acquisitionDurations[];

// parameters
extern struct param_local_str param_local;
extern Packet_TM_LFR_PARAMETER_DUMP_t parameter_dump_packet;

// registers
extern time_management_regs_t *time_management_regs;
extern volatile spectral_matrix_regs_t *spectral_matrix_regs;

extern rtems_name misc_name[];
extern rtems_id Task_id[]; /* array of task ids */

ring_node *getRingNodeForAveraging(unsigned char frequencyChannel);
// ISR
rtems_isr spectral_matrices_isr(rtems_vector_number vector);

//******************
// Spectral Matrices
void reset_nb_sm(void);
// SM
void SM_init_rings(void);
void SM_reset_current_ring_nodes(void);
// ASM
void ASM_generic_init_ring(ring_node_asm *ring, unsigned char nbNodes);

//*****************
// Basic Parameters

void BP_reset_current_ring_nodes(void);
void BP_init_header(bp_packet *packet, unsigned int apid, unsigned char sid,
                    unsigned int packetLength, unsigned char blkNr);
void BP_init_header_with_spare(bp_packet_with_spare *packet, unsigned int apid,
                               unsigned char sid, unsigned int packetLength,
                               unsigned char blkNr);
void BP_send(char *data, rtems_id queue_id, unsigned int nbBytesToSend,
             unsigned int sid);
void BP_send_s1_s2(char *data, rtems_id queue_id, unsigned int nbBytesToSend,
                   unsigned int sid);

//******************
// general functions
void reset_sm_status(void);
void reset_spectral_matrix_regs(void);
void set_time(unsigned char *time, unsigned char *timeInBuffer);
unsigned long long int get_acquisition_time(unsigned char *timePtr);
unsigned char getSID(rtems_event_set event);

extern rtems_status_code get_message_queue_id_prc1(rtems_id *queue_id);
extern rtems_status_code get_message_queue_id_prc2(rtems_id *queue_id);

void init_kcoeff_sbm_from_kcoeff_norm(float *input_kcoeff, float *output_kcoeff,
                                      unsigned char nb_bins_norm);

#endif // FSW_PROCESSING_H_INCLUDED

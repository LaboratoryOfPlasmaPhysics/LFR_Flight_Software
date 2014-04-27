#ifndef AVF0_PRC0_H
#define AVF0_PRC0_H

#include "fsw_processing.h"

typedef struct {
    unsigned int norm_bp1;
    unsigned int norm_bp2;
    unsigned int norm_asm;
    unsigned int burst_sbm_bp1;
    unsigned int burst_sbm_bp2;
    unsigned int burst_bp1;
    unsigned int burst_bp2;
    unsigned int sbm1_bp1;
    unsigned int sbm1_bp2;
    unsigned int sbm2_bp1;
    unsigned int sbm2_bp2;
} nb_sm_before_bp_asm_f0;

extern struct ring_node_sm *current_ring_node_sm_f0;
extern struct ring_node_sm *ring_node_for_averaging_sm_f0;

extern rtems_status_code get_message_queue_id_prc0( rtems_id *queue_id );

void reset_nb_sm_f0( unsigned char lfrMode );

rtems_task avf0_task( rtems_task_argument lfrRequestedMode );
rtems_task prc0_task( rtems_task_argument lfrRequestedMode );

#endif // AVF0_PRC0_H

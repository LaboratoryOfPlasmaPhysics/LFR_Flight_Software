#ifndef AVF1_PRC1_H
#define AVF1_PRC1_H

#include "fsw_processing.h"

typedef struct {
    unsigned int norm_bp1;
    unsigned int norm_bp2;
    unsigned int norm_asm;
    unsigned int burst_sbm_bp1;
    unsigned int burst_sbm_bp2;
    unsigned int burst_bp1;
    unsigned int burst_bp2;
    unsigned int sbm2_bp1;
    unsigned int sbm2_bp2;
} nb_sm_before_bp_asm_f1;

extern struct ring_node_sm *ring_node_for_averaging_sm_f1;

extern rtems_status_code get_message_queue_id_prc1( rtems_id *queue_id );

void reset_nb_sm_f1( unsigned char lfrMode );
void SM_average_f1( float *averaged_spec_mat_f0, float *averaged_spec_mat_f1,
                  ring_node_sm *ring_node_tab[],
                  unsigned int nbAverageNormF0, unsigned int nbAverageSBM1F0 );

rtems_task avf1_task( rtems_task_argument lfrRequestedMode );
rtems_task prc1_task( rtems_task_argument lfrRequestedMode );

#endif // AVF1_PRC1_H

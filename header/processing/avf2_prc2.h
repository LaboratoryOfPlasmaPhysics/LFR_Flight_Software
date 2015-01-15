#ifndef AVF2_PRC2_H
#define AVF2_PRC2_H

#include "fsw_processing.h"
#include "basic_parameters.h"

typedef struct {
    unsigned int norm_bp1;
    unsigned int norm_bp2;
    unsigned int norm_asm;
} nb_sm_before_bp_asm_f2;

//************
// RTEMS TASKS
rtems_task avf2_task( rtems_task_argument lfrRequestedMode );
rtems_task prc2_task( rtems_task_argument lfrRequestedMode );

//**********
// FUNCTIONS

void reset_nb_sm_f2( void );
void SM_average_f2(float *averaged_spec_mat_f2, ring_node *ring_node, unsigned int nbAverageNormF2 , asm_msg *msgForMATR);
void init_k_coefficients_f2( void );

//*******
// EXTERN
extern rtems_status_code get_message_queue_id_prc2( rtems_id *queue_id );

#endif // AVF2_PRC2_H

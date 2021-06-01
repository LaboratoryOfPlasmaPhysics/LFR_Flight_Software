#ifndef AVF1_PRC1_H
#define AVF1_PRC1_H

#include "basic_parameters.h"
#include "fsw_init.h"
#include "fsw_processing.h"

typedef struct
{
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

//************
// RTEMS TASKS
rtems_task avf1_task(rtems_task_argument lfrRequestedMode);
rtems_task prc1_task(rtems_task_argument lfrRequestedMode);

//**********
// FUNCTIONS

void reset_nb_sm_f1(unsigned char lfrMode);
void init_k_coefficients_prc1(void);

//*******
// EXTERN
extern rtems_status_code get_message_queue_id_prc1(rtems_id* queue_id);

#endif // AVF1_PRC1_H

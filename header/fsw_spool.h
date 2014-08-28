#ifndef FSW_SPOOL_H_INCLUDED
#define FSW_SPOOL_H_INCLUDED

#include <rtems.h>
#include <stdio.h>

#include "fsw_params.h"
#include "fsw_processing.h"

rtems_name name_spool_rate_monotonic;   // name of the SPOOL rate monotonic
rtems_id spool_period_id;               // id of the SPOOL rate monotonic period

extern unsigned char lfrCurrentMode;

extern waveform_picker_regs_new_t *waveform_picker_regs;
extern spectral_matrix_regs_t *spectral_matrix_regs;

// WAVEFORMS
extern ring_node *current_ring_node_f0;
extern ring_node *ring_node_to_send_swf_f0;
extern ring_node *current_ring_node_f1;
extern ring_node *ring_node_to_send_swf_f1;
extern ring_node *ring_node_to_send_cwf_f1;
extern ring_node *current_ring_node_f2;
extern ring_node *ring_node_to_send_swf_f2;
extern ring_node *ring_node_to_send_cwf_f2;
extern ring_node *current_ring_node_f3;
extern ring_node *ring_node_to_send_cwf_f3;

// SPECTRAL MATRICES
unsigned int spool_nb_sm_f0;
unsigned int spool_nb_sm_f1;
extern ring_node_sm *current_ring_node_sm_f0;
extern ring_node_sm *current_ring_node_sm_f1;
extern ring_node_sm *current_ring_node_sm_f2;
extern ring_node_sm *ring_node_for_averaging_sm_f0;
extern ring_node_sm *ring_node_for_averaging_sm_f1;
extern ring_node_sm *ring_node_for_averaging_sm_f2;

extern rtems_id    Task_id[];   /* array of task ids */

extern bool swf_f0_ready;
extern bool swf_f1_ready;
extern bool swf_f2_ready;

extern bool wake_up_task_wfrm;
extern bool wake_up_task_cwf_f1;
extern bool wake_up_task_cwf_f2_burst;
extern bool wake_up_task_cwf_f2_sbm2;
extern bool wake_up_task_cwf_f3;

//***********
// RTEMS_TASK
rtems_task spoo_task( rtems_task_argument argument );

// OTHER FUNCTIONS
void spool_waveforms( void );
void spool_spectral_matrices_f0( void );
void spool_spectral_matrices_f1( void );
void spool_spectral_matrices_f2( void );
void spool_spectral_matrices( void );
void spool_reset_nb_sm( void );

#endif // FSW_SPOOL_H_INCLUDED

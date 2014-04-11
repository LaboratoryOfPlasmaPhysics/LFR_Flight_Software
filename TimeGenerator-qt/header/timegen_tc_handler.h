#ifndef TIMEGEN_TC_HANDLER_H_INCLUDED
#define TIMEGEN_TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "tc_load_dump_parameters.h"
#include "tc_acceptance.h"
#include "tm_lfr_tc_exe.h"

// MODE PARAMETERS
extern unsigned int maxCount;

//****
// ISR
rtems_isr commutation_isr1( rtems_vector_number vector );
rtems_isr commutation_isr2( rtems_vector_number vector );

//***********
// RTEMS TASK
rtems_task actn_task( rtems_task_argument unused );

//***********
// TC ACTIONS
int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_enable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_disable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int action_update_time(ccsdsTelecommandPacket_t *TC);

// mode transition
int transition_validation(unsigned char requestedMode);
int stop_current_mode( void );
int enter_mode(unsigned char mode);
int restart_science_tasks();
int suspend_science_tasks();
void launch_waveform_picker( unsigned char mode );
void launch_spectral_matrix( unsigned char mode );
void set_irq_on_new_ready_matrix(unsigned char value );
void set_run_matrix_spectral( unsigned char value );
void launch_spectral_matrix_simu( unsigned char mode );

// other functions
void updateLFRCurrentMode();
void update_last_TC_exe(ccsdsTelecommandPacket_t *TC );
void update_last_TC_rej(ccsdsTelecommandPacket_t *TC );
void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id );

extern rtems_status_code get_message_queue_id_send( rtems_id *queue_id );
extern rtems_status_code get_message_queue_id_recv( rtems_id *queue_id );

#endif // TIMEGEN_TC_HANDLER_H_INCLUDED



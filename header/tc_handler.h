#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "tc_load_dump_parameters.h"
#include "tc_acceptance.h"
#include "tm_lfr_tc_exe.h"
#include "wf_handler.h"
#include "fsw_processing.h"

#include "lfr_cpu_usage_report.h"

//****
// ISR
rtems_isr commutation_isr1( rtems_vector_number vector );
rtems_isr commutation_isr2( rtems_vector_number vector );

//***********
// RTEMS TASK
rtems_task actn_task( rtems_task_argument unused );

//***********
// TC ACTIONS
int action_reset( ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time );
int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_update_info( ccsdsTelecommandPacket_t *TC, rtems_id queue_id );
int action_enable_calibration( ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time );
int action_disable_calibration( ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time );
int action_update_time( ccsdsTelecommandPacket_t *TC);

// mode transition
int check_mode_value( unsigned char requestedMode );
int check_mode_transition( unsigned char requestedMode );
int check_transition_date( unsigned int transitionCoarseTime );
int stop_current_mode( void );
int enter_mode( unsigned char mode , unsigned int transitionCoarseTime );
int restart_science_tasks( unsigned char lfrRequestedMode );
int suspend_science_tasks();
void launch_waveform_picker( unsigned char mode , unsigned int transitionCoarseTime );
void launch_spectral_matrix( void );
void launch_spectral_matrix_simu( void );
void set_sm_irq_onNewMatrix( unsigned char value );
void set_sm_irq_onError( unsigned char value );

// other functions
void updateLFRCurrentMode();
void set_lfr_soft_reset( unsigned char value );
void reset_lfr( void );
// CALIBRATION
void setCalibrationPrescaler( unsigned int prescaler );
void setCalibrationDivisor( unsigned int divisionFactor );
void setCalibrationData( void );
void setCalibrationReload( bool state);
void setCalibrationEnable( bool state );
void setCalibrationInterleaved( bool state );
void setCalibration( bool state );
void configureCalibration( bool interleaved );
//
void update_last_TC_exe( ccsdsTelecommandPacket_t *TC , unsigned char *time );
void update_last_TC_rej(ccsdsTelecommandPacket_t *TC , unsigned char *time );
void close_action( ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id );

extern rtems_status_code get_message_queue_id_send( rtems_id *queue_id );
extern rtems_status_code get_message_queue_id_recv( rtems_id *queue_id );

#endif // TC_HANDLER_H_INCLUDED




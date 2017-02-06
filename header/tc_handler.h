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

#define MAX_DELTA_COARSE_TIME   3
#define NB_SCIENCE_TASKS        10
#define NB_ASM_TASKS            6
#define STATUS_0    0
#define STATUS_1    1
#define STATUS_2    2
#define STATUS_3    3
#define STATUS_4    4
#define STATUS_5    5
#define STATUS_6    6
#define STATUS_7    7
#define STATUS_8    8
#define STATUS_9    9

#define CAL_F0              625
#define CAL_F1              10000
#define CAL_FS              160256.410
#define CAL_SCALE_FACTOR    (0.250 / 0.000654) // 191, 500 mVpp, 2 sinus waves => 500 mVpp each, amplitude = 250 mV
#define CAL_NB_PTS          256
#define CAL_DATA_MASK       0xfff
#define CAL_F_DIVISOR       38  // 25 MHz => 160 256 (39 - 1)
// INTERLEAVED MODE
#define CAL_FS_INTER          240384.615
#define CAL_NB_PTS_INTER      384
#define CAL_DATA_MASK_INTER   0x3f
#define CAL_DATA_SHIFT_INTER  12
#define BYTES_FOR_2_SAMPLES   3   // one need 3 bytes = 24 bits to store 3 samples of 12 bits in interleaved mode
#define STEPS_FOR_STORAGE_INTER 128
#define CAL_F_DIVISOR_INTER 26  // 25 MHz => 240 384

extern unsigned int lastValidEnterModeTime;
extern unsigned char oneTcLfrUpdateTimeReceived;

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
void update_last_valid_transition_date( unsigned int transitionCoarseTime );
int check_transition_date( unsigned int transitionCoarseTime );
int stop_spectral_matrices( void );
int stop_current_mode( void );
int enter_mode_standby(void );
int enter_mode_normal( unsigned int transitionCoarseTime );
int enter_mode_burst( unsigned int transitionCoarseTime );
int enter_mode_sbm1( unsigned int transitionCoarseTime );
int enter_mode_sbm2( unsigned int transitionCoarseTime );
int restart_science_tasks( unsigned char lfrRequestedMode );
int restart_asm_tasks(unsigned char lfrRequestedMode );
int suspend_science_tasks(void);
int suspend_asm_tasks( void );
void launch_waveform_picker( unsigned char mode , unsigned int transitionCoarseTime );
void launch_spectral_matrix( void );
void set_sm_irq_onNewMatrix( unsigned char value );
void set_sm_irq_onError( unsigned char value );

// other functions
void updateLFRCurrentMode(unsigned char requestedMode);
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




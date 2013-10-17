#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include "fsw_init.h"
#include "tc_load_dump_parameters.h"
#include "tm_lfr_tc_exe.h"

// MODE PARAMETERS
extern struct param_sbm1_str param_sbm1;
extern struct param_sbm2_str param_sbm2;
extern time_management_regs_t *time_management_regs;
extern waveform_picker_regs_t *waveform_picker_regs;
extern gptimer_regs_t         *gptimer_regs;

//****
// ISR
rtems_isr commutation_isr1( rtems_vector_number vector );
rtems_isr commutation_isr2( rtems_vector_number vector );

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt( unsigned char D, unsigned int Chk);
void initLookUpTableForCRC( void );
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);
void updateLFRCurrentMode();

//*********************
// ACCEPTANCE FUNCTIONS
int tc_acceptance(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV, rtems_id queue_queu_id, rtems_id queue_pkts_id);
int tc_parser(ccsdsTelecommandPacket_t * TCPacket, unsigned int TC_LEN_RCV);
int tc_check_type( unsigned char packetType );
int tc_check_subtype( unsigned char packetType );
int tc_check_length( unsigned char packetType, unsigned int length );
int tc_check_crc( ccsdsTelecommandPacket_t * TCPacket, unsigned int length );

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused );
rtems_task actn_task( rtems_task_argument unused );
rtems_task dumb_task( rtems_task_argument unused );

//***********
// TC ACTIONS
int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_update_info(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_enable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_disable_calibration(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_update_time(ccsdsTelecommandPacket_t *TC);

// mode transition
int transition_validation(unsigned char requestedMode);
int stop_current_mode();
int enter_mode(unsigned char mode, ccsdsTelecommandPacket_t *TC);
int enter_standby_mode();
int enter_normal_mode();
int enter_burst_mode();
int enter_sbm1_mode();
int enter_sbm2_mode();
int restart_science_tasks();
int suspend_science_tasks();

// other functions
void update_last_TC_exe(ccsdsTelecommandPacket_t *TC);
void update_last_TC_rej(ccsdsTelecommandPacket_t *TC);
void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id);

#endif // TC_HANDLER_H_INCLUDED




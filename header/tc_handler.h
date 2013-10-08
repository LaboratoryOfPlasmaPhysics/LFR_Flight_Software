#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <bsp.h>        // for the LEON_Unmask_interrupt function
#include <stdio.h>
#include <unistd.h>     // for the read call
#include <sys/ioctl.h>  // for the ioctl call
#include <ccsds_types.h>
#include <grspw.h>

#include "fsw_init.h"
#include "fsw_misc.h"

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
int TC_acceptance(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV, rtems_id queue_id);
unsigned char TC_parser(ccsdsTelecommandPacket_t * TMPacket, unsigned int TC_LEN_RCV);

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int packetLength,
                              TMHeader_t *TMHeader, unsigned char tc_sid);

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused );
rtems_task actn_task( rtems_task_argument unused );
rtems_task dumb_task( rtems_task_argument unused );

//***********
// TC ACTIONS
int action_reset(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_load_common_par(ccsdsTelecommandPacket_t *TC);
int action_load_normal_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_load_burst_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_load_sbm1_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_load_sbm2_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int action_dump_par(ccsdsTelecommandPacket_t *TC);
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
int send_tm_lfr_tc_exe_success(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_not_executable(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_not_implemented(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);
int send_tm_lfr_tc_exe_error(ccsdsTelecommandPacket_t *TC, rtems_id queue_id);

#endif // TC_HANDLER_H_INCLUDED




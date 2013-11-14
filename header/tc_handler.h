#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <leon.h>

#include "tc_load_dump_parameters.h"
#include "tc_acceptance.h"
#include "tm_lfr_tc_exe.h"
#include "wf_handler.h"

// MODE PARAMETERS
extern struct param_sbm1_str param_sbm1;
extern struct param_sbm2_str param_sbm2;
extern time_management_regs_t *time_management_regs;
extern waveform_picker_regs_t *waveform_picker_regs;
extern gptimer_regs_t         *gptimer_regs;
extern rtems_name  misc_name[5];
extern rtems_id    Task_id[20];         /* array of task ids */
extern unsigned char lfrCurrentMode;
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
void updateLFRCurrentMode();
void update_last_TC_exe(ccsdsTelecommandPacket_t *TC, unsigned char *time);
void update_last_TC_rej(ccsdsTelecommandPacket_t *TC, unsigned char *time);
void close_action(ccsdsTelecommandPacket_t *TC, int result, rtems_id queue_id);

#endif // TC_HANDLER_H_INCLUDED




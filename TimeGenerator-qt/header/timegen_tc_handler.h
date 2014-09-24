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
rtems_task act__task( rtems_task_argument unused );

//***********
// TC ACTIONS
int timegen_action_enter_mode(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time);
int timegen_action_update_time(ccsdsTelecommandPacket_t *TC);

extern rtems_status_code get_message_queue_id_send( rtems_id *queue_id );
extern rtems_status_code get_message_queue_id_recv( rtems_id *queue_id );

#endif // TIMEGEN_TC_HANDLER_H_INCLUDED




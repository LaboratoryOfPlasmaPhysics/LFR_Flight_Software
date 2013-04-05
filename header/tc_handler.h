#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <bsp.h>        // for the LEON_Unmask_interrupt function
#include <stdio.h>
#include <unistd.h>     // for the read call
#include <sys/ioctl.h>  // for the ioctl call
#include <ccsds_types.h>
#include <grspw.h>
#include <fsw_init.h>

extern int fdSPW;
extern rtems_name misc_name[ ];
extern rtems_name misc_id[ ];
extern rtems_id Task_id[ ];         // array of task ids
// MODE PARAMETERS
extern struct param_norm_str param_norm;
extern struct param_sbm1_str param_sbm1;
extern struct param_sbm2_str param_sbm2;
extern time_management_regs_t *time_management_regs;
extern unsigned char param_common[];

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned char currentTC_COMPUTED_CRC[2];
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
unsigned char currentTC_processedFlag;

//****
// ISR
rtems_isr commutation_isr1( rtems_vector_number vector );
rtems_isr commutation_isr2( rtems_vector_number vector );

//**********************
// GENERAL USE FUNCTIONS
unsigned int lookUpTableForCRC[256];
void initLookUpTableForCRC();
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

//*********************
// ACCEPTANCE FUNCTIONS
unsigned char acceptTM(ccsdsTelecommandPacket_t * TMPacket, unsigned int tc_len_recv);

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int packetLength,
                              unsigned int coarseTime, unsigned int fineTime, TMHeader_t *TMHeader);
unsigned char TM_build_data(ccsdsTelecommandPacket_t *TC, char* data, unsigned int SID, unsigned char *computed_CRC);
int TC_checker(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV);

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused );
rtems_task actn_task( rtems_task_argument unused );
rtems_task dumb_task( rtems_task_argument unused );
int create_message_queue();

//***********
// TC ACTIONS
int action_default(ccsdsTelecommandPacket_t *TC);
int action_default_alt(ccsdsTelecommandPacket_t *TC);
int send_tm_lfr_tc_exe_success(ccsdsTelecommandPacket_t *TC);
//
int action_load_norm(ccsdsTelecommandPacket_t *TC);
int action_enter(ccsdsTelecommandPacket_t *TC);
int action_updt_time(ccsdsTelecommandPacket_t *TC);

#endif // TC_HANDLER_H_INCLUDED




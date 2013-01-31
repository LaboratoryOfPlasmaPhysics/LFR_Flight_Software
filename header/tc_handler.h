#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <rtems.h>
#include <stdio.h>
#include <unistd.h>     // for the read call
#include <sys/ioctl.h>  // for the ioctl call
#include <ccsds_types.h>
#include <grspw.h>
#include <fsw_init.h>

extern int fdSPW;
extern rtems_name misc_name[ ];
extern rtems_name misc_id[ ];
extern rtems_id Task_id[ ];         /* array of task ids */

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned char currentTC_COMPUTED_CRC[2];
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
unsigned char currentTC_processedFlag;

//**********************
// GENERAL USE FUNCTIONS
unsigned int lookUpTableForCRC[256];
void InitLookUpTableForCRC();
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

//*********************
// ACCEPTANCE FUNCTIONS
unsigned char acceptTM(ccsdsTelecommandPacket_t * TMPacket, unsigned int tc_len_recv);

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int SID, unsigned int packetLength,
                              unsigned int coarseTime, unsigned int fineTime, TMHeader_t *TMHeader);
unsigned char TM_build_data(ccsdsTelecommandPacket_t *TC, char* data, unsigned int SID, unsigned char *computed_CRC);
int TC_checker(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV);

//***********
// RTEMS TASK
rtems_task recv_task( rtems_task_argument unused );
rtems_task actn_task( rtems_task_argument unused );
int create_message_queue();

//***********
// TC ACTIONS
int default_action(ccsdsTelecommandPacket_t *TC);

#endif // TC_HANDLER_H_INCLUDED

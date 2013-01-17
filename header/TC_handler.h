#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <ccsds_types.h>

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned char currentTC_COMPUTED_CRC[2];
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
ccsdsTelemetryPacket_t currentTM;
ccsdsTelecommandPacket_t currentTC;
unsigned char currentTC_processedFlag;

unsigned int lookUpTableForCRC[256];
void InitLookUpTableForCRC();
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

unsigned char TM_checker(ccsdsTelecommandPacket_t * TMPacket);
unsigned char TM_acceptance_generator(ccsdsTelecommandPacket_t * TCPacket, unsigned int code, ccsdsTelemetryPacket_t * TMPacket);
unsigned char TM_not_implemented_generator(ccsdsTelecommandPacket_t * TCPacket, ccsdsTelemetryPacket_t * TMPacket);

#endif // TC_HANDLER_H_INCLUDED

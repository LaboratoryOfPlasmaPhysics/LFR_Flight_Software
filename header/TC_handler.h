#ifndef TC_HANDLER_H_INCLUDED
#define TC_HANDLER_H_INCLUDED

#include <stdio.h>
#include <ccsds_types.h>

unsigned char currentTC_LEN_RCV[2]; //  SHALL be equal to the current TC packet estimated packet length field
unsigned char currentTC_COMPUTED_CRC[2];
unsigned int currentTC_LEN_RCV_AsUnsignedInt;
unsigned int currentTM_length;
unsigned char currentTC_processedFlag;

unsigned int lookUpTableForCRC[256];
void InitLookUpTableForCRC();
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

unsigned char acceptTM(ccsdsTelecommandPacket_t * TMPacket, unsigned int TC_LEN_RCV);

unsigned char TM_build_header( enum TM_TYPE tm_type, unsigned int SID, unsigned int packetLength,
                              unsigned int coarseTime, unsigned int fineTime, TMHeader_t *TMHeader);
unsigned char TM_build_data(ccsdsTelecommandPacket_t *TC, char* data, unsigned int SID, unsigned char *computed_CRC);
unsigned int TC_checker(ccsdsTelecommandPacket_t *TC, unsigned int TC_LEN_RCV,
                         TMHeader_t *TM_Header, unsigned int *hlen, char *data);

#endif // TC_HANDLER_H_INCLUDED

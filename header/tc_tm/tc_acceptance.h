#ifndef TC_ACCEPTANCE_H_INCLUDED
#define TC_ACCEPTANCE_H_INCLUDED

#include "fsw_params.h"

#define BIT_0 0x01
#define BIT_1 0x02
#define BIT_2 0x04
#define BIT_3 0x08
#define BIT_4 0x10
#define BIT_5 0x20
#define BIT_6 0x40
#define BIT_7 0x80

#define CONST_CRC_0 0x1021
#define CONST_CRC_1 0x2042
#define CONST_CRC_2 0x4084
#define CONST_CRC_3 0x8108
#define CONST_CRC_4 0x1231
#define CONST_CRC_5 0x2462
#define CONST_CRC_6 0x48c4
#define CONST_CRC_7 0x9188

#define CRC_RESET 0xffff

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt(unsigned char D, unsigned int Chk);
void initLookUpTableForCRC(void);
void GetCRCAsTwoBytes(const unsigned char *data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

//*********************
// ACCEPTANCE FUNCTIONS
int tc_parser(const ccsdsTelecommandPacket_t * const TCPacket, unsigned int estimatedPacketLength,
    unsigned char* computed_CRC);
int tc_check_type(unsigned char packetType);
int tc_check_type_subtype(unsigned char packetType, unsigned char packetSubType);
int tc_check_sid(unsigned char sid);
int tc_check_length(unsigned char packetType, unsigned int length);
int tc_check_crc(const ccsdsTelecommandPacket_t * const TCPacket, unsigned int length, unsigned char* computed_CRC);

#endif // TC_ACCEPTANCE_H_INCLUDED

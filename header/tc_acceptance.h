#ifndef TC_ACCEPTANCE_H_INCLUDED
#define TC_ACCEPTANCE_H_INCLUDED

//#include "tm_lfr_tc_exe.h"
#include "fsw_params.h"

//**********************
// GENERAL USE FUNCTIONS
unsigned int Crc_opt( unsigned char D, unsigned int Chk);
void initLookUpTableForCRC( void );
void GetCRCAsTwoBytes(unsigned char* data, unsigned char* crcAsTwoBytes, unsigned int sizeOfData);

//*********************
// ACCEPTANCE FUNCTIONS
int tc_parser(ccsdsTelecommandPacket_t * TCPacket, unsigned int TC_LEN_RCV, unsigned char *computed_CRC);
int tc_check_type( unsigned char packetType );
int tc_check_type_subtype( unsigned char packetType, unsigned char packetSubType );
int tc_check_sid( unsigned char sid );
int tc_check_length( unsigned char packetType, unsigned int length );
int tc_check_crc(ccsdsTelecommandPacket_t * TCPacket, unsigned int length , unsigned char *computed_CRC);

#endif // TC_ACCEPTANCE_H_INCLUDED




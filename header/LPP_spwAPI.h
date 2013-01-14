#ifndef SPWAPI_LPP_H
#define SPWAPI_LPP_H

#include <asm-leon/leonstack.h>
#include <..\header\LPP_grspw.h>

#define LPP_TIME_MANAGER 0x80000600

struct spectral_matrices_regs_str
{
    volatile unsigned int ctrl;
    volatile unsigned int address1;
    volatile unsigned int address2;
};
typedef struct spectral_matrices_regs_str spectral_matrices_regs_t;

struct packet_type_str
{
    char time_packet;
    char tc_packet;
} packet_type;

struct spwHeader_str
{
    unsigned char targetLogicalAddress;
    unsigned char protocolIdentifier;
    unsigned char reserved;
    unsigned char userApplication;
};
typedef struct spwHeader_str spwHeader_t;

struct time_manager_regs_str
{
    volatile int ctrl;
    volatile int next_coarse_time;
    volatile int current_coarse_time;
    volatile int fine_time;
};
typedef struct time_manager_regs_str time_manager_regs_t;

// GENERAL PURPOSE FUNCTIONS

inline int loadmemAPI(int addr);

inline char loadb(int addr);

char *almalloc(int sz);

////////////////
// SPW FUNCTIONS
////////////////

void spw_reset(grspwregs_t *regs);

void spw_init(int nodeaddr, int clkdiv, int destkey, int rxmaxlen, grspwregs_t *regs);

void set_txd(char *dataAddress, unsigned int dataLength, char *headerAddress, unsigned int headerLength, volatile int *txd);

int spw_checkrx(int* size, volatile int *rxd, grspwregs_t *regs);

int spw_checktx(grspwregs_t *regs);

void enable_transmitter_descriptor(unsigned int headerLength, volatile int *txd);

void enable_transmitter(grspwregs_t *regs);

void enable_receiver_descriptor(volatile char *rx, volatile int *rxd);

void enable_receiver(grspwregs_t *regs);

void send_data(unsigned int headerLength, volatile int *txd, grspwregs_t *regs, char* tx, char value);

int enable_timecode_reception(grspwregs_t *regs);

int check_time(grspwregs_t *grspw_regs);

//////////////
// GENERAL USE
//////////////

void send_fine_time(unsigned int fine_time);

//////
// IRQ
//////

int irqhandler_receive_CCSDS(int irq, void * args, struct leonbare_pt_regs *leon_regs);

int enable_irq(int irq);

int disable_irq(int irq);

int force_irq(int irq);

#endif // SPWAPI_LPP_H

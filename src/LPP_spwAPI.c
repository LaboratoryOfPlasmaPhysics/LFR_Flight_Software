#include <..\header\LPP_spwAPI.h>
#include <stdlib.h>
#include <stdio.h>

volatile int *lreg = (int*) 0x80000200;  // the IRQMP controller registers are at 0x80000200

inline int loadmemAPI(int addr)
{
    int tmp;
    asm volatile (" lda [%1]1, %0 " // LDA => Load Word from Alternate space
        : "=r" (tmp)
        : "r" (addr)
    );
    return tmp;
}

inline char loadb(int addr)
{
    char tmp;
    asm volatile (" lduba [%1]1, %0 "
        : "=r" (tmp)
        : "r" (addr)
    );
    return tmp;
}

char *almalloc(int sz)
{
    char *tmp;
    tmp = (char*) malloc(2*sz);
    tmp = (char*) (((int)tmp+sz) & ~(sz-1));
    return(tmp);
}

//////
// SPW
//////

void spw_reset(grspwregs_t *regs)
{
    regs->ctrl = GRSPW_CTRL_RS; // make complete reset of the spacewire node, self clearing
}

void spw_init(int nodeaddr, int clkdiv, int destkey, int rxmaxlen, grspwregs_t *regs)
{
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE; // link start (LS)
                                                // RMAP enable (RE)
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE;
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE;
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE;
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE;
    regs->ctrl = GRSPW_CTRL_LS | GRSPW_CTRL_RE;
    regs->status = 0xffffffff;
    regs->nodeaddr = nodeaddr;
    regs->clkdiv = clkdiv;
    regs->destkey = destkey;
    regs->time = 0;
    regs->dmactrl = GRSPW_DMACTRL_RA | GRSPW_DMACTRL_TA | GRSPW_DMACTRL_PR | GRSPW_DMACTRL_PS | GRSPW_DMACTRL_AI;
    regs->rxmaxlen = rxmaxlen;
    regs->txdesc = 0;
    regs->rxdesc = 0;
}

void set_txd(char *dataAddress, unsigned int dataLength, char *headerAddress, unsigned int headerLength, volatile int *txd)
{
    txd[4] = 0;
    txd[3] = (int) dataAddress;     // data address
    txd[2] = dataLength;            // data length
    txd[1] = (int) headerAddress;   // header address
    txd[0] = (1 << 13) | (1 << 12) | headerLength;  // bit 13 WR = 1 next descriptor read will be 1st one in the table (at the base address)
                                                    // bit 12 EN = 1 enable transmitter descriptor
                                                    // bit 0 to 7 stand for the header length
}

int spw_checkrx(int* size, volatile int *rxd, grspwregs_t *regs)
{
    volatile int tmp = 0;
    tmp = regs->dmactrl;
    if ( ( (tmp >> 6) & 1 ) )
    { // bit 6 PR packet received
        *size = loadmemAPI((int) &rxd[0]) & 0x00001fff;
        regs->dmactrl = regs->dmactrl | 1 << 6; // clear bit 6 PR (packet received) by writing a one
        return 1;
    }
    else return 0;
}

int spw_checktx(grspwregs_t *regs)
{
    if ( ( (regs->dmactrl >> 5) & 1 )==1 )
    { // bit 5 PS (packet sent), cleared when written with a one
        regs->dmactrl = regs->dmactrl | 1 << 5; // clear bit 5 PS (packet sent) by writing a one
        return 1;
    }
    else return 0;
}

void enable_transmitter_descriptor(unsigned int headerLength, volatile int *txd)
{
    txd[0] = GRSPW_TXBD_WR | GRSPW_TXBD_EN;
    //WR = 1 next descriptor read will be 1st one in the table (at the base address)
    //EN = 1 enable transmitter descriptor
}

void enable_transmitter(grspwregs_t *regs)
{
    regs->dmactrl = regs->dmactrl | GRSPW_DMACTRL_TE;
}

void enable_receiver_descriptor(volatile char *rx, volatile int *rxd)
{
    rxd[1] = (int) rx;
    rxd[0] = GRSPW_RXBD_IE | GRSPW_RXBD_WR | GRSPW_RXBD_EN;
                                    // IE = 1 Interrupt enable
                                    // WR = 1 next descriptor will be the 1st one in the desc table (at the base address)
                                    // EN = 1 activate the descriptor
}

void enable_receiver(grspwregs_t *regs)
{
    regs->dmactrl = regs->dmactrl | 1 << 11 | 1 << 2;
}

int enable_timecode_reception(grspwregs_t *regs)
{
    regs->ctrl = regs->ctrl | GRSPW_CTRL_TR;
    return 1;
}

int check_time(grspwregs_t *grspw_regs)
{
        int tmp = loadmemAPI((int)&(grspw_regs->status)) & 1;
        if (tmp) {
                grspw_regs->status = loadmemAPI((int)&(grspw_regs->status)) | 1;
        }
        return tmp;
}

//////////////
// GENERAL USE
//////////////
void send_fine_time(unsigned int fine_time){
    spacewire_PARAMETERS.tx[0] = (char) fine_time; // send the fine time value to LPPMON
    spacewire_PARAMETERS.tx[1] = (char) (fine_time>>8);
    spacewire_PARAMETERS.tx[2] = (char) (fine_time>>16);
    spacewire_PARAMETERS.tx[3] = (char) (fine_time>>24);
    enable_transmitter_descriptor(HEADERLEN, spacewire_PARAMETERS.txd);   // enable the descriptor for transmission
    enable_transmitter(spacewire_PARAMETERS.regs);                        // enable the transmission
}

//////
// IRQ
//////

int irqhandler_receive_CCSDS(int irq, void * args, struct leonbare_pt_regs * leon_regs)
{
    if (spw_checkrx(spacewire_PARAMETERS.size, spacewire_PARAMETERS.rxd, spacewire_PARAMETERS.regs)==1)
    {
        packet_type.time_packet = 1;
        return 1;
    }
    return 0;
}

int enable_irq(int irq)
{
    lreg[ICLEAR/4] = (1 << irq); // clear any pending irq
    lreg[IMASK/4] = lreg[IMASK/4] | (1 << irq); // unmask irq
    return 1;
}

int disable_irq(int irq) { lreg[IMASK/4] &= ~(1 << irq); return 1;}

int force_irq(int irq) { lreg[IFORCE/4] = lreg[IFORCE/4] | (1 << irq); return 1;}



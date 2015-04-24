#include <stdio.h>

#include "grspw.h"
#include "fsw_params.h"

#define DSU_TIME_TAG_COUNTER 0x90000008

//**********
// IRQ LINES
#define IRQ_GRSPW 11
#define IRQ_SPARC_GRSPW 0x1b     // see sparcv8.pdf p.76 for interrupt levels

extern void *catch_interrupt(void func(), int irq);
int *lreg = (int *) 0x80000000;

#define ICLEAR 0x20c
#define IMASK  0x240
#define IFORCE 0x208

void enable_irq (int irq)
{
    lreg[ICLEAR/4] = (1 << irq);	// clear any pending irq
    lreg[IMASK/4] |= (1 << irq);	// unmaks irq
}

void disable_irq (int irq) { lreg[IMASK/4] &= ~(1 << irq); }	// mask irq

void force_irq (int irq) { lreg[IFORCE/4] = (1 << irq); }	// force irq

/* NOTE: NEVER put printf() or other stdio routines in interrupt handlers,
   they are not re-entrant. This (bad) example is just a demo */

unsigned char processTimecode = 0;

void irqhandler(int irq)
{
    processTimecode = 1;
}

int main( void )
{
    unsigned int *grspwCtrlReg;
    unsigned int k;
    volatile unsigned int *reg;
    float aux;
    unsigned int counter = 0;

    printf("hello world!\n");

    grspwCtrlReg = (unsigned int*) REGS_ADDR_GRSPW;
    grspw_set_ie( 1, grspwCtrlReg );
    grspw_set_tq( 1, grspwCtrlReg );
    grspw_set_tr( 1, grspwCtrlReg );

    catch_interrupt(irqhandler, IRQ_GRSPW);
    enable_irq( IRQ_GRSPW );
    force_irq( IRQ_GRSPW );

    reg = (volatile unsigned int *) DSU_TIME_TAG_COUNTER;

    while(1)
    {
        if (processTimecode == 1)
        {
            counter ++;
            printf("timecode counter = %d\n", counter);
            processTimecode = 0;
        }
        else
        {
            printf(".");
        }

//        for (k=0; k<100000;k++)
//        {
//            aux = aux + *reg ;
//        }
    }

    return 0;
}

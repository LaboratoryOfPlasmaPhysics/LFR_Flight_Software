#include <..\header\LPP_grspw.h>
#include <stdio.h>

#define REG_WRITE(addr, val) (*(volatile unsigned int *)(addr) = (unsigned int)(val))
#define REG_READ(addr) (*(volatile unsigned int *)(addr))

/* Return Current Link State */
unsigned int grspw_link_state(grspwregs_t *regs)
{
	return ((regs->status & GRSPW_STAT_LS) >> GRSPW_STAT_LS_BIT);
}

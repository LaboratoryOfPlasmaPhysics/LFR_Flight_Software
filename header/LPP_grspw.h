#ifndef GRSPW_H_INCLUDED
#define GRSPW_H_INCLUDED

#define GRSPW_APB_ADDR 0x80000500

#define NODEADDR 0xfe
#define NODEADDR_M7A3P1000 0xfd
#define CLKDIV 0x0303
#define DESTKEY 0
#define RXMAXLEN 64
#define HEADERLEN 4

#define IFORCE 0x08
#define ICLEAR 0x0c
#define IMASK 0x40
#define GRSPW_IRQ 10

#define SPW_LINK_STATE_RESET 0
#define SPW_LINK_STATE_WAIT 1
#define SPW_LINK_STATE_READY 2
#define SPW_LINK_STATE_STARTED 3
#define SPW_LINK_STATE_CONNECTING 4
#define SPW_LINK_STATE_RUN 5

/////////////////////////////////////
/* GRSPW - Control Register - 0x00 */
#define GRSPW_CTRL_RA_BIT	31  // RMAP avaialble
#define GRSPW_CTRL_RX_BIT	30  // RX unaligned access
#define GRSPW_CTRL_RC_BIT	29  // RMAP CRC available
#define GRSPW_CTRL_NCH_BIT	27  // 28:27 number of DMA channels minus 1
#define GRSPW_CTRL_PO_BIT	26  // number of ports minus 1
#define GRSPW_CTRL_PS_BIT	21  // Port select
#define GRSPW_CTRL_NP_BIT	20  // No port force
#define GRSPW_CTRL_RD_BIT	17  // RMAP buffer disable
#define GRSPW_CTRL_RE_BIT	16  // RMAP Enable
#define GRSPW_CTRL_TR_BIT	11  // Time Rx enable
#define GRSPW_CTRL_TT_BIT	10  // Time Tx enable
#define GRSPW_CTRL_LI_BIT	9   // Link error IRQ
#define GRSPW_CTRL_TQ_BIT	8   // Tick-out IRQ
#define GRSPW_CTRL_RS_BIT	6   // Reset
#define GRSPW_CTRL_PM_BIT	5   // Promiscuous mode
#define GRSPW_CTRL_TI_BIT	4   // Tick In
#define GRSPW_CTRL_IE_BIT	3   // Interrupt enable
#define GRSPW_CTRL_AS_BIT	2   // Autostart
#define GRSPW_CTRL_LS_BIT	1   // Link Start
#define GRSPW_CTRL_LD_BIT	0   // Lind Disable

#define GRSPW_CTRL_RA	(1<<GRSPW_CTRL_RA_BIT)
#define GRSPW_CTRL_RX	(1<<GRSPW_CTRL_RX_BIT)
#define GRSPW_CTRL_RC	(1<<GRSPW_CTRL_RC_BIT)
#define GRSPW_CTRL_NCH	(0x3<<GRSPW_CTRL_NCH_BIT)
#define GRSPW_CTRL_PO	(1<<GRSPW_CTRL_PO_BIT)
#define GRSPW_CTRL_PS	(1<<GRSPW_CTRL_PS_BIT)
#define GRSPW_CTRL_NP	(1<<GRSPW_CTRL_NP_BIT)
#define GRSPW_CTRL_RD	(1<<GRSPW_CTRL_RD_BIT)
#define GRSPW_CTRL_RE	(1<<GRSPW_CTRL_RE_BIT)
#define GRSPW_CTRL_TR	(1<<GRSPW_CTRL_TR_BIT)
#define GRSPW_CTRL_TT	(1<<GRSPW_CTRL_TT_BIT)
#define GRSPW_CTRL_LI	(1<<GRSPW_CTRL_LI_BIT)
#define GRSPW_CTRL_TQ	(1<<GRSPW_CTRL_TQ_BIT)
#define GRSPW_CTRL_RS	(1<<GRSPW_CTRL_RS_BIT)
#define GRSPW_CTRL_PM	(1<<GRSPW_CTRL_PM_BIT)
#define GRSPW_CTRL_TI	(1<<GRSPW_CTRL_TI_BIT)
#define GRSPW_CTRL_IE	(1<<GRSPW_CTRL_IE_BIT)
#define GRSPW_CTRL_AS	(1<<GRSPW_CTRL_AS_BIT)
#define GRSPW_CTRL_LS	(1<<GRSPW_CTRL_LS_BIT)
#define GRSPW_CTRL_LD	(1<<GRSPW_CTRL_LD_BIT)

////////////////////////////////////
/* GRSPW - Status Register - 0x04 */
#define GRSPW_STAT_LS_BIT	21  // Link State
#define GRSPW_STAT_AP_BIT	9   // Active Port
#define GRSPW_STAT_EE_BIT	8   // Early EOP-EEP
#define GRSPW_STAT_IA_BIT	7   // Invalid Address
#define GRSPW_STAT_PE_BIT	4   // Parity Error
#define GRSPW_STAT_DE_BIT	3   // Disconnect Error
#define GRSPW_STAT_ER_BIT	2   // Escape Error
#define GRSPW_STAT_CE_BIT	1   // Credit Error
#define GRSPW_STAT_TO_BIT	0   // Tick Out

#define GRSPW_STAT_LS	(0x7<<GRSPW_STAT_LS_BIT)
#define GRSPW_STAT_AP	(1<<GRSPW_STAT_AP_BIT)
#define GRSPW_STAT_EE	(1<<GRSPW_STAT_EE_BIT)
#define GRSPW_STAT_IA	(1<<GRSPW_STAT_IA_BIT)
#define GRSPW_STAT_WE	(1<<GRSPW_STAT_WE_BIT)
#define GRSPW_STAT_PE	(1<<GRSPW_STAT_PE_BIT)
#define GRSPW_STAT_DE	(1<<GRSPW_STAT_DE_BIT)
#define GRSPW_STAT_ER	(1<<GRSPW_STAT_ER_BIT)
#define GRSPW_STAT_CE	(1<<GRSPW_STAT_CE_BIT)
#define GRSPW_STAT_TO	(1<<GRSPW_STAT_TO_BIT)

/////////////////////////////////////////////
/* GRSPW - Default Address Register - 0x08 */
#define GRSPW_DEF_ADDR_BIT	0   // Default address
#define GRSPW_DEF_MASK_BIT	8   // Default mask
#define GRSPW_DEF_ADDR	(0xff<<GRSPW_DEF_ADDR_BIT)
#define GRSPW_DEF_MASK	(0xff<<GRSPW_DEF_MASK_BIT)

///////////////////////////////////////////
/* GRSPW - Clock Divisor Register - 0x0C */
#define GRSPW_CLKDIV_START_BIT	8   // Clock divisor startup
#define GRSPW_CLKDIV_RUN_BIT	0   // Clock divisor run
#define GRSPW_CLKDIV_START	(0xff<<GRSPW_CLKDIV_START_BIT)
#define GRSPW_CLKDIV_RUN	(0xff<<GRSPW_CLKDIV_RUN_BIT)
#define GRSPW_CLKDIV_MASK	(GRSPW_CLKDIV_START|GRSPW_CLKDIV_RUN)

/////////////////////////////////////////////
/* GRSPW - Destination key Register - 0x10 */
#define GRSPW_DK_DESTKEY_BIT	0   // Destination key
#define GRSPW_DK_DESTKEY	(0xff<<GRSPW_DK_DESTKEY_BIT)

//////////////////////////////////
/* GRSPW - Time Register - 0x14 */
#define GRSPW_TIME_CTRL_BIT	0   // Time counter
#define GRSPW_TIME_CNT_BIT	6   // Time control flags
#define GRSPW_TIME_CTRL		(0x3f<<GRSPW_TIME_CTRL_BIT)
#define GRSPW_TIME_TCNT		(0x3<<GRSPW_TIME_CNT_BIT)

///////////////////////////////////////////
/* GRSPW - DMA Control Register - 0x20*N */
#define GRSPW_DMACTRL_LE_BIT	16  // Link error disable
#define GRSPW_DMACTRL_SP_BIT	15  // Strip PID
#define GRSPW_DMACTRL_SA_BIT	14  // Strip address
#define GRSPW_DMACTRL_EN_BIT	13  // Enable address
#define GRSPW_DMACTRL_NS_BIT	12  // No spill
#define GRSPW_DMACTRL_RD_BIT	11  // Rx descriptors available
#define GRSPW_DMACTRL_RX_BIT	10  // RX active
#define GRSPW_DMACTRL_AT_BIT	9   // Abort TX
#define GRSPW_DMACTRL_RA_BIT	8   // RX AHB error
#define GRSPW_DMACTRL_TA_BIT	7   // TX AHB error
#define GRSPW_DMACTRL_PR_BIT	6   // Packet received
#define GRSPW_DMACTRL_PS_BIT	5   // Packet sent
#define GRSPW_DMACTRL_AI_BIT	4   // AHB error interrupt
#define GRSPW_DMACTRL_RI_BIT	3   // Receive interrupt
#define GRSPW_DMACTRL_TI_BIT	2   // Transmit interrupt
#define GRSPW_DMACTRL_RE_BIT	1   // Receiver enable
#define GRSPW_DMACTRL_TE_BIT	0   // Transmitter enable

#define GRSPW_DMACTRL_LE	(1<<GRSPW_DMACTRL_LE_BIT)
#define GRSPW_DMACTRL_SP	(1<<GRSPW_DMACTRL_SP_BIT)
#define GRSPW_DMACTRL_SA	(1<<GRSPW_DMACTRL_SA_BIT)
#define GRSPW_DMACTRL_EN	(1<<GRSPW_DMACTRL_EN_BIT)
#define GRSPW_DMACTRL_NS	(1<<GRSPW_DMACTRL_NS_BIT)
#define GRSPW_DMACTRL_RD	(1<<GRSPW_DMACTRL_RD_BIT)
#define GRSPW_DMACTRL_RX	(1<<GRSPW_DMACTRL_RX_BIT)
#define GRSPW_DMACTRL_AT	(1<<GRSPW_DMACTRL_AT_BIT)
#define GRSPW_DMACTRL_RA	(1<<GRSPW_DMACTRL_RA_BIT)
#define GRSPW_DMACTRL_TA	(1<<GRSPW_DMACTRL_TA_BIT)
#define GRSPW_DMACTRL_PR	(1<<GRSPW_DMACTRL_PR_BIT)
#define GRSPW_DMACTRL_PS	(1<<GRSPW_DMACTRL_PS_BIT)
#define GRSPW_DMACTRL_AI	(1<<GRSPW_DMACTRL_AI_BIT)
#define GRSPW_DMACTRL_RI	(1<<GRSPW_DMACTRL_RI_BIT)
#define GRSPW_DMACTRL_TI	(1<<GRSPW_DMACTRL_TI_BIT)
#define GRSPW_DMACTRL_RE	(1<<GRSPW_DMACTRL_RE_BIT)
#define GRSPW_DMACTRL_TE	(1<<GRSPW_DMACTRL_TE_BIT)

//////////////////////////////////////////////////////////////////////
/* GRSPW - DMA Channel Max Packet Length Register - (0x20*N + 0x04) */
#define GRSPW_DMARXLEN_MAX_BIT	0   // RX maximum length
#define GRSPW_DMARXLEN_MAX	(0xffffff<<GRSPW_DMARXLEN_MAX_BIT)

/* GRSPW - DMA Channel Address Register - (0x20*N + 0x10) */
#define GRSPW_DMAADR_ADDR_BIT	0   // address
#define GRSPW_DMAADR_MASK_BIT	8   // Mask
#define GRSPW_DMAADR_ADDR	(0xff<<GRSPW_DMAADR_ADDR_BIT)
#define GRSPW_DMAADR_MASK	(0xff<<GRSPW_DMAADR_MASK_BIT)

/* RX Buffer Descriptor */
struct grspw_rxbd {
   volatile unsigned int ctrl;
   volatile unsigned int addr;
};

/* TX Buffer Descriptor */
struct grspw_txbd {
   volatile unsigned int ctrl;
   volatile unsigned int haddr;
   volatile unsigned int dlen;
   volatile unsigned int daddr;
};

///////////////////////////////////////////
/* GRSPW - DMA Receive descriptor word 0 */
#define GRSPW_RXBD_LEN_BIT 0
#define GRSPW_RXBD_LEN	(0x1ffffff<<GRSPW_RXBD_LEN_BIT)
#define GRSPW_RXBD_EN	(1<<25) // Enable
#define GRSPW_RXBD_WR	(1<<26) // Wrap
#define GRSPW_RXBD_IE	(1<<27) // Interrupt enable
#define GRSPW_RXBD_EP	(1<<28) // EEP termination
#define GRSPW_RXBD_HC	(1<<29) // Header CRC
#define GRSPW_RXBD_DC	(1<<30) // Data CRC
#define GRSPW_RXBD_TR	(1<<31) // Truncated

////////////////////////////////////////////
/* GRSPW - DMA Transmit descriptor word 0 */
#define GRSPW_TXBD_HLEN	(0xff<<0)   // header length
#define GRSPW_TXBD_NCL	(0xf<<8)    // non-CRC bytes
#define GRSPW_TXBD_EN	(1<<12)     // Enable
#define GRSPW_TXBD_WR	(1<<13)     // Wrap
#define GRSPW_TXBD_IE	(1<<14)     // Interrupt enable
#define GRSPW_TXBD_LE	(1<<15)     // Link error
#define GRSPW_TXBD_HC	(1<<16)     // Append header CRC
#define GRSPW_TXBD_DC	(1<<17)     // Append data CRC

struct grspwregs_str
{
    volatile int ctrl;
    volatile int status;
    volatile int nodeaddr;
    volatile int clkdiv;
    volatile int destkey;
    volatile int time;
    volatile int unused[2];
    volatile int dmactrl;
    volatile int rxmaxlen;
    volatile int txdesc;
    volatile int rxdesc;
};
typedef struct grspwregs_str grspwregs_t;

struct spacewire_PARAMETERS_str
{
        int *size;
        volatile char *rx;
        volatile int *rxd;
        char *tx;
        volatile int *txd;
        grspwregs_t *regs;
} spacewire_PARAMETERS;

////////////
// FUNCTIONS
unsigned int grspw_link_state(grspwregs_t *);

#endif // GRSPW_H_INCLUDED

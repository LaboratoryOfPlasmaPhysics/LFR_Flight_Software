#ifndef FSW_RTEMS_CONFIG_H_INCLUDED
#define FSW_RTEMS_CONFIG_H_INCLUDED

#define GRSPW_DEVICE_NAME "/dev/grspw0"
#define UART_DEVICE_NAME "/dev/console"

//**********
// LFR MODES
#define LFR_MODE_STANDBY 0
#define LFR_MODE_NORMAL 1
#define LFR_MODE_BURST 2
#define LFR_MODE_SBM1 3
#define LFR_MODE_SBM2 4

//*****************************
// APB REGISTERS BASE ADDRESSES
#define REGS_ADDR_APBUART 0x80000100
#define REGS_ADDR_GPTIMER 0x80000300
#define REGS_ADDR_GRSPW 0x80000500
#define REGS_ADDR_TIME_MANAGEMENT 0x80000600
#define REGS_ADDR_SPECTRAL_MATRICES 0x80000700

#define APBUART_CTRL_REG_MASK_DB 0xfffff7ff
#define APBUART_SCALER_RELOAD_VALUE 0x00000050      // 25 MHz => about 38400

//**********
// IRQ LINES
#define IRQ_SM 9
#define IRQ_SPARC_SM 0x19 // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_WF 10
#define IRQ_SPARC_WF 0x1a // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_TIME1 12
#define IRQ_SPARC_TIME1 0x1c // see sparcv8.pdf p.76 for interrupt levels
#define IRQ_TIME2 13
#define IRQ_SPARC_TIME2 0x1d // see sparcv8.pdf p.76 for interrupt levels

//*****
// TIME
#define CLKDIV_SM_SIMULATOR 9999
#define CLKDIV_WF_SIMULATOR 9999999
#define TIMER_SM_SIMULATOR 1
#define TIMER_WF_SIMULATOR 2
#define HK_PERIOD 100 // 100 * 10ms => 1sec

//******
// RTEMS
#define TASKID_RECV 1
#define TASKID_ACTN 2
#define TASKID_SPIQ 3
#define TASKID_SMIQ 4
#define TASKID_STAT 5
#define TASKID_AVF0 6
#define TASKID_BPF0 7
#define TASKID_WFRM 8
#define TASKID_DUMB 9
#define TASKID_HOUS 10

#define ACTION_MSG_QUEUE_COUNT 10

//*******
// MACROS
#define PRINT_TASK_STATISTICS
#define PRINT_MESSAGES_ON_CONSOLE // enable or disable the printf instructions
#ifdef PRINT_MESSAGES_ON_CONSOLE
#define PRINTF(x) printf(x);
#define PRINTF1(x,y) printf(x,y);
#define PRINTF2(x,y,z) printf(x,y,z);
#else
#define PRINTF(x) ;
#define PRINTF1(x,y) ;
#define PRINTF2(x,y,z) ;
#endif

#define NB_SAMPLES_PER_SNAPSHOT 2048
#define NB_BYTES_SWF_BLK 2 * 6
#define NB_WORDS_SWF_BLK 3

struct param_norm_str{
    unsigned int sy_lfr_n_swf_l; // length of the snapshots
    unsigned int sy_lfr_n_swf_p; // time between two snapshots
    unsigned int sy_lfr_n_asm_p; // time between two asm
    unsigned char sy_lfr_n_bp_p0; // timebetween two products BP1 set
    unsigned char sy_lfr_n_bp_p1; // time between two products BP2 set
};

struct param_burst_str{
    unsigned char sy_lfr_b_bp_p0; // timebetween two products BP1 set
    unsigned char sy_lfr_b_bp_p1; // time between two products BP2 set
};

struct param_sbm1_str{
    unsigned char sy_lfr_s1_bp_p0; // timebetween two products BP1 set
    unsigned char sy_lfr_s1_bp_p1; // time between two products BP2 set
};

struct param_sbm2_str{
    unsigned char sy_lfr_s2_bp_p0; // timebetween two products BP1 set
    unsigned char sy_lfr_s2_bp_p1; // time between two products BP2 set
};

extern volatile int wf_snap_f0[ ]; // 24576 bytes
extern volatile int wf_snap_f1[ ]; // 24576 bytes
extern volatile int wf_snap_f2[ ]; // 24576 bytes
extern volatile int wf_cont_f3[ ]; // 24576 bytes

#endif // FSW_RTEMS_CONFIG_H_INCLUDED

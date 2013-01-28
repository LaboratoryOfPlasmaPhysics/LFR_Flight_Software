#ifndef FSW_RTEMS_H_INCLUDED
#define FSW_RTEMS_H_INCLUDED

#define REGS_ADDRESS_GPTIMER 0x80000300
#define REGS_ADDRESS_SPECTRAL_MATRICES 0x80000700

#define IRQ_SPECTRAL_MATRICES 9
#define IRQ_WAVEFORMS 10

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

volatile int waveform_snapshot_f0[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int waveform_snapshot_f1[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int waveform_snapshot_f2[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes
volatile int waveform_continuous_f3[ NB_SAMPLES_PER_SNAPSHOT * NB_BYTES_SWF_BLK ]; // 24576 bytes

#endif // FSW_RTEMS_H_INCLUDED

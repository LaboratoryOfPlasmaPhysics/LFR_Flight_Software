#include <..\header\LPP_spwAPI.h>
#include <..\header\basic_parameters.h>
#include <..\header\ccsds_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <asm-leon/timer.h>
#include <asm-leon/irq.h>

#define FINE_TIME_RESET time_manager_regs->ctrl = 0x00000001;

volatile int didtick  = 0;
volatile int cnt  = 0;
volatile int dide  = 0;

struct timerevent e;    /* this has to be a global variable, it is inserted in a linked list by addtimer() */
struct irqaction action_receive_CCSDS;     /* this has to be a global variable as it will be inserted in a list */
void register_event();

int event(void *arg)
{
	dide = 1;
	register_event();
	return 0;
}

void register_event()
{
	/* add an event 1s from now */
	do_gettimeofday((struct timeval*) &e.expire);
	e.handler = event;
	e.expire.tv_sec = e.expire.tv_sec + 1;
	addtimer(&e);
}

int main()
{
    volatile char *rx;
    char *tx;
    volatile char *rmaphdr;
    volatile char *rmap;
    volatile int *rxd;
    volatile int *txd;
    int size;
    int i;
    int j;
    unsigned char current_nb_average = 0;
    spwHeader_t spwHeader;

    grspwregs_t *grspw_regs;
    time_manager_regs_t *time_manager_regs;
    spectral_matrices_regs_t *spectral_matrices_regs;

    grspw_regs = (grspwregs_t*) GRSPW_APB_ADDR;                       // the grspw registers are at the address GRSPW_APB_ADDR
    time_manager_regs = (time_manager_regs_t*) LPP_TIME_MANAGER;    // the time manager registers are at the address LPP_TIME_MANAGER
    spectral_matrices_regs = (spectral_matrices_regs_t*) LPP_SPECTRAL_MATRIX_CTRL; // the spectral matrices registers are at this address

    rx = (char*) malloc(64);
    tx = (char*) malloc(64);
    rmaphdr = (char*) malloc(32);
    rmap = (char*) malloc(64);
    rxd = (int*) almalloc(1024);                // memory allocation for the receiver descriptors table
                                                // almalloc => 1024 bytes aligned address
    txd = (int*) almalloc(1024);                // memory allocation for the transmitter descriptors table
                                                // almalloc => 1024 bytes aligned address

    // GRSPW initilization
    spw_reset(grspw_regs);                                          // RESET THE SPACEWIRE LINK
    spw_init(NODEADDR, CLKDIV, DESTKEY, RXMAXLEN, grspw_regs);      // INITIALIZE THE SPACEWIRE REGISTERS
    enable_timecode_reception(grspw_regs);                          // enable time-code reception

    while( grspw_link_state(grspw_regs) != SPW_LINK_STATE_RUN ) {}  // wait for the link to be in run state

    // INITIALIZE THE TRANSMITTER RECEIVER DESCRIPTORS TABLES
    for(i=0; i<8; i++) { // initialize the receiver and transmitter descriptor tables
        rxd[i] = 0;
        txd[i] = 0;
    }
    grspw_regs->txdesc = (int) txd;
    grspw_regs->rxdesc = (int) rxd;

    for (j=0; j<64; j++){ // initialize the rx and tx data tables
        rx[j] = j;
        tx[j] = 10;
    }

    // INITIALIZE THE TRANSMISSION
    spwHeader.targetLogicalAddress = 0x21;  // initialize the SpaceWire Header
    spwHeader.protocolIdentifier = 0x02;
    spwHeader.reserved = 0x00;
    spwHeader.userApplication = 0x00;
    set_txd(tx, 32, (char*) &spwHeader, HEADERLEN, txd); // initialize the transmitter descriptor 0

    INIT_CCSDS_TELEMETRY_HEADER

    // INITIALIZE THE RECEPTION
    enable_receiver_descriptor(rx, rxd);    // enable the reception of a SpaceWire packet

    // START SPW TRANSMISSION / RECEPTION
    grspw_regs->dmactrl = GRSPW_DMACTRL_NS | GRSPW_DMACTRL_RD | GRSPW_DMACTRL_RI | GRSPW_DMACTRL_RE | GRSPW_DMACTRL_TE;
                                    // NS = 1 wait for a descriptor to be activated
                                    // RD = 1 there are enabled descriptors
                                    // RI = 1 receive interrupts
                                    // RE = 1 receiver enabled
                                    // TE = 1 transmitter enabled

    // TIME FUNCTIONS
    leonbare_init_ticks();  // initialize the time abilities of the Leon3
	register_event();       // create an event

	// SETS AN IRQ HANDLER UPON RECEPTION OF A SPACEWIRE PACKET
	spacewire_PARAMETERS.size = &size;        // size of the receive packet
    spacewire_PARAMETERS.rx = rx;             // location of the data
    spacewire_PARAMETERS.rxd = rxd;           // associated descriptor
    spacewire_PARAMETERS.tx = tx;             // location of the data
    spacewire_PARAMETERS.txd = txd;           // associated descriptor
    spacewire_PARAMETERS.regs = grspw_regs;   // grspw module registers
	action_receive_CCSDS.handler = (irqhandler) irqhandler_receive_CCSDS;   // create the irq handler
	chained_catch_interrupt(GRSPW_IRQ, &action_receive_CCSDS);              // associate the handler with the GRSPW_IRQ
	enable_irq(GRSPW_IRQ);  // enable the interruption driven by the grspw module upon reception of a packet

	packet_type.time_packet = 0;
	packet_type.tc_packet = 0;

	//write spectral matrices location in the registers
	spectral_matrices_regs->address1 = (unsigned int) spectral_matrix_f0_a;
	spectral_matrices_regs->address2 = (unsigned int) spectral_matrix_f0_b;

	printf("let's work...\n");

	while(1) { // infinite loop
	    if (packet_type.time_packet == 1) {
	        FINE_TIME_RESET
	        time_manager_regs->next_coarse_time = (rx[10+4]<<24) + (rx[11+4]<<16) + (rx[12+4]<<8) + (rx[13+4]);
	        send_fine_time(time_manager_regs->fine_time);
            enable_receiver_descriptor(spacewire_PARAMETERS.rx, spacewire_PARAMETERS.rxd);
            enable_receiver(spacewire_PARAMETERS.regs);
            packet_type.time_packet = 0;
	    }
        if ((spectral_matrices_regs->ctrl & 1) == 1) {
            FINE_TIME_RESET
            for (i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++) {
                averaged_spectral_matrix_f0[i] = averaged_spectral_matrix_f0[i] + spectral_matrix_f0_a[i];
            }
            send_fine_time(time_manager_regs->fine_time);
            spectral_matrices_regs->ctrl = spectral_matrices_regs->ctrl & 0xfffffffe;
            current_nb_average++;
        }
        if ( ( (spectral_matrices_regs->ctrl>>1) & 1) == 1) {
            FINE_TIME_RESET
            for (i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++) {
                averaged_spectral_matrix_f0[i] = averaged_spectral_matrix_f0[i] + spectral_matrix_f0_b[i];
            }
            send_fine_time(time_manager_regs->fine_time);
            spectral_matrices_regs->ctrl = spectral_matrices_regs->ctrl & 0xfffffffd;
            current_nb_average++;
        }
        if ( current_nb_average == NB_AVERAGE_NORMAL_f0 ) {
            FINE_TIME_RESET
            for(i=0;i<NB_BINS_COMPRESSED_MATRIX_f0;i++){
                j = 17 + i * 8;
                compressed_spectral_matrix_f0[i]= (averaged_spectral_matrix_f0[j]
                                                    + averaged_spectral_matrix_f0[j+1]
                                                    + averaged_spectral_matrix_f0[j+2]
                                                    + averaged_spectral_matrix_f0[j+3]
                                                    + averaged_spectral_matrix_f0[j+4]
                                                    + averaged_spectral_matrix_f0[j+5]
                                                    + averaged_spectral_matrix_f0[j+6]
                                                    + averaged_spectral_matrix_f0[j+7])/(8*NB_AVERAGE_NORMAL_f0);
            }

            BP1_set();  // computation of the BP1 set and building of the corresponding telemetry product

            // sending of the BP1 set
            set_txd( LFR_BP1_F0, NB_BINS_COMPRESSED_MATRIX_f0*9, // initialize the transmitter descriptor 0
                        (char*) &ccsdsTelemetryHeader, CCSDS_TELEMETRY_HEADER_LENGTH, txd );
            enable_transmitter_descriptor(CCSDS_TELEMETRY_HEADER_LENGTH, spacewire_PARAMETERS.txd); // enable the descriptor for transmission
            enable_transmitter(spacewire_PARAMETERS.regs);                                          // enable the transmission

            for (i=0; i<TOTAL_SIZE_SPECTRAL_MATRIX; i++) {
                averaged_spectral_matrix_f0[i] = 0;
            }
            //send_fine_time(time_manager_regs->fine_time);
            current_nb_average = 0;
        }
	}

    return 0;
}


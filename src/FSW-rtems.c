//*************************
// GPL reminder to be added
//*************************

#define OFFSET_COARSE_TIME 4+10-1 // -1 => the receiver suppresses the target logical address
#define OFFSET_FINE_TIME OFFSET_COARSE_TIME+4
#define TC_FIFO_SIZE 5

#define ADDRESS_APBUART_REGISTERS 0x80000100
#define ADDRESS_GRSPW_REGISTERS 0x80000500
#define APBUART_CTRL_REG_MASK_DB 0xfffff7ff

#define GRSPW_DEVICE_NAME "/dev/grspw0"
#define UART_DEVICE_NAME "/dev/console"

#include <rtems.h>

/* configuration information */

#define CONFIGURE_INIT
#include <bsp.h> /* for device driver prototypes */

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS             10
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS         (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY	100
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 1

#include <rtems/confdefs.h>

/* If --drvmgr was enabled during the configuration of the RTEMS kernel */
#ifdef RTEMS_DRVMGR_STARTUP
    #ifdef LEON3
    /* Add Timer and UART Driver */
        #ifdef CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
            #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GPTIMER
        #endif
        #ifdef CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
            #define CONFIGURE_DRIVER_AMBAPP_GAISLER_APBUART
        #endif
    #endif
    #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GRSPW   /* GRSPW Driver */
    #include <drvmgr/drvmgr_confdefs.h>
#endif

#include <rtems.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>

//#include <FSW-config.c>
#include <grspw.h>
#include <apbuart.h>
#include <FSW-config.h>
#include <TC_handler.h>
#include <FSW-rtems-processing.h>
#include <grlibregs.h>

/*char *tmGeneratorMsg[2] = { "NOTHING_TO_DO",
                            "TM_GENERATED"
};*/

// RTEMS TASKS
rtems_task Init( rtems_task_argument argument);	/* forward declaration needed */
rtems_task spw_recv_task(rtems_task_argument argument);
rtems_task spw_spiq_task(rtems_task_argument argument);
rtems_task spw_stat_task(rtems_task_argument argument);
rtems_task spw_wfrm_task(rtems_task_argument argument);
// ISR
rtems_isr waveforms_isr( rtems_vector_number vector );

int send_console_outputs_on_serial_port();
int create_all_tasks();
int start_all_tasks();
int configure_spw_link();
int configure_timer_for_sm_simulation();
int configure_timer_for_wf_simulation();
extern int rtems_cpu_usage_report();
extern int rtems_cpu_usage_reset();

char *link_status(int status);
void print_statistics(spw_stats *);
extern int sched_yield();
extern int errno;

char *lstates[6] = {"Error-reset",
                    "Error-wait",
                    "Ready",
                    "Started",
                    "Connecting",
                    "Run"
};

int fdSPW;
int fdUART;

rtems_id   Task_id[10];         /* array of task ids */
rtems_name Task_name[10];       /* array of task names */

rtems_task Init( rtems_task_argument ignored ) {
    rtems_status_code status;

    send_console_outputs_on_serial_port();

    create_all_tasks();
    start_all_tasks();

    configure_spw_link();
    configure_timer_for_sm_simulation();
    configure_timer_for_wf_simulation();

    InitLookUpTableForCRC(); // in TC_handler.h

    status = rtems_task_delete(RTEMS_SELF);
}

rtems_task spw_recv_task( rtems_task_argument unused ) {
    rtems_status_code status;
    int len = 0;
    unsigned int i = 0;
    unsigned int data_length = 0;
    ccsdsTelecommandPacket_t currentTC;
    spw_ioctl_pkt_send spw_ioctl_send;
    TMHeader_t TM_header;
    char data[100];

    for(i=0; i<100; i++) data[i] = 0;

    PRINTF("In RECV *** \n")

    while(1){
        len = read(fdSPW, (char*) &currentTC, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
        if (len == -1){ // error during the read call
            PRINTF("In RECV *** last read call returned -1\n")
            if (rtems_event_send( Task_id[3], SPW_LINKERR_EVENT ) != RTEMS_SUCCESSFUL)
                PRINTF("IN RECV *** Error: rtems_event_send SPW_LINKERR_EVENT\n")
            if (rtems_task_suspend(RTEMS_SELF) != RTEMS_SUCCESSFUL)
                PRINTF("In RECV *** Error: rtems_task_suspend(RTEMS_SELF)\n")
        }
        else {
            //PRINTF1("In RECV *** Got Message of length %d\n", len)
            currentTC_LEN_RCV[0] = 0x00;
            currentTC_LEN_RCV[1] = (unsigned char) len - CCSDS_TC_TM_PACKET_OFFSET - 3; //  build the corresponding packet size field
            currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) len - CCSDS_TC_TM_PACKET_OFFSET - 3; // => -3 is for Prot ID, Reserved and User App bytes
            // CHECK THE TC AND BUILD THE APPROPRIATE TM
            data_length = TC_checker(&currentTC, currentTC_LEN_RCV_AsUnsignedInt,
                       &TM_header, &spw_ioctl_send.hlen, data);
            spw_ioctl_send.hlen = TM_HEADER_LEN + 4; // + 4 is for the protocole extra header
            spw_ioctl_send.hdr = (char*) &TM_header;
            spw_ioctl_send.dlen = data_length;
            spw_ioctl_send.data = data;
            //printf("hlen %d, dlen %d\n", spw_ioctl_send.hlen, spw_ioctl_send.dlen);
            // SEND PACKET
            status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send );
            if (status!=RTEMS_SUCCESSFUL) printf("In TC_checker *** Error SPACEWIRE_IOCTRL_SEND\n");
            //PRINTF1("In TC_checker *** packet of size %d sent\n", spw_ioctl_send.sent)
        }
    }

    close(fdSPW);
}

rtems_task spw_spiq_task(rtems_task_argument unused){
    rtems_event_set event_out;
    struct grspw_regs_str *grspw_regs;
    grspw_regs = (struct grspw_regs_str *) ADDRESS_GRSPW_REGISTERS;

    while(1){
        PRINTF("In SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        if (rtems_task_suspend(Task_id[1])!=RTEMS_SUCCESSFUL)   // suspend RECV task
            PRINTF("In SPIQ *** Error suspending RECV Task\n")

        configure_spw_link();

        if (rtems_task_restart(Task_id[1], 1)!=RTEMS_SUCCESSFUL) // restart RECV task
            PRINTF("In SPIQ *** Error resume RECV Task\n")
    }
}

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[8], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        printf("In spectral_matrices_isr *** Error sending event to WFRM\n");
}

rtems_task spw_wfrm_task(rtems_task_argument argument)
{
    unsigned int length;
    unsigned int i = 0;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send;
    rtems_event_set event_out;
    gptimer_regs_t *gptimer_regs;
    gptimer_regs = (gptimer_regs_t *) REGS_ADDRESS_GPTIMER;
    ExtendedTMHeader_t header;

    header.targetLogicalAddress = CCSDS_DESTINATION_ID;
    header.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header.reserved = 0x00;
    header.userApplication = CCSDS_USER_APP;
    header.packetID[0] = 0x0c;
    header.packetID[1] = 0xcc;
    header.packetSequenceControl[0] = 0x00;
    header.packetSequenceControl[1] = 0x00;
    header.packetLength[0] = 0x00;
    header.packetLength[1] = 0x00;
    header.dataFieldHeader[0] = 0x10;
    header.dataFieldHeader[1] = 0x15; // service type
    header.dataFieldHeader[2] = 0x03; // service subtype
    header.dataFieldHeader[3] = CCSDS_DESTINATION_ID;
    header.dataFieldHeader[4] = 0xaa;
    header.dataFieldHeader[5] = 0xbb;
    header.dataFieldHeader[6] = 0xcc;
    header.dataFieldHeader[7] = 0xdd;
    header.dataFieldHeader[8] = 0xee;
    header.dataFieldHeader[9] = 0xff;

    header.auxiliaryHeader[0] = 0x00;
    header.auxiliaryHeader[1] = 0x1f;
    header.auxiliaryHeader[2] = 0x07; // PKT_CNT
    header.auxiliaryHeader[3] = 0x00; // PKT_NR
    header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
    header.auxiliaryHeader[5] = 0x00; // BLK_NR LSB

    // BUILD THE PACKET HEADER
    spw_ioctl_send.hlen = TM_HEADER_LEN + 4 + 6; // + 4 is for the protocole extra header, + 6 is for the auxiliary header
    spw_ioctl_send.hdr = (char*) &header;

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        for (i=0; i<7; i++) // send F0
        {
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
            }
            spw_ioctl_send.data = (char*) &waveform_snapshot_f0[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F0; // SID
            // SEND PACKET
            status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send );
            if (status!=RTEMS_SUCCESSFUL) printf("In WFRM *** Error SPACEWIRE_IOCTRL_SEND\n");
            //sched_yield();
        }
        for (i=0; i<7; i++) // send F1
        {
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
            }
            spw_ioctl_send.data = (char*) &waveform_snapshot_f1[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F1; // SID
            // SEND PACKET
            status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send );
            if (status!=RTEMS_SUCCESSFUL) printf("In WFRM *** Error SPACEWIRE_IOCTRL_SEND\n");
            //sched_yield();
        }
        for (i=0; i<7; i++) // send F0
        {
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
            }
            spw_ioctl_send.data = (char*) &waveform_snapshot_f2[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F2; // SID
            // SEND PACKET
            status = ioctl( fdSPW, SPACEWIRE_IOCTRL_SEND, &spw_ioctl_send );
            if (status!=RTEMS_SUCCESSFUL) printf("In WFRM *** Error SPACEWIRE_IOCTRL_SEND\n");
            //sched_yield();
        }
        // irq processed, reset the related register of the timer unit
        gptimer_regs->timer3_ctrl = gptimer_regs->timer3_ctrl | 0x00000010;
    }
}

int create_all_tasks()
{
    rtems_status_code status;

    Task_name[1] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[3] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[4] = rtems_build_name( 'S', 'M', 'I', 'Q' );
    Task_name[5] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[6] = rtems_build_name( 'A', 'V', 'F', '0' );
    Task_name[7] = rtems_build_name( 'B', 'P', 'F', '0' );
    Task_name[8] = rtems_build_name( 'W', 'F', 'R', 'M' );

    // RECV
    status = rtems_task_create(
        Task_name[1], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[1]
    );
    // SPIQ
    status = rtems_task_create(
        Task_name[3], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[3]
    );
    // SMIQ
    status = rtems_task_create(
        Task_name[4], 10, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[4]
    );
    // STAT
    status = rtems_task_create(
        Task_name[5], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[5]
    );
    // AVF0
    status = rtems_task_create(
        Task_name[6], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES  | RTEMS_NO_PREEMPT,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[6]
    );
    // BPF0
    status = rtems_task_create(
        Task_name[7], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[7]
    );
    // WFRM
    status = rtems_task_create(
        Task_name[8], 100, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[8]
    );

    return 0;
}

int start_all_tasks()
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[3], spw_spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_SPIQ\n")

    status = rtems_task_start( Task_id[1], spw_recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_RECV\n")

    //status = rtems_task_start( Task_id[4], spw_bppr_task_rate_monotonic, 1 );
    status = rtems_task_start( Task_id[4], spw_smiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPPR\n")

    status = rtems_task_start( Task_id[5], spw_stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_STAT\n")

    status = rtems_task_start( Task_id[6], spw_avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_AVF0\n")

    status = rtems_task_start( Task_id[7], spw_bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPF0\n")

    status = rtems_task_start( Task_id[8], spw_wfrm_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_WFRM\n")

    return 0;
}

int send_console_outputs_on_serial_port() // Send the console outputs on the serial port
{
    struct apbuart_regs_str *apbuart_regs;

    apbuart_regs = (struct apbuart_regs_str *) ADDRESS_APBUART_REGISTERS;
    apbuart_regs->ctrl = apbuart_regs->ctrl & APBUART_CTRL_REG_MASK_DB;
    PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

    return 0;
}

int configure_spw_link()
{
    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
    if (fdSPW<0) PRINTF("In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, 0) != RTEMS_SUCCESSFUL){
        PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" not started, retry\n")
        close(fdSPW); // close the device
        fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
        if (fdSPW<0) PRINTF("In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
        rtems_task_wake_after(100);
    }

    PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    // sets a few parameters of the link
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RMAPEN, 1);      // sets the RMAP enable bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_RMAPEN\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[3]);    // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 1);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    //status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DESTKEY, CCSDS_DESTINATION_ID);  // sets the destination key
    //if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

int configure_timer_for_sm_simulation() // configure the timer for the spectral matrices simulation
{
    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    status = rtems_interrupt_catch( spectral_matrices_isr, 0x19, &old_isr_handler) ; // 0x19 comes from sparcv8.pdf p.76 <=> interrupt_level_9
    if (status==RTEMS_SUCCESSFUL) PRINTF("In configure_timer_for_sm_simulation *** rtems_interrupt_catch successfullly configured\n")

    gptimer_regs_t *gptimer_regs;
    gptimer_regs = (gptimer_regs_t *) REGS_ADDRESS_GPTIMER;
    gptimer_regs->timer2_reload = 9999; // 10ms timer, base clock frequency is 1 MHz
    gptimer_regs->timer2_ctrl = gptimer_regs->timer2_ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer2_ctrl = gptimer_regs->timer2_ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer2_ctrl = gptimer_regs->timer2_ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer2_ctrl = gptimer_regs->timer2_ctrl | 0x00000008;  // IE interrupt enable

    LEON_Unmask_interrupt( IRQ_SPECTRAL_MATRICES );

    return 1;
}

int configure_timer_for_wf_simulation() // configure the timer for the spectral matrices simulation
{
    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    status = rtems_interrupt_catch( waveforms_isr, 0x1a, &old_isr_handler) ; // 0x19 comes from sparcv8.pdf p.76 <=> interrupt_level_10
    if (status==RTEMS_SUCCESSFUL) PRINTF("In configure_timer_for_wf_simulation *** rtems_interrupt_catch successfullly configured\n")

    gptimer_regs_t *gptimer_regs;
    gptimer_regs = (gptimer_regs_t *) REGS_ADDRESS_GPTIMER;
    gptimer_regs->timer3_reload = 7999999; // 1s timer, base clock frequency is 1 MHz
    gptimer_regs->timer3_ctrl = gptimer_regs->timer3_ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer3_ctrl = gptimer_regs->timer3_ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer3_ctrl = gptimer_regs->timer3_ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer3_ctrl = gptimer_regs->timer3_ctrl | 0x00000008;  // IE interrupt enable

    LEON_Unmask_interrupt( IRQ_WAVEFORMS );

    return 1;
}

rtems_task spw_stat_task(rtems_task_argument argument){
    int i;
    i = 0;
    PRINTF("In STAT *** \n")
    while(1){
        rtems_task_wake_after(1000);
        PRINTF1("%d\n", i)
        if (i == 2) {
            rtems_cpu_usage_report();
            rtems_cpu_usage_reset();
            i = 0;
        }
        else i++;
    }
}

char *link_status(int status){
        return lstates[status];
}

void print_statistics(spw_stats *stats)
{
        //printf(" ******** STATISTICS ********  \n");
        printf("Transmit link errors: %i\n", stats->tx_link_err);
        printf("Receiver RMAP header CRC errors: %i\n", stats->rx_rmap_header_crc_err);
        printf("Receiver RMAP data CRC errors: %i\n", stats->rx_rmap_data_crc_err);
        printf("Receiver EEP errors: %i\n", stats->rx_eep_err);
        printf("Receiver truncation errors: %i\n", stats->rx_truncated);
        printf("Parity errors: %i\n", stats->parity_err);
        printf("Escape errors: %i\n", stats->escape_err);
        printf("Credit errors: %i\n", stats->credit_err);
        printf("Disconnect errors: %i\n", stats->disconnect_err);
        printf("Write synchronization errors: %i\n", stats->write_sync_err);
        printf("Early EOP/EEP: %i\n", stats->early_ep);
        printf("Invalid Node Address: %i\n", stats->invalid_address);
        printf("Packets transmitted: %i\n", stats->packets_sent);
        printf("Packets received: %i\n", stats->packets_received);
}






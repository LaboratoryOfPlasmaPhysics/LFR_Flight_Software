//*************************
// GPL reminder to be added
//*************************

//#define PRINT_MESSAGES_ON_CONSOLE // enable or disable the printf instructions
#ifdef PRINT_MESSAGES_ON_CONSOLE
#define PRINTF(x) printf(x);
#define PRINTF1(x,y) printf(x,y);
#define PRINTF2(x,y,z) printf(x,y,z);
#else
#define PRINTF(x) ;
#define PRINTF1(x,y) ;
#define PRINTF2(x,y,z) ;
#endif

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

#include <FSW-config.c>
#include <grspw.h>
#include <apbuart.h>
#include <TC_handler.h>
#include <FSW-rtems-processing.h>

char *errorCCSDSMsg[8] = { "ILLEGAL_APID 0",
                            "WRONG_LEN_PACKET 1",
                            "INCOR_CHECKSUM 2",
                            "ILL_TYPE 3",
                            "ILL_SUBTYPE 4",
                            "WRONG_APP_DATA 5",
                            "WRONG_CMD_CODE 6",
                            "CCSDS_TM_VALID 7"
};

char *tmGeneratorMsg[2] = { "NOTHING_TO_DO",
                            "TM_GENERATED"
};

rtems_task Init( rtems_task_argument argument);	/* forward declaration needed */
rtems_task spw_recv_task(rtems_task_argument argument);
rtems_task spw_tcck_task(rtems_task_argument argument);
rtems_task spw_spiq_task(rtems_task_argument argument);
rtems_task spw_stat_task(rtems_task_argument argument);
int configure_spw_link();
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

struct tc_fifo_str{
    struct tc_fifo_str *next;
    unsigned char *tc_pkt;
    unsigned int tc_pktlength;
    unsigned char ready;
};
struct tc_fifo_str tc_fifo[TC_FIFO_SIZE];
struct tc_fifo_str *tc_being_processed;
struct tc_fifo_str *tc_pending_reception;

struct apbuart_regs_str{
    volatile unsigned int data;
    volatile unsigned int status;
    volatile unsigned int ctrl;
    volatile unsigned int scaler;
    volatile unsigned int fifoDebug;
};

int fdSPW;
int fdUART;

rtems_id   Task_id[10];         /* array of task ids */
rtems_name Task_name[10];       /* array of task names */
rtems_name sem_tc_fifo_name;
rtems_id sem_tc_fifo_id;

rtems_task Init( rtems_task_argument ignored ) {
    rtems_status_code status;
    //struct apbuart_regs_str *apbuart_regs;
    rtems_isr_entry old_isr_handler;

    Task_name[1] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[2] = rtems_build_name( 'T', 'C', 'C', 'K' );
    Task_name[3] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[4] = rtems_build_name( 'B', 'P', 'P', 'R' );
    Task_name[5] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[6] = rtems_build_name( 'A', 'V', 'F', '0' );
    Task_name[7] = rtems_build_name( 'B', 'P', 'F', '0' );
    sem_tc_fifo_name = rtems_build_name( 'S', 'E', 'M', '0' );

    // RECV
    status = rtems_task_create(
        Task_name[1], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[1]
    );
    // TCCK
    status = rtems_task_create(
        Task_name[2], 200, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[2]
    );
    // SPIQ
    status = rtems_task_create(
        Task_name[3], 50, RTEMS_MINIMUM_STACK_SIZE * 2,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[3]
    );
    // BPPR
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

    status = rtems_semaphore_create(
        sem_tc_fifo_name,
        1,
        RTEMS_BINARY_SEMAPHORE,
        1,
        &sem_tc_fifo_id
    );

    //********************************************
    // Send the console outputs on the serial port
    //apbuart_regs = (struct apbuart_regs_str *) ADDRESS_APBUART_REGISTERS;
    //apbuart_regs->ctrl = apbuart_regs->ctrl & APBUART_CTRL_REG_MASK_DB;
    //PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")
    //
    //********************************************

    configure_spw_link();

    status = rtems_interrupt_catch( spectral_matrices_isr, 0x1c, &old_isr_handler) ; // 0x1c comes from sparcv8.pdf p.76
    if (status==RTEMS_SUCCESSFUL) PRINTF("In INIT *** rtems_interrupt_catch successfullly configured\n")

    LEON_Unmask_interrupt( IRQ_SPECTRAL_MATRICES );

    status = rtems_task_start( Task_id[2], spw_tcck_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_TCCK\n")

    status = rtems_task_start( Task_id[3], spw_spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_SPIQ\n")

    status = rtems_task_start( Task_id[1], spw_recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_RECV\n")

    //status = rtems_task_start( Task_id[4], spw_bppr_task_rate_monotonic, 1 );
    //if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPPR\n")

    status = rtems_task_start( Task_id[5], spw_stat_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_STAT\n")

    status = rtems_task_start( Task_id[6], spw_avf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_AVF0\n")

    status = rtems_task_start( Task_id[7], spw_bpf0_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In INIT *** Error starting TASK_BPF0\n")

    InitLookUpTableForCRC(); // in TC_handler.h

    status = rtems_task_delete(RTEMS_SELF);
}

rtems_task spw_recv_task_old( rtems_task_argument unused ) {
    rtems_status_code status;
    int len = 0;
    unsigned int i;

    PRINTF("In RECV *** \n")

    status = rtems_semaphore_obtain( sem_tc_fifo_id, RTEMS_WAIT, RTEMS_NO_TIMEOUT );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error: rtems_semaphore_obtain\n")
    for (i = 0; i<TC_FIFO_SIZE; i++){ // initialize the tc fifo ring
        tc_fifo[i].tc_pktlength = 0;
        tc_fifo[i].ready = 0;
    }
    status = rtems_semaphore_release( sem_tc_fifo_id );
    if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error: rtems_semaphore_release\n")

    tc_being_processed = tc_fifo;
    tc_pending_reception = tc_fifo;

    while(1){
        if (tc_pending_reception->ready == 0){
            PRINTF("In RECV *** Just before read\n")
            len = read(fdSPW, tc_pending_reception->tc_pkt, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
            PRINTF("In RECV *** Just after read\n")
            if (len == -1) { // error during the read call
                PRINTF("In RECV *** last read call returned -1\n")
                if (rtems_event_send( Task_id[3], SPW_LINKERR_EVENT ) != RTEMS_SUCCESSFUL)
                    PRINTF("IN RECV *** Error: rtems_event_send SPW_LINKERR_EVENT\n")
                if (rtems_task_suspend(RTEMS_SELF) != RTEMS_SUCCESSFUL)
                    PRINTF("In RECV *** Error: rtems_task_suspend(RTEMS_SELF)\n")
            }
            else {
                tc_pending_reception->tc_pktlength = len;
                tc_pending_reception->ready = 1;
                PRINTF1("In RECV *** Got Message of length %d\n",tc_pending_reception->tc_pktlength)
                status = rtems_event_send( Task_id[2], RTEMS_EVENT_0 ); // sending an event to the task TCCK
                if (status == RTEMS_INVALID_ID) PRINTF("IN TASK RECV *** invalid task id when sending RTEMS_EVENT_0\n")
                tc_pending_reception = tc_pending_reception->next;
                sched_yield();
            }
        }
        else {
            status = rtems_event_send( Task_id[2], RTEMS_EVENT_0 ); // sending an event to the task TCCK
            if (status == RTEMS_INVALID_ID) PRINTF("IN TASK RECV *** invalid task id when sending RTEMS_EVENT_0\n")
            sched_yield();  // if not executed, the system blocks when the packets arrive faster than the processing
                            // this call gives back the hand to the scheduler and allows TCCK to process the TC packets
        }
    }

    close(fdSPW);
}

rtems_task spw_recv_task( rtems_task_argument unused ) {
    rtems_status_code status;
    int len = 0;

    PRINTF("In RECV BIS *** \n")

    currentTC_processedFlag = 1; // reset the flag

    while(1){
        while(currentTC_processedFlag == 0) sched_yield();
        len = read(fdSPW, (char*) &currentTC, CCSDS_TC_PKT_MAX_SIZE); // the call to read is blocking
        if (len == -1){ // error during the read call
            PRINTF("In RECV *** last read call returned -1\n")
            if (rtems_event_send( Task_id[3], SPW_LINKERR_EVENT ) != RTEMS_SUCCESSFUL)
                PRINTF("IN RECV *** Error: rtems_event_send SPW_LINKERR_EVENT\n")
            if (rtems_task_suspend(RTEMS_SELF) != RTEMS_SUCCESSFUL)
                PRINTF("In RECV *** Error: rtems_task_suspend(RTEMS_SELF)\n")
        }
        else {
            currentTC_processedFlag = 0;
            PRINTF1("In RECV *** Got Message of length %d\n", len)
            currentTC_LEN_RCV[0] = 0x00;
            currentTC_LEN_RCV[1] = (unsigned char) len - CCSDS_TC_TM_PACKET_OFFSET - 3; //  build the corresponding packet size field
            currentTC_LEN_RCV_AsUnsignedInt = (unsigned int) len - CCSDS_TC_TM_PACKET_OFFSET - 3; // => -3 is for Prot ID, Reserved and User App bytes
            status = rtems_event_send( Task_id[2], RTEMS_EVENT_0 ); // sending an event to the task TCCK
            if (status == RTEMS_INVALID_ID) PRINTF("IN TASK RECV *** invalid task id when sending RTEMS_EVENT_0\n")
        }
    }

    close(fdSPW);
}

rtems_task spw_tcck_task( rtems_task_argument unused ) {
    rtems_status_code status;
    rtems_event_set event_out;
    unsigned char result;
    unsigned int code;

    PRINTF("In TCCK ***\n")
    while(1){
        status = rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an event to process a TC packet
        if (status == RTEMS_SUCCESSFUL)
        {
            GetCRCAsTwoBytes((unsigned char*)currentTC.packetID, currentTC_COMPUTED_CRC, currentTC_LEN_RCV_AsUnsignedInt + 5);
            code = TM_checker(&currentTC);
            PRINTF1("     %s\n", errorCCSDSMsg[code])
            if (code == 7){     // if the TC is valid, the TM_LFR_TC_EXE_NOT_IMPLEMENTED packet is sent
                result = TM_not_implemented_generator(&currentTC, &currentTM);
                PRINTF1("     %s\n", tmGeneratorMsg[result])
                result = write( fdSPW, (char*)&currentTM, currentTM_length );
                if (result==-1) printf("IN TCCK *** error sending TM of size: %d\n", currentTM_length);
            }
            else{               // if the TC is not valid, the TM_LFR_TC_EXE_CORRUPTED is sent
                result =  TM_acceptance_generator(&currentTC, code, &currentTM);
                PRINTF1("     %s\n", tmGeneratorMsg[result])
                result = write( fdSPW, (char*)&currentTM, currentTM_length );
                if (result==-1) printf("IN TCCK *** error sending TM of size: %d\n", currentTM_length);
            }

            currentTC_processedFlag = 1;
        }
    }
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
    //
    //status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DESTKEY, CCSDS_DESTINATION_ID);  // sets the destinatino key
    //if (status!=RTEMS_SUCCESSFUL) PRINTF("In RECV *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    PRINTF("In configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
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




#include <fsw_misc.h>
#include <fsw_params.h>

extern rtems_id Task_id[];         /* array of task ids */
extern int fdSPW;
extern TMHeader_t housekeeping_header;
extern char housekeeping_data[];
extern Packet_TM_LFR_HK_t housekeeping_packet;

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() )
{ // configure the timer for the waveforms simulation
    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    status = rtems_interrupt_catch( timer_isr, interrupt_level, &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    //if (status==RTEMS_SUCCESSFUL) PRINTF("In configure_timer_for_wf_simulation *** rtems_interrupt_catch successfullly configured\n")

    gptimer_regs->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000008;  // IE interrupt enable

    return 1;
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

int send_console_outputs_on_serial_port( void ) // Send the console outputs on the serial port
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;

    apbuart_regs->ctrl = apbuart_regs->ctrl & APBUART_CTRL_REG_MASK_DB;
    PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

    return 0;
}

int set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value)
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) regs;

    apbuart_regs->scaler = value;
    PRINTF1("OK  *** COM port scaler reload register set to %x\n", value)

    return 0;
}

//************
// RTEMS TASKS

rtems_task stat_task(rtems_task_argument argument)
{
    int i;
    int j;
    i = 0;
    j = 0;
    PRINTF("in STAT *** \n")
    while(1){
        rtems_task_wake_after(1000);
        PRINTF1("%d\n", j)
        if (i == 2) {
            #ifdef PRINT_TASK_STATISTICS
            rtems_cpu_usage_report();
            rtems_cpu_usage_reset();
            #endif
            i = 0;
        }
        else i++;
        j++;
    }
}

rtems_task hous_task(rtems_task_argument argument)
{
    int result;
    rtems_status_code status;

    PRINTF("in HOUS ***\n")

    if (rtems_rate_monotonic_ident( HK_name, &HK_id)!=RTEMS_SUCCESSFUL) {
        status = rtems_rate_monotonic_create( HK_name, &HK_id );
        if( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "rtems_rate_monotonic_create failed with status of %d\n", status )
        }
    }

    housekeeping_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    housekeeping_packet.protocolIdentifier = 0x02;
    housekeeping_packet.reserved = 0x00;
    housekeeping_packet.userApplication = 0x00;
    housekeeping_packet.packetID[0] = 0x0c;
    housekeeping_packet.packetID[1] = 0xc4;
    housekeeping_packet.packetSequenceControl[0] = 0xc0;
    housekeeping_packet.packetSequenceControl[1] = 0x00;
    housekeeping_packet.packetLength[0] = 0x00;
    housekeeping_packet.packetLength[1] = 0x77;
    housekeeping_packet.dataFieldHeader[0] = 0x10;
    housekeeping_packet.dataFieldHeader[1] = 0x03;
    housekeeping_packet.dataFieldHeader[2] = 0x19;
    housekeeping_packet.dataFieldHeader[3] = 0x00;

    status = rtems_rate_monotonic_cancel(HK_id);
    if( status != RTEMS_SUCCESSFUL )
    PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status )
    else
    PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n")

    while(1){ // launch the rate monotonic task
        status = rtems_rate_monotonic_period( HK_id, HK_PERIOD );
        if ( status != RTEMS_SUCCESSFUL ){
            PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_period *** code %d\n", status);
        }
        else
        {
            housekeeping_packet.dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
            housekeeping_packet.dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
            housekeeping_packet.dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
            housekeeping_packet.dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
            housekeeping_packet.dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
            housekeeping_packet.dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
            housekeeping_packet.data[0] = CCSDS_DESTINATION_ID_DPU;
            result = write ( fdSPW, &housekeeping_packet, LEN_TM_LFR_HK);
            if (result==-1)
            {
                PRINTF("ERR *** in HOUS *** HK send\n");
            }
        }
    }

    PRINTF("in HOUS *** deleting task\n")

    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
    exit( 1 );
}


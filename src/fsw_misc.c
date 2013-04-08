#include <fsw_misc.h>
#include <fsw_params.h>

extern rtems_id Task_id[];         /* array of task ids */
extern int fdSPW;
extern TMHeader_t housekeeping_header;
extern char housekeeping_data[];

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

int send_console_outputs_on_serial_port() // Send the console outputs on the serial port
{
    struct apbuart_regs_str *apbuart_regs;

    apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;
    apbuart_regs->ctrl = apbuart_regs->ctrl & APBUART_CTRL_REG_MASK_DB;
    PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

    return 0;
}

rtems_task stat_task(rtems_task_argument argument)
{
    int i;
    i = 0;
    PRINTF("In STAT *** \n")
    while(1){
        rtems_task_wake_after(1000);
        PRINTF1("%d\n", i)
        if (i == 2) {
            #ifdef PRINT_TASK_STATISTICS
            rtems_cpu_usage_report();
            rtems_cpu_usage_reset();
            #endif
            i = 0;
        }
        else i++;
    }
}

rtems_task hous_task(rtems_task_argument argument)
{
    rtems_status_code status;

    spw_ioctl_pkt_send spw_ioctl_send;

    rtems_name  name;
    rtems_id    period;
    name = rtems_build_name( 'H', 'O', 'U', 'S' );
    status = rtems_rate_monotonic_create( name, &period );
    if( status != RTEMS_SUCCESSFUL ) {
        printf( "rtems_rate_monotonic_create failed with status of %d\n", status );
        exit( 1 );
    }

    // filling the structure for the spacewire transmission
    spw_ioctl_send.hlen = TM_HEADER_LEN + 4; // + 4 is for the protocole extra header
    spw_ioctl_send.hdr = (char*) &housekeeping_header;
    spw_ioctl_send.dlen = LENGTH_TM_LFR_HK - 10 + 1;
    spw_ioctl_send.data = housekeeping_data;

    housekeeping_header.targetLogicalAddress = CCSDS_DESTINATION_ID;
    housekeeping_header.protocolIdentifier = 0x02;
    housekeeping_header.reserved = 0x00;
    housekeeping_header.userApplication = 0x00;
    housekeeping_header.packetID[0] = 0x0c;
    housekeeping_header.packetID[1] = 0xc4;
    housekeeping_header.packetSequenceControl[0] = 0xc0;
    housekeeping_header.packetSequenceControl[1] = 0x00;
    housekeeping_header.packetLength[0] = 0x00;
    housekeeping_header.packetLength[1] = 0x77;
    housekeeping_header.dataFieldHeader[0] = 0x10;
    housekeeping_header.dataFieldHeader[1] = 0x03;
    housekeeping_header.dataFieldHeader[2] = 0x19;
    housekeeping_header.dataFieldHeader[3] = 0x00;

    printf("In HOUS ***\n");

    while(1){ // launch the rate monotonic task
        if ( rtems_rate_monotonic_period( period, HK_PERIOD ) == RTEMS_TIMEOUT ){
            printf( "ERR *** in hous_task *** RTEMS_TIMEOUT\n" );
            break;
        }
        housekeeping_header.dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
        housekeeping_header.dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
        housekeeping_header.dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
        housekeeping_header.dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
        housekeeping_header.dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
        housekeeping_header.dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
        status = write_spw(&spw_ioctl_send);
    }

    status = rtems_rate_monotonic_delete( period );
    if ( status != RTEMS_SUCCESSFUL ) {
        printf( "rtems_rate_monotonic_delete failed with status of %d.\n", status );
        exit( 1 );
    }
    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
    exit( 1 );
}


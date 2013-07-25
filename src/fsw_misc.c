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
    if (status==RTEMS_SUCCESSFUL)
    {
        PRINTF("In configure_timer *** rtems_interrupt_catch successfullly configured\n")
    }

    timer_set_clock_divider( gptimer_regs, timer, clock_divider);

    return 1;
}

int timer_start(gptimer_regs_t *gptimer_regs, unsigned char timer)
{
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000008;  // IE interrupt enable

    return 1;
}

int timer_stop(gptimer_regs_t *gptimer_regs, unsigned char timer)
{
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xfffffffe;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xffffffef;  // IE interrupt enable
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any

    return 1;
}

int timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider)
{
    gptimer_regs->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
    
    return 1;
}

void update_spacewire_statistics()
{
    rtems_status_code status;
    spw_stats spacewire_stats_grspw;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &spacewire_stats_grspw );

    spacewire_stats.packets_received = spacewire_stats_backup.packets_received
            + spacewire_stats_grspw.packets_received;
    spacewire_stats.packets_sent = spacewire_stats_backup.packets_sent
            + spacewire_stats_grspw.packets_sent;
    spacewire_stats.parity_err = spacewire_stats_backup.parity_err
            + spacewire_stats_grspw.parity_err;
    spacewire_stats.disconnect_err = spacewire_stats_backup.disconnect_err
            + spacewire_stats_grspw.disconnect_err;
    spacewire_stats.escape_err = spacewire_stats_backup.escape_err
            + spacewire_stats_grspw.escape_err;
    spacewire_stats.credit_err = spacewire_stats_backup.credit_err
            + spacewire_stats_grspw.credit_err;
    spacewire_stats.write_sync_err = spacewire_stats_backup.write_sync_err
            + spacewire_stats_grspw.write_sync_err;
    spacewire_stats.rx_rmap_header_crc_err = spacewire_stats_backup.rx_rmap_header_crc_err
            + spacewire_stats_grspw.rx_rmap_header_crc_err;
    spacewire_stats.rx_rmap_data_crc_err = spacewire_stats_backup.rx_rmap_data_crc_err
            + spacewire_stats_grspw.rx_rmap_data_crc_err;
    spacewire_stats.early_ep = spacewire_stats_backup.early_ep
            + spacewire_stats_grspw.early_ep;
    spacewire_stats.invalid_address = spacewire_stats_backup.invalid_address
            + spacewire_stats_grspw.invalid_address;
    spacewire_stats.rx_eep_err = spacewire_stats_backup.rx_eep_err
            +  spacewire_stats_grspw.rx_eep_err;
    spacewire_stats.rx_truncated = spacewire_stats_backup.rx_truncated
            + spacewire_stats_grspw.rx_truncated;
    //spacewire_stats.tx_link_err;

    //****************************
    // DPU_SPACEWIRE_IF_STATISTICS
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[0] = (unsigned char) (spacewire_stats.packets_received >> 8);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_rcv_cnt[1] = (unsigned char) (spacewire_stats.packets_received);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[0] = (unsigned char) (spacewire_stats.packets_sent >> 8);
    housekeeping_packet.hk_lfr_dpu_spw_pkt_sent_cnt[1] = (unsigned char) (spacewire_stats.packets_sent);
    //housekeeping_packet.hk_lfr_dpu_spw_tick_out_cnt;
    //housekeeping_packet.hk_lfr_dpu_spw_last_timc;

    //******************************************
    // ERROR COUNTERS / SPACEWIRE / LOW SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_parity = (unsigned char) spacewire_stats.parity_err;
    housekeeping_packet.hk_lfr_dpu_spw_disconnect = (unsigned char) spacewire_stats.disconnect_err;
    housekeeping_packet.hk_lfr_dpu_spw_escape = (unsigned char) spacewire_stats.escape_err;
    housekeeping_packet.hk_lfr_dpu_spw_credit = (unsigned char) spacewire_stats.credit_err;
    housekeeping_packet.hk_lfr_dpu_spw_write_sync = (unsigned char) spacewire_stats.write_sync_err;
    // housekeeping_packet.hk_lfr_dpu_spw_rx_ahb;
    // housekeeping_packet.hk_lfr_dpu_spw_tx_ahb;
    housekeeping_packet.hk_lfr_dpu_spw_header_crc = (unsigned char) spacewire_stats.rx_rmap_header_crc_err;
    housekeeping_packet.hk_lfr_dpu_spw_data_crc = (unsigned char) spacewire_stats.rx_rmap_data_crc_err;

    //*********************************************
    // ERROR COUNTERS / SPACEWIRE / MEDIUM SEVERITY
    housekeeping_packet.hk_lfr_dpu_spw_early_eop = (unsigned char) spacewire_stats.early_ep;
    housekeeping_packet.hk_lfr_dpu_spw_invalid_addr = (unsigned char) spacewire_stats.invalid_address;
    housekeeping_packet.hk_lfr_dpu_spw_eep = (unsigned char) spacewire_stats.rx_eep_err;
    housekeeping_packet.hk_lfr_dpu_spw_rx_too_big = (unsigned char) spacewire_stats.rx_truncated;

}

int send_console_outputs_on_apbuart_port( void ) // Send the console outputs on the apbuart port
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
    PRINTF1("OK  *** apbuart port scaler reload register set to 0x%x\n", value)

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

    if (rtems_rate_monotonic_ident( HK_name, &HK_id) != RTEMS_SUCCESSFUL) {
        status = rtems_rate_monotonic_create( HK_name, &HK_id );
        if( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "rtems_rate_monotonic_create failed with status of %d\n", status )
        }
    }

    housekeeping_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    housekeeping_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    housekeeping_packet.reserved = 0x00;
    housekeeping_packet.userApplication = 0x00;
    housekeeping_packet.packetID[0] = (unsigned char) (TM_PACKET_ID_HK >> 8);
    housekeeping_packet.packetID[1] = (unsigned char) (TM_PACKET_ID_HK);
    housekeeping_packet.packetSequenceControl[0] = 0xc0;
    housekeeping_packet.packetSequenceControl[1] = 0x00;
    housekeeping_packet.packetLength[0] = 0x00;
    housekeeping_packet.packetLength[1] = 0x77;
    housekeeping_packet.dataFieldHeader[0] = 0x10;
    housekeeping_packet.dataFieldHeader[1] = TM_TYPE_HK;
    housekeeping_packet.dataFieldHeader[2] = TM_SUBTYPE_HK;
    housekeeping_packet.dataFieldHeader[3] = TM_DESTINATION_ID_GROUND;

    status = rtems_rate_monotonic_cancel(HK_id);
    if( status != RTEMS_SUCCESSFUL ) {
        PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status )
    }
    else {
        PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n")
    }

    while(1){ // launch the rate monotonic task
        status = rtems_rate_monotonic_period( HK_id, HK_PERIOD );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_period *** code %d\n", status);
        }
        else {
            housekeeping_packet.dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
            housekeeping_packet.dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
            housekeeping_packet.dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
            housekeeping_packet.dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
            housekeeping_packet.dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
            housekeeping_packet.dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
            housekeeping_packet.sid = SID_HK;

            update_spacewire_statistics();

            // SEND PACKET
            result = write ( fdSPW, &housekeeping_packet, LEN_TM_LFR_HK);
            if (status == -1) {
                while (true) {
                    if (status != RTEMS_SUCCESSFUL) {
                        result = write ( fdSPW, &housekeeping_packet, LEN_TM_LFR_HK);
                        PRINTF("x")
                        sched_yield();
                    }
                    else {
                        PRINTF("\n")
                        break;
                    }
                }
            }
        }
    }

    PRINTF("in HOUS *** deleting task\n")

    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
    exit( 1 );
}


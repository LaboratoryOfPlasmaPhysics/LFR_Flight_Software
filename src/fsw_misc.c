/** General usage functions and RTEMS tasks.
 *
 * @file
 * @author P. LEROY
 *
 */

#include "fsw_misc.h"

void timer_configure(unsigned char timer, unsigned int clock_divider,
                    unsigned char interrupt_level, rtems_isr (*timer_isr)() )
{
    /** This function configures a GPTIMER timer instantiated in the VHDL design.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     * @param interrupt_level is the interrupt level that the timer drives.
     * @param timer_isr is the interrupt subroutine that will be attached to the IRQ driven by the timer.
     *
     * Interrupt levels are described in the SPARC documentation sparcv8.pdf p.76
     *
     */

    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    gptimer_regs->timer[timer].ctrl = 0x00;  // reset the control register

    status = rtems_interrupt_catch( timer_isr, interrupt_level, &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status!=RTEMS_SUCCESSFUL)
    {
        PRINTF("in configure_timer *** ERR rtems_interrupt_catch\n")
    }

    timer_set_clock_divider( timer, clock_divider);
}

void timer_start(unsigned char timer)
{
    /** This function starts a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000008;  // IE interrupt enable
}

void timer_stop(unsigned char timer)
{
    /** This function stops a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xfffffffe;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xffffffef;  // IE interrupt enable
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any
}

void timer_set_clock_divider(unsigned char timer, unsigned int clock_divider)
{
    /** This function sets the clock divider of a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     *
     */

    gptimer_regs->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
}

// WATCHDOG

rtems_isr watchdog_isr( rtems_vector_number vector )
{
    rtems_status_code status_code;

    status_code = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_12 );

    PRINTF("watchdog_isr *** this is the end, exit(0)\n");

    exit(0);
}

void watchdog_configure(void)
{
    /** This function configure the watchdog.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     * The watchdog is a timer provided by the GPTIMER IP core of the GRLIB.
     *
     */

    LEON_Mask_interrupt( IRQ_GPTIMER_WATCHDOG );    // mask gptimer/watchdog interrupt during configuration

    timer_configure( TIMER_WATCHDOG, CLKDIV_WATCHDOG, IRQ_SPARC_GPTIMER_WATCHDOG, watchdog_isr );

    LEON_Clear_interrupt( IRQ_GPTIMER_WATCHDOG );   // clear gptimer/watchdog interrupt
}

void watchdog_stop(void)
{
    LEON_Mask_interrupt( IRQ_GPTIMER_WATCHDOG );                  // mask gptimer/watchdog interrupt line
    timer_stop( TIMER_WATCHDOG );
    LEON_Clear_interrupt( IRQ_GPTIMER_WATCHDOG );                 // clear gptimer/watchdog interrupt
}

void watchdog_reload(void)
{
    /** This function reloads the watchdog timer counter with the timer reload value.
     *
     * @param void
     *
     * @return void
     *
     */

    gptimer_regs->timer[TIMER_WATCHDOG].ctrl = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | 0x00000004;  // LD load value from the reload register
}

void watchdog_start(void)
{
    /** This function starts the watchdog timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     */    

    LEON_Clear_interrupt( IRQ_GPTIMER_WATCHDOG );

    gptimer_regs->timer[TIMER_WATCHDOG].ctrl = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | 0x00000010;  // clear pending IRQ if any
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer[TIMER_WATCHDOG].ctrl = gptimer_regs->timer[TIMER_WATCHDOG].ctrl | 0x00000008;  // IE interrupt enable

    LEON_Unmask_interrupt( IRQ_GPTIMER_WATCHDOG );

}

int enable_apbuart_transmitter( void )  // set the bit 1, TE Transmitter Enable to 1 in the APBUART control register
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;

    apbuart_regs->ctrl = APBUART_CTRL_REG_MASK_TE;

    return 0;
}

void set_apbuart_scaler_reload_register(unsigned int regs, unsigned int value)
{
    /** This function sets the scaler reload register of the apbuart module
     *
     * @param regs is the address of the apbuart registers in memory
     * @param value is the value that will be stored in the scaler register
     *
     * The value shall be set by the software to get data on the serial interface.
     *
     */

    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) regs;

    apbuart_regs->scaler = value;

    BOOT_PRINTF1("OK  *** apbuart port scaler reload register set to 0x%x\n", value)
}

//************
// RTEMS TASKS

rtems_task load_task(rtems_task_argument argument)
{
    BOOT_PRINTF("in LOAD *** \n")

    rtems_status_code status;
    unsigned int i;
    unsigned int j;
    rtems_name name_watchdog_rate_monotonic;  // name of the watchdog rate monotonic
    rtems_id watchdog_period_id;              // id of the watchdog rate monotonic period

    name_watchdog_rate_monotonic = rtems_build_name( 'L', 'O', 'A', 'D' );

    status = rtems_rate_monotonic_create( name_watchdog_rate_monotonic, &watchdog_period_id );
    if( status != RTEMS_SUCCESSFUL ) {
        PRINTF1( "in LOAD *** rtems_rate_monotonic_create failed with status of %d\n", status )
    }

    i = 0;
    j = 0;

    watchdog_configure();

    watchdog_start();

    set_sy_lfr_watchdog_enabled( true );

    while(1){
        status = rtems_rate_monotonic_period( watchdog_period_id, WATCHDOG_PERIOD );
        watchdog_reload();
        i = i + 1;
        if ( i == 10 )
        {
            i = 0;
            j = j + 1;
            PRINTF1("%d\n", j)
        }
#ifdef DEBUG_WATCHDOG
        if (j == 3 )
        {
            status = rtems_task_delete(RTEMS_SELF);
        }
#endif
    }
}

rtems_task hous_task(rtems_task_argument argument)
{
    rtems_status_code status;
    rtems_status_code spare_status;
    rtems_id queue_id;
    rtems_rate_monotonic_period_status period_status;

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in HOUS *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in HOUS ***\n");

    if (rtems_rate_monotonic_ident( name_hk_rate_monotonic, &HK_id) != RTEMS_SUCCESSFUL) {
        status = rtems_rate_monotonic_create( name_hk_rate_monotonic, &HK_id );
        if( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "rtems_rate_monotonic_create failed with status of %d\n", status );
        }
    }

    status = rtems_rate_monotonic_cancel(HK_id);
    if( status != RTEMS_SUCCESSFUL ) {
        PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status );
    }
    else {
        DEBUG_PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n");
    }

    // startup phase
    status = rtems_rate_monotonic_period( HK_id, SY_LFR_TIME_SYN_TIMEOUT_in_ticks );
    status = rtems_rate_monotonic_get_status( HK_id, &period_status );
    DEBUG_PRINTF1("startup HK, HK_id status = %d\n", period_status.state)
    while(period_status.state != RATE_MONOTONIC_EXPIRED )   // after SY_LFR_TIME_SYN_TIMEOUT ms, starts HK anyway
    {
        if ((time_management_regs->coarse_time & 0x80000000) == 0x00000000) // check time synchronization
        {
            break;  // break if LFR is synchronized
        }
        else
        {
            status = rtems_rate_monotonic_get_status( HK_id, &period_status );
//            sched_yield();
            status = rtems_task_wake_after( 10 );        // wait SY_LFR_DPU_CONNECT_TIMEOUT 100 ms = 10 * 10 ms
        }
    }
    status = rtems_rate_monotonic_cancel(HK_id);
    DEBUG_PRINTF1("startup HK, HK_id status = %d\n", period_status.state)

    set_hk_lfr_reset_cause( POWER_ON );

    while(1){ // launch the rate monotonic task
        status = rtems_rate_monotonic_period( HK_id, HK_PERIOD );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "in HOUS *** ERR period: %d\n", status);
            spare_status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_6 );
        }
        else {
            housekeeping_packet.packetSequenceControl[0] = (unsigned char) (sequenceCounterHK >> 8);
            housekeeping_packet.packetSequenceControl[1] = (unsigned char) (sequenceCounterHK     );
            increment_seq_counter( &sequenceCounterHK );

            housekeeping_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
            housekeeping_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
            housekeeping_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
            housekeeping_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
            housekeeping_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
            housekeeping_packet.time[5] = (unsigned char) (time_management_regs->fine_time);

            spacewire_update_hk_lfr_link_state( &housekeeping_packet.lfr_status_word[0] );

            spacewire_read_statistics();

            update_hk_with_grspw_stats();

            set_hk_lfr_time_not_synchro();

            housekeeping_packet.hk_lfr_q_sd_fifo_size_max = hk_lfr_q_sd_fifo_size_max;
            housekeeping_packet.hk_lfr_q_rv_fifo_size_max = hk_lfr_q_rv_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p0_fifo_size_max = hk_lfr_q_p0_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p1_fifo_size_max = hk_lfr_q_p1_fifo_size_max;
            housekeeping_packet.hk_lfr_q_p2_fifo_size_max = hk_lfr_q_p2_fifo_size_max;

            housekeeping_packet.sy_lfr_common_parameters_spare  = parameter_dump_packet.sy_lfr_common_parameters_spare;
            housekeeping_packet.sy_lfr_common_parameters        = parameter_dump_packet.sy_lfr_common_parameters;
            get_temperatures( housekeeping_packet.hk_lfr_temp_scm );
            get_v_e1_e2_f3(   housekeeping_packet.hk_lfr_sc_v_f3  );
            get_cpu_load( (unsigned char *) &housekeeping_packet.hk_lfr_cpu_load );

            hk_lfr_le_me_he_update();

            housekeeping_packet.hk_lfr_sc_rw_f_flags = cp_rpw_sc_rw_f_flags;

            // SEND PACKET
            status =  rtems_message_queue_send( queue_id, &housekeeping_packet,
                                                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
            if (status != RTEMS_SUCCESSFUL) {
                PRINTF1("in HOUS *** ERR send: %d\n", status)
            }
        }
    }

    PRINTF("in HOUS *** deleting task\n")

    status = rtems_task_delete( RTEMS_SELF ); // should not return

    return;
}

rtems_task dumb_task( rtems_task_argument unused )
{
    /** This RTEMS taks is used to print messages without affecting the general behaviour of the software.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The DUMB taks waits for RTEMS events and print messages depending on the incoming events.
     *
     */

    unsigned int i;
    unsigned int intEventOut;
    unsigned int coarse_time = 0;
    unsigned int fine_time = 0;
    rtems_event_set event_out;

    char *DumbMessages[15] = {"in DUMB *** default",                                            // RTEMS_EVENT_0
                        "in DUMB *** timecode_irq_handler",                                     // RTEMS_EVENT_1
                        "in DUMB *** f3 buffer changed",                                        // RTEMS_EVENT_2
                        "in DUMB *** in SMIQ *** Error sending event to AVF0",                  // RTEMS_EVENT_3
                        "in DUMB *** spectral_matrices_isr *** Error sending event to SMIQ",    // RTEMS_EVENT_4
                        "in DUMB *** waveforms_simulator_isr",                                  // RTEMS_EVENT_5
                        "VHDL SM *** two buffers f0 ready",                                     // RTEMS_EVENT_6
                        "ready for dump",                                                       // RTEMS_EVENT_7
                        "VHDL ERR *** spectral matrix",                                         // RTEMS_EVENT_8
                        "tick",                                                                 // RTEMS_EVENT_9
                        "VHDL ERR *** waveform picker",                                         // RTEMS_EVENT_10
                        "VHDL ERR *** unexpected ready matrix values",                          // RTEMS_EVENT_11
                        "WATCHDOG timer",                                                       // RTEMS_EVENT_12
                        "TIMECODE timer",                                                       // RTEMS_EVENT_13
                        "TIMECODE ISR"                                                          // RTEMS_EVENT_14
    };

    BOOT_PRINTF("in DUMB *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3
                            | RTEMS_EVENT_4 | RTEMS_EVENT_5 | RTEMS_EVENT_6 | RTEMS_EVENT_7
                            | RTEMS_EVENT_8 | RTEMS_EVENT_9 | RTEMS_EVENT_12 | RTEMS_EVENT_13
                            | RTEMS_EVENT_14,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut =  (unsigned int) event_out;
        for ( i=0; i<32; i++)
        {
            if ( ((intEventOut >> i) & 0x0001) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                if (i==12)
                {
                    PRINTF1("%s\n", DumbMessages[12])
                }
                if (i==13)
                {
                    PRINTF1("%s\n", DumbMessages[13])
                }
                if (i==14)
                {
                    PRINTF1("%s\n", DumbMessages[1])
                }
            }
        }
    }
}

//*****************************
// init housekeeping parameters

void init_housekeeping_parameters( void )
{
    /** This function initialize the housekeeping_packet global variable with default values.
     *
     */

    unsigned int i = 0;
    unsigned char *parameters;
    unsigned char sizeOfHK;

    sizeOfHK = sizeof( Packet_TM_LFR_HK_t );

    parameters = (unsigned char*) &housekeeping_packet;

    for(i = 0; i< sizeOfHK; i++)
    {
        parameters[i] = 0x00;
    }

    housekeeping_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    housekeeping_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    housekeeping_packet.reserved = DEFAULT_RESERVED;
    housekeeping_packet.userApplication = CCSDS_USER_APP;
    housekeeping_packet.packetID[0] = (unsigned char) (APID_TM_HK >> 8);
    housekeeping_packet.packetID[1] = (unsigned char) (APID_TM_HK);
    housekeeping_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    housekeeping_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    housekeeping_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_HK >> 8);
    housekeeping_packet.packetLength[1] = (unsigned char) (PACKET_LENGTH_HK     );
    housekeeping_packet.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    housekeeping_packet.serviceType = TM_TYPE_HK;
    housekeeping_packet.serviceSubType = TM_SUBTYPE_HK;
    housekeeping_packet.destinationID = TM_DESTINATION_ID_GROUND;
    housekeeping_packet.sid = SID_HK;

    // init status word
    housekeeping_packet.lfr_status_word[0] = DEFAULT_STATUS_WORD_BYTE0;
    housekeeping_packet.lfr_status_word[1] = DEFAULT_STATUS_WORD_BYTE1;
    // init software version
    housekeeping_packet.lfr_sw_version[0] = SW_VERSION_N1;
    housekeeping_packet.lfr_sw_version[1] = SW_VERSION_N2;
    housekeeping_packet.lfr_sw_version[2] = SW_VERSION_N3;
    housekeeping_packet.lfr_sw_version[3] = SW_VERSION_N4;
    // init fpga version
    parameters = (unsigned char *) (REGS_ADDR_VHDL_VERSION);
    housekeeping_packet.lfr_fpga_version[0] = parameters[1]; // n1
    housekeeping_packet.lfr_fpga_version[1] = parameters[2]; // n2
    housekeeping_packet.lfr_fpga_version[2] = parameters[3]; // n3

    housekeeping_packet.hk_lfr_q_sd_fifo_size = MSG_QUEUE_COUNT_SEND;
    housekeeping_packet.hk_lfr_q_rv_fifo_size = MSG_QUEUE_COUNT_RECV;
    housekeeping_packet.hk_lfr_q_p0_fifo_size = MSG_QUEUE_COUNT_PRC0;
    housekeeping_packet.hk_lfr_q_p1_fifo_size = MSG_QUEUE_COUNT_PRC1;
    housekeeping_packet.hk_lfr_q_p2_fifo_size = MSG_QUEUE_COUNT_PRC2;
}

void increment_seq_counter( unsigned short *packetSequenceControl )
{
    /** This function increment the sequence counter passes in argument.
     *
     * The increment does not affect the grouping flag. In case of an overflow, the counter is reset to 0.
     *
     */

    unsigned short segmentation_grouping_flag;
    unsigned short sequence_cnt;

    segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << 8;   // keep bits 7 downto 6
    sequence_cnt                = (*packetSequenceControl) & 0x3fff;    // [0011 1111 1111 1111]

    if ( sequence_cnt < SEQ_CNT_MAX)
    {
        sequence_cnt = sequence_cnt + 1;
    }
    else
    {
        sequence_cnt = 0;
    }

    *packetSequenceControl = segmentation_grouping_flag | sequence_cnt ;
}

void getTime( unsigned char *time)
{
    /** This function write the current local time in the time buffer passed in argument.
     *
     */

    time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    time[3] = (unsigned char) (time_management_regs->coarse_time);
    time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    time[5] = (unsigned char) (time_management_regs->fine_time);
}

unsigned long long int getTimeAsUnsignedLongLongInt( )
{
    /** This function write the current local time in the time buffer passed in argument.
     *
     */
    unsigned long long int time;

    time = ( (unsigned long long int) (time_management_regs->coarse_time & 0x7fffffff) << 16 )
            + time_management_regs->fine_time;

    return time;
}

void send_dumb_hk( void )
{
    Packet_TM_LFR_HK_t dummy_hk_packet;
    unsigned char *parameters;
    unsigned int i;
    rtems_id queue_id;

    dummy_hk_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    dummy_hk_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    dummy_hk_packet.reserved = DEFAULT_RESERVED;
    dummy_hk_packet.userApplication = CCSDS_USER_APP;
    dummy_hk_packet.packetID[0] = (unsigned char) (APID_TM_HK >> 8);
    dummy_hk_packet.packetID[1] = (unsigned char) (APID_TM_HK);
    dummy_hk_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    dummy_hk_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    dummy_hk_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_HK >> 8);
    dummy_hk_packet.packetLength[1] = (unsigned char) (PACKET_LENGTH_HK     );
    dummy_hk_packet.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    dummy_hk_packet.serviceType = TM_TYPE_HK;
    dummy_hk_packet.serviceSubType = TM_SUBTYPE_HK;
    dummy_hk_packet.destinationID = TM_DESTINATION_ID_GROUND;
    dummy_hk_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    dummy_hk_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    dummy_hk_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    dummy_hk_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
    dummy_hk_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    dummy_hk_packet.time[5] = (unsigned char) (time_management_regs->fine_time);
    dummy_hk_packet.sid = SID_HK;

    // init status word
    dummy_hk_packet.lfr_status_word[0] = 0xff;
    dummy_hk_packet.lfr_status_word[1] = 0xff;
    // init software version
    dummy_hk_packet.lfr_sw_version[0] = SW_VERSION_N1;
    dummy_hk_packet.lfr_sw_version[1] = SW_VERSION_N2;
    dummy_hk_packet.lfr_sw_version[2] = SW_VERSION_N3;
    dummy_hk_packet.lfr_sw_version[3] = SW_VERSION_N4;
    // init fpga version
    parameters = (unsigned char *) (REGS_ADDR_WAVEFORM_PICKER + 0xb0);
    dummy_hk_packet.lfr_fpga_version[0] = parameters[1]; // n1
    dummy_hk_packet.lfr_fpga_version[1] = parameters[2]; // n2
    dummy_hk_packet.lfr_fpga_version[2] = parameters[3]; // n3

    parameters = (unsigned char *) &dummy_hk_packet.hk_lfr_cpu_load;

    for (i=0; i<100; i++)
    {
        parameters[i] = 0xff;
    }

    get_message_queue_id_send( &queue_id );

    rtems_message_queue_send( queue_id, &dummy_hk_packet,
                                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
}

void get_temperatures( unsigned char *temperatures )
{
    unsigned char* temp_scm_ptr;
    unsigned char* temp_pcb_ptr;
    unsigned char* temp_fpga_ptr;

    // SEL1 SEL0
    // 0    0       => PCB
    // 0    1       => FPGA
    // 1    0       => SCM

    temp_scm_ptr  = (unsigned char *) &time_management_regs->temp_scm;
    temp_pcb_ptr =  (unsigned char *) &time_management_regs->temp_pcb;
    temp_fpga_ptr = (unsigned char *) &time_management_regs->temp_fpga;

    temperatures[0] = temp_scm_ptr[2];
    temperatures[1] = temp_scm_ptr[3];
    temperatures[2] = temp_pcb_ptr[2];
    temperatures[3] = temp_pcb_ptr[3];
    temperatures[4] = temp_fpga_ptr[2];
    temperatures[5] = temp_fpga_ptr[3];
}

void get_v_e1_e2_f3( unsigned char *spacecraft_potential )
{
    unsigned char* v_ptr;
    unsigned char* e1_ptr;
    unsigned char* e2_ptr;

    v_ptr  = (unsigned char *) &waveform_picker_regs->v;
    e1_ptr = (unsigned char *) &waveform_picker_regs->e1;
    e2_ptr = (unsigned char *) &waveform_picker_regs->e2;

    spacecraft_potential[0] = v_ptr[2];
    spacecraft_potential[1] = v_ptr[3];
    spacecraft_potential[2] = e1_ptr[2];
    spacecraft_potential[3] = e1_ptr[3];
    spacecraft_potential[4] = e2_ptr[2];
    spacecraft_potential[5] = e2_ptr[3];
}

void get_cpu_load( unsigned char *resource_statistics )
{
    unsigned char cpu_load;

    cpu_load = lfr_rtems_cpu_usage_report();

    // HK_LFR_CPU_LOAD
    resource_statistics[0] = cpu_load;

    // HK_LFR_CPU_LOAD_MAX
    if (cpu_load > resource_statistics[1])
    {
         resource_statistics[1] = cpu_load;
    }

    // CPU_LOAD_AVE
    resource_statistics[2] = 0;

#ifndef PRINT_TASK_STATISTICS
        rtems_cpu_usage_reset();
#endif

}

void set_hk_lfr_sc_potential_flag( bool state )
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] | 0x40;   // [0100 0000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] & 0xbf;   // [1011 1111]
    }
}

void set_sy_lfr_watchdog_enabled( bool state )
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] | 0x10;   // [0001 0000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] & 0xef;   // [1110 1111]
    }
}

void set_hk_lfr_calib_enable( bool state )
{
    if (state == true)
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] | 0x08;   // [0000 1000]
    }
    else
    {
        housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] & 0xf7;   // [1111 0111]
    }
}

void set_hk_lfr_reset_cause( enum lfr_reset_cause_t lfr_reset_cause )
{
    housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1] & 0xf8; // [1111 1000]

    housekeeping_packet.lfr_status_word[1] = housekeeping_packet.lfr_status_word[1]
            | (lfr_reset_cause & 0x07 );   // [0000 0111]

}

void hk_lfr_le_me_he_update()
{
    unsigned int hk_lfr_le_cnt;
    unsigned int hk_lfr_me_cnt;
    unsigned int hk_lfr_he_cnt;
    unsigned int current_hk_lfr_le_cnt;
    unsigned int current_hk_lfr_me_cnt;
    unsigned int current_hk_lfr_he_cnt;

    hk_lfr_le_cnt = 0;
    hk_lfr_me_cnt = 0;
    hk_lfr_he_cnt = 0;
    current_hk_lfr_le_cnt = ((unsigned int) housekeeping_packet.hk_lfr_le_cnt[0]) * 256 + housekeeping_packet.hk_lfr_le_cnt[1];
    current_hk_lfr_me_cnt = ((unsigned int) housekeeping_packet.hk_lfr_me_cnt[0]) * 256 + housekeeping_packet.hk_lfr_me_cnt[1];
    current_hk_lfr_he_cnt = ((unsigned int) housekeeping_packet.hk_lfr_he_cnt[0]) * 256 + housekeeping_packet.hk_lfr_he_cnt[1];

    //update the low severity error counter
    hk_lfr_le_cnt =
            current_hk_lfr_le_cnt
            + housekeeping_packet.hk_lfr_dpu_spw_parity
            + housekeeping_packet.hk_lfr_dpu_spw_disconnect
            + housekeeping_packet.hk_lfr_dpu_spw_escape
            + housekeeping_packet.hk_lfr_dpu_spw_credit
            + housekeeping_packet.hk_lfr_dpu_spw_write_sync
            + housekeeping_packet.hk_lfr_timecode_erroneous
            + housekeeping_packet.hk_lfr_timecode_missing
            + housekeeping_packet.hk_lfr_timecode_invalid
            + housekeeping_packet.hk_lfr_time_timecode_it
            + housekeeping_packet.hk_lfr_time_not_synchro
            + housekeeping_packet.hk_lfr_time_timecode_ctr
            + housekeeping_packet.hk_lfr_ahb_correctable;
    // housekeeping_packet.hk_lfr_dpu_spw_rx_ahb => not handled by the grspw driver
    // housekeeping_packet.hk_lfr_dpu_spw_tx_ahb => not handled by the grspw driver

    //update the medium severity error counter
    hk_lfr_me_cnt =
            current_hk_lfr_me_cnt
            + housekeeping_packet.hk_lfr_dpu_spw_early_eop
            + housekeeping_packet.hk_lfr_dpu_spw_invalid_addr
            + housekeeping_packet.hk_lfr_dpu_spw_eep
            + housekeeping_packet.hk_lfr_dpu_spw_rx_too_big;

    //update the high severity error counter
    hk_lfr_he_cnt = 0;

    // update housekeeping packet counters, convert unsigned int numbers in 2 bytes numbers
    // LE
    housekeeping_packet.hk_lfr_le_cnt[0] = (unsigned char) ((hk_lfr_le_cnt & 0xff00) >> 8);
    housekeeping_packet.hk_lfr_le_cnt[1] = (unsigned char)  (hk_lfr_le_cnt & 0x00ff);
    // ME
    housekeeping_packet.hk_lfr_me_cnt[0] = (unsigned char) ((hk_lfr_me_cnt & 0xff00) >> 8);
    housekeeping_packet.hk_lfr_me_cnt[1] = (unsigned char)  (hk_lfr_me_cnt & 0x00ff);
    // HE
    housekeeping_packet.hk_lfr_he_cnt[0] = (unsigned char) ((hk_lfr_he_cnt & 0xff00) >> 8);
    housekeeping_packet.hk_lfr_he_cnt[1] = (unsigned char)  (hk_lfr_he_cnt & 0x00ff);

}

void set_hk_lfr_time_not_synchro()
{
    static unsigned char synchroLost = 1;
    int synchronizationBit;

    // get the synchronization bit
    synchronizationBit = (time_management_regs->coarse_time & 0x80000000) >> 31;    // 1000 0000 0000 0000

    switch (synchronizationBit)
    {
    case 0:
        if (synchroLost == 1)
        {
            synchroLost = 0;
        }
        break;
    case 1:
        if (synchroLost == 0 )
        {
            synchroLost = 1;
            increase_unsigned_char_counter(&housekeeping_packet.hk_lfr_time_not_synchro);
            update_hk_lfr_last_er_fields( RID_LE_LFR_TIME, CODE_NOT_SYNCHRO );
        }
        break;
    default:
        PRINTF1("in hk_lfr_time_not_synchro *** unexpected value for synchronizationBit = %d\n", synchronizationBit);
        break;
    }

}

void set_hk_lfr_ahb_correctable()   // CRITICITY L
{
    /** This function builds the error counter hk_lfr_ahb_correctable using the statistics provided
     * by the Cache Control Register (ASI 2, offset 0) and in the Register Protection Control Register (ASR16) on the
     * detected errors in the cache, in the integer unit and in the floating point unit.
     *
     * @param void
     *
     * @return void
     *
     * All errors are summed to set the value of the hk_lfr_ahb_correctable counter.
     *
    */

    unsigned int ahb_correctable;
    unsigned int instructionErrorCounter;
    unsigned int dataErrorCounter;
    unsigned int fprfErrorCounter;
    unsigned int iurfErrorCounter;

    CCR_getInstructionAndDataErrorCounters( &instructionErrorCounter, &dataErrorCounter);
    ASR16_get_FPRF_IURF_ErrorCounters( &fprfErrorCounter, &iurfErrorCounter);

    ahb_correctable = instructionErrorCounter
            + dataErrorCounter
            + fprfErrorCounter
            + iurfErrorCounter
            + housekeeping_packet.hk_lfr_ahb_correctable;

    housekeeping_packet.hk_lfr_ahb_correctable = (unsigned char) (ahb_correctable & 0xff);  // [1111 1111]

}

/** General usage functions and RTEMS tasks.
 *
 * @file
 * @author P. LEROY
 *
 */

#include "fsw_misc.h"

void configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
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

    timer_set_clock_divider( gptimer_regs, timer, clock_divider);
}

void timer_start(gptimer_regs_t *gptimer_regs, unsigned char timer)
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

void timer_stop(gptimer_regs_t *gptimer_regs, unsigned char timer)
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

void timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider)
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

int send_console_outputs_on_apbuart_port( void ) // Send the console outputs on the apbuart port
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;

    apbuart_regs->ctrl = APBUART_CTRL_REG_MASK_TE;

    return 0;
}

int enable_apbuart_transmitter( void )  // set the bit 1, TE Transmitter Enable to 1 in the APBUART control register
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;

    apbuart_regs->ctrl = apbuart_regs->ctrl | APBUART_CTRL_REG_MASK_TE;

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

rtems_task stat_task(rtems_task_argument argument)
{
    int i;
    int j;
    i = 0;
    j = 0;
    BOOT_PRINTF("in STAT *** \n")
    while(1){
        rtems_task_wake_after(1000);
        PRINTF1("%d\n", j)
        if (i == CPU_USAGE_REPORT_PERIOD) {
//            #ifdef PRINT_TASK_STATISTICS
//            rtems_cpu_usage_report();
//            rtems_cpu_usage_reset();
//            #endif
            i = 0;
        }
        else i++;
        j++;
    }
}

rtems_task hous_task(rtems_task_argument argument)
{
    rtems_status_code status;
    rtems_id queue_id;
    rtems_rate_monotonic_period_status period_status;

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in HOUS *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in HOUS ***\n")

    if (rtems_rate_monotonic_ident( name_hk_rate_monotonic, &HK_id) != RTEMS_SUCCESSFUL) {
        status = rtems_rate_monotonic_create( name_hk_rate_monotonic, &HK_id );
        if( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "rtems_rate_monotonic_create failed with status of %d\n", status )
        }
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

    status = rtems_rate_monotonic_cancel(HK_id);
    if( status != RTEMS_SUCCESSFUL ) {
        PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status )
    }
    else {
        DEBUG_PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n")
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

    while(1){ // launch the rate monotonic task
        status = rtems_rate_monotonic_period( HK_id, HK_PERIOD );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "in HOUS *** ERR period: %d\n", status);
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_6 );
        }
        else {
            housekeeping_packet.packetSequenceControl[0] = (unsigned char) sequenceCounterHK >> 8;
            housekeeping_packet.packetSequenceControl[1] = (unsigned char) sequenceCounterHK;
            increment_seq_counter( &sequenceCounterHK );

            housekeeping_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
            housekeeping_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
            housekeeping_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
            housekeeping_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
            housekeeping_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
            housekeeping_packet.time[5] = (unsigned char) (time_management_regs->fine_time);

            spacewire_update_statistics();

            get_v_e1_e2_f3( housekeeping_packet.hk_lfr_sc_v_f3 );
            get_cpu_load( (unsigned char *) &housekeeping_packet.hk_lfr_cpu_load );

            // SEND PACKET
            status =  rtems_message_queue_urgent( queue_id, &housekeeping_packet,
                                                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
            if (status != RTEMS_SUCCESSFUL) {
                PRINTF1("in HOUS *** ERR send: %d\n", status)
            }
        }
    }

    PRINTF("in HOUS *** deleting task\n")

    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
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

    char *DumbMessages[12] = {"in DUMB *** default",                                            // RTEMS_EVENT_0
                        "in DUMB *** timecode_irq_handler",                                     // RTEMS_EVENT_1
                        "in DUMB *** f3 buffer changed",                                        // RTEMS_EVENT_2
                        "in DUMB *** in SMIQ *** Error sending event to AVF0",                  // RTEMS_EVENT_3
                        "in DUMB *** spectral_matrices_isr *** Error sending event to SMIQ",    // RTEMS_EVENT_4
                        "in DUMB *** waveforms_simulator_isr",                                  // RTEMS_EVENT_5
                        "ERR HK",                                                               // RTEMS_EVENT_6
                        "ready for dump",                                                       // RTEMS_EVENT_7
                        "VHDL ERR *** spectral matrix",                                         // RTEMS_EVENT_8
                        "tick",                                                                 // RTEMS_EVENT_9
                        "VHDL ERR *** waveform picker",                                         // RTEMS_EVENT_10
                        "VHDL ERR *** unexpected ready matrix values"                           // RTEMS_EVENT_11
    };

    BOOT_PRINTF("in DUMB *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3
                            | RTEMS_EVENT_4 | RTEMS_EVENT_5 | RTEMS_EVENT_6 | RTEMS_EVENT_7
                            | RTEMS_EVENT_8 | RTEMS_EVENT_9,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut =  (unsigned int) event_out;
        for ( i=0; i<32; i++)
        {
            if ( ((intEventOut >> i) & 0x0001) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                printf("in DUMB *** coarse: %x, fine: %x, %s\n", coarse_time, fine_time, DumbMessages[i]);
                if (i==8)
                {
                    PRINTF1("spectral_matrix_regs->status = %x\n", spectral_matrix_regs->status)
                }
                if (i==10)
                {
                    PRINTF1("waveform_picker_regs->status = %x\n", waveform_picker_regs->status)
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

    parameters = (unsigned char*) &housekeeping_packet.lfr_status_word;
    for(i = 0; i< SIZE_HK_PARAMETERS; i++)
    {
        parameters[i] = 0x00;
    }
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
}

void increment_seq_counter_old( unsigned char *packet_sequence_control)
{
    /** This function increment the sequence counter psased in argument.
     *
     * The increment does not affect the grouping flag. In case of an overflow, the counter is reset to 0.
     *
     */

    unsigned short sequence_cnt;
    unsigned short segmentation_grouping_flag;
    unsigned short new_packet_sequence_control;

    segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << 8;   // keep bits 7 downto 6
    sequence_cnt                = (unsigned short) (
                ( (packet_sequence_control[0] & 0x3f) << 8 )    // keep bits 5 downto 0
                                + packet_sequence_control[1]
            );

    new_packet_sequence_control = segmentation_grouping_flag | sequence_cnt ;

    packet_sequence_control[0] = (unsigned char) (new_packet_sequence_control >> 8);
    packet_sequence_control[1] = (unsigned char) (new_packet_sequence_control     );

    if ( sequence_cnt < SEQ_CNT_MAX)
    {
        sequence_cnt = sequence_cnt + 1;
    }
    else
    {
        sequence_cnt = 0;
    }
}

void increment_seq_counter( unsigned short *packetSequenceControl )
{
    /** This function increment the sequence counter psased in argument.
     *
     * The increment does not affect the grouping flag. In case of an overflow, the counter is reset to 0.
     *
     */

    unsigned short sequence_cnt;
    unsigned short segmentation_grouping_flag;

    segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << 8;   // keep bits 7 downto 6
    sequence_cnt                = (*packetSequenceControl) & 0x3fff;       // [0011 1111 1111 1111]

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

    rtems_message_queue_urgent( queue_id, &dummy_hk_packet,
                                PACKET_LENGTH_HK + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
}

void get_v_e1_e2_f3_old( unsigned char *spacecraft_potential )
{
    unsigned int coarseTime;
    unsigned int acquisitionTime;
    unsigned int deltaT = 0;
    unsigned char *bufferPtr;

    unsigned int offset_in_samples;
    unsigned int offset_in_bytes;
    unsigned char f3 = 16;    // v, e1 and e2 will be picked up each second, f3 = 16 Hz

    if (lfrCurrentMode == LFR_MODE_STANDBY)
    {
        spacecraft_potential[0] = 0x00;
        spacecraft_potential[1] = 0x00;
        spacecraft_potential[2] = 0x00;
        spacecraft_potential[3] = 0x00;
        spacecraft_potential[4] = 0x00;
        spacecraft_potential[5] = 0x00;
    }
    else
    {
        coarseTime = time_management_regs->coarse_time & 0x7fffffff;
        bufferPtr = (unsigned char*) current_ring_node_f3->buffer_address;
        acquisitionTime = (unsigned int) ( ( bufferPtr[2] & 0x7f ) << 24 )
                + (unsigned int) ( bufferPtr[3] << 16 )
                + (unsigned int) ( bufferPtr[0] << 8  )
                + (unsigned int) ( bufferPtr[1]       );
        if ( coarseTime > acquisitionTime )
        {
            deltaT = coarseTime - acquisitionTime;
            offset_in_samples = (deltaT-1) * f3 ;
        }
        else if( coarseTime == acquisitionTime )
        {
            bufferPtr = (unsigned char*) current_ring_node_f3->previous->buffer_address;    // pick up v e1 and e2 in the previous f3 buffer
            offset_in_samples = NB_SAMPLES_PER_SNAPSHOT-1;
        }
        else
        {
            offset_in_samples = 0;
            PRINTF2("ERR *** in get_v_e1_e2_f3 *** coarseTime = %x, acquisitionTime = %x\n", coarseTime, acquisitionTime)
        }

        if ( offset_in_samples > (NB_SAMPLES_PER_SNAPSHOT - 1) )
        {
            PRINTF1("ERR *** in get_v_e1_e2_f3 *** trying to read out of the buffer, counter = %d\n", offset_in_samples)
            offset_in_samples = NB_SAMPLES_PER_SNAPSHOT -1;
        }
        offset_in_bytes = TIME_OFFSET_IN_BYTES + offset_in_samples * NB_WORDS_SWF_BLK * 4;
        spacecraft_potential[0] = bufferPtr[ offset_in_bytes + 0];
        spacecraft_potential[1] = bufferPtr[ offset_in_bytes + 1];
        spacecraft_potential[2] = bufferPtr[ offset_in_bytes + 2];
        spacecraft_potential[3] = bufferPtr[ offset_in_bytes + 3];
        spacecraft_potential[4] = bufferPtr[ offset_in_bytes + 4];
        spacecraft_potential[5] = bufferPtr[ offset_in_bytes + 5];
    }
}

void get_v_e1_e2_f3( unsigned char *spacecraft_potential )
{
    unsigned int coarseTime;
    unsigned int acquisitionTime;
    unsigned int deltaT = 0;
    unsigned char *bufferPtr;

    unsigned int offset_in_samples;
    unsigned int offset_in_bytes;
    unsigned char f3 = 16;    // v, e1 and e2 will be picked up each second, f3 = 16 Hz

    if (lfrCurrentMode == LFR_MODE_STANDBY)
    {
        spacecraft_potential[0] = 0x00;
        spacecraft_potential[1] = 0x00;
        spacecraft_potential[2] = 0x00;
        spacecraft_potential[3] = 0x00;
        spacecraft_potential[4] = 0x00;
        spacecraft_potential[5] = 0x00;
    }
    else
    {
        coarseTime = time_management_regs->coarse_time & 0x7fffffff;
        bufferPtr = (unsigned char*) current_ring_node_f3->buffer_address;
        acquisitionTime = (unsigned int) ( ( bufferPtr[0] & 0x7f ) << 24 )
                + (unsigned int) ( bufferPtr[1] << 16 )
                + (unsigned int) ( bufferPtr[2] << 8  )
                + (unsigned int) ( bufferPtr[3]       );
        if ( coarseTime > acquisitionTime )
        {
            deltaT = coarseTime - acquisitionTime;
            offset_in_samples = (deltaT-1) * f3 ;
        }
        else if( coarseTime == acquisitionTime )
        {
            bufferPtr = (unsigned char*) current_ring_node_f3->previous->buffer_address;    // pick up v e1 and e2 in the previous f3 buffer
            offset_in_samples = NB_SAMPLES_PER_SNAPSHOT-1;
        }
        else
        {
            offset_in_samples = 0;
            PRINTF2("ERR *** in get_v_e1_e2_f3 *** coarseTime = %x, acquisitionTime = %x\n", coarseTime, acquisitionTime)
        }

        if ( offset_in_samples > (NB_SAMPLES_PER_SNAPSHOT - 1) )
        {
            PRINTF1("ERR *** in get_v_e1_e2_f3 *** trying to read out of the buffer, counter = %d\n", offset_in_samples)
            offset_in_samples = NB_SAMPLES_PER_SNAPSHOT -1;
        }
        offset_in_bytes = TIME_OFFSET_IN_BYTES + offset_in_samples * NB_WORDS_SWF_BLK * 4;
        spacecraft_potential[0] = bufferPtr[ offset_in_bytes + 0];
        spacecraft_potential[1] = bufferPtr[ offset_in_bytes + 1];
        spacecraft_potential[2] = bufferPtr[ offset_in_bytes + 2];
        spacecraft_potential[3] = bufferPtr[ offset_in_bytes + 3];
        spacecraft_potential[4] = bufferPtr[ offset_in_bytes + 4];
        spacecraft_potential[5] = bufferPtr[ offset_in_bytes + 5];
    }
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




/** General usage functions and RTEMS tasks.
 *
 * @file
 * @author P. LEROY
 *
 */

#include "fsw_misc.h"

char *DumbMessages[6] = {"in DUMB *** default",                                             // RTEMS_EVENT_0
                    "in DUMB *** timecode_irq_handler",                                     // RTEMS_EVENT_1
                    "in DUMB *** waveforms_isr",                                            // RTEMS_EVENT_2
                    "in DUMB *** in SMIQ *** Error sending event to AVF0",                  // RTEMS_EVENT_3
                    "in DUMB *** spectral_matrices_isr *** Error sending event to SMIQ",    // RTEMS_EVENT_4
                    "in DUMB *** waveforms_simulator_isr"                                   // RTEMS_EVENT_5
};

int configure_timer(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider,
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
     * @return
     *
     * Interrupt levels are described in the SPARC documentation sparcv8.pdf p.76
     *
     */

    rtems_status_code status;
    rtems_isr_entry old_isr_handler;

    status = rtems_interrupt_catch( timer_isr, interrupt_level, &old_isr_handler) ; // see sparcv8.pdf p.76 for interrupt levels
    if (status!=RTEMS_SUCCESSFUL)
    {
        PRINTF("in configure_timer *** ERR rtems_interrupt_catch\n")
    }

    timer_set_clock_divider( gptimer_regs, timer, clock_divider);

    return 1;
}

int timer_start(gptimer_regs_t *gptimer_regs, unsigned char timer)
{
    /** This function starts a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     * @return 1
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000004;  // LD load value from the reload register
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000001;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000002;  // RS restart
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000008;  // IE interrupt enable

    return 1;
}

int timer_stop(gptimer_regs_t *gptimer_regs, unsigned char timer)
{
    /** This function stops a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     *
     * @return 1
     *
     */

    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xfffffffe;  // EN enable the timer
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl & 0xffffffef;  // IE interrupt enable
    gptimer_regs->timer[timer].ctrl = gptimer_regs->timer[timer].ctrl | 0x00000010;  // clear pending IRQ if any

    return 1;
}

int timer_set_clock_divider(gptimer_regs_t *gptimer_regs, unsigned char timer, unsigned int clock_divider)
{
    /** This function sets the clock divider of a GPTIMER timer.
     *
     * @param gptimer_regs points to the APB registers of the GPTIMER IP core.
     * @param timer is the number of the timer in the IP core (several timers can be instantiated).
     * @param clock_divider is the divider of the 1 MHz clock that will be configured.
     *
     * @return 1
     *
     */

    gptimer_regs->timer[timer].reload = clock_divider; // base clock frequency is 1 MHz
    
    return 1;
}

int send_console_outputs_on_apbuart_port( void ) // Send the console outputs on the apbuart port
{
    struct apbuart_regs_str *apbuart_regs = (struct apbuart_regs_str *) REGS_ADDR_APBUART;

    apbuart_regs->ctrl = apbuart_regs->ctrl & APBUART_CTRL_REG_MASK_DB;
    PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

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

    status =  rtems_message_queue_ident( misc_name[QUEUE_SEND], 0, &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in HOUS *** ERR %d\n", status)
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
    housekeeping_packet.packetID[0] = (unsigned char) (TM_PACKET_ID_HK >> 8);
    housekeeping_packet.packetID[1] = (unsigned char) (TM_PACKET_ID_HK);
    housekeeping_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    housekeeping_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    housekeeping_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_HK >> 8);
    housekeeping_packet.packetLength[1] = (unsigned char) (PACKET_LENGTH_HK     );
    housekeeping_packet.spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
    housekeeping_packet.serviceType = TM_TYPE_HK;
    housekeeping_packet.serviceSubType = TM_SUBTYPE_HK;
    housekeeping_packet.destinationID = TM_DESTINATION_ID_GROUND;

    status = rtems_rate_monotonic_cancel(HK_id);
    if( status != RTEMS_SUCCESSFUL ) {
        PRINTF1( "ERR *** in HOUS *** rtems_rate_monotonic_cancel(HK_id) ***code: %d\n", status )
    }
    else {
        DEBUG_PRINTF("OK  *** in HOUS *** rtems_rate_monotonic_cancel(HK_id)\n")
    }

    while(1){ // launch the rate monotonic task
        status = rtems_rate_monotonic_period( HK_id, HK_PERIOD );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "in HOUS *** ERR period: %d\n", status);
        }
        else {
            housekeeping_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
            housekeeping_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
            housekeeping_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
            housekeeping_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
            housekeeping_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
            housekeeping_packet.time[5] = (unsigned char) (time_management_regs->fine_time);
            housekeeping_packet.sid = SID_HK;

            spacewire_update_statistics();

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

    BOOT_PRINTF("in DUMB *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3 | RTEMS_EVENT_4 | RTEMS_EVENT_5,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT
        intEventOut =  (unsigned int) event_out;
        for ( i=0; i<32; i++)
        {
            if ( ((intEventOut >> i) & 0x0001) != 0)
            {
                coarse_time = time_management_regs->coarse_time;
                fine_time = time_management_regs->fine_time;
                printf("in DUMB *** time = coarse: %x, fine: %x, %s\n", coarse_time, fine_time, DumbMessages[i]);
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
    char *parameters;

    parameters = (char*) &housekeeping_packet.lfr_status_word;
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

    updateLFRCurrentMode();
}


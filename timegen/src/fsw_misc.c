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
                }
                if (i==10)
                {
                }
            }
        }
    }
}

//*****************************
// init housekeeping parameters

void increment_seq_counter( unsigned short *packetSequenceControl )
{
    /** This function increment the sequence counter psased in argument.
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

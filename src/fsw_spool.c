/** Functions and tasks related to waveform packet generation.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle waveforms, in snapshot or continuous format.\n
 *
 */

#include "fsw_spool.h"

//*********************
// Interrupt SubRoutine

void spool_waveforms( void )
{
    /** This is the interrupt sub routine called by the waveform picker core.
     *
     * This ISR launch different actions depending mainly on two pieces of information:
     * 1. the values read in the registers of the waveform picker.
     * 2. the current LFR mode.
     *
     */

    rtems_status_code status;

    if ( (lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_BURST)  // in BURST the data are used to place v, e1 and e2 in the HK packet
         || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
    { // in modes other than STANDBY and BURST, send the CWF_F3 data
        if ((waveform_picker_regs->status & 0x08) == 0x08){     // [1000] f3 is full
            // (1) change the receiving buffer for the waveform picker
            ring_node_to_send_cwf_f3 = current_ring_node_f3;
            current_ring_node_f3 = current_ring_node_f3->next;
            waveform_picker_regs->addr_data_f3 = current_ring_node_f3->buffer_address;
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF3], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2);
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffff777; // reset f3 bits to 0, [1111 0111 0111 0111]
        }
    }

    switch(lfrCurrentMode)
    {
        //********
        // STANDBY
        case(LFR_MODE_STANDBY):
        break;

        //******
        // NORMAL
        case(LFR_MODE_NORMAL):
        if ( (waveform_picker_regs->status & 0xff8) != 0x00)    // [1000] check the error bits
        {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
        }
        if ( (waveform_picker_regs->status & 0x07) == 0x07)    // [0111] check the f2, f1, f0 full bits
        {
            // change F0 ring node
            ring_node_to_send_swf_f0 = current_ring_node_f0;
            current_ring_node_f0 = current_ring_node_f0->next;
            waveform_picker_regs->addr_data_f0 = current_ring_node_f0->buffer_address;
            // change F1 ring node
            ring_node_to_send_swf_f1 = current_ring_node_f1;
            current_ring_node_f1 = current_ring_node_f1->next;
            waveform_picker_regs->addr_data_f1 = current_ring_node_f1->buffer_address;
            // change F2 ring node
            ring_node_to_send_swf_f2 = current_ring_node_f2;
            current_ring_node_f2 = current_ring_node_f2->next;
            waveform_picker_regs->addr_data_f2 = current_ring_node_f2->buffer_address;
            //
            if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL)
            {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffff888; // [1000 1000 1000]
        }
        break;

        //******
        // BURST
        case(LFR_MODE_BURST):
        if ( (waveform_picker_regs->status & 0x04) == 0x04 ){ // [0100] check the f2 full bit
            // (1) change the receiving buffer for the waveform picker
            ring_node_to_send_cwf_f2 = current_ring_node_f2;
            current_ring_node_f2 = current_ring_node_f2->next;
            waveform_picker_regs->addr_data_f2 = current_ring_node_f2->buffer_address;
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1111 1011 1011 1011] f2 bit = 0
        }
        break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
        if ( (waveform_picker_regs->status & 0x02) == 0x02 ) { // [0010] check the f1 full bit
            // (1) change the receiving buffer for the waveform picker
            ring_node_to_send_cwf_f1 = current_ring_node_f1;
            current_ring_node_f1 = current_ring_node_f1->next;
            waveform_picker_regs->addr_data_f1 = current_ring_node_f1->buffer_address;
            // (2) send an event for the the CWF1 task for transmission (and snapshot extraction if needed)
            status = rtems_event_send( Task_id[TASKID_CWF1], RTEMS_EVENT_MODE_SBM1 );
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffddd; // [1111 1101 1101 1101] f1 bits = 0
        }
        if ( (waveform_picker_regs->status & 0x01) == 0x01 ) { // [0001] check the f0 full bit
            swf_f0_ready = true;
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffeee; // [1111 1110 1110 1110] f0 bits = 0
        }
        if ( (waveform_picker_regs->status & 0x04) == 0x04 ) { // [0100] check the f2 full bit
            swf_f2_ready = true;
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1111 1011 1011 1011] f2 bits = 0
        }
        break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
        if ( (waveform_picker_regs->status & 0x04) == 0x04 ){ // [0100] check the f2 full bit
            // (1) change the receiving buffer for the waveform picker
            ring_node_to_send_cwf_f2 = current_ring_node_f2;
            current_ring_node_f2 = current_ring_node_f2->next;
            waveform_picker_regs->addr_data_f2 = current_ring_node_f2->buffer_address;
            // (2) send an event for the waveforms transmission
            status = rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_SBM2 );
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1111 1011 1011 1011] f2 bit = 0
        }
        if ( (waveform_picker_regs->status & 0x01) == 0x01 ) { // [0001] check the f0 full bit
            swf_f0_ready = true;
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffeee; // [1111 1110 1110 1110] f0 bits = 0
        }
        if ( (waveform_picker_regs->status & 0x02) == 0x02 ) { // [0010] check the f1 full bit
            swf_f1_ready = true;
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffddd; // [1111 1101 1101 1101] f1, f0 bits = 0
        }
        break;

        //********
        // DEFAULT
        default:
        break;
    }
}

void spool_waveforms_alt( void )
{
    // WFRM
    if (wake_up_task_wfrm == true)
    {
        rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL );
        wake_up_task_wfrm = false;
    }
    // CWF_F1
    if (wake_up_task_cwf_f1 == true)
    {
        rtems_event_send( Task_id[TASKID_CWF1], RTEMS_EVENT_MODE_SBM1 );
        wake_up_task_cwf_f1 = false;
    }
    // CWF_F2 BURST
    if (wake_up_task_cwf_f2_burst == true)
    {
        rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST );
        wake_up_task_cwf_f2_burst = false;
    }
    // CWF_F2 SBM2
    if (wake_up_task_cwf_f2_sbm2 == true)
    {
        rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_SBM2 );
        wake_up_task_cwf_f2_sbm2 = false;
    }
    // CWF_F3
    if (wake_up_task_cwf_f3 == true)
    {
        rtems_event_send( Task_id[TASKID_CWF3], RTEMS_EVENT_0 );
        wake_up_task_cwf_f3 = false;
    }
}

void spool_spectral_matrices( void )
{
    // STATUS REGISTER
    // input_fifo_write(2) *** input_fifo_write(1) *** input_fifo_write(0)
    //           10                    9                       8
    // buffer_full ** bad_component_err ** f2_1 ** f2_0 ** f1_1 ** f1_0 ** f0_1 ** f0_0
    //      7                  6             5       4       3       2       1       0

    unsigned char status_f0;
    unsigned char status_f1;
    unsigned char status_f2;
    static unsigned int counter = 0;

    rtems_interrupt_level level;

    rtems_interrupt_disable( level );

    status_f0 = spectral_matrix_regs->status & 0x03;   // [0011] get the status_ready_matrix_f0_x bits
    status_f1 = (spectral_matrix_regs->status & 0x0c) >> 2;   // [0011] get the status_ready_matrix_f0_x bits
    status_f2 = (spectral_matrix_regs->status & 0x30) >> 4;   // [0011] get the status_ready_matrix_f0_x bits

//    if ( status_f0 == 0x03)
//    {
//        printf("%d \n", counter);
//    }
    if ( status_f1 == 0x03)
    {
        printf("f1 %d \n", counter);
    }
    if ( status_f2 == 0x03)
    {
        printf("f2 %d \n", counter);
    }

    spectral_matrices_isr_f0();

    spectral_matrices_isr_f1();

    spectral_matrices_isr_f2();

    spectral_matrix_isr_error_handler();

    rtems_interrupt_enable( level );

    counter = counter + 1;
}

//************
// RTEMS TASKS

rtems_task spoo_task(rtems_task_argument argument)
{
    rtems_status_code status;

    BOOT_PRINTF("in SPOOL ***\n")

    if (rtems_rate_monotonic_ident( name_spool_rate_monotonic, &spool_period_id) != RTEMS_SUCCESSFUL) {
        status = rtems_rate_monotonic_create( name_spool_rate_monotonic, &spool_period_id );
        if( status != RTEMS_SUCCESSFUL ) {
            PRINTF1( "in SPOO *** rtems_rate_monotonic_create failed with status %d\n", status )
        }
    }

    status = rtems_rate_monotonic_cancel( spool_period_id );
    if( status != RTEMS_SUCCESSFUL )
    {
        PRINTF1( "ERR *** in SPOOL *** rtems_rate_monotonic_cancel(spool_period_id) ***code: %d\n", status )
    }
    else
    {
        DEBUG_PRINTF("OK  *** in SPOOL *** rtems_rate_monotonic_cancel(spool_period_id)\n")
    }

    while(1){ // launch the rate monotonic task
//        status = rtems_rate_monotonic_period( spool_period_id, SPOOL_TIMEOUT_in_ticks );
        rtems_task_wake_after( SPOOL_TIMEOUT_in_ticks / 2 );
        spool_waveforms_alt();
        spool_spectral_matrices();
//        if ( status != RTEMS_SUCCESSFUL )
//        {
//            PRINTF1( "in SPOOL *** ERR period: %d\n", status);
//        }
//        else
//        {
//            spool_waveforms();
//            spool_spectral_matrices();
//        }
    }

    PRINTF("in SPOOL *** deleting task\n")

    status = rtems_task_delete( RTEMS_SELF ); // should not return
    printf( "rtems_task_delete returned with status of %d.\n", status );
    return;
}

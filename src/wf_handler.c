/** Functions and tasks related to waveform packet generation.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle waveforms, in snapshot or continuous format.\n
 *
 */

#include "wf_handler.h"

//*****************
// waveform headers
// SWF
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F0[7];
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F1[7];
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F2[7];
// CWF
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F1[       NB_PACKETS_PER_GROUP_OF_CWF         ];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F2_BURST[ NB_PACKETS_PER_GROUP_OF_CWF         ];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F2_SBM2[  NB_PACKETS_PER_GROUP_OF_CWF         ];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F3[       NB_PACKETS_PER_GROUP_OF_CWF         ];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F3_light[ NB_PACKETS_PER_GROUP_OF_CWF_LIGHT   ];

//***************
// waveform rings
// F0
ring_node waveform_ring_f0[NB_RING_NODES_F0];
ring_node *current_ring_node_f0;
ring_node *ring_node_to_send_swf_f0;
// F1
ring_node waveform_ring_f1[NB_RING_NODES_F1];
ring_node *current_ring_node_f1;
ring_node *ring_node_to_send_swf_f1;
ring_node *ring_node_to_send_cwf_f1;
// F2
ring_node waveform_ring_f2[NB_RING_NODES_F2];
ring_node *current_ring_node_f2;
ring_node *ring_node_to_send_swf_f2;
ring_node *ring_node_to_send_cwf_f2;
// F3
ring_node waveform_ring_f3[NB_RING_NODES_F3];
ring_node *current_ring_node_f3;
ring_node *ring_node_to_send_cwf_f3;

bool extractSWF = false;
bool swf_f0_ready = false;
bool swf_f1_ready = false;
bool swf_f2_ready = false;

int wf_snap_extracted[ (NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK) ];
ring_node ring_node_wf_snap_extracted;

//*********************
// Interrupt SubRoutine

void reset_extractSWF( void )
{
    extractSWF = false;
    swf_f0_ready = false;
    swf_f1_ready = false;
    swf_f2_ready = false;
}

void waveforms_isr_f3( void )
{
    rtems_status_code spare_status;

    if ( (lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_BURST)  // in BURST the data are used to place v, e1 and e2 in the HK packet
         || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
    { // in modes other than STANDBY and BURST, send the CWF_F3 data
        //***
        // F3
        if ( (waveform_picker_regs->status & 0xc0) != 0x00 ) {  // [1100 0000] check the f3 full bits
            ring_node_to_send_cwf_f3    = current_ring_node_f3->previous;
            current_ring_node_f3        = current_ring_node_f3->next;
            if ((waveform_picker_regs->status & 0x40) == 0x40){         // [0100 0000] f3 buffer 0 is full
                ring_node_to_send_cwf_f3->coarseTime    = waveform_picker_regs->f3_0_coarse_time;
                ring_node_to_send_cwf_f3->fineTime      = waveform_picker_regs->f3_0_fine_time;
                waveform_picker_regs->addr_data_f3_0    = current_ring_node_f3->buffer_address;
                waveform_picker_regs->status            = waveform_picker_regs->status & 0x00008840; // [1000 1000 0100 0000]
            }
            else if ((waveform_picker_regs->status & 0x80) == 0x80){     // [1000 0000] f3 buffer 1 is full
                ring_node_to_send_cwf_f3->coarseTime    = waveform_picker_regs->f3_1_coarse_time;
                ring_node_to_send_cwf_f3->fineTime      = waveform_picker_regs->f3_1_fine_time;
                waveform_picker_regs->addr_data_f3_1    = current_ring_node_f3->buffer_address;
                waveform_picker_regs->status            = waveform_picker_regs->status & 0x00008880; // [1000 1000 1000 0000]
            }
            if (rtems_event_send( Task_id[TASKID_CWF3], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
                spare_status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 );
            }
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2);
        }
    }
}

void waveforms_isr_normal( void )
{
    rtems_status_code status;

    if ( ( (waveform_picker_regs->status & 0x30) != 0x00 )      // [0011 0000] check the f2 full bits
         && ( (waveform_picker_regs->status & 0x0c) != 0x00 )   // [0000 1100] check the f1 full bits
         && ( (waveform_picker_regs->status & 0x03) != 0x00 ))  // [0000 0011] check the f0 full bits
    {
        //***
        // F0
        ring_node_to_send_swf_f0    = current_ring_node_f0->previous;
        current_ring_node_f0        = current_ring_node_f0->next;
        if ( (waveform_picker_regs->status & 0x01) == 0x01)
        {

            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_0_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_0_fine_time;
            waveform_picker_regs->addr_data_f0_0    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001101; // [0001 0001 0000 0001]
        }
        else if ( (waveform_picker_regs->status & 0x02) == 0x02)
        {
            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_1_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_1_fine_time;
            waveform_picker_regs->addr_data_f0_1    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001102; // [0001 0001 0000 0010]
        }

        //***
        // F1
        ring_node_to_send_swf_f1    = current_ring_node_f1->previous;
        current_ring_node_f1        = current_ring_node_f1->next;
        if ( (waveform_picker_regs->status & 0x04) == 0x04)
        {
            ring_node_to_send_swf_f1->coarseTime    = waveform_picker_regs->f1_0_coarse_time;
            ring_node_to_send_swf_f1->fineTime      = waveform_picker_regs->f1_0_fine_time;
            waveform_picker_regs->addr_data_f1_0    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002204; // [0010 0010 0000 0100] f1 bits = 0
        }
        else if ( (waveform_picker_regs->status & 0x08) == 0x08)
        {
            ring_node_to_send_swf_f1->coarseTime    = waveform_picker_regs->f1_1_coarse_time;
            ring_node_to_send_swf_f1->fineTime      = waveform_picker_regs->f1_1_fine_time;
            waveform_picker_regs->addr_data_f1_1    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002208; // [0010 0010 0000 1000] f1 bits = 0
        }

        //***
        // F2
        ring_node_to_send_swf_f2    = current_ring_node_f2->previous;
        current_ring_node_f2        = current_ring_node_f2->next;
        if ( (waveform_picker_regs->status & 0x10) == 0x10)
        {
            ring_node_to_send_swf_f2->coarseTime    = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_swf_f2->fineTime      = waveform_picker_regs->f2_0_fine_time;
            waveform_picker_regs->addr_data_f2_0    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004410; // [0100 0100 0001 0000]
        }
        else if ( (waveform_picker_regs->status & 0x20) == 0x20)
        {
            ring_node_to_send_swf_f2->coarseTime    = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_swf_f2->fineTime      = waveform_picker_regs->f2_1_fine_time;
            waveform_picker_regs->addr_data_f2_1    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004420; // [0100 0100 0010 0000]
        }
        //
        status = rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL );
        if ( status != RTEMS_SUCCESSFUL)
        {
            status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 );
        }
    }
}

void waveforms_isr_burst( void )
{
    rtems_status_code spare_status;

    if ( (waveform_picker_regs->status & 0x30) != 0 ){ // [0100] check the f2 full bit
        // (1) change the receiving buffer for the waveform picker
        ring_node_to_send_cwf_f2    = current_ring_node_f2->previous;
        current_ring_node_f2        = current_ring_node_f2->next;
        if ( (waveform_picker_regs->status & 0x10) == 0x10)
        {
            ring_node_to_send_cwf_f2->coarseTime    = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_cwf_f2->fineTime      = waveform_picker_regs->f2_0_fine_time;
            waveform_picker_regs->addr_data_f2_0    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004410; // [0100 0100 0001 0000]
        }
        else if ( (waveform_picker_regs->status & 0x20) == 0x20)
        {
            ring_node_to_send_cwf_f2->coarseTime    = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_cwf_f2->fineTime      = waveform_picker_regs->f2_1_fine_time;
            waveform_picker_regs->addr_data_f2_1    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004420; // [0100 0100 0010 0000]
        }
        // (2) send an event for the waveforms transmission
        if (rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST ) != RTEMS_SUCCESSFUL) {
            spare_status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 );
        }
    }
}

void waveforms_isr_sbm1( void )
{
    rtems_status_code status;
    rtems_status_code spare_status;

    //***
    // F1
    if ( (waveform_picker_regs->status & 0x0c) != 0x00 ) {  // [0000 1100] check the f1 full bits
        // (1) change the receiving buffer for the waveform picker
        ring_node_to_send_cwf_f1    = current_ring_node_f1->previous;
        current_ring_node_f1        = current_ring_node_f1->next;
        if ( (waveform_picker_regs->status & 0x04) == 0x04)
        {
            ring_node_to_send_cwf_f1->coarseTime    = waveform_picker_regs->f1_0_coarse_time;
            ring_node_to_send_cwf_f1->fineTime      = waveform_picker_regs->f1_0_fine_time;
            waveform_picker_regs->addr_data_f1_0    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002204; // [0010 0010 0000 0100] f1 bits = 0
        }
        else if ( (waveform_picker_regs->status & 0x08) == 0x08)
        {
            ring_node_to_send_cwf_f1->coarseTime    = waveform_picker_regs->f1_1_coarse_time;
            ring_node_to_send_cwf_f1->fineTime      = waveform_picker_regs->f1_1_fine_time;
            waveform_picker_regs->addr_data_f1_1    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002208; // [0010 0010 0000 1000] f1 bits = 0
        }
        // (2) send an event for the the CWF1 task for transmission (and snapshot extraction if needed)
        status = rtems_event_send( Task_id[TASKID_CWF1], RTEMS_EVENT_MODE_SBM1 );
    }

    //***
    // F0
    if ( (waveform_picker_regs->status & 0x03) != 0x00 ) {  // [0000 0011] check the f0 full bits
        swf_f0_ready = true;
        // change f0 buffer
        ring_node_to_send_swf_f0    = current_ring_node_f0->previous;
        current_ring_node_f0        = current_ring_node_f0->next;
        if ( (waveform_picker_regs->status & 0x01) == 0x01)
        {

            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_0_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_0_fine_time;
            waveform_picker_regs->addr_data_f0_0    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001101; // [0001 0001 0000 0001]
        }
        else if ( (waveform_picker_regs->status & 0x02) == 0x02)
        {
            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_1_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_1_fine_time;
            waveform_picker_regs->addr_data_f0_1    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001102; // [0001 0001 0000 0010]
        }
    }

    //***
    // F2
    if ( (waveform_picker_regs->status & 0x30) != 0x00 ) {  // [0011 0000] check the f2 full bits
        swf_f2_ready = true;
        // change f2 buffer
        ring_node_to_send_swf_f2    = current_ring_node_f2->previous;
        current_ring_node_f2        = current_ring_node_f2->next;
        if ( (waveform_picker_regs->status & 0x10) == 0x10)
        {
            ring_node_to_send_swf_f2->coarseTime    = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_swf_f2->fineTime      = waveform_picker_regs->f2_0_fine_time;
            waveform_picker_regs->addr_data_f2_0    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004410; // [0100 0100 0001 0000]
        }
        else if ( (waveform_picker_regs->status & 0x20) == 0x20)
        {
            ring_node_to_send_swf_f2->coarseTime    = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_swf_f2->fineTime      = waveform_picker_regs->f2_1_fine_time;
            waveform_picker_regs->addr_data_f2_1    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004420; // [0100 0100 0010 0000]
        }
        // start the snapshots transmission
        if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL)
        {
            spare_status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_0 );
        }
    }
}

void waveforms_isr_sbm2( void )
{
    rtems_status_code status;

    //***
    // F2
    if ( (waveform_picker_regs->status & 0x30) != 0x00 ) {  // [0011 0000] check the f2 full bit
        // (1) change the receiving buffer for the waveform picker
        ring_node_to_send_cwf_f2    = current_ring_node_f2->previous;
        current_ring_node_f2        = current_ring_node_f2->next;
        if ( (waveform_picker_regs->status & 0x10) == 0x10)
        {
            ring_node_to_send_cwf_f2->coarseTime    = waveform_picker_regs->f2_0_coarse_time;
            ring_node_to_send_cwf_f2->fineTime      = waveform_picker_regs->f2_0_fine_time;
            waveform_picker_regs->addr_data_f2_0    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004410; // [0100 0100 0001 0000]
        }
        else if ( (waveform_picker_regs->status & 0x20) == 0x20)
        {
            ring_node_to_send_cwf_f2->coarseTime    = waveform_picker_regs->f2_1_coarse_time;
            ring_node_to_send_cwf_f2->fineTime      = waveform_picker_regs->f2_1_fine_time;
            waveform_picker_regs->addr_data_f2_1    = current_ring_node_f2->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00004420; // [0100 0100 0010 0000]
        }
        // (2) send an event for the waveforms transmission
        status = rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_SBM2 );
    }

    //***
    // F0
    if ( (waveform_picker_regs->status & 0x03) != 0x00 ) {  // [0000 0011] check the f0 full bit
        swf_f0_ready = true;
        // change f0 buffer
        ring_node_to_send_swf_f0    = current_ring_node_f0->previous;
        current_ring_node_f0        = current_ring_node_f0->next;
        if ( (waveform_picker_regs->status & 0x01) == 0x01)
        {

            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_0_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_0_fine_time;
            waveform_picker_regs->addr_data_f0_0    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001101; // [0001 0001 0000 0001]
        }
        else if ( (waveform_picker_regs->status & 0x02) == 0x02)
        {
            ring_node_to_send_swf_f0->coarseTime    = waveform_picker_regs->f0_1_coarse_time;
            ring_node_to_send_swf_f0->fineTime      = waveform_picker_regs->f0_1_fine_time;
            waveform_picker_regs->addr_data_f0_1    = current_ring_node_f0->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00001102; // [0001 0001 0000 0010]
        }
    }

    //***
    // F1
    if ( (waveform_picker_regs->status & 0x0c) != 0x00 ) {  // [0000 1100] check the f1 full bit
        swf_f1_ready = true;
        ring_node_to_send_swf_f1    = current_ring_node_f1->previous;
        current_ring_node_f1        = current_ring_node_f1->next;
        if ( (waveform_picker_regs->status & 0x04) == 0x04)
        {
            ring_node_to_send_swf_f1->coarseTime    = waveform_picker_regs->f1_0_coarse_time;
            ring_node_to_send_swf_f1->fineTime      = waveform_picker_regs->f1_0_fine_time;
            waveform_picker_regs->addr_data_f1_0    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002204; // [0010 0010 0000 0100] f1 bits = 0
        }
        else if ( (waveform_picker_regs->status & 0x08) == 0x08)
        {
            ring_node_to_send_swf_f1->coarseTime    = waveform_picker_regs->f1_1_coarse_time;
            ring_node_to_send_swf_f1->fineTime      = waveform_picker_regs->f1_1_fine_time;
            waveform_picker_regs->addr_data_f1_1    = current_ring_node_f1->buffer_address;
            waveform_picker_regs->status            = waveform_picker_regs->status & 0x00002208; // [0010 0010 0000 1000] f1 bits = 0
        }
    }
}

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    /** This is the interrupt sub routine called by the waveform picker core.
     *
     * This ISR launch different actions depending mainly on two pieces of information:
     * 1. the values read in the registers of the waveform picker.
     * 2. the current LFR mode.
     *
     */

    // STATUS
    // new error        error buffer full
    // 15 14 13 12      11 10 9  8
    // f3 f2 f1 f0      f3 f2 f1 f0
    //
    // ready buffer
    // 7    6    5    4    3    2    1    0
    // f3_1 f3_0 f2_1 f2_0 f1_1 f1_0 f0_1 f0_0

    rtems_status_code spare_status;

    waveforms_isr_f3();

    if ( (waveform_picker_regs->status & 0xff00) != 0x00)    // [1111 1111 0000 0000] check the error bits
    {
        spare_status = rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_10 );
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
        waveforms_isr_normal();
        break;

        //******
        // BURST
        case(LFR_MODE_BURST):
        waveforms_isr_burst();
        break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
        waveforms_isr_sbm1();
        break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
        waveforms_isr_sbm2();
        break;

        //********
        // DEFAULT
        default:
        break;
    }
}

//************
// RTEMS TASKS

rtems_task wfrm_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    /** This RTEMS task is dedicated to the transmission of snapshots of the NORMAL mode.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packets are sent by this task:
     * - TM_LFR_SCIENCE_NORMAL_SWF_F0
     * - TM_LFR_SCIENCE_NORMAL_SWF_F1
     * - TM_LFR_SCIENCE_NORMAL_SWF_F2
     *
     */

    rtems_event_set event_out;
    rtems_id queue_id;
    rtems_status_code status;
    bool resynchronisationEngaged;

    resynchronisationEngaged = false;

    init_header_snapshot_wf_table( SID_NORM_SWF_F0, headerSWF_F0 );
    init_header_snapshot_wf_table( SID_NORM_SWF_F1, headerSWF_F1 );
    init_header_snapshot_wf_table( SID_NORM_SWF_F2, headerSWF_F2 );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in WFRM *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in WFRM ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_MODE_NORMAL | RTEMS_EVENT_MODE_SBM1
                            | RTEMS_EVENT_MODE_SBM2 | RTEMS_EVENT_MODE_SBM2_WFRM,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if(resynchronisationEngaged == false)
        {   // engage resynchronisation
            snapshot_resynchronization( (unsigned char *)  ring_node_to_send_swf_f0->buffer_address);
            resynchronisationEngaged = true;
        }
        else
        {   // reset delta_snapshot to the nominal value
            PRINTF("no resynchronisation, reset delta_snapshot to the nominal value\n")
            set_wfp_delta_snapshot();
            resynchronisationEngaged = false;
        }
        //

        if (event_out == RTEMS_EVENT_MODE_NORMAL)
        {
            DEBUG_PRINTF("WFRM received RTEMS_EVENT_MODE_NORMAL\n")
            send_waveform_SWF( ring_node_to_send_swf_f0, SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF( ring_node_to_send_swf_f1, SID_NORM_SWF_F1, headerSWF_F1, queue_id);
            send_waveform_SWF( ring_node_to_send_swf_f2, SID_NORM_SWF_F2, headerSWF_F2, queue_id);
        }
        if (event_out == RTEMS_EVENT_MODE_SBM1)
        {
            DEBUG_PRINTF("WFRM received RTEMS_EVENT_MODE_SBM1\n")
            send_waveform_SWF( ring_node_to_send_swf_f0,    SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF( &ring_node_wf_snap_extracted, SID_NORM_SWF_F1, headerSWF_F1, queue_id);
            send_waveform_SWF( ring_node_to_send_swf_f2,    SID_NORM_SWF_F2, headerSWF_F2, queue_id);
        }
        if (event_out == RTEMS_EVENT_MODE_SBM2)
        {
            DEBUG_PRINTF("WFRM received RTEMS_EVENT_MODE_SBM2\n")
            send_waveform_SWF( ring_node_to_send_swf_f0,    SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF( ring_node_to_send_swf_f1,    SID_NORM_SWF_F1, headerSWF_F1, queue_id);
            send_waveform_SWF( &ring_node_wf_snap_extracted, SID_NORM_SWF_F2, headerSWF_F2, queue_id);
        }
    }
}

rtems_task cwf3_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f3.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this task:
     * - TM_LFR_SCIENCE_NORMAL_CWF_F3
     *
     */

    rtems_event_set event_out;
    rtems_id queue_id;
    rtems_status_code status;

    init_header_continuous_wf_table( SID_NORM_CWF_LONG_F3, headerCWF_F3 );
    init_header_continuous_cwf3_light_table( headerCWF_F3_light );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in CWF3 *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in CWF3 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_0,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if ( (lfrCurrentMode == LFR_MODE_NORMAL)
             || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode==LFR_MODE_SBM2) )
        {
            if ( (parameter_dump_packet.sy_lfr_n_cwf_long_f3 & 0x01) == 0x01)
            {
                PRINTF("send CWF_LONG_F3\n")
                send_waveform_CWF( ring_node_to_send_cwf_f3,
                            SID_NORM_CWF_LONG_F3, headerCWF_F3, queue_id );
            }
            else
            {
                PRINTF("send CWF_F3 (light)\n")
                send_waveform_CWF3_light( ring_node_to_send_cwf_f3,
                            headerCWF_F3_light, queue_id );
            }

        }
        else
        {
            PRINTF1("in CWF3 *** lfrCurrentMode is %d, no data will be sent\n", lfrCurrentMode)
        }
    }
}

rtems_task cwf2_task(rtems_task_argument argument)  // ONLY USED IN BURST AND SBM2
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f2.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this function:
     * - TM_LFR_SCIENCE_BURST_CWF_F2
     * - TM_LFR_SCIENCE_SBM2_CWF_F2
     *
     */

    rtems_event_set event_out;
    rtems_id queue_id;
    rtems_status_code status;

    init_header_continuous_wf_table( SID_BURST_CWF_F2, headerCWF_F2_BURST );
    init_header_continuous_wf_table( SID_SBM2_CWF_F2, headerCWF_F2_SBM2 );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in CWF2 *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in CWF2 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_MODE_BURST | RTEMS_EVENT_MODE_SBM2,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if (event_out == RTEMS_EVENT_MODE_BURST)
        {
            send_waveform_CWF( ring_node_to_send_cwf_f2, SID_BURST_CWF_F2, headerCWF_F2_BURST, queue_id );
        }
        if (event_out == RTEMS_EVENT_MODE_SBM2)
        {
            send_waveform_CWF( ring_node_to_send_cwf_f2, SID_SBM2_CWF_F2, headerCWF_F2_SBM2, queue_id );
            // launch snapshot extraction if needed
            if (extractSWF == true)
            {
                ring_node_to_send_swf_f2 = ring_node_to_send_cwf_f2;
                // extract the snapshot
                build_snapshot_from_ring( ring_node_to_send_swf_f2, 2 );
                // send the snapshot when built
                status = rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2 );
                extractSWF = false;
            }
            if (swf_f0_ready && swf_f1_ready)
            {
                extractSWF = true;
                swf_f0_ready = false;
                swf_f1_ready = false;
            }
        }
    }
}

rtems_task cwf1_task(rtems_task_argument argument)  // ONLY USED IN SBM1
{
    /** This RTEMS task is dedicated to the transmission of continuous waveforms at f1.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The following data packet is sent by this function:
     * - TM_LFR_SCIENCE_SBM1_CWF_F1
     *
     */

    rtems_event_set event_out;
    rtems_id queue_id;
    rtems_status_code status;

    init_header_continuous_wf_table( SID_SBM1_CWF_F1, headerCWF_F1 );

    status =  get_message_queue_id_send( &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in CWF1 *** ERR get_message_queue_id_send %d\n", status)
    }

    BOOT_PRINTF("in CWF1 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_MODE_SBM1,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        send_waveform_CWF( ring_node_to_send_cwf_f1, SID_SBM1_CWF_F1, headerCWF_F1, queue_id );
        // launch snapshot extraction if needed
        if (extractSWF == true)
        {
            ring_node_to_send_swf_f1 = ring_node_to_send_cwf_f1;
            // launch the snapshot extraction
            status = rtems_event_send( Task_id[TASKID_SWBD], RTEMS_EVENT_MODE_SBM1 );
            extractSWF = false;
        }
        if (swf_f0_ready == true)
        {
            extractSWF = true;
            swf_f0_ready = false;   // this step shall be executed only one time
        }
        if ((swf_f1_ready == true) && (swf_f2_ready == true))   // swf_f1 is ready after the extraction
        {
            status = rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM1 );
            swf_f1_ready = false;
            swf_f2_ready = false;
        }
    }
}

rtems_task swbd_task(rtems_task_argument argument)
{
    /** This RTEMS task is dedicated to the building of snapshots from different continuous waveforms buffers.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     */

    rtems_event_set event_out;

    BOOT_PRINTF("in SWBD ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_MODE_SBM1 | RTEMS_EVENT_MODE_SBM2,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if (event_out == RTEMS_EVENT_MODE_SBM1)
        {
            build_snapshot_from_ring( ring_node_to_send_swf_f1, 1 );
            swf_f1_ready = true;    // the snapshot has been extracted and is ready to be sent
        }
        else
        {
            PRINTF1("in SWBD *** unexpected rtems event received %x\n", (int) event_out)
        }
    }
}

//******************
// general functions

void WFP_init_rings( void )
{
    // F0 RING
    init_waveform_ring( waveform_ring_f0, NB_RING_NODES_F0, wf_snap_f0 );
    // F1 RING
    init_waveform_ring( waveform_ring_f1, NB_RING_NODES_F1, wf_snap_f1 );
    // F2 RING
    init_waveform_ring( waveform_ring_f2, NB_RING_NODES_F2, wf_snap_f2 );
    // F3 RING
    init_waveform_ring( waveform_ring_f3, NB_RING_NODES_F3, wf_cont_f3 );

    ring_node_wf_snap_extracted.buffer_address = (int) wf_snap_extracted;

    DEBUG_PRINTF1("waveform_ring_f0 @%x\n", (unsigned int) waveform_ring_f0)
    DEBUG_PRINTF1("waveform_ring_f1 @%x\n", (unsigned int) waveform_ring_f1)
    DEBUG_PRINTF1("waveform_ring_f2 @%x\n", (unsigned int) waveform_ring_f2)
    DEBUG_PRINTF1("waveform_ring_f3 @%x\n", (unsigned int) waveform_ring_f3)
}

void init_waveform_ring(ring_node waveform_ring[], unsigned char nbNodes, volatile int wfrm[] )
{
    unsigned char i;

    waveform_ring[0].next            = (ring_node*) &waveform_ring[ 1 ];
    waveform_ring[0].previous        = (ring_node*) &waveform_ring[ nbNodes - 1 ];
    waveform_ring[0].buffer_address  = (int) &wfrm[0];

    waveform_ring[nbNodes-1].next           = (ring_node*) &waveform_ring[ 0 ];
    waveform_ring[nbNodes-1].previous       = (ring_node*) &waveform_ring[ nbNodes - 2 ];
    waveform_ring[nbNodes-1].buffer_address = (int) &wfrm[ (nbNodes-1) * WFRM_BUFFER ];

    for(i=1; i<nbNodes-1; i++)
    {
        waveform_ring[i].next            = (ring_node*) &waveform_ring[ i + 1 ];
        waveform_ring[i].previous        = (ring_node*) &waveform_ring[ i - 1 ];
        waveform_ring[i].buffer_address  = (int) &wfrm[ i * WFRM_BUFFER ];
    }
}

void WFP_reset_current_ring_nodes( void )
{
    current_ring_node_f0      = waveform_ring_f0;
    ring_node_to_send_swf_f0  = waveform_ring_f0;

    current_ring_node_f1      = waveform_ring_f1;
    ring_node_to_send_cwf_f1  = waveform_ring_f1;
    ring_node_to_send_swf_f1  = waveform_ring_f1;

    current_ring_node_f2      = waveform_ring_f2;
    ring_node_to_send_cwf_f2  = waveform_ring_f2;
    ring_node_to_send_swf_f2  = waveform_ring_f2;

    current_ring_node_f3      = waveform_ring_f3;
    ring_node_to_send_cwf_f3  = waveform_ring_f3;
}

int init_header_snapshot_wf_table( unsigned int sid, Header_TM_LFR_SCIENCE_SWF_t *headerSWF)
{
    unsigned char i;
    int return_value;

    return_value = LFR_SUCCESSFUL;

    for (i=0; i<7; i++)
    {
        headerSWF[ i ].targetLogicalAddress = CCSDS_DESTINATION_ID;
        headerSWF[ i ].protocolIdentifier = CCSDS_PROTOCOLE_ID;
        headerSWF[ i ].reserved = DEFAULT_RESERVED;
        headerSWF[ i ].userApplication = CCSDS_USER_APP;
        headerSWF[ i ].packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
        headerSWF[ i ].packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
        headerSWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
        if (i == 6)
        {
            headerSWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_224 >> 8);
            headerSWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_224     );
            headerSWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_224 >> 8);
            headerSWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_224     );
        }
        else
        {
            headerSWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_304 >> 8);
            headerSWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_304     );
            headerSWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_304 >> 8);
            headerSWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_304     );
        }
        headerSWF[ i ].packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
        headerSWF[ i ].pktCnt = DEFAULT_PKTCNT;  // PKT_CNT
        headerSWF[ i ].pktNr = i+1;    // PKT_NR
        // DATA FIELD HEADER
        headerSWF[ i ].spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
        headerSWF[ i ].serviceType = TM_TYPE_LFR_SCIENCE; // service type
        headerSWF[ i ].serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
        headerSWF[ i ].destinationID = TM_DESTINATION_ID_GROUND;
        // AUXILIARY DATA HEADER
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].sid = sid;
        headerSWF[ i ].hkBIA = DEFAULT_HKBIA;
    }

    return return_value;
}

int init_header_continuous_wf_table( unsigned int sid, Header_TM_LFR_SCIENCE_CWF_t *headerCWF )
{
    unsigned int i;
    int return_value;

    return_value = LFR_SUCCESSFUL;

    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF; i++)
    {
        headerCWF[ i ].targetLogicalAddress = CCSDS_DESTINATION_ID;
        headerCWF[ i ].protocolIdentifier = CCSDS_PROTOCOLE_ID;
        headerCWF[ i ].reserved = DEFAULT_RESERVED;
        headerCWF[ i ].userApplication = CCSDS_USER_APP;
        if ( (sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2) )
        {
            headerCWF[ i ].packetID[0] = (unsigned char) (APID_TM_SCIENCE_SBM1_SBM2 >> 8);
            headerCWF[ i ].packetID[1] = (unsigned char) (APID_TM_SCIENCE_SBM1_SBM2);
        }
        else
        {
            headerCWF[ i ].packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
            headerCWF[ i ].packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);
        }
        headerCWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
        headerCWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_336 >> 8);
        headerCWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_336     );
        headerCWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_CWF >> 8);
        headerCWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_CWF     );
        headerCWF[ i ].packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
        // DATA FIELD HEADER
        headerCWF[ i ].spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
        headerCWF[ i ].serviceType = TM_TYPE_LFR_SCIENCE; // service type
        headerCWF[ i ].serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
        headerCWF[ i ].destinationID = TM_DESTINATION_ID_GROUND;
        // AUXILIARY DATA HEADER
        headerCWF[ i ].sid = sid;
        headerCWF[ i ].hkBIA = DEFAULT_HKBIA;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
    }

    return return_value;
}

int init_header_continuous_cwf3_light_table( Header_TM_LFR_SCIENCE_CWF_t *headerCWF )
{
    unsigned int i;
    int return_value;

    return_value = LFR_SUCCESSFUL;

    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF_LIGHT; i++)
    {
        headerCWF[ i ].targetLogicalAddress = CCSDS_DESTINATION_ID;
        headerCWF[ i ].protocolIdentifier = CCSDS_PROTOCOLE_ID;
        headerCWF[ i ].reserved = DEFAULT_RESERVED;
        headerCWF[ i ].userApplication = CCSDS_USER_APP;

        headerCWF[ i ].packetID[0] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST >> 8);
        headerCWF[ i ].packetID[1] = (unsigned char) (APID_TM_SCIENCE_NORMAL_BURST);

        headerCWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
        headerCWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_672 >> 8);
        headerCWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_672     );
        headerCWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_CWF_SHORT_F3 >> 8);
        headerCWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_CWF_SHORT_F3     );

        headerCWF[ i ].packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
        // DATA FIELD HEADER
        headerCWF[ i ].spare1_pusVersion_spare2 = DEFAULT_SPARE1_PUSVERSION_SPARE2;
        headerCWF[ i ].serviceType = TM_TYPE_LFR_SCIENCE; // service type
        headerCWF[ i ].serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
        headerCWF[ i ].destinationID = TM_DESTINATION_ID_GROUND;
        // AUXILIARY DATA HEADER
        headerCWF[ i ].sid = SID_NORM_CWF_F3;
        headerCWF[ i ].hkBIA = DEFAULT_HKBIA;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
        headerCWF[ i ].time[0] = 0x00;
    }

    return return_value;
}

int send_waveform_SWF( ring_node *ring_node_to_send, unsigned int sid,
                       Header_TM_LFR_SCIENCE_SWF_t *headerSWF, rtems_id queue_id )
{
    /** This function sends SWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerSWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_SWF;
    int *dataPtr;

    spw_ioctl_send_SWF.hlen = TM_HEADER_LEN + 4 + 12; // + 4 is for the protocole extra header, + 12 is for the auxiliary header
    spw_ioctl_send_SWF.options = 0;

    ret = LFR_DEFAULT;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (int*) ring_node_to_send->buffer_address;

    for (i=0; i<7; i++) // send waveform
    {
        spw_ioctl_send_SWF.data = (char*) &dataPtr[ (i * BLK_NR_304 * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_SWF.hdr = (char*) &headerSWF[ i ];
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send_SWF.dlen = BLK_NR_224 * NB_BYTES_SWF_BLK;
        }
        else {
            spw_ioctl_send_SWF.dlen = BLK_NR_304 * NB_BYTES_SWF_BLK;
        }
        // SET PACKET SEQUENCE COUNTER
        increment_seq_counter_source_id( headerSWF[ i ].packetSequenceControl, sid );
        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, sid, i, headerSWF[ i ].acquisitionTime );
        //
        headerSWF[ i ].time[0] = headerSWF[ i ].acquisitionTime[0];
        headerSWF[ i ].time[1] = headerSWF[ i ].acquisitionTime[1];
        headerSWF[ i ].time[2] = headerSWF[ i ].acquisitionTime[2];
        headerSWF[ i ].time[3] = headerSWF[ i ].acquisitionTime[3];
        headerSWF[ i ].time[4] = headerSWF[ i ].acquisitionTime[4];
        headerSWF[ i ].time[5] = headerSWF[ i ].acquisitionTime[5];
        // SEND PACKET
        status =  rtems_message_queue_send( queue_id, &spw_ioctl_send_SWF, ACTION_MSG_SPW_IOCTL_SEND_SIZE);
        if (status != RTEMS_SUCCESSFUL) {
            printf("%d-%d, ERR %d\n", sid, i, (int) status);
            ret = LFR_DEFAULT;
        }
        rtems_task_wake_after(TIME_BETWEEN_TWO_SWF_PACKETS);  // 300 ms between each packet => 7 * 3 = 21 packets => 6.3 seconds
    }

    return ret;
}

int send_waveform_CWF(ring_node *ring_node_to_send, unsigned int sid,
                      Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id)
{
    /** This function sends CWF CCSDS packets (F2, F1 or F0).
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param sid is the source identifier of the data that will be sent.
     * @param headerCWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * One group of 2048 samples is sent as 7 consecutive packets, 6 packets containing 340 blocks and 8 packets containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    int *dataPtr;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (int*) ring_node_to_send->buffer_address;

    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF; i++) // send waveform
    {
        spw_ioctl_send_CWF.data = (char*) &dataPtr[ (i * BLK_NR_CWF * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_CWF.hdr = (char*) &headerCWF[ i ];
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF * NB_BYTES_SWF_BLK;
        // SET PACKET SEQUENCE COUNTER
        increment_seq_counter_source_id( headerCWF[ i ].packetSequenceControl, sid );
        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, sid, i, headerCWF[ i ].acquisitionTime);
        //
        headerCWF[ i ].time[0] = headerCWF[ i ].acquisitionTime[0];
        headerCWF[ i ].time[1] = headerCWF[ i ].acquisitionTime[1];
        headerCWF[ i ].time[2] = headerCWF[ i ].acquisitionTime[2];
        headerCWF[ i ].time[3] = headerCWF[ i ].acquisitionTime[3];
        headerCWF[ i ].time[4] = headerCWF[ i ].acquisitionTime[4];
        headerCWF[ i ].time[5] = headerCWF[ i ].acquisitionTime[5];
        // SEND PACKET
        if (sid == SID_NORM_CWF_LONG_F3)
        {
            status =  rtems_message_queue_send( queue_id, &spw_ioctl_send_CWF, sizeof(spw_ioctl_send_CWF));
            if (status != RTEMS_SUCCESSFUL) {
                printf("%d-%d, ERR %d\n", sid, i, (int) status);
                ret = LFR_DEFAULT;
            }
            rtems_task_wake_after(TIME_BETWEEN_TWO_CWF3_PACKETS);
        }
        else
        {
            status =  rtems_message_queue_send( queue_id, &spw_ioctl_send_CWF, sizeof(spw_ioctl_send_CWF));
            if (status != RTEMS_SUCCESSFUL) {
                printf("%d-%d, ERR %d\n", sid, i, (int) status);
                ret = LFR_DEFAULT;
            }
        }
    }

    return ret;
}

int send_waveform_CWF3_light( ring_node *ring_node_to_send, Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id )
{
    /** This function sends CWF_F3 CCSDS packets without the b1, b2 and b3 data.
     *
     * @param waveform points to the buffer containing the data that will be send.
     * @param headerCWF points to a table of headers that have been prepared for the data transmission.
     * @param queue_id is the id of the rtems queue to which spw_ioctl_pkt_send structures will be send. The structures
     * contain information to setup the transmission of the data packets.
     *
     * By default, CWF_F3 packet are send without the b1, b2 and b3 data. This function rebuilds a data buffer
     * from the incoming data and sends it in 7 packets, 6 containing 340 blocks and 1 one containing 8 blocks.
     *
     */

    unsigned int i;
    int ret;
    unsigned int coarseTime;
    unsigned int fineTime;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    char *sample;
    int *dataPtr;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;

    coarseTime  = ring_node_to_send->coarseTime;
    fineTime    = ring_node_to_send->fineTime;
    dataPtr     = (int*) ring_node_to_send->buffer_address;

    //**********************
    // BUILD CWF3_light DATA
    for ( i=0; i< NB_SAMPLES_PER_SNAPSHOT; i++)
    {
        sample = (char*) &dataPtr[ (i * NB_WORDS_SWF_BLK) ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK)     ] = sample[ 0 ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK) + 1 ] = sample[ 1 ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK) + 2 ] = sample[ 2 ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK) + 3 ] = sample[ 3 ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK) + 4 ] = sample[ 4 ];
        wf_cont_f3_light[ (i * NB_BYTES_CWF3_LIGHT_BLK) + 5 ] = sample[ 5 ];
    }

    //*********************
    // SEND CWF3_light DATA
    for (i=0; i<NB_PACKETS_PER_GROUP_OF_CWF_LIGHT; i++) // send waveform
    {
        spw_ioctl_send_CWF.data = (char*) &wf_cont_f3_light[ (i * BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK) ];
        spw_ioctl_send_CWF.hdr = (char*) &headerCWF[ i ];
        // BUILD THE DATA
        spw_ioctl_send_CWF.dlen = BLK_NR_CWF_SHORT_F3 * NB_BYTES_CWF3_LIGHT_BLK;
        // SET PACKET SEQUENCE COUNTER
        increment_seq_counter_source_id( headerCWF[ i ].packetSequenceControl, SID_NORM_CWF_F3 );
        // SET PACKET TIME
        compute_acquisition_time( coarseTime, fineTime, SID_NORM_CWF_F3, i, headerCWF[ i ].acquisitionTime );
        //
        headerCWF[ i ].time[0] = headerCWF[ i ].acquisitionTime[0];
        headerCWF[ i ].time[1] = headerCWF[ i ].acquisitionTime[1];
        headerCWF[ i ].time[2] = headerCWF[ i ].acquisitionTime[2];
        headerCWF[ i ].time[3] = headerCWF[ i ].acquisitionTime[3];
        headerCWF[ i ].time[4] = headerCWF[ i ].acquisitionTime[4];
        headerCWF[ i ].time[5] = headerCWF[ i ].acquisitionTime[5];
        // SEND PACKET
        status =  rtems_message_queue_send( queue_id, &spw_ioctl_send_CWF, sizeof(spw_ioctl_send_CWF));
        if (status != RTEMS_SUCCESSFUL) {
            printf("%d-%d, ERR %d\n", SID_NORM_CWF_F3, i, (int) status);
            ret = LFR_DEFAULT;
        }
        rtems_task_wake_after(TIME_BETWEEN_TWO_CWF3_PACKETS);
    }

    return ret;
}

int send_ring_node_CWF( ring_node *ring_node_to_send )
{
    int status;

    status = LFR_SUCCESSFUL;
//    status =  rtems_message_queue_send( queue_id, ring_node_to_send, 4 );

    return status;
}

void compute_acquisition_time( unsigned int coarseTime, unsigned int fineTime,
                               unsigned int sid, unsigned char pa_lfr_pkt_nr, unsigned char * acquisitionTime )
{
    unsigned long long int acquisitionTimeAsLong;
    unsigned char localAcquisitionTime[6];
    double deltaT;

    deltaT = 0.;

    localAcquisitionTime[0] = (unsigned char) ( coarseTime >> 24 );
    localAcquisitionTime[1] = (unsigned char) ( coarseTime >> 16 );
    localAcquisitionTime[2] = (unsigned char) ( coarseTime >> 8  );
    localAcquisitionTime[3] = (unsigned char) ( coarseTime       );
    localAcquisitionTime[4] = (unsigned char) ( fineTime   >> 8  );
    localAcquisitionTime[5] = (unsigned char) ( fineTime         );

    acquisitionTimeAsLong = ( (unsigned long long int) localAcquisitionTime[0] << 40 )
            + ( (unsigned long long int) localAcquisitionTime[1] << 32 )
            + ( (unsigned long long int) localAcquisitionTime[2] << 24 )
            + ( (unsigned long long int) localAcquisitionTime[3] << 16 )
            + ( (unsigned long long int) localAcquisitionTime[4] << 8  )
            + ( (unsigned long long int) localAcquisitionTime[5]       );

    switch( sid )
    {
    case SID_NORM_SWF_F0:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_304 * 65536. / 24576. ;
        break;

    case SID_NORM_SWF_F1:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_304 * 65536. / 4096. ;
        break;

    case SID_NORM_SWF_F2:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_304 * 65536. / 256. ;
        break;

    case SID_SBM1_CWF_F1:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_CWF * 65536. / 4096. ;
        break;

    case SID_SBM2_CWF_F2:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_CWF * 65536. / 256. ;
        break;

    case SID_BURST_CWF_F2:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_CWF * 65536. / 256. ;
        break;

    case SID_NORM_CWF_F3:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_CWF_SHORT_F3 * 65536. / 16. ;
        break;

    case SID_NORM_CWF_LONG_F3:
        deltaT = ( (double ) (pa_lfr_pkt_nr) ) * BLK_NR_CWF * 65536. / 16. ;
        break;

    default:
        PRINTF1("in compute_acquisition_time *** ERR unexpected sid %d", sid)
        deltaT = 0.;
        break;
    }

    acquisitionTimeAsLong = acquisitionTimeAsLong + (unsigned long long int) deltaT;
    //
    acquisitionTime[0] = (unsigned char) (acquisitionTimeAsLong >> 40);
    acquisitionTime[1] = (unsigned char) (acquisitionTimeAsLong >> 32);
    acquisitionTime[2] = (unsigned char) (acquisitionTimeAsLong >> 24);
    acquisitionTime[3] = (unsigned char) (acquisitionTimeAsLong >> 16);
    acquisitionTime[4] = (unsigned char) (acquisitionTimeAsLong >> 8 );
    acquisitionTime[5] = (unsigned char) (acquisitionTimeAsLong      );

}

void build_snapshot_from_ring( ring_node *ring_node_to_send, unsigned char frequencyChannel )
{
    unsigned int i;
    unsigned long long int centerTime_asLong;
    unsigned long long int acquisitionTimeF0_asLong;
    unsigned long long int acquisitionTime_asLong;
    unsigned long long int bufferAcquisitionTime_asLong;
    unsigned char *ptr1;
    unsigned char *ptr2;
    unsigned char *timeCharPtr;
    unsigned char nb_ring_nodes;
    unsigned long long int frequency_asLong;
    unsigned long long int nbTicksPerSample_asLong;
    unsigned long long int nbSamplesPart1_asLong;
    unsigned long long int sampleOffset_asLong;

    unsigned int deltaT_F0;
    unsigned int deltaT_F1;
    unsigned long long int deltaT_F2;

    deltaT_F0 = 2731;      // (2048. / 24576. / 2.) * 65536. = 2730.667;
    deltaT_F1 = 16384;  // (2048. / 4096.  / 2.) * 65536. = 16384;
    deltaT_F2 = 262144; // (2048. / 256.   / 2.) * 65536. = 262144;
    sampleOffset_asLong = 0x00;

    // (1) get the f0 acquisition time
    acquisitionTimeF0_asLong = get_acquisition_time( (unsigned char *) &ring_node_to_send->coarseTime );

    // (2) compute the central reference time
    centerTime_asLong = acquisitionTimeF0_asLong + deltaT_F0;

    // (3) compute the acquisition time of the current snapshot
    switch(frequencyChannel)
    {
    case 1: // 1 is for F1 = 4096 Hz
        acquisitionTime_asLong = centerTime_asLong - deltaT_F1;
        nb_ring_nodes = NB_RING_NODES_F1;
        frequency_asLong = 4096;
        nbTicksPerSample_asLong = 16;  // 65536 / 4096;
        break;
    case 2: // 2 is for F2 = 256 Hz
        acquisitionTime_asLong = centerTime_asLong - deltaT_F2;
        nb_ring_nodes = NB_RING_NODES_F2;
        frequency_asLong = 256;
        nbTicksPerSample_asLong = 256;  // 65536 / 256;
        break;
    default:
        acquisitionTime_asLong = centerTime_asLong;
        frequency_asLong = 256;
        nbTicksPerSample_asLong = 256;
        break;
    }

    //****************************************************************************
    // (4) search the ring_node with the acquisition time <= acquisitionTime_asLong
    for (i=0; i<nb_ring_nodes; i++)
    {
        PRINTF1("%d ... ", i)
        bufferAcquisitionTime_asLong = get_acquisition_time( (unsigned char *) ring_node_to_send->coarseTime );
        if (bufferAcquisitionTime_asLong <= acquisitionTime_asLong)
        {
            PRINTF1("buffer found with acquisition time = %llx\n", bufferAcquisitionTime_asLong)
            break;
        }
        ring_node_to_send = ring_node_to_send->previous;
    }

    // (5) compute the number of samples to take in the current buffer
    sampleOffset_asLong = ((acquisitionTime_asLong - bufferAcquisitionTime_asLong) * frequency_asLong ) >> 16;
    nbSamplesPart1_asLong = NB_SAMPLES_PER_SNAPSHOT - sampleOffset_asLong;
    PRINTF2("sampleOffset_asLong = %llx, nbSamplesPart1_asLong = %llx\n", sampleOffset_asLong, nbSamplesPart1_asLong)

    // (6) compute the final acquisition time
    acquisitionTime_asLong = bufferAcquisitionTime_asLong +
            sampleOffset_asLong * nbTicksPerSample_asLong;

    // (7) copy the acquisition time at the beginning of the extrated snapshot
    ptr1 = (unsigned char*) &acquisitionTime_asLong;
    // fine time
    ptr2 = (unsigned char*) &ring_node_wf_snap_extracted.fineTime;
    ptr2[2] = ptr1[ 4 + 2 ];
    ptr2[3] = ptr1[ 5 + 2 ];
    // coarse time
    ptr2 = (unsigned char*) &ring_node_wf_snap_extracted.coarseTime;
    ptr2[0] = ptr1[ 0 + 2 ];
    ptr2[1] = ptr1[ 1 + 2 ];
    ptr2[2] = ptr1[ 2 + 2 ];
    ptr2[3] = ptr1[ 3 + 2 ];

    // re set the synchronization bit
    timeCharPtr = (unsigned char*) &ring_node_to_send->coarseTime;
    ptr2[0] = ptr2[0] | (timeCharPtr[0] & 0x80); // [1000 0000]

    if ( (nbSamplesPart1_asLong >= NB_SAMPLES_PER_SNAPSHOT) | (nbSamplesPart1_asLong < 0) )
    {
        nbSamplesPart1_asLong = 0;
    }
    // copy the part 1 of the snapshot in the extracted buffer
    for ( i = 0; i < (nbSamplesPart1_asLong * NB_WORDS_SWF_BLK); i++ )
    {
        wf_snap_extracted[i] =
                ((int*) ring_node_to_send->buffer_address)[ i + (sampleOffset_asLong * NB_WORDS_SWF_BLK) ];
    }
    // copy the part 2 of the snapshot in the extracted buffer
    ring_node_to_send = ring_node_to_send->next;
    for ( i = (nbSamplesPart1_asLong * NB_WORDS_SWF_BLK); i < (NB_SAMPLES_PER_SNAPSHOT * NB_WORDS_SWF_BLK); i++ )
    {
        wf_snap_extracted[i] =
                ((int*) ring_node_to_send->buffer_address)[ (i-(nbSamplesPart1_asLong * NB_WORDS_SWF_BLK)) ];
    }
}

void snapshot_resynchronization( unsigned char *timePtr )
{
    unsigned long long int acquisitionTime;
    unsigned long long int centerTime;
    unsigned long long int previousTick;
    unsigned long long int nextTick;
    unsigned long long int deltaPreviousTick;
    unsigned long long int deltaNextTick;
    unsigned int deltaTickInF2;
    double deltaPrevious;
    double deltaNext;

    acquisitionTime = get_acquisition_time( timePtr );

    // compute center time
    centerTime = acquisitionTime + 2731;    // (2048. / 24576. / 2.) * 65536. = 2730.667;
    previousTick = centerTime - (centerTime & 0xffff);
    nextTick = previousTick + 65536;

    deltaPreviousTick   = centerTime - previousTick;
    deltaNextTick       = nextTick - centerTime;

    deltaPrevious       = ((double) deltaPreviousTick) / 65536. * 1000.;
    deltaNext           = ((double) deltaNextTick) / 65536. * 1000.;

    printf("delta previous = %f ms, delta next = %f ms\n", deltaPrevious, deltaNext);
    printf("delta previous = %llu, delta next = %llu\n", deltaPreviousTick, deltaNextTick);

    // which tick is the closest
    if (deltaPreviousTick > deltaNextTick)
    {
        deltaTickInF2 = floor( (deltaNext * 256. / 1000.) ); // the division by 2 is important here
        waveform_picker_regs->delta_snapshot = waveform_picker_regs->delta_snapshot + deltaTickInF2;
        printf("correction of = + %u\n", deltaTickInF2);
    }
    else
    {
        deltaTickInF2 = floor( (deltaPrevious * 256. / 1000.) ); // the division by 2 is important here
        waveform_picker_regs->delta_snapshot = waveform_picker_regs->delta_snapshot - deltaTickInF2;
        printf("correction of = - %u\n", deltaTickInF2);
    }
}

//**************
// wfp registers
void reset_wfp_burst_enable( void )
{
    /** This function resets the waveform picker burst_enable register.
     *
     * The burst bits [f2 f1 f0] and the enable bits [f3 f2 f1 f0] are set to 0.
     *
     */

    // [1000 000] burst f2, f1, f0     enable f3, f2, f1, f0
    waveform_picker_regs->run_burst_enable = waveform_picker_regs->run_burst_enable & 0x80;
}

void reset_wfp_status( void )
{
    /** This function resets the waveform picker status register.
     *
     * All status bits are set to 0 [new_err full_err full].
     *
     */

    waveform_picker_regs->status = 0xffff;
}

void reset_wfp_buffer_addresses( void )
{
    // F0
    waveform_picker_regs->addr_data_f0_0 = current_ring_node_f0->buffer_address; // 0x08
    current_ring_node_f0 = current_ring_node_f0->next;
    waveform_picker_regs->addr_data_f0_1 = current_ring_node_f0->buffer_address; // 0x0c
    // F1
    waveform_picker_regs->addr_data_f1_0 = current_ring_node_f1->buffer_address; // 0x10
    current_ring_node_f1 = current_ring_node_f1->next;
    waveform_picker_regs->addr_data_f1_1 = current_ring_node_f1->buffer_address; // 0x14
    // F2
    waveform_picker_regs->addr_data_f2_0 = current_ring_node_f2->buffer_address; // 0x18
    current_ring_node_f2 = current_ring_node_f2->next;
    waveform_picker_regs->addr_data_f2_1 = current_ring_node_f2->buffer_address; // 0x1c
    // F3
    waveform_picker_regs->addr_data_f3_0 = current_ring_node_f3->buffer_address; // 0x20
    current_ring_node_f3 = current_ring_node_f3->next;
    waveform_picker_regs->addr_data_f3_1 = current_ring_node_f3->buffer_address; // 0x24
}

void reset_waveform_picker_regs( void )
{
    /** This function resets the waveform picker module registers.
    *
    * The registers affected by this function are located at the following offset addresses:
    * - 0x00 data_shaping
    * - 0x04 run_burst_enable
    * - 0x08 addr_data_f0
    * - 0x0C addr_data_f1
    * - 0x10 addr_data_f2
    * - 0x14 addr_data_f3
    * - 0x18 status
    * - 0x1C delta_snapshot
    * - 0x20 delta_f0
    * - 0x24 delta_f0_2
    * - 0x28 delta_f1
    * - 0x2c delta_f2
    * - 0x30 nb_data_by_buffer
    * - 0x34 nb_snapshot_param
    * - 0x38 start_date
    * - 0x3c nb_word_in_buffer
    *
    */

    set_wfp_data_shaping();                                     // 0x00 *** R1 R0 SP1 SP0 BW

    reset_wfp_burst_enable();                                   // 0x04 *** [run *** burst f2, f1, f0 *** enable f3, f2, f1, f0 ]

    reset_wfp_buffer_addresses();

    reset_wfp_status();                                         // 0x18

    set_wfp_delta_snapshot();   // 0x1c *** 300 s => 0x12bff

    set_wfp_delta_f0_f0_2();    // 0x20, 0x24

    set_wfp_delta_f1();         // 0x28

    set_wfp_delta_f2();         // 0x2c

    DEBUG_PRINTF1("delta_snapshot %x\n",    waveform_picker_regs->delta_snapshot)
    DEBUG_PRINTF1("delta_f0 %x\n",          waveform_picker_regs->delta_f0)
    DEBUG_PRINTF1("delta_f0_2 %x\n",        waveform_picker_regs->delta_f0_2)
    DEBUG_PRINTF1("delta_f1 %x\n",          waveform_picker_regs->delta_f1)
    DEBUG_PRINTF1("delta_f2 %x\n",          waveform_picker_regs->delta_f2)
    // 2688 = 8 * 336
    waveform_picker_regs->nb_data_by_buffer = 0xa7f; // 0x30 *** 2688 - 1 => nb samples -1
    waveform_picker_regs->snapshot_param    = 0xa80; // 0x34 *** 2688 => nb samples
    waveform_picker_regs->start_date        = 0x7fffffff;  // 0x38
    //
    // coarse time and fine time registers are not initialized, they are volatile
    //
    waveform_picker_regs->buffer_length     = 0x1f8;// buffer length in burst = 3 * 2688 / 16 = 504 = 0x1f8
}

void set_wfp_data_shaping( void )
{
    /** This function sets the data_shaping register of the waveform picker module.
     *
     * The value is read from one field of the parameter_dump_packet structure:\n
     * bw_sp0_sp1_r0_r1
     *
     */

    unsigned char data_shaping;

    // get the parameters for the data shaping [BW SP0 SP1 R0 R1] in sy_lfr_common1 and configure the register
    // waveform picker : [R1 R0 SP1 SP0 BW]

    data_shaping = parameter_dump_packet.bw_sp0_sp1_r0_r1;

    waveform_picker_regs->data_shaping =
              ( (data_shaping & 0x10) >> 4 )     // BW
            + ( (data_shaping & 0x08) >> 2 )     // SP0
            + ( (data_shaping & 0x04)      )     // SP1
            + ( (data_shaping & 0x02) << 2 )     // R0
            + ( (data_shaping & 0x01) << 4 );    // R1
}

void set_wfp_burst_enable_register( unsigned char mode )
{
    /** This function sets the waveform picker burst_enable register depending on the mode.
     *
     * @param mode is the LFR mode to launch.
     *
     * The burst bits shall be before the enable bits.
     *
     */

    // [0000 0000] burst f2, f1, f0 enable f3 f2 f1 f0
    // the burst bits shall be set first, before the enable bits
    switch(mode) {
    case(LFR_MODE_NORMAL):
        waveform_picker_regs->run_burst_enable = 0x00;  // [0000 0000] no burst enable
        waveform_picker_regs->run_burst_enable = 0x0f; // [0000 1111] enable f3 f2 f1 f0
        break;
    case(LFR_MODE_BURST):
        waveform_picker_regs->run_burst_enable = 0x40;  // [0100 0000] f2 burst enabled
//        waveform_picker_regs->run_burst_enable =  waveform_picker_regs->run_burst_enable | 0x04; // [0100] enable f2
        waveform_picker_regs->run_burst_enable =  waveform_picker_regs->run_burst_enable | 0x0c; // [1100] enable f3 AND f2
        break;
    case(LFR_MODE_SBM1):
        waveform_picker_regs->run_burst_enable = 0x20;  // [0010 0000] f1 burst enabled
        waveform_picker_regs->run_burst_enable =  waveform_picker_regs->run_burst_enable | 0x0f; // [1111] enable f3 f2 f1 f0
        break;
    case(LFR_MODE_SBM2):
        waveform_picker_regs->run_burst_enable = 0x40;  // [0100 0000] f2 burst enabled
        waveform_picker_regs->run_burst_enable =  waveform_picker_regs->run_burst_enable | 0x0f; // [1111] enable f3 f2 f1 f0
        break;
    default:
        waveform_picker_regs->run_burst_enable = 0x00;  // [0000 0000] no burst enabled, no waveform enabled
        break;
    }
}

void set_wfp_delta_snapshot( void )
{
    /** This function sets the delta_snapshot register of the waveform picker module.
     *
     * The value is read from two (unsigned char) of the parameter_dump_packet structure:
     * - sy_lfr_n_swf_p[0]
     * - sy_lfr_n_swf_p[1]
     *
     */

    unsigned int delta_snapshot;
    unsigned int delta_snapshot_in_T2;

    delta_snapshot = parameter_dump_packet.sy_lfr_n_swf_p[0]*256
            + parameter_dump_packet.sy_lfr_n_swf_p[1];

    delta_snapshot_in_T2 = delta_snapshot * 256;
    waveform_picker_regs->delta_snapshot = delta_snapshot_in_T2 - 1;    // max 4 bytes
}

void set_wfp_delta_f0_f0_2( void )
{
    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;
    float delta_f0_in_float;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = parameter_dump_packet.sy_lfr_n_swf_l[0] * 256 + parameter_dump_packet.sy_lfr_n_swf_l[1];
    delta_f0_in_float =nb_samples_per_snapshot / 2. * ( 1. / 256. - 1. / 24576.) * 256.;

    waveform_picker_regs->delta_f0      =  delta_snapshot - floor( delta_f0_in_float );
    waveform_picker_regs->delta_f0_2    = 0x7;         // max 7 bits
}

void set_wfp_delta_f1( void )
{
    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;
    float delta_f1_in_float;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = parameter_dump_packet.sy_lfr_n_swf_l[0] * 256 + parameter_dump_packet.sy_lfr_n_swf_l[1];
    delta_f1_in_float = nb_samples_per_snapshot / 2. * ( 1. / 256. - 1. / 4096.) * 256.;

    waveform_picker_regs->delta_f1 = delta_snapshot - floor( delta_f1_in_float );
}

void set_wfp_delta_f2()
{
    unsigned int delta_snapshot;
    unsigned int nb_samples_per_snapshot;

    delta_snapshot = waveform_picker_regs->delta_snapshot;
    nb_samples_per_snapshot = parameter_dump_packet.sy_lfr_n_swf_l[0] * 256 + parameter_dump_packet.sy_lfr_n_swf_l[1];

    waveform_picker_regs->delta_f2 = delta_snapshot - nb_samples_per_snapshot / 2;
}

//*****************
// local parameters

void increment_seq_counter_source_id( unsigned char *packet_sequence_control, unsigned int sid )
{
    /** This function increments the parameter "sequence_cnt" depending on the sid passed in argument.
     *
     * @param packet_sequence_control is a pointer toward the parameter sequence_cnt to update.
     * @param sid is the source identifier of the packet being updated.
     *
     * REQ-LFR-SRS-5240 / SSS-CP-FS-590
     * The sequence counters shall wrap around from 2^14 to zero.
     * The sequence counter shall start at zero at startup.
     *
     * REQ-LFR-SRS-5239 / SSS-CP-FS-580
     * All TM_LFR_SCIENCE_ packets are sent to ground, i.e. destination id = 0
     *
     */

    unsigned short *sequence_cnt;
    unsigned short segmentation_grouping_flag;
    unsigned short new_packet_sequence_control;
    rtems_mode initial_mode_set;
    rtems_mode current_mode_set;
    rtems_status_code status;

    //******************************************
    // CHANGE THE MODE OF THE CALLING RTEMS TASK
    status =  rtems_task_mode( RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &initial_mode_set );

    if ( (sid == SID_NORM_SWF_F0)    || (sid == SID_NORM_SWF_F1) || (sid == SID_NORM_SWF_F2)
         || (sid == SID_NORM_CWF_F3) || (sid == SID_NORM_CWF_LONG_F3)
         || (sid == SID_BURST_CWF_F2)
         || (sid == SID_NORM_ASM_F0) || (sid == SID_NORM_ASM_F1) || (sid == SID_NORM_ASM_F2)
         || (sid == SID_NORM_BP1_F0) || (sid == SID_NORM_BP1_F1) || (sid == SID_NORM_BP1_F2)
         || (sid == SID_NORM_BP2_F0) || (sid == SID_NORM_BP2_F1) || (sid == SID_NORM_BP2_F2)
         || (sid == SID_BURST_BP1_F0) || (sid == SID_BURST_BP2_F0)
         || (sid == SID_BURST_BP1_F1) || (sid == SID_BURST_BP2_F1) )
    {
        sequence_cnt = (unsigned short *) &sequenceCounters_SCIENCE_NORMAL_BURST;
    }
    else if ( (sid ==SID_SBM1_CWF_F1) || (sid ==SID_SBM2_CWF_F2)
              || (sid == SID_SBM1_BP1_F0) || (sid == SID_SBM1_BP2_F0)
              || (sid == SID_SBM2_BP1_F0) || (sid == SID_SBM2_BP2_F0)
              || (sid == SID_SBM2_BP1_F1) || (sid == SID_SBM2_BP2_F1) )
    {
        sequence_cnt = (unsigned short *) &sequenceCounters_SCIENCE_SBM1_SBM2;
    }
    else
    {
        sequence_cnt = (unsigned short *) NULL;
        PRINTF1("in increment_seq_counter_source_id *** ERR apid_destid %d not known\n", sid)
    }

    if (sequence_cnt != NULL)
    {
        segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << 8;
        *sequence_cnt                = (*sequence_cnt) & 0x3fff;

        new_packet_sequence_control = segmentation_grouping_flag | (*sequence_cnt) ;

        packet_sequence_control[0] = (unsigned char) (new_packet_sequence_control >> 8);
        packet_sequence_control[1] = (unsigned char) (new_packet_sequence_control     );

        // increment the sequence counter
        if ( *sequence_cnt < SEQ_CNT_MAX)
        {
            *sequence_cnt = *sequence_cnt + 1;
        }
        else
        {
            *sequence_cnt = 0;
        }
    }

    //***********************************
    // RESET THE MODE OF THE CALLING TASK
    status =  rtems_task_mode( initial_mode_set, RTEMS_PREEMPT_MASK, &current_mode_set );
}

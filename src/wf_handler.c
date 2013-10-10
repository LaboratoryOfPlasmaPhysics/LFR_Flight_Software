#include <wf_handler.h>

// SWF
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F0[7];
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F1[7];
Header_TM_LFR_SCIENCE_SWF_t headerSWF_F2[7];
// CWF
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F1[7];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F2_BURST[7];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F2_SBM2[7];
Header_TM_LFR_SCIENCE_CWF_t headerCWF_F3[7];

unsigned char doubleSendCWF1 = 0;
unsigned char doubleSendCWF2 = 0;

rtems_isr waveforms_isr( rtems_vector_number vector )
{

#ifdef GSA
#else
    if ( (lfrCurrentMode == LFR_MODE_NORMAL)
         || (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) )
    { // in modes other than STANDBY and BURST, send the CWF_F3 data
        if ((waveform_picker_regs->status & 0x08) == 0x08){     // [1000] f3 is full
            // (1) change the receiving buffer for the waveform picker
            if (waveform_picker_regs->addr_data_f3 == (int) wf_cont_f3) {
                waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3_bis);
            }
            else {
                waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3);
            }
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF3], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffff777; // reset f3 bits to 0, [1111 0111 0111 0111]
        }
    }
#endif

    switch(lfrCurrentMode)
    {
        //********
        // STANDBY
        case(LFR_MODE_STANDBY):
        break;

        //******
        // NORMAL
        case(LFR_MODE_NORMAL):
#ifdef GSA
        PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
        if ( (waveform_picker_regs->burst_enable & 0x7) == 0x0 ){ // if no channel is enable
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
        }
        else {
            if ( (waveform_picker_regs->status & 0x7) == 0x7 ){ // f2 f1 and f0 are full
                waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable & 0x08;
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                waveform_picker_regs->status = waveform_picker_regs->status & 0x00;
                waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable | 0x07; // [0111] enable f2 f1 f0
            }
       }
#endif
        break;

        //******
        // BURST
        case(LFR_MODE_BURST):
#ifdef GSA
        PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
        if ((waveform_picker_regs->status & 0x04) == 0x04){ // [0100] check the f2 full bit
            // (1) change the receiving buffer for the waveform picker
            if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2_bis);
            }
            else {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);
            }
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_BURST ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1111 1011 1011 1011] f2 bits = 0
        }
#endif
        break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
#ifdef GSA
        PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
        if ((waveform_picker_regs->status & 0x02) == 0x02){ // [0010] check the f1 full bit
            // (1) change the receiving buffer for the waveform picker
            if ( param_local.local_sbm1_nb_cwf_sent == (param_local.local_sbm1_nb_cwf_max-1) )
            {
                waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1_norm);
            }
            else if ( waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1_norm )
            {
                doubleSendCWF1 = 1;
                waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);
            }
            else if ( waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1 ) {
                waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1_bis);
            }
            else {
                waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);
            }
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF1], RTEMS_EVENT_MODE_SBM1 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffddd; // [1111 1101 1101 1101] f1 bit = 0
        }
        if ( ( (waveform_picker_regs->status & 0x05) == 0x05 ) ) { // [0101] check the f2 and f0 full bit
            if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffaaa; // [1111 1010 1010 1010] f2 and f0 bits = 0
            reset_local_sbm1_nb_cwf_sent();
        }

#endif
        break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
#ifdef GSA
        PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
        if ((waveform_picker_regs->status & 0x04) == 0x04){ // [0100] check the f2 full bit
            // (1) change the receiving buffer for the waveform picker
            if ( param_local.local_sbm2_nb_cwf_sent == (param_local.local_sbm2_nb_cwf_max-1) )
            {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2_norm);
            }
            else if ( waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2_norm ) {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);
                doubleSendCWF2 = 1;
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2_WFRM ) != RTEMS_SUCCESSFUL) {
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                reset_local_sbm2_nb_cwf_sent();
            }
            else if ( waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2 ) {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2_bis);
            }
            else {
                waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);
            }
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF2], RTEMS_EVENT_MODE_SBM2 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1111 1011 1011 1011] f2 bit = 0
        }
        if ( ( (waveform_picker_regs->status & 0x03) == 0x03 ) ) { // [0011] f3 f2 f1 f0, f1 and f0 are full
            if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffccc; // [1111 1100 1100 1100] f1, f0 bits = 0
        }
#endif
        break;

        //********
        // DEFAULT
        default:
        break;
    }
}

rtems_isr waveforms_simulator_isr( rtems_vector_number vector )
{
    unsigned char lfrMode;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    switch(lfrMode) {
    case (LFR_MODE_STANDBY):
        break;
    case (LFR_MODE_NORMAL):
        if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
            rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_5 );
        }
        break;
    case (LFR_MODE_BURST):
        break;
    case (LFR_MODE_SBM1):
        break;
    case (LFR_MODE_SBM2):
        break;
    }
}

rtems_task wfrm_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    rtems_event_set event_out;
    rtems_id queue_id;
    rtems_status_code status;

    init_header_snapshot_wf_table( SID_NORM_SWF_F0, headerSWF_F0 );
    init_header_snapshot_wf_table( SID_NORM_SWF_F1, headerSWF_F1 );
    init_header_snapshot_wf_table( SID_NORM_SWF_F2, headerSWF_F2 );

    init_waveforms();

    status =  rtems_message_queue_ident( misc_name[QUEUE_PKTS], 0, &queue_id );
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in WFRM *** ERR getting queue id, %d\n", status)
    }

    BOOT_PRINTF("in WFRM ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_MODE_NORMAL | RTEMS_EVENT_MODE_SBM1
                            | RTEMS_EVENT_MODE_SBM2 | RTEMS_EVENT_MODE_SBM2_WFRM,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);

        if (event_out == RTEMS_EVENT_MODE_NORMAL)
        {
            send_waveform_SWF(wf_snap_f0, SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF(wf_snap_f1, SID_NORM_SWF_F1, headerSWF_F1, queue_id);
            send_waveform_SWF(wf_snap_f2, SID_NORM_SWF_F2, headerSWF_F2, queue_id);
#ifdef GSA
            waveform_picker_regs->status = waveform_picker_regs->status & 0xf888; // [1111 1000 1000 1000] f2, f1, f0 bits =0
#endif
        }
        else if (event_out == RTEMS_EVENT_MODE_SBM1)
        {
            send_waveform_SWF(wf_snap_f0, SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF(wf_snap_f1_norm, SID_NORM_SWF_F1, headerSWF_F1, queue_id);
            send_waveform_SWF(wf_snap_f2, SID_NORM_SWF_F2, headerSWF_F2, queue_id);
#ifdef GSA
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffaaa; // [1111 1010 1010 1010] f2, f0 bits = 0
#endif
        }
        else if (event_out == RTEMS_EVENT_MODE_SBM2)
        {
            send_waveform_SWF(wf_snap_f0, SID_NORM_SWF_F0, headerSWF_F0, queue_id);
            send_waveform_SWF(wf_snap_f1, SID_NORM_SWF_F1, headerSWF_F1, queue_id);
#ifdef GSA
            waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffccc; // [1111 1100 1100 1100] f1, f0 bits = 0
#endif
        }
        else if (event_out == RTEMS_EVENT_MODE_SBM2_WFRM)
        {
            send_waveform_SWF(wf_snap_f2_norm, SID_NORM_SWF_F2, headerSWF_F2, queue_id);
        }
        else
        {
            PRINTF("in WFRM *** unexpected event")
        }


#ifdef GSA
        // irq processed, reset the related register of the timer unit
        gptimer_regs->timer[TIMER_WF_SIMULATOR].ctrl = gptimer_regs->timer[TIMER_WF_SIMULATOR].ctrl | 0x00000010;
        // clear the interruption
        LEON_Unmask_interrupt( IRQ_WF );
#endif
    }
}

rtems_task cwf3_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    rtems_event_set event_out;
    rtems_id queue_id;

    init_header_continuous_wf_table( SID_NORM_CWF_F3, headerCWF_F3 );

    queue_id = get_pkts_queue_id();

    BOOT_PRINTF("in CWF3 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_0,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        PRINTF("send CWF F3 \n")
#ifdef GSA
#else
        if (waveform_picker_regs->addr_data_f3 == (int) wf_cont_f3) {
            send_waveform_CWF( wf_cont_f3_bis, SID_NORM_CWF_F3, headerCWF_F3, queue_id );
        }
        else {
            send_waveform_CWF( wf_cont_f3, SID_NORM_CWF_F3, headerCWF_F3, queue_id );
        }
#endif
    }
}

rtems_task cwf2_task(rtems_task_argument argument)  // ONLY USED IN BURST AND SBM2
{
    rtems_event_set event_out;
    rtems_id queue_id;

    init_header_continuous_wf_table( SID_BURST_CWF_F2, headerCWF_F2_BURST );
    init_header_continuous_wf_table( SID_SBM2_CWF_F2, headerCWF_F2_SBM2 );

    queue_id = get_pkts_queue_id();

    BOOT_PRINTF("in CWF2 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_MODE_BURST | RTEMS_EVENT_MODE_SBM2,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);

        if (event_out == RTEMS_EVENT_MODE_BURST)
        {
            // F2
#ifdef GSA
#else
            if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                send_waveform_CWF( wf_snap_f2_bis, SID_BURST_CWF_F2, headerCWF_F2_BURST, queue_id );
            }
            else {
                send_waveform_CWF( wf_snap_f2, SID_BURST_CWF_F2, headerCWF_F2_BURST, queue_id );
            }
        #endif
        }

        else if (event_out == RTEMS_EVENT_MODE_SBM2)
        {
#ifdef GSA
#else
            if (doubleSendCWF2 == 1)
            {
                doubleSendCWF2 = 0;
                send_waveform_CWF( wf_snap_f2_norm, SID_SBM2_CWF_F2, headerCWF_F2_SBM2, queue_id );
            }
            else if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                send_waveform_CWF( wf_snap_f2_bis, SID_SBM2_CWF_F2, headerCWF_F2_SBM2, queue_id );
            }
            else {
                send_waveform_CWF( wf_snap_f2, SID_SBM2_CWF_F2, headerCWF_F2_SBM2, queue_id );
            }
            param_local.local_sbm2_nb_cwf_sent ++;
#endif
        }
        else
        {
            PRINTF1("in CWF2 *** ERR mode = %d\n", lfrCurrentMode)
        }
    }
}

rtems_task cwf1_task(rtems_task_argument argument)  // ONLY USED IN SBM1
{
    rtems_event_set event_out;
    rtems_id queue_id;

    init_header_continuous_wf_table( SID_SBM1_CWF_F1, headerCWF_F1 );

    queue_id = get_pkts_queue_id();

    BOOT_PRINTF("in CWF1 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_MODE_SBM1,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        if (event_out == RTEMS_EVENT_MODE_SBM1)
        {
#ifdef GSA
#else
            if (doubleSendCWF1 == 1)
            {
                doubleSendCWF1 = 0;
                send_waveform_CWF( wf_snap_f1_norm, SID_SBM1_CWF_F1, headerCWF_F1, queue_id );
            }
            else if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
               send_waveform_CWF( wf_snap_f1_bis, SID_SBM1_CWF_F1, headerCWF_F1, queue_id );
            }
            else {
                send_waveform_CWF( wf_snap_f1, SID_SBM1_CWF_F1, headerCWF_F1, queue_id );
            }
            param_local.local_sbm1_nb_cwf_sent ++;
#endif
        }
        else
        {
            PRINTF1("in CWF1 *** ERR mode = %d\n", lfrCurrentMode)
        }
    }
}

//******************
// general functions
void init_waveforms( void )
{
    int i = 0;

    for (i=0; i< NB_SAMPLES_PER_SNAPSHOT; i++)
    {
        //***
        // F0
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET ] = 0x88887777;     //
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET  ] = 0x22221111;    //
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET  ] = 0x44443333;    //

        //***
        // F1
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET  ] = 0x22221111;
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET  ] = 0x44443333;
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET  ] = 0xaaaa0000;

        //***
        // F2
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET  ] = 0x44443333;
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET  ] = 0x22221111;
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET  ] = 0xaaaa0000;

        //***
        // F3
        //wf_cont_f3[ (i* NB_WORDS_SWF_BLK) + 0 ] = val1;
        //wf_cont_f3[ (i* NB_WORDS_SWF_BLK) + 1 ] = val2;
        //wf_cont_f3[ (i* NB_WORDS_SWF_BLK) + 2 ] = 0xaaaa0000;
    }
}

int init_header_snapshot_wf_table( unsigned int sid, Header_TM_LFR_SCIENCE_SWF_t *headerSWF)
{
    unsigned char i;

    for (i=0; i<7; i++)
    {
        headerSWF[ i ].targetLogicalAddress = CCSDS_DESTINATION_ID;
        headerSWF[ i ].protocolIdentifier = CCSDS_PROTOCOLE_ID;
        headerSWF[ i ].reserved = DEFAULT_RESERVED;
        headerSWF[ i ].userApplication = CCSDS_USER_APP;
        headerSWF[ i ].packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST >> 8);
        headerSWF[ i ].packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST);
        if (i == 0)
        {
            headerSWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_FIRST;
            headerSWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_340 >> 8);
            headerSWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_340     );
            headerSWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_340 >> 8);
            headerSWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_340     );
        }
        else if (i == 6)
        {
            headerSWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_LAST;
            headerSWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_8 >> 8);
            headerSWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_8     );
            headerSWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_8 >> 8);
            headerSWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_8     );
        }
        else
        {
            headerSWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_CONTINUATION;
            headerSWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_SWF_340 >> 8);
            headerSWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_SWF_340     );
            headerSWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_340 >> 8);
            headerSWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_340     );
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
        headerSWF[ i ].sid = sid;
        headerSWF[ i ].hkBIA = DEFAULT_HKBIA;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
        headerSWF[ i ].time[0] = 0x00;
    }
    return LFR_SUCCESSFUL;
}

int init_header_continuous_wf_table( unsigned int sid, Header_TM_LFR_SCIENCE_CWF_t *headerCWF )
{
    unsigned int i;

    for (i=0; i<7; i++)
    {
        headerCWF[ i ].targetLogicalAddress = CCSDS_DESTINATION_ID;
        headerCWF[ i ].protocolIdentifier = CCSDS_PROTOCOLE_ID;
        headerCWF[ i ].reserved = DEFAULT_RESERVED;
        headerCWF[ i ].userApplication = CCSDS_USER_APP;
        if ( (sid == SID_SBM1_CWF_F1) || (sid == SID_SBM2_CWF_F2) )
        {
            headerCWF[ i ].packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2 >> 8);
            headerCWF[ i ].packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_SBM1_SBM2);
        }
        else
        {
            headerCWF[ i ].packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST >> 8);
            headerCWF[ i ].packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL_BURST);
        }
        if (i == 0)
        {
            headerCWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_FIRST;
            headerCWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_340 >> 8);
            headerCWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_340     );
            headerCWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_340 >> 8);
            headerCWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_340     );
        }
        else if (i == 6)
        {
            headerCWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_LAST;
            headerCWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_8 >> 8);
            headerCWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_8     );
            headerCWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_8 >> 8);
            headerCWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_8     );
        }
        else
        {
            headerCWF[ i ].packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_CONTINUATION;
            headerCWF[ i ].packetLength[0] = (unsigned char) (TM_LEN_SCI_CWF_340 >> 8);
            headerCWF[ i ].packetLength[1] = (unsigned char) (TM_LEN_SCI_CWF_340     );
            headerCWF[ i ].blkNr[0] = (unsigned char) (BLK_NR_340 >> 8);
            headerCWF[ i ].blkNr[1] = (unsigned char) (BLK_NR_340     );
        }
        headerCWF[ i ].packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
        // PKT_CNT
        // PKT_NR
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
    return LFR_SUCCESSFUL;
}

void reset_waveforms( void )
{
    int i = 0;

    for (i=0; i< NB_SAMPLES_PER_SNAPSHOT; i++)
    {
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET] = 0x10002000;
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET] = 0x20001000;
        wf_snap_f0[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET] = 0x40008000;

        //***
        // F1
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET] = 0x1000f000;
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET] = 0xf0001000;
        wf_snap_f1[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET] = 0x40008000;

        //***
        // F2
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 0 + TIME_OFFSET] = 0x40008000;
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 1 + TIME_OFFSET] = 0x20001000;
        wf_snap_f2[ (i* NB_WORDS_SWF_BLK) + 2 + TIME_OFFSET] = 0x10002000;

        //***
        // F3
        /*wf_cont_f3[ i* NB_WORDS_SWF_BLK + 0 ] = build_value(   i, i );   // v  and 1
        wf_cont_f3[ i* NB_WORDS_SWF_BLK + 1 ] = build_value(   i, i );   // e2 and b1
        wf_cont_f3[ i* NB_WORDS_SWF_BLK + 2 ] = build_value(   i, i );   // b2 and b3*/
    }
}

int send_waveform_SWF( volatile int *waveform, unsigned int sid,
                       Header_TM_LFR_SCIENCE_SWF_t *headerSWF, rtems_id queue_id )
{
    unsigned int i;
    int ret;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_SWF;

    spw_ioctl_send_SWF.hlen = TM_HEADER_LEN + 4 + 12; // + 4 is for the protocole extra header, + 12 is for the auxiliary header
    spw_ioctl_send_SWF.options = 0;

    ret = LFR_DEFAULT;

    for (i=0; i<7; i++) // send waveform
    {
        spw_ioctl_send_SWF.data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_SWF.hdr = (char*) &headerSWF[ i ];
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send_SWF.dlen = 8 * NB_BYTES_SWF_BLK;
        }
        else {
            spw_ioctl_send_SWF.dlen = 340 * NB_BYTES_SWF_BLK;
        }
        // SET PACKET TIME
        headerSWF[ i ].time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        headerSWF[ i ].time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        headerSWF[ i ].time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        headerSWF[ i ].time[3] = (unsigned char) (time_management_regs->coarse_time);
        headerSWF[ i ].time[4] = (unsigned char) (time_management_regs->fine_time>>8);
        headerSWF[ i ].time[5] = (unsigned char) (time_management_regs->fine_time);
        headerSWF[ i ].acquisitionTime[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        headerSWF[ i ].acquisitionTime[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        headerSWF[ i ].acquisitionTime[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        headerSWF[ i ].acquisitionTime[3] = (unsigned char) (time_management_regs->coarse_time);
        headerSWF[ i ].acquisitionTime[4] = (unsigned char) (time_management_regs->fine_time>>8);
        headerSWF[ i ].acquisitionTime[5] = (unsigned char) (time_management_regs->fine_time);
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

int send_waveform_CWF(volatile int *waveform, unsigned int sid,
                      Header_TM_LFR_SCIENCE_CWF_t *headerCWF, rtems_id queue_id)
{
    unsigned int i;
    int ret;
    rtems_status_code status;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.options = 0;

    ret = LFR_DEFAULT;

    for (i=0; i<7; i++) // send waveform
    {
        int coarseTime = 0x00;
        int fineTime = 0x00;
        spw_ioctl_send_CWF.data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) ];
        spw_ioctl_send_CWF.hdr = (char*) &headerCWF[ i ];
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send_CWF.dlen = 8 * NB_BYTES_SWF_BLK;
        }
        else {
            spw_ioctl_send_CWF.dlen = 340 * NB_BYTES_SWF_BLK;
        }
        // SET PACKET TIME
        coarseTime = time_management_regs->coarse_time;
        fineTime = time_management_regs->fine_time;
        headerCWF[ i ].time[0] = (unsigned char) (coarseTime>>24);
        headerCWF[ i ].time[1] = (unsigned char) (coarseTime>>16);
        headerCWF[ i ].time[2] = (unsigned char) (coarseTime>>8);
        headerCWF[ i ].time[3] = (unsigned char) (coarseTime);
        headerCWF[ i ].time[4] = (unsigned char) (fineTime>>8);
        headerCWF[ i ].time[5] = (unsigned char) (fineTime);
        headerCWF[ i ].acquisitionTime[0] = (unsigned char) (coarseTime>>24);
        headerCWF[ i ].acquisitionTime[1] = (unsigned char) (coarseTime>>16);
        headerCWF[ i ].acquisitionTime[2] = (unsigned char) (coarseTime>>8);
        headerCWF[ i ].acquisitionTime[3] = (unsigned char) (coarseTime);
        headerCWF[ i ].acquisitionTime[4] = (unsigned char) (fineTime>>8);
        headerCWF[ i ].acquisitionTime[5] = (unsigned char) (fineTime);
        // SEND PACKET
        if (sid == SID_NORM_CWF_F3)
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

//**************
// wfp registers
void set_wfp_data_shaping()
{
    unsigned char data_shaping;

    // get the parameters for the data shaping [BW SP0 SP1 R0 R1] in sy_lfr_common1 and configure the register
    // waveform picker : [R1 R0 SP1 SP0 BW]

    data_shaping = parameter_dump_packet.bw_sp0_sp1_r0_r1;

#ifdef GSA
#else
    waveform_picker_regs->data_shaping =
              ( (data_shaping & 0x10) >> 4 )     // BW
            + ( (data_shaping & 0x08) >> 2 )     // SP0
            + ( (data_shaping & 0x04)      )     // SP1
            + ( (data_shaping & 0x02) << 2 )     // R0
            + ( (data_shaping & 0x01) << 4 );    // R1
#endif
}

char set_wfp_delta_snapshot()
{
    char ret;
    unsigned int delta_snapshot;
    ret = LFR_DEFAULT;

    delta_snapshot = parameter_dump_packet.sy_lfr_n_swf_p[0]*256
            + parameter_dump_packet.sy_lfr_n_swf_p[1];

#ifdef GSA
#else
    unsigned char aux = 0;
    if ( delta_snapshot < MIN_DELTA_SNAPSHOT )
    {
        aux = MIN_DELTA_SNAPSHOT;
        ret = LFR_DEFAULT;
    }
    else
    {
        aux = delta_snapshot ;
        ret = LFR_SUCCESSFUL;
    }
    waveform_picker_regs->delta_snapshot = aux;             // max 2 bytes
#endif

    return ret;
}

void set_wfp_burst_enable_register( unsigned char mode)
{
#ifdef GSA
#else
    // [0000 0000] burst f2, f1, f0 enable f3 f2 f1 f0
    // the burst bits shall be set first, before the enable bits
    switch(mode) {
    case(LFR_MODE_NORMAL):
        waveform_picker_regs->burst_enable = 0x00;  // [0000 0000] no burst enable
        waveform_picker_regs->burst_enable = 0x0f; // [0000 1111] enable f3 f2 f1 f0
        break;
    case(LFR_MODE_BURST):
        waveform_picker_regs->burst_enable = 0x40;  // [0100 0000] f2 burst enabled
        waveform_picker_regs->burst_enable =  waveform_picker_regs->burst_enable | 0x04; // [0100] enable f2
        break;
    case(LFR_MODE_SBM1):
        waveform_picker_regs->burst_enable = 0x20;  // [0010 0000] f1 burst enabled
        waveform_picker_regs->burst_enable =  waveform_picker_regs->burst_enable | 0x0f; // [1111] enable f3 f2 f1 f0
        break;
    case(LFR_MODE_SBM2):
        waveform_picker_regs->burst_enable = 0x40;  // [0100 0000] f2 burst enabled
        waveform_picker_regs->burst_enable =  waveform_picker_regs->burst_enable | 0x0f; // [1111] enable f3 f2 f1 f0
        break;
    default:
        waveform_picker_regs->burst_enable = 0x00;  // [0000 0000] no burst enabled, no waveform enabled
        break;
    }
#endif
}

void reset_wfp_burst_enable()
{
#ifdef GSA
#else
    waveform_picker_regs->burst_enable = 0x00;              // burst f2, f1, f0     enable f3, f2, f1, f0
#endif
}

void reset_wfp_status()
{
#ifdef GSA
#else
    waveform_picker_regs->status = 0x00;              // burst f2, f1, f0     enable f3, f2, f1, f0
#endif
}

void reset_waveform_picker_regs()
{
#ifdef GSA
#else
    set_wfp_data_shaping();
    reset_wfp_burst_enable();
    waveform_picker_regs->addr_data_f0 = (int) (wf_snap_f0);    //
    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);    //
    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);    //
    waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3);    //
    set_wfp_delta_snapshot();                           // time in seconds between two snapshots
    waveform_picker_regs->delta_f2_f1 = 0xffff;         // 0x16800 => 92160 (max 4 bytes)
    waveform_picker_regs->delta_f2_f0 = 0x17c00;        // 97 280 (max 5 bytes)
    waveform_picker_regs->nb_burst_available = 0x180;   // max 3 bytes, size of the buffer in burst (1 burst = 16 x 4 octets)
    waveform_picker_regs->nb_snapshot_param = 0x7ff;    // max 3 octets, 2048 - 1
    waveform_picker_regs->status = 0x00;                //
#endif
}

//*****************
// local parameters
void set_local_sbm1_nb_cwf_max()
{
    // (2 snapshots of 2048 points per seconds) * (period of the NORM snashots) - 8 s (duration of the f2 snapshot)
    param_local.local_sbm1_nb_cwf_max = 2 *
            (parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
            + parameter_dump_packet.sy_lfr_n_swf_p[1]) - 8; // 16 CWF1 parts during 1 SWF2
}

void set_local_sbm2_nb_cwf_max()
{
    // (period of the NORM snashots) / (8 seconds per snapshot at f2 = 256 Hz)
    param_local.local_sbm2_nb_cwf_max = (parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
            + parameter_dump_packet.sy_lfr_n_swf_p[1]) / 8;
}

void set_local_nb_interrupt_f0_MAX()
{
    param_local.local_nb_interrupt_f0_MAX = ( (parameter_dump_packet.sy_lfr_n_asm_p[0]) * 256
            + parameter_dump_packet.sy_lfr_n_asm_p[1] ) * 100;
}

void reset_local_sbm1_nb_cwf_sent()
{
    param_local.local_sbm1_nb_cwf_sent = 0;
}

void reset_local_sbm2_nb_cwf_sent()
{
    param_local.local_sbm2_nb_cwf_sent = 0;
}

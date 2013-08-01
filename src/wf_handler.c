#include <wf_handler.h>

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    unsigned char lfrMode;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

#ifdef GSA
#else
    if ( (lfrMode != LFR_MODE_STANDBY) & (lfrMode != LFR_MODE_BURST) )
    { // in modes other than STANDBY and BURST, send the CWF_F3 data
        if ((waveform_picker_regs->status & 0x08) == 0x08){     // f3 is full
            // (1) change the receiving buffer for the waveform picker
            if (waveform_picker_regs->addr_data_f3 == (int) wf_cont_f3) {
                waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3_bis);
            }
            else {
                waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3);
            }
            // (2) send an event for the waveforms transmission
            if (rtems_event_send( Task_id[TASKID_CWF3], RTEMS_EVENT_MODE_NORMAL_CWF_F3 ) != RTEMS_SUCCESSFUL) {
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            waveform_picker_regs->status = waveform_picker_regs->status & 0x0007; // reset the f3 full bit to 0
        }
    }
#endif

    switch(lfrMode)
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
            if ( (waveform_picker_regs->burst_enable & 0x7) == 0x0 ){   // if no channel is enable
                rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
            }
            else {
                if ( (waveform_picker_regs->status & 0x7) == 0x7 ){     // f2 f1 and f0 are full
                    waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable & 0x08;
                    if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                        rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                    }
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
            if (waveform_picker_regs->burst_enable == 0x44) {
                if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2_bis);
                }
                else {
                    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);
                }
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_BURST ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
            }
            waveform_picker_regs->status = 0x00;
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
                if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
                    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1_bis);
                }
                else {
                    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);
                }
                // (2) send an event for the waveforms transmission
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM1 ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                waveform_picker_regs->status = waveform_picker_regs->status & 0x000d; // reset the f1 full bit to 0
            }
            if ( ( (waveform_picker_regs->status & 0x05) == 0x05 ) ) { // [0101] check the f2 and f0 full bit
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable | 0x05; // [0101] // enable f2 and f0
                waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffaaa; // set to 0 the bits related to f2 and f0
            }

#endif
            break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
#ifdef GSA
            PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
            if ((waveform_picker_regs->status & 0x04) == 0x04){ // check the f2 full bit
                // (1) change the receiving buffer for the waveform picker
                if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2_bis);
                }
                else {
                    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);
                }
                // (2) send an event for the waveforms transmission
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2 ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffbbb; // [1011]
            }
            if ( ( (waveform_picker_regs->status & 0x03) == 0x03 ) ) { // [0011] f3 f2 f1 f0, check the f2 and f0 full bit
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
                waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable | 0x03; // [0011] // enable f2 and f0
                waveform_picker_regs->status = waveform_picker_regs->status & 0xfffffccc; // set to 0 the bits related to f1 and f0
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
    unsigned int i;
    unsigned int intEventOut;
    spw_ioctl_pkt_send spw_ioctl_send_SWF;
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    rtems_event_set event_out;
    Header_TM_LFR_SCIENCE_SWF_t headerSWF;
    Header_TM_LFR_SCIENCE_CWF_t headerCWF;

    init_header_snapshot_wf( &headerSWF );
    init_header_continuous_wf( &headerCWF );

    // BUILD THE PACKET HEADERS
    spw_ioctl_send_SWF.hlen = TM_HEADER_LEN + 4 + 12; // + 4 is for the protocole extra header, + 12 is for the auxiliary header
    spw_ioctl_send_SWF.hdr = (char*) &headerSWF;
    spw_ioctl_send_SWF.options = 0;

    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.hdr = (char*) &headerCWF;
    spw_ioctl_send_CWF.options = 0;

    init_waveforms();

    PRINTF("in WFRM ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3 | RTEMS_EVENT_4,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        intEventOut =  (unsigned int) event_out;
        for (i = 0; i< 5; i++) {
            if ( ( (intEventOut >> i) & 0x0001) != 0 ) {

                switch(i) {

                case(LFR_MODE_NORMAL):
                    send_waveform_norm( &headerSWF, &spw_ioctl_send_SWF );
                    break;

                case(LFR_MODE_BURST):
                    send_waveform_burst( &headerCWF, &spw_ioctl_send_CWF );
                    break;

                case(LFR_MODE_SBM1):
                    send_waveform_sbm1( &headerCWF, &spw_ioctl_send_CWF );
#ifdef GSA
#else
                    if ( param_local.local_sbm1_nb_cwf_sent == (param_local.local_sbm1_nb_cwf_max-1) ) {
                        // send the f1 buffer as a NORM snapshot
                        if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
                           send_waveform_SWF( &headerSWF, wf_snap_f1_bis, SID_NORM_SWF_F1, &spw_ioctl_send_SWF );
                        }
                        else {
                            send_waveform_SWF( &headerSWF, wf_snap_f1, SID_NORM_SWF_F1, &spw_ioctl_send_SWF );
                        }
                        reset_local_sbm1_nb_cwf_sent();
                    }
                    else {
                        param_local.local_sbm1_nb_cwf_sent ++;
                    }
#endif
                    break;

                case(LFR_MODE_SBM2):
                    send_waveform_sbm2( &headerCWF, &spw_ioctl_send_CWF );
#ifdef GSA
#else
                    if ( param_local.local_sbm2_nb_cwf_sent == (param_local.local_sbm2_nb_cwf_max-1) ) {
                        // send the f2 buffer as a NORM snapshot
                        if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                           send_waveform_SWF( &headerSWF, wf_snap_f2_bis, SID_NORM_SWF_F2, &spw_ioctl_send_SWF );
                        }
                        else {
                            send_waveform_SWF( &headerSWF, wf_snap_f2, SID_NORM_SWF_F2, &spw_ioctl_send_SWF );
                        }
                        reset_local_sbm2_nb_cwf_sent();
                    }
                    else {
                        param_local.local_sbm2_nb_cwf_sent ++;
                    }
#endif
                    break;

                default:
                    break;
                }
            }
        }
    }
}

rtems_task cwf3_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    spw_ioctl_pkt_send spw_ioctl_send_CWF;
    rtems_event_set event_out;
    Header_TM_LFR_SCIENCE_CWF_t headerCWF;

    init_header_continuous_wf( &headerCWF );

    // BUILD THE PACKET HEADER
    spw_ioctl_send_CWF.hlen = TM_HEADER_LEN + 4 + 10; // + 4 is for the protocole extra header, + 10 is for the auxiliary header
    spw_ioctl_send_CWF.hdr = (char*) &headerCWF;
    spw_ioctl_send_CWF.options = 0;

    PRINTF("in CWF3 ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive( RTEMS_EVENT_5,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        PRINTF("send CWF F3 \n")
        send_waveform_norm_cwf_f3( &headerCWF, &spw_ioctl_send_CWF );
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

void init_header_snapshot_wf( Header_TM_LFR_SCIENCE_SWF_t *header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL);
    header->packetSequenceControl[0] = 0xc0;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = 0x00;
    header->packetLength[1] = 0x00;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = 0x10;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    header->sid = 0x00;
    header->hkBIA = 0x1f;
    header->pktCnt = 0x07;    // PKT_CNT
    header->pktNr = 0x00;     // PKT_NR
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->blkNr[0] = 0x00;  // BLK_NR MSB
    header->blkNr[1] = 0x00;  // BLK_NR LSB
}

void init_header_continuous_wf( Header_TM_LFR_SCIENCE_CWF_t *header)
{
    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = 0x00;
    header->packetID[1] = 0x00;
    header->packetSequenceControl[0] = 0xc0;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = 0x00;
    header->packetLength[1] = 0x00;
    // DATA FIELD HEADER
    header->spare1_pusVersion_spare2 = 0x10;
    header->serviceType = TM_TYPE_LFR_SCIENCE; // service type
    header->serviceSubType = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->destinationID = TM_DESTINATION_ID_GROUND;
    // AUXILIARY DATA HEADER
    header->sid = 0x00;
    header->hkBIA = 0x1f;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->time[0] = 0x00;
    header->blkNr[0] = 0x00;  // BLK_NR MSB
    header->blkNr[1] = 0x00;  // BLK_NR LSB
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

void send_waveform_SWF( Header_TM_LFR_SCIENCE_SWF_t *header, volatile int *waveform,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send)
{
    unsigned int i = 0;
    unsigned int length = 0;
    rtems_status_code status;

    header->sid = (unsigned char) sid;

    for (i=0; i<7; i++) // send waveform
    {
        header->pktNr = (unsigned char) i+1; // PKT_NR
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send->dlen = 8 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_SWF_8;
            header->blkNr[0] = 0x00; // BLK_NR MSB
            header->blkNr[1] = 0x08; // BLK_NR LSB
        }
        else {
            spw_ioctl_send->dlen = 340 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_SWF_340;
            header->blkNr[0] = 0x01; // BLK_NR MSB
            header->blkNr[1] = 0x54; // BLK_NR LSB
        }
        spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) ];
        // BUILD THE HEADER
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        header->sid = (unsigned char) sid;   // SID
        // SET PACKET TIME
        header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->time[3] = (unsigned char) (time_management_regs->coarse_time);
        header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->time[5] = (unsigned char) (time_management_regs->fine_time);
        header->acquisitionTime[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->acquisitionTime[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->acquisitionTime[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->acquisitionTime[3] = (unsigned char) (time_management_regs->coarse_time);
        header->acquisitionTime[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->acquisitionTime[5] = (unsigned char) (time_management_regs->fine_time);
        // SEND PACKET
        status = write_spw(spw_ioctl_send);
        if (status != RTEMS_SUCCESSFUL) {
            while (true) {
                if (status != RTEMS_SUCCESSFUL) {
                    status = write_spw(spw_ioctl_send);
                    //PRINTF1("%d", i)
                    sched_yield();
                }
                else {
                    //PRINTF("\n")
                    break;
                }
            }
        }
    }
}

void send_waveform_CWF( Header_TM_LFR_SCIENCE_CWF_t *header, volatile int *waveform,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send)
{
    unsigned int i = 0;
    unsigned int length = 0;
    rtems_status_code status;

    header->sid = (unsigned char) sid;

    for (i=0; i<7; i++) // send waveform
    {
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send->dlen = 8 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_CWF_8;
            header->blkNr[0] = 0x00; // BLK_NR MSB
            header->blkNr[1] = 0x08; // BLK_NR LSB
        }
        else {
            spw_ioctl_send->dlen = 340 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_CWF_340;
            header->blkNr[0] = 0x01; // BLK_NR MSB
            header->blkNr[1] = 0x54; // BLK_NR LSB
        }
        spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) ];
        // BUILD THE HEADER
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        // SET PACKET TIME
        header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->time[3] = (unsigned char) (time_management_regs->coarse_time);
        header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->time[5] = (unsigned char) (time_management_regs->fine_time);
        header->acquisitionTime[0] = (unsigned char) (time_management_regs->coarse_time>>24);
        header->acquisitionTime[1] = (unsigned char) (time_management_regs->coarse_time>>16);
        header->acquisitionTime[2] = (unsigned char) (time_management_regs->coarse_time>>8);
        header->acquisitionTime[3] = (unsigned char) (time_management_regs->coarse_time);
        header->acquisitionTime[4] = (unsigned char) (time_management_regs->fine_time>>8);
        header->acquisitionTime[5] = (unsigned char) (time_management_regs->fine_time);
        // SEND PACKET
        status = write_spw(spw_ioctl_send);
        if (status != RTEMS_SUCCESSFUL) {
            while (true) {
                if (status != RTEMS_SUCCESSFUL) {
                    status = write_spw(spw_ioctl_send);
                    //PRINTF1("%d", i)
                    sched_yield();
                }
                else {
                    //PRINTF("\n")
                    break;
                }
            }
        }
    }
}

void send_waveform_norm(Header_TM_LFR_SCIENCE_SWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send)
{
    unsigned char lfrMode;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL);
    // TIME
    header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    header->time[3] = (unsigned char) (time_management_regs->coarse_time);
    header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    header->time[5] = (unsigned char) (time_management_regs->fine_time);

    switch(lfrMode) {

    case LFR_MODE_NORMAL:
        send_waveform_SWF( header, wf_snap_f0, SID_NORM_SWF_F0, spw_ioctl_send);
        send_waveform_SWF( header, wf_snap_f1, SID_NORM_SWF_F1, spw_ioctl_send);
        send_waveform_SWF( header, wf_snap_f2, SID_NORM_SWF_F2, spw_ioctl_send);
#ifdef GSA
#else
        waveform_picker_regs->status = waveform_picker_regs->status & 0x00;
        waveform_picker_regs->burst_enable = waveform_picker_regs->burst_enable | 0x07; // [0111] enable f2 f1 f0
#endif
        break;

    case LFR_MODE_SBM1:
        send_waveform_SWF( header, wf_snap_f0, SID_NORM_SWF_F0, spw_ioctl_send);
        // F1 data are sent by the send_waveform_sbm1 function
        send_waveform_SWF( header, wf_snap_f2, SID_NORM_SWF_F2, spw_ioctl_send);
        break;

    case LFR_MODE_SBM2:
        send_waveform_SWF( header, wf_snap_f0, SID_NORM_SWF_F0, spw_ioctl_send);
        send_waveform_SWF( header, wf_snap_f1, SID_NORM_SWF_F1, spw_ioctl_send);
        // F2 data are sent by the send_waveform_sbm2 function
        break;

    default:
        break;
    }

#ifdef GSA
    // irq processed, reset the related register of the timer unit
    gptimer_regs->timer[TIMER_WF_SIMULATOR].ctrl = gptimer_regs->timer[TIMER_WF_SIMULATOR].ctrl | 0x00000010;
    // clear the interruption
    LEON_Unmask_interrupt( IRQ_WF );
#endif

}

void send_waveform_norm_cwf_f3(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send)
{
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_NORMAL);
    // TIME
    header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    header->time[3] = (unsigned char) (time_management_regs->coarse_time);
    header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    header->time[5] = (unsigned char) (time_management_regs->fine_time);

    //*******************************
    // send continuous waveform at f3
    // F3
#ifdef GSA
#else
    if (waveform_picker_regs->addr_data_f3 == (int) wf_cont_f3) {
       send_waveform_CWF( header, wf_cont_f3_bis, SID_NORM_CWF_F3, spw_ioctl_send);
    }
    else {
        send_waveform_CWF( header, wf_cont_f3, SID_NORM_CWF_F3, spw_ioctl_send);
    }
#endif
}

void send_waveform_burst(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send)
{
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2 >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2);
    // TIME
    header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    header->time[3] = (unsigned char) (time_management_regs->coarse_time);
    header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    header->time[5] = (unsigned char) (time_management_regs->fine_time);
    // ACQUISITION TIME

    // F2
#ifdef GSA
#else
    if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
       send_waveform_CWF( header, wf_snap_f2_bis, SID_BURST_CWF_F2, spw_ioctl_send);
    }
    else {
        send_waveform_CWF( header, wf_snap_f2, SID_BURST_CWF_F2, spw_ioctl_send);
    }
#endif
}

void send_waveform_sbm1(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send)
{
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2 >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2);
    // TIME
    header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    header->time[3] = (unsigned char) (time_management_regs->coarse_time);
    header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    header->time[5] = (unsigned char) (time_management_regs->fine_time);

    // F1
#ifdef GSA
#else
    if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
       send_waveform_CWF( header, wf_snap_f1_bis, SID_SBM1_CWF_F1, spw_ioctl_send );
    }
    else {
        send_waveform_CWF( header, wf_snap_f1, SID_SBM1_CWF_F1, spw_ioctl_send );
    }
#endif
}

void send_waveform_sbm2(Header_TM_LFR_SCIENCE_CWF_t *header, spw_ioctl_pkt_send *spw_ioctl_send)
{
    header->packetID[0] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2 >> 8);
    header->packetID[1] = (unsigned char) (TM_PACKET_ID_SCIENCE_BURST_SBM1_SBM2);
    // TIME
    header->time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    header->time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    header->time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    header->time[3] = (unsigned char) (time_management_regs->coarse_time);
    header->time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    header->time[5] = (unsigned char) (time_management_regs->fine_time);

    // F2
#ifdef GSA
#else
    if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
        send_waveform_CWF( header, wf_snap_f2_bis, SID_SBM2_CWF_F2, spw_ioctl_send);
    }
    else {
        send_waveform_CWF( header, wf_snap_f2, SID_SBM2_CWF_F2, spw_ioctl_send);
    }
#endif
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
    waveform_picker_regs->delta_f2_f1 = 0xffff;         // max 4 bytes
    waveform_picker_regs->delta_f2_f0 = 0x17c00;        // max 5 bytes
    waveform_picker_regs->nb_burst_available = 0x180;   // max 3 bytes, size of the buffer in burst (1 burst = 16 x 4 octets)
    waveform_picker_regs->nb_snapshot_param = 0x7ff;    // max 3 octets, 2048 - 1
    waveform_picker_regs->status = 0x00;                //
#endif
}

//*****************
// local parameters
void set_local_sbm1_nb_cwf_max()
{
    // (2 snapshots of 2048 points per seconds) * (period of the NORM snashots)
    param_local.local_sbm1_nb_cwf_max = 2 * (
                parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
                + parameter_dump_packet.sy_lfr_n_swf_p[1]
             );
}

void set_local_sbm2_nb_cwf_max()
{
    // (period of the NORM snashots) / (8 seconds per snapshot at f2 = 256 Hz)
    param_local.local_sbm2_nb_cwf_max = (
                parameter_dump_packet.sy_lfr_n_swf_p[0] * 256
                + parameter_dump_packet.sy_lfr_n_swf_p[1]
             )/ 8;
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

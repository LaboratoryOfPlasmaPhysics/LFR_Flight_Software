#include <wf_handler.h>

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    unsigned char lfrMode;
    lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4;

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
            if ( (waveform_picker_regs->burst_enable & 0x7) == 0x0 ){// if no channel is enable
                if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveform_isr *** Error sending event to DUMB\n");
                }
            }
            else {
                if ( (waveform_picker_regs->status & 0x7) == 0x7 ){         // f2 f1 and f0 are full
                    waveform_picker_regs->burst_enable = 0x00;
                    if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                        if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 ) != RTEMS_SUCCESSFUL) {
                            PRINTF("in waveform_isr *** Error sending event to DUMB\n");
                        }
                    }
                }
            }
#endif
            break;

        //******
        // BURST
        case(LFR_MODE_BURST):
            break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
#ifdef GSA
            PRINTF("in waveform_isr *** unexpected waveform picker interruption\n")
#else
            if (waveform_picker_regs->burst_enable == 0x22) {
                if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
                    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1_bis);
                }
                else {
                    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);
                }
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM1 ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
            }
            waveform_picker_regs->status = 0x00;
#endif
            break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
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
                if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_SBM2 ) != RTEMS_SUCCESSFUL) {
                    PRINTF("in waveforms_isr *** Error sending event to WFRM\n")
                    rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_2 );
                }
            }
            waveform_picker_regs->status = 0x00;
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

    switch(lfrMode)
    {
        //********
        // STANDBY
        case(LFR_MODE_STANDBY):
            break;

        //******
        // NORMAL
        case(LFR_MODE_NORMAL):
            if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_MODE_NORMAL ) != RTEMS_SUCCESSFUL) {
                PRINTF("ERR *** in waveforms_isr *** error sending event to WFRM\n");
            }
            break;

        //******
        // BURST
        case(LFR_MODE_BURST):
            break;

        //*****
        // SBM1
        case(LFR_MODE_SBM1):
            break;

        //*****
        // SBM2
        case(LFR_MODE_SBM2):
            break;

        //********
        // DEFAULT
        default:
            break;
    }
}

rtems_task wfrm_task(rtems_task_argument argument) //used with the waveform picker VHDL IP
{
    spw_ioctl_pkt_send spw_ioctl_send;
    rtems_event_set event_out;
    ExtendedTMHeader_t header;

    header.targetLogicalAddress = CCSDS_DESTINATION_ID;
    header.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header.reserved = 0x00;
    header.userApplication = CCSDS_USER_APP;
    header.packetID[0] = 0x0c;
    header.packetID[1] = 0xcc;
    header.packetSequenceControl[0] = 0x00;
    header.packetSequenceControl[1] = 0x00;
    header.packetLength[0] = 0x00;
    header.packetLength[1] = 0x00;
    header.dataFieldHeader[0] = 0x10;
    header.dataFieldHeader[1] = TM_TYPE_LFR_SCIENCE; // service type
    header.dataFieldHeader[2] = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header.dataFieldHeader[3] = CCSDS_DESTINATION_ID_GROUND;

    header.auxiliaryHeader[0] = 0x00;
    header.auxiliaryHeader[1] = 0x1f;
    header.auxiliaryHeader[2] = 0x07; // PKT_CNT
    header.auxiliaryHeader[3] = 0x00; // PKT_NR
    header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
    header.auxiliaryHeader[5] = 0x00; // BLK_NR LSB

    // BUILD THE PACKET HEADER
    spw_ioctl_send.hlen = TM_HEADER_LEN + 4 + 6; // + 4 is for the protocole extra header, + 6 is for the auxiliary header
    spw_ioctl_send.hdr = (char*) &header;

    init_waveforms();

    PRINTF("in WFRM ***\n")

    while(1){
        // wait for an RTEMS_EVENT
        rtems_event_receive(RTEMS_EVENT_0 | RTEMS_EVENT_1 | RTEMS_EVENT_2 | RTEMS_EVENT_3 | RTEMS_EVENT_4,
                            RTEMS_WAIT | RTEMS_EVENT_ANY, RTEMS_NO_TIMEOUT, &event_out);
        header.dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
        header.dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
        header.dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
        header.dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
        header.dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
        header.dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);

        switch(event_out)
        {
            //********
            // STANDBY
            case(RTEMS_EVENT_MODE_STANDBY):
                break;

            //*******
            // NORMAL
            case(RTEMS_EVENT_MODE_NORMAL):
                //***************
                // send snapshots
                // F0
                send_waveform( &header, wf_snap_f0, SID_NORM_SWF_F0, &spw_ioctl_send);
                // F1
                send_waveform( &header, wf_snap_f1, SID_NORM_SWF_F1, &spw_ioctl_send);
                // F2
                send_waveform( &header, wf_snap_f2, SID_NORM_SWF_F2, &spw_ioctl_send);
#ifdef GSA
                // irq processed, reset the related register of the timer unit
                gptimer_regs->timer[2].ctrl = gptimer_regs->timer[2].ctrl | 0x00000010;
#else
                // irq processed, reset the related register of the waveform picker
                waveform_picker_regs->status = 0x00;
                waveform_picker_regs->burst_enable = 0x07;
#endif
                break;

            //*****
            // SBM1
            case(RTEMS_EVENT_MODE_SBM1):
                // F1
                if (waveform_picker_regs->addr_data_f1 == (int) wf_snap_f1) {
                   send_waveform( &header, wf_snap_f1_bis, SID_NORM_SWF_F1, &spw_ioctl_send);
                }
                else {
                    send_waveform( &header, wf_snap_f1, SID_NORM_SWF_F1, &spw_ioctl_send);
                }
                break;

            //*****
            // SBM2
            case(RTEMS_EVENT_MODE_SBM2):
                // F2
                if (waveform_picker_regs->addr_data_f2 == (int) wf_snap_f2) {
                    send_waveform( &header, wf_snap_f2_bis, SID_NORM_SWF_F2, &spw_ioctl_send);
                }
                else {
                    send_waveform( &header, wf_snap_f2, SID_NORM_SWF_F2, &spw_ioctl_send);
                }
                break;

            //********
            // DEFAULT
            default:
                break;
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

void init_waveform_header( ExtendedTMHeader_t * header, unsigned int sid )
{

    header->targetLogicalAddress = CCSDS_DESTINATION_ID;
    header->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    header->reserved = 0x00;
    header->userApplication = CCSDS_USER_APP;
    header->packetID[0] = 0x0c;
    header->packetID[1] = 0xcc;
    header->packetSequenceControl[0] = 0x00;
    header->packetSequenceControl[1] = 0x00;
    header->packetLength[0] = 0x00;
    header->packetLength[1] = 0x00;
    header->dataFieldHeader[0] = 0x10;
    header->dataFieldHeader[1] = TM_TYPE_LFR_SCIENCE; // service type
    header->dataFieldHeader[2] = TM_SUBTYPE_LFR_SCIENCE; // service subtype
    header->dataFieldHeader[3] = CCSDS_DESTINATION_ID_GROUND;

    header->auxiliaryHeader[0] = sid;
    header->auxiliaryHeader[1] = 0x1f;
    header->auxiliaryHeader[2] = 0x07; // PKT_CNT
    header->auxiliaryHeader[3] = 0x00; // PKT_NR
    header->auxiliaryHeader[4] = 0x00; // BLK_NR MSB
    header->auxiliaryHeader[5] = 0x00; // BLK_NR LSB
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

void send_waveform( ExtendedTMHeader_t *header, volatile int *waveform,
                    unsigned int sid, spw_ioctl_pkt_send *spw_ioctl_send)
{
    unsigned int i = 0;
    unsigned int length = 0;
    rtems_status_code status;

    for (i=0; i<7; i++) // send waveform
    {
        header->auxiliaryHeader[3] = (unsigned char) i+1; // PKT_NR
        // BUILD THE DATA
        if (i==6) {
            spw_ioctl_send->dlen = 8 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_NORM_SWF_8;
            header->auxiliaryHeader[4] = 0x00; // BLK_NR MSB
            header->auxiliaryHeader[5] = 0x08; // BLK_NR LSB
        }
        else {
            spw_ioctl_send->dlen = 340 * NB_BYTES_SWF_BLK;
            length = TM_LEN_SCI_NORM_SWF_340;
            header->auxiliaryHeader[4] = 0x01; // BLK_NR MSB
            header->auxiliaryHeader[5] = 0x54; // BLK_NR LSB
        }
        if (sid == SID_NORM_SWF_F0) {
             spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) + (1 * TIME_OFFSET) ];
        }
        else if (sid == SID_NORM_SWF_F1) {
             spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) + (1 * TIME_OFFSET) ];
        }
        else if (sid == SID_NORM_SWF_F2) {
             spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) + (1 * TIME_OFFSET) ];
        }
        else {
             spw_ioctl_send->data = (char*) &waveform[ (i * 340 * NB_WORDS_SWF_BLK) ];
        }
        // BUILD THE HEADER
        header->packetLength[0] = (unsigned char) (length>>8);
        header->packetLength[1] = (unsigned char) (length);
        header->auxiliaryHeader[0] = sid; // SID
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

int build_value(int value1, int value0)
{
    int aux  = 0;
    int aux1 = 0;
    int aux0 = 0;
    int value1_aux = 0;
    int value0_aux = 0;

    value1_aux = value1;
    value0_aux = value0;

    //******
    // B3 B2
    if (value1_aux > 8191) value1_aux = 8191;
    if (value1_aux < -8192) value1_aux = -8192;
    aux1 = ( (int) ( ( (unsigned char) (value1_aux / 256 ) ) << 8 ) )
         + ( (int) (   (unsigned char) (value1_aux       ) )      );

    //******
    // B1 B0
    if (value0_aux > 8191) value0_aux = 8191;
    if (value0_aux < -8192) value0_aux = -8192;
    aux0 = ( (int) ( ( (unsigned char) (value0_aux / 256) ) << 8 ) )
         + ( (int) (   (unsigned char) (value0_aux      ) )      );

    aux = (aux1 << 16) + aux0;

    return aux;
}

void init_waveform_picker_regs()
{
    set_data_shaping_parameters(param_common.sy_lfr_common1);
    waveform_picker_regs->burst_enable = 0x00;              // burst f2, f1, f0     enable f3, f2, f1, f0
    waveform_picker_regs->addr_data_f0 = (int) (wf_snap_f0);  //
    waveform_picker_regs->addr_data_f1 = (int) (wf_snap_f1);  //
    waveform_picker_regs->addr_data_f2 = (int) (wf_snap_f2);  //
    waveform_picker_regs->addr_data_f3 = (int) (wf_cont_f3);  //
    waveform_picker_regs->status = 0x00;                    //
    waveform_picker_regs->delta_snapshot = 0x5;             // max 2 bytes
    waveform_picker_regs->delta_f2_f1 = 0xffff;             // max 4 bytes
    waveform_picker_regs->delta_f2_f0 = 0x17c00;            // max 5 bytes
    waveform_picker_regs->nb_burst_available = 0x180;       // max 3 bytes, size of the buffer in burst (1 burst = 16 x 4 octets)
    waveform_picker_regs->nb_snapshot_param = 0x7ff;        // max 3 octets, 2048 - 1
    //waveform_picker_regs->delta_snapshot = 0x2;         // max 2 bytes, = period / 2
    //waveform_picker_regs->delta_f2_f1 = 0x2d00;         // max 4 bytes
    //waveform_picker_regs->delta_f2_f0 = 0x2f80;         // max 5 bytes
    //waveform_picker_regs->nb_burst_available = 0x30;    // max 3 bytes, size of the buffer in burst (1 burst = 16 x 4 octets)
    //waveform_picker_regs->nb_snapshot_param = 0xff;     // max 3 octets, 256 - 1
}

void set_data_shaping_parameters(unsigned char parameters)
{
    // get the parameters for the data shaping [BW SP0 SP1 R0 R1] in sy_lfr_common1 and configure the register
    // waveform picker : [R1 R0 SP1 SP0 BW]
    waveform_picker_regs->data_shaping =
              ( (parameters & 0x10) >> 4 )     // BW
            + ( (parameters & 0x08) >> 2 )     // SP0
            + ( (parameters & 0x04)      )     // SP1
            + ( (parameters & 0x02) << 2 )     // R0
            + ( (parameters & 0x01) << 4 );    // R1
}

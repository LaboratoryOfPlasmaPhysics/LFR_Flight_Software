#include <wf_handler.h>

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        printf("In waveforms_isr *** Error sending event to WFRM\n");
}

rtems_task wfrm_task(rtems_task_argument argument)
{
    unsigned int length;
    unsigned int i = 0;
    spw_ioctl_pkt_send spw_ioctl_send;
    rtems_event_set event_out;
    gptimer_regs_t *gptimer_regs;
    gptimer_regs = (gptimer_regs_t *) REGS_ADDR_GPTIMER;
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
    header.dataFieldHeader[1] = 0x15; // service type
    header.dataFieldHeader[2] = 0x03; // service subtype
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

    PRINTF("in WFRM *** \n")

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        header.dataFieldHeader[4] = (unsigned char) (time_management_regs->coarse_time>>24);
        header.dataFieldHeader[5] = (unsigned char) (time_management_regs->coarse_time>>16);
        header.dataFieldHeader[6] = (unsigned char) (time_management_regs->coarse_time>>8);
        header.dataFieldHeader[7] = (unsigned char) (time_management_regs->coarse_time);
        header.dataFieldHeader[8] = (unsigned char) (time_management_regs->fine_time>>8);
        header.dataFieldHeader[9] = (unsigned char) (time_management_regs->fine_time);
        for (i=0; i<7; i++) // send F0
        {
            header.auxiliaryHeader[3] = (unsigned char) i+1; // PKT_NR
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
                header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x08; // BLK_NR LSB
            }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                header.auxiliaryHeader[4] = 0x01; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x54; // BLK_NR LSB
            }
            spw_ioctl_send.data = (char*) &wf_snap_f0[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F0; // SID
            // SEND PACKET
            write_spw(&spw_ioctl_send);
        }
        for (i=0; i<7; i++) // send F1
        {
            header.auxiliaryHeader[3] = (unsigned char) i+1; // PKT_NR
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
                header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x08; // BLK_NR LSB
            }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                header.auxiliaryHeader[4] = 0x01; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x54; // BLK_NR LSB
            }
            spw_ioctl_send.data = (char*) &wf_snap_f1[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F1; // SID
            // SEND PACKET
            write_spw(&spw_ioctl_send);
        }
        for (i=0; i<7; i++) // send F2
        {
            header.auxiliaryHeader[3] = (unsigned char) i+1; // PKT_NR
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
                header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x08; // BLK_NR LSB
            }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                header.auxiliaryHeader[4] = 0x01; // BLK_NR MSB
                header.auxiliaryHeader[5] = 0x54; // BLK_NR LSB
            }
            spw_ioctl_send.data = (char*) &wf_snap_f2[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F2; // SID
            // SEND PACKET
            write_spw(&spw_ioctl_send);
        }
        // irq processed, reset the related register of the timer unit
        gptimer_regs->timer[2].ctrl = gptimer_regs->timer[2].ctrl | 0x00000010;
    }
}

//******************
// general functions
void init_waveforms()
{
    int i = 0;

    for (i=0; i< NB_SAMPLES_PER_SNAPSHOT; i++)
    {
        wf_snap_f0[ i* NB_WORDS_SWF_BLK + 0 ] = buil_value(   i, 2*i );   // v  and e1
        wf_snap_f0[ i* NB_WORDS_SWF_BLK + 1 ] = buil_value( 3*i, 4*i );   // e2 and b1
        wf_snap_f0[ i* NB_WORDS_SWF_BLK + 2 ] = buil_value(   i, 2*i );   // b2 and b3
        wf_snap_f1[ i* NB_WORDS_SWF_BLK + 0 ] = buil_value(   i, 2*i );   // v  and 1
        wf_snap_f1[ i* NB_WORDS_SWF_BLK + 1 ] = buil_value(   i, 2*i );   // e2 and b1
        wf_snap_f1[ i* NB_WORDS_SWF_BLK + 2 ] = buil_value(   i, 2*i );   // b2 and b3
        wf_snap_f2[ i* NB_WORDS_SWF_BLK + 0 ] = buil_value(   i, 2*i );   // v  and 1
        wf_snap_f2[ i* NB_WORDS_SWF_BLK + 1 ] = buil_value(   i, 2*i );   // e2 and b1
        wf_snap_f2[ i* NB_WORDS_SWF_BLK + 2 ] = buil_value(   i, 2*i );   // b2 and b3
        wf_cont_f3[ i* NB_WORDS_SWF_BLK + 0 ] = buil_value(   i, 2*i );   // v  and 1
        wf_cont_f3[ i* NB_WORDS_SWF_BLK + 1 ] = buil_value(   i, 2*i );   // e2 and b1
        wf_cont_f3[ i* NB_WORDS_SWF_BLK + 2 ] = buil_value(   i, 2*i );   // b2 and b3
    }
}

int buil_value(int value1, int value0)
{
    int aux  = 0;
    int aux1 = 0;
    int aux0 = 0;
    //******
    // B3 B2
    aux1 = ( (int) ( (char) (value1 >> 8) ) << 8 )
         + ( (int) ( (char) (value1     ) )      );

    //******
    // B1 B0
    aux0 = ( (int) ( (char) (value0 >> 8) ) << 8  )
         + ( (int) ( (char) (value0     ) )       );

    aux = (aux1 << 16) + aux0;

    return aux;
}




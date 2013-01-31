#include <wf_handler.h>

rtems_isr waveforms_isr( rtems_vector_number vector )
{
    if (rtems_event_send( Task_id[TASKID_WFRM], RTEMS_EVENT_0 ) != RTEMS_SUCCESSFUL)
        printf("In spectral_matrices_isr *** Error sending event to WFRM\n");
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
    header.dataFieldHeader[3] = CCSDS_DESTINATION_ID;
    header.dataFieldHeader[4] = 0xaa;
    header.dataFieldHeader[5] = 0xbb;
    header.dataFieldHeader[6] = 0xcc;
    header.dataFieldHeader[7] = 0xdd;
    header.dataFieldHeader[8] = 0xee;
    header.dataFieldHeader[9] = 0xff;

    header.auxiliaryHeader[0] = 0x00;
    header.auxiliaryHeader[1] = 0x1f;
    header.auxiliaryHeader[2] = 0x07; // PKT_CNT
    header.auxiliaryHeader[3] = 0x00; // PKT_NR
    header.auxiliaryHeader[4] = 0x00; // BLK_NR MSB
    header.auxiliaryHeader[5] = 0x00; // BLK_NR LSB

    // BUILD THE PACKET HEADER
    spw_ioctl_send.hlen = TM_HEADER_LEN + 4 + 6; // + 4 is for the protocole extra header, + 6 is for the auxiliary header
    spw_ioctl_send.hdr = (char*) &header;

    while(1){
        rtems_event_receive(RTEMS_EVENT_0, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an RTEMS_EVENT0
        for (i=0; i<7; i++) // send F0
        {
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
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
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
            }
            spw_ioctl_send.data = (char*) &wf_snap_f1[i * 340 * NB_BYTES_SWF_BLK];
            // BUILD THE HEADER
            header.packetLength[0] = (unsigned char) (length>>8);
            header.packetLength[1] = (unsigned char) (length);
            header.auxiliaryHeader[0] = SID_NORM_SWF_F1; // SID
            // SEND PACKET
            write_spw(&spw_ioctl_send);
        }
        for (i=0; i<7; i++) // send F0
        {
            // BUILD THE DATA
            if (i==6) {
                spw_ioctl_send.dlen = 8 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_340;
                }
            else {
                spw_ioctl_send.dlen = 340 * NB_BYTES_SWF_BLK;
                length = TM_LEN_SCI_NORM_SWF_8;
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

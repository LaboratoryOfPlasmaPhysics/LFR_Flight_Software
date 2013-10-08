#include "fsw_spacewire.h"

char *lstates[6] = {"Error-reset",
                    "Error-wait",
                    "Ready",
                    "Started",
                    "Connecting",
                    "Run"
};

// RTEMS TASK
rtems_task spiq_task(rtems_task_argument unused)
{
    rtems_event_set event_out;
    rtems_status_code status;
    unsigned char lfrMode;

    while(true){
        BOOT_PRINTF("in SPIQ *** Waiting for SPW_LINKERR_EVENT\n")
        rtems_event_receive(SPW_LINKERR_EVENT, RTEMS_WAIT, RTEMS_NO_TIMEOUT, &event_out); // wait for an SPW_LINKERR_EVENT

        lfrMode = (housekeeping_packet.lfr_status_word[0] & 0xf0) >> 4; // get the current mode

        status = spacewire_wait_for_link();

        if (status != RTEMS_SUCCESSFUL)
        {
            //****************
            // STOP THE SYSTEM
            spacewire_compute_stats_offsets();
            stop_current_mode();
            if (rtems_task_suspend(Task_id[TASKID_RECV])!=RTEMS_SUCCESSFUL) {   // suspend RECV task
                PRINTF("in SPIQ *** Error suspending RECV Task\n")
            }
            if (rtems_task_suspend(Task_id[TASKID_HOUS])!=RTEMS_SUCCESSFUL) {   // suspend HOUS task
                PRINTF("in SPIQ *** Error suspending HOUS Task\n")
            }

            //***************************
            // RESTART THE SPACEWIRE LINK
            spacewire_configure_link();

            //*******************
            // RESTART THE SYSTEM
            //ioctl(fdSPW, SPACEWIRE_IOCTRL_CLR_STATISTICS);   // clear statistics
            status = rtems_task_restart( Task_id[TASKID_HOUS], 1 );
            if (status != RTEMS_SUCCESSFUL) {
                PRINTF1("in SPIQ *** Error restarting HOUS Task *** code %d\n", status)
            }
            status = rtems_task_restart(Task_id[TASKID_RECV], 1);
            if ( status != RTEMS_SUCCESSFUL) {
                PRINTF("in SPIQ *** Error restarting RECV Task\n")
            }
            enter_mode(lfrMode, NULL); // enter the mode that was running before the SpaceWire interruption
        }
    }
}

int spacewire_configure_link( void )
{
    rtems_status_code status;

    close(fdSPW); // close the device if it is already open
    BOOT_PRINTF("OK  *** in configure_spw_link *** try to open "GRSPW_DEVICE_NAME"\n")
    fdSPW = open(GRSPW_DEVICE_NAME, O_RDWR); // open the device. the open call reset the hardware
    if ( fdSPW<0 ) {
        PRINTF("ERR *** in configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
    }

    while(ioctl(fdSPW, SPACEWIRE_IOCTRL_START, -1) != RTEMS_SUCCESSFUL){
        PRINTF(".")
        fflush( stdout );
        close( fdSPW ); // close the device
        fdSPW = open( GRSPW_DEVICE_NAME, O_RDWR ); // open the device. the open call reset the hardware
        if (fdSPW<0) {
            PRINTF("ERR *** In configure_spw_link *** Error opening"GRSPW_DEVICE_NAME"\n")
        }
        rtems_task_wake_after(100);
    }

    BOOT_PRINTF("OK  *** In configure_spw_link *** "GRSPW_DEVICE_NAME" opened and started successfully\n")

    spacewire_set_NP(1, REGS_ADDR_GRSPW); // [N]o [P]ort force
    spacewire_set_RE(1, REGS_ADDR_GRSPW); // [R]MAP [E]nable, the dedicated call seems to  break the no port force configuration

    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_RXBLOCK, 1);              // sets the blocking mode for reception
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_RXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_EVENT_ID, Task_id[TASKID_SPIQ]); // sets the task ID to which an event is sent when a
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_EVENT_ID\n") // link-error interrupt occurs
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_DISABLE_ERR, 0);          // automatic link-disabling due to link-error interrupts
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_DISABLE_ERR\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ, 1);         // sets the link-error interrupt bit
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_LINK_ERR_IRQ\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK, 0);             // transmission blocks
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL, 0);      // transmission blocks on full
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TXBLOCK_ON_FULL\n")
    //
    status = ioctl(fdSPW, SPACEWIRE_IOCTRL_SET_TCODE_CTRL, 0x0909); // [Time Rx : Time Tx : Link error : Tick-out IRQ]
    if (status!=RTEMS_SUCCESSFUL) PRINTF("in SPIQ *** Error SPACEWIRE_IOCTRL_SET_TCODE_CTRL,\n")

    BOOT_PRINTF("OK  *** in configure_spw_link *** "GRSPW_DEVICE_NAME" configured successfully\n")

    return RTEMS_SUCCESSFUL;
}

int spacewire_wait_for_link(void)
{
    unsigned int i;
    int linkStatus;
    rtems_status_code status = RTEMS_UNSATISFIED;

    for(i = 0; i< 10; i++){
        PRINTF(".")
        fflush( stdout );
        ioctl(fdSPW, SPACEWIRE_IOCTRL_GET_LINK_STATUS, &linkStatus);   // get the link status
        PRINTF1("in spacewire_wait_for_link *** link status is: %s\n", lstates[linkStatus])
        if ( linkStatus == 5) {
            PRINTF("in spacewire_wait_for_link *** link is running\n")
            status = RTEMS_SUCCESSFUL;
            break;
        }
        rtems_task_wake_after(100);
    }

    return status;
}

void spacewire_set_NP(unsigned char val, unsigned int regAddr) // [N]o [P]ort force
{
    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1) {
        *spwptr = *spwptr | 0x00100000; // [NP] set the No port force bit
    }
    if (val== 0) {
        *spwptr = *spwptr & 0xffdfffff;
    }
}

void spacewire_set_RE(unsigned char val, unsigned int regAddr) // [R]MAP [E]nable
{
    unsigned int *spwptr = (unsigned int*) regAddr;

    if (val == 1)
    {
        *spwptr = *spwptr | 0x00010000; // [RE] set the RMAP Enable bit
    }
    if (val== 0)
    {
        *spwptr = *spwptr & 0xfffdffff;
    }
}

void spacewire_compute_stats_offsets()
{
    spw_stats spacewire_stats_grspw;
    rtems_status_code status;

    status = ioctl( fdSPW, SPACEWIRE_IOCTRL_GET_STATISTICS, &spacewire_stats_grspw );

    spacewire_stats_backup.packets_received = spacewire_stats_grspw.packets_received
            + spacewire_stats.packets_received;
    spacewire_stats_backup.packets_sent = spacewire_stats_grspw.packets_sent
            + spacewire_stats.packets_sent;
    spacewire_stats_backup.parity_err = spacewire_stats_grspw.parity_err
            + spacewire_stats.parity_err;
    spacewire_stats_backup.disconnect_err = spacewire_stats_grspw.disconnect_err
            + spacewire_stats.disconnect_err;
    spacewire_stats_backup.escape_err = spacewire_stats_grspw.escape_err
            + spacewire_stats.escape_err;
    spacewire_stats_backup.credit_err = spacewire_stats_grspw.credit_err
            + spacewire_stats.credit_err;
    spacewire_stats_backup.write_sync_err = spacewire_stats_grspw.write_sync_err
            + spacewire_stats.write_sync_err;
    spacewire_stats_backup.rx_rmap_header_crc_err = spacewire_stats_grspw.rx_rmap_header_crc_err
            + spacewire_stats.rx_rmap_header_crc_err;
    spacewire_stats_backup.rx_rmap_data_crc_err = spacewire_stats_grspw.rx_rmap_data_crc_err
            + spacewire_stats.rx_rmap_data_crc_err;
    spacewire_stats_backup.early_ep = spacewire_stats_grspw.early_ep
            + spacewire_stats.early_ep;
    spacewire_stats_backup.invalid_address = spacewire_stats_grspw.invalid_address
            + spacewire_stats.invalid_address;
    spacewire_stats_backup.rx_eep_err = spacewire_stats_grspw.rx_eep_err
            + spacewire_stats.rx_eep_err;
    spacewire_stats_backup.rx_truncated = spacewire_stats_grspw.rx_truncated
            + spacewire_stats.rx_truncated;
}

void timecode_irq_handler(void *pDev, void *regs, int minor, unsigned int tc)
{
    //if (rtems_event_send( Task_id[TASKID_DUMB], RTEMS_EVENT_1 ) != RTEMS_SUCCESSFUL) {
    //    printf("In timecode_irq_handler *** Error sending event to DUMB\n");
    //}
}

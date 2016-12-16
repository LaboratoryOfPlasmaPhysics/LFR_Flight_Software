/** This is the RTEMS initialization module.
 *
 * @file
 * @author P. LEROY
 *
 * This module contains two very different information:
 * - specific instructions to configure the compilation of the RTEMS executive
 * - functions related to the fligth softwre initialization, especially the INIT RTEMS task
 *
 */

//*************************
// GPL reminder to be added
//*************************

#include <rtems.h>

/* configuration information */

#define CONFIGURE_INIT

#include <bsp.h> /* for device driver prototypes */

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS 20
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_EXTRA_TASK_STACKS (3 * RTEMS_MINIMUM_STACK_SIZE)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 32
#define CONFIGURE_INIT_TASK_PRIORITY 1 // instead of 100
#define CONFIGURE_INIT_TASK_MODE (RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT)
#define CONFIGURE_INIT_TASK_ATTRIBUTES (RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT)
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 5
#define CONFIGURE_MAXIMUM_TIMERS 5  // STAT (1s), send SWF (0.3s), send CWF3 (1s)
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 5
#ifdef PRINT_STACK_REPORT
    #define CONFIGURE_STACK_CHECKER_ENABLED
#endif

#include <rtems/confdefs.h>

/* If --drvmgr was enabled during the configuration of the RTEMS kernel */
#ifdef RTEMS_DRVMGR_STARTUP
    #ifdef LEON3
    /* Add Timer and UART Driver */
        #ifdef CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
            #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GPTIMER
        #endif
        #ifdef CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
            #define CONFIGURE_DRIVER_AMBAPP_GAISLER_APBUART
        #endif
    #endif
    #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GRSPW   /* GRSPW Driver */
    #include <drvmgr/drvmgr_confdefs.h>
#endif

#include "fsw_init.h"
#include "fsw_config.c"

rtems_id rtems_task_id_updt;
rtems_name rtems_task_name_updt;

rtems_task Init( rtems_task_argument ignored )
{
    /** This is the RTEMS INIT taks, it the first task launched by the system.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The INIT task create and run all other RTEMS tasks.
     *
     */

    unsigned char *vhdlVersion;

    reset_local_time();

    rtems_cpu_usage_reset();

    rtems_status_code status;
    rtems_status_code status_spw;

    struct grgpio_regs_str *grgpio_regs = (struct grgpio_regs_str *) REGS_ADDR_GRGPIO;

    // UART settings
    send_console_outputs_on_apbuart_port();
    set_apbuart_scaler_reload_register(REGS_ADDR_APBUART, APBUART_SCALER_RELOAD_VALUE);
    enable_apbuart_transmitter();
    DEBUG_PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

    PRINTF("\n\n\n\n\n")
    PRINTF("*************************\n")
    PRINTF("** timegen for LFR     **\n")
    PRINTF1("** %d.", SW_VERSION_N1)
    PRINTF1("%d."   , SW_VERSION_N2)
    PRINTF1("%d."   , SW_VERSION_N3)
    PRINTF1("%d             **\n", SW_VERSION_N4)

    vhdlVersion = (unsigned char *) (REGS_ADDR_VHDL_VERSION);
    PRINTF("** VHDL                **\n")
    PRINTF1("** %d.", vhdlVersion[1])
    PRINTF1("%d."   , vhdlVersion[2])
    PRINTF1("%d              **\n", vhdlVersion[3])
    PRINTF("*************************\n")
    PRINTF("\n\n")

    create_names();                             // create all names

    status = create_message_queues();           // create message queues
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** ERR in create_message_queues, code %d", status)
    }

    status = create_all_tasks();                // create all tasks
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** ERR in create_all_tasks, code %d\n", status)
    }

    // **************************
    // <SPACEWIRE INITIALIZATION>
    grspw_timecode_callback = &timecode_irq_handler;

    status_spw = spacewire_open_link();     // (1) open the link
    if ( status_spw != RTEMS_SUCCESSFUL )
    {
        PRINTF1("in INIT *** ERR spacewire_open_link code %d\n", status_spw )
    }

    if ( status_spw == RTEMS_SUCCESSFUL )   // (2) configure the link
    {
        status_spw = spacewire_configure_link( fdSPW );
        if ( status_spw != RTEMS_SUCCESSFUL )
        {
            PRINTF1("in INIT *** ERR spacewire_configure_link code %d\n", status_spw )
        }
    }

    if ( status_spw == RTEMS_SUCCESSFUL)    // (3) start the link
    {
        status_spw = spacewire_start_link( fdSPW );
        if (  status_spw != RTEMS_SUCCESSFUL )
        {
            PRINTF1("in INIT *** ERR spacewire_start_link code %d\n",  status_spw )
        }
    }
    // </SPACEWIRE INITIALIZATION>
    // ***************************

    status = start_all_tasks();     // start all tasks
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** ERR in start_all_tasks, code %d", status)
    }

    // start RECV and SEND *AFTER* SpaceWire Initialization, due to the timeout of the start call during the initialization
    status = start_recv_send_tasks();
    if ( status != RTEMS_SUCCESSFUL )
    {
        PRINTF1("in INIT *** ERR start_recv_send_tasks code %d\n",  status )
    }

    // suspend science tasks, they will be restarted later depending on the mode
    status = suspend_science_tasks();   // suspend science tasks (not done in stop_current_mode if current mode = STANDBY)
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** in suspend_science_tasks *** ERR code: %d\n", status)
    }

    // if the spacewire link is not up then send an event to the SPIQ task for link recovery
    if ( status_spw != RTEMS_SUCCESSFUL )
    {
        status = rtems_event_send( Task_id[TASKID_SPIQ], SPW_LINKERR_EVENT );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1("in INIT *** ERR rtems_event_send to SPIQ code %d\n",  status )
        }
    }

    //*********************************
    // init GPIO for trigger generation
    grgpio_regs->io_port_direction_register =
            grgpio_regs->io_port_direction_register | 0x04; // [0000 0100], 0 = output disabled, 1 = output enabled
    grgpio_regs->io_port_direction_register =
            grgpio_regs->io_port_direction_register | 0x08; // [0000 1000], 0 = output disabled, 1 = output enabled

    BOOT_PRINTF("delete INIT\n")

    status = rtems_task_delete(RTEMS_SELF);

}

void init_local_mode_parameters( void )
{
    /** This function initialize the param_local global variable with default values.
     *
     */

    unsigned int i;

    // LOCAL PARAMETERS

    BOOT_PRINTF1("local_sbm1_nb_cwf_max %d \n", param_local.local_sbm1_nb_cwf_max)
    BOOT_PRINTF1("local_sbm2_nb_cwf_max %d \n", param_local.local_sbm2_nb_cwf_max)
    BOOT_PRINTF1("nb_interrupt_f0_MAX = %d\n", param_local.local_nb_interrupt_f0_MAX)

    // init sequence counters

    for(i = 0; i<SEQ_CNT_NB_DEST_ID; i++)
    {
        sequenceCounters_TC_EXE[i] = 0x00;
    }
    sequenceCounters_SCIENCE_NORMAL_BURST = 0x00;
    sequenceCounters_SCIENCE_SBM1_SBM2 =    0x00;
    sequenceCounterHK =                     TM_PACKET_SEQ_CTRL_STANDALONE << 8;
    sequenceCounterParameterDump =          TM_PACKET_SEQ_CTRL_STANDALONE << 8;
}

void reset_local_time( void )
{
    time_management_regs->ctrl = 0x02;  // software reset, coarse time = 0x80000000
}

void create_names( void ) // create all names for tasks and queues
{
    /** This function creates all RTEMS names used in the software for tasks and queues.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - successful completion
     *
     */

    // task names
    Task_name[TASKID_RECV] = rtems_build_name( 'R', 'E', 'C', 'V' );
    Task_name[TASKID_ACTN] = rtems_build_name( 'A', 'C', 'T', 'N' );
    Task_name[TASKID_SPIQ] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[TASKID_STAT] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[TASKID_DUMB] = rtems_build_name( 'D', 'U', 'M', 'B' );
    Task_name[TASKID_SEND] = rtems_build_name( 'S', 'E', 'N', 'D' );
    Task_name[TASKID_WTDG] = rtems_build_name( 'W', 'T', 'D', 'G' );

    rtems_task_name_updt   = rtems_build_name( 'U', 'P', 'D', 'T' );

    misc_name[QUEUE_RECV] = rtems_build_name( 'Q', '_', 'R', 'V' );
    misc_name[QUEUE_SEND] = rtems_build_name( 'Q', '_', 'S', 'D' );

}

int create_all_tasks( void ) // create all tasks which run in the software
{
    /** This function creates all RTEMS tasks used in the software.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - task created successfully
     * - RTEMS_INVALID_ADDRESS - id is NULL
     * - RTEMS_INVALID_NAME - invalid task name
     * - RTEMS_INVALID_PRIORITY - invalid task priority
     * - RTEMS_MP_NOT_CONFIGURED - multiprocessing not configured
     * - RTEMS_TOO_MANY - too many tasks created
     * - RTEMS_UNSATISFIED - not enough memory for stack/FP context
     * - RTEMS_TOO_MANY - too many global objects
     *
     */

    rtems_status_code status;

    //**********
    // SPACEWIRE
    // RECV
    status = rtems_task_create(
        Task_name[TASKID_RECV], TASK_PRIORITY_RECV, RTEMS_MINIMUM_STACK_SIZE,
        RTEMS_DEFAULT_MODES,
        RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_RECV]
    );
    if (status == RTEMS_SUCCESSFUL) // SEND
    {
        status = rtems_task_create(
            Task_name[TASKID_SEND], TASK_PRIORITY_SEND, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SEND]
        );
    }
    if (status == RTEMS_SUCCESSFUL) // WTDG
    {
        status = rtems_task_create(
            Task_name[TASKID_WTDG], TASK_PRIORITY_WTDG, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_WTDG]
        );
    }
    if (status == RTEMS_SUCCESSFUL) // ACTN
    {
        status = rtems_task_create(
            Task_name[TASKID_ACTN], TASK_PRIORITY_ACTN, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_ACTN]
        );
    }
    if (status == RTEMS_SUCCESSFUL) // SPIQ
    {
        status = rtems_task_create(
            Task_name[TASKID_SPIQ], TASK_PRIORITY_SPIQ, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SPIQ]
        );
    }

    //*****
    // MISC
    if (status == RTEMS_SUCCESSFUL) // STAT
    {
        status = rtems_task_create(
            Task_name[TASKID_STAT], TASK_PRIORITY_STAT, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_STAT]
        );
    }
    if (status == RTEMS_SUCCESSFUL) // DUMB
    {
        status = rtems_task_create(
            Task_name[TASKID_DUMB], TASK_PRIORITY_DUMB, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_DUMB]
        );
    }
    if (status == RTEMS_SUCCESSFUL) // UPDT
    {
        status = rtems_task_create(
            rtems_task_name_updt, TASK_PRIORITY_HOUS, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES, &rtems_task_id_updt
        );
    }

    return status;
}

int start_recv_send_tasks( void )
{
    rtems_status_code status;

    status = rtems_task_start( Task_id[TASKID_RECV], recv_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        BOOT_PRINTF("in INIT *** Error starting TASK_RECV\n")
    }

    if (status == RTEMS_SUCCESSFUL)     // SEND
    {
        status = rtems_task_start( Task_id[TASKID_SEND], send_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_SEND\n")
        }
    }

    return status;
}

int start_all_tasks( void ) // start all tasks except SEND RECV and HOUS
{
    /** This function starts all RTEMS tasks used in the software.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - ask started successfully
     * - RTEMS_INVALID_ADDRESS - invalid task entry point
     * - RTEMS_INVALID_ID - invalid task id
     * - RTEMS_INCORRECT_STATE - task not in the dormant state
     * - RTEMS_ILLEGAL_ON_REMOTE_OBJECT - cannot start remote task
     *
     */
    // starts all the tasks fot eh flight software

    rtems_status_code status;

    //**********
    // SPACEWIRE
    status = rtems_task_start( Task_id[TASKID_SPIQ], spiq_task, 1 );
    if (status!=RTEMS_SUCCESSFUL) {
        BOOT_PRINTF("in INIT *** Error starting TASK_SPIQ\n")
    }

    if (status == RTEMS_SUCCESSFUL)     // WTDG
    {
        status = rtems_task_start( Task_id[TASKID_WTDG], wtdg_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_WTDG\n")
        }
    }

    if (status == RTEMS_SUCCESSFUL)     // ACTN
    {
        status = rtems_task_start( Task_id[TASKID_ACTN], actn_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_ACTN\n")
        }
    }

    //*****
    // MISC
    if (status == RTEMS_SUCCESSFUL)     // DUMB
    {
        status = rtems_task_start( Task_id[TASKID_DUMB], dumb_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_DUMB\n")
        }
    }
    if (status == RTEMS_SUCCESSFUL)     // STAT
    {
        status = rtems_task_start( Task_id[TASKID_STAT], stat_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_STAT\n")
        }
    }
    if (status == RTEMS_SUCCESSFUL)     // UPDT
    {
        status = rtems_task_start( rtems_task_id_updt, updt_task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_UPDT\n")
        }
    }

    return status;
}

rtems_status_code create_message_queues( void ) // create the two message queues used in the software
{
    rtems_status_code status_recv;
    rtems_status_code status_send;

    rtems_status_code ret;
    rtems_id queue_id;

    ret = LFR_SUCCESSFUL;

    //****************************************
    // create the queue for handling valid TCs
    status_recv = rtems_message_queue_create( misc_name[QUEUE_RECV],
                                              MSG_QUEUE_COUNT_RECV, CCSDS_TC_PKT_MAX_SIZE,
                                         RTEMS_FIFO | RTEMS_LOCAL, &queue_id );
    if ( status_recv != RTEMS_SUCCESSFUL ) {
        PRINTF1("in create_message_queues *** ERR creating QUEU queue, %d\n", status_recv)
    }

    //************************************************
    // create the queue for handling TM packet sending
    status_send = rtems_message_queue_create( misc_name[QUEUE_SEND],
                                              MSG_QUEUE_COUNT_SEND, MSG_QUEUE_SIZE_SEND,
                                      RTEMS_FIFO | RTEMS_LOCAL, &queue_id );
    if ( status_send != RTEMS_SUCCESSFUL ) {
        PRINTF1("in create_message_queues *** ERR creating PKTS queue, %d\n", status_send)
    }

    if ( status_recv != RTEMS_SUCCESSFUL )
    {
        ret = status_recv;
    }
    else if( status_send != RTEMS_SUCCESSFUL )
    {
        ret = status_send;
    }

    return ret;
}

rtems_status_code get_message_queue_id_send( rtems_id *queue_id )
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name( 'Q', '_', 'S', 'D' );

    status =  rtems_message_queue_ident( queue_name, 0, queue_id );

    return status;
}

rtems_status_code get_message_queue_id_recv( rtems_id *queue_id )
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name( 'Q', '_', 'R', 'V' );

    status =  rtems_message_queue_ident( queue_name, 0, queue_id );

    return status;
}

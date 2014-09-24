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
#define CONFIGURE_MAXIMUM_DRIVERS 16
#define CONFIGURE_MAXIMUM_PERIODS 5
#define CONFIGURE_MAXIMUM_TIMERS 5  // STAT (1s), send SWF (0.3s), send CWF3 (1s)
#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES 2
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

#include "timegen_init.h"
#include "fsw_config.c"

rtems_task Init( rtems_task_argument ignored )
{
    /** This is the RTEMS INIT taks, it the first task launched by the system.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The INIT task create and run all other RTEMS tasks.
     *
     */

    rtems_status_code status;
    rtems_status_code status_spw;

//    initCoarseTime();

    // UART settings
    send_console_outputs_on_apbuart_port();
    set_apbuart_scaler_reload_register(REGS_ADDR_APBUART, APBUART_SCALER_RELOAD_VALUE);
    enable_apbuart_transmitter();
    DEBUG_PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n")

    PRINTF("\n\n\n\n\n")
    PRINTF("*************************\n")
    PRINTF("** Time Generator      **\n")
    PRINTF1("** %d.", SW_VERSION_N1)
    PRINTF1("%d.", SW_VERSION_N2)
    PRINTF1("%d.", SW_VERSION_N3)
    PRINTF1("%d             **\n", SW_VERSION_N4)
    PRINTF("*************************\n")
    PRINTF("\n\n")

//    init_local_mode_parameters();
//    init_housekeeping_parameters();

//    updateLFRCurrentMode();

//    BOOT_PRINTF1("in INIT *** lfrCurrentMode is %d\n", lfrCurrentMode)

    create_names();                             // create all names

    status = create_message_queues();           // create message queues
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** ERR in create_message_queues, code %d", status)
    }

    status = create_all_tasks();                // create all tasks
    if (status != RTEMS_SUCCESSFUL)
    {
        PRINTF1("in INIT *** ERR in create_all_tasks, code %d", status)
    }

    // **************************
    // <SPACEWIRE INITIALIZATION>
    grspw_timecode_callback = &timegen_timecode_irq_handler;

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

    // if the spacewire link is not up then send an event to the SPIQ task for link recovery
    if ( status_spw != RTEMS_SUCCESSFUL )
    {
        status = rtems_event_send( Task_id[TASKID_SPIQ], SPW_LINKERR_EVENT );
        if ( status != RTEMS_SUCCESSFUL ) {
            PRINTF1("in INIT *** ERR rtems_event_send to SPIQ code %d\n",  status )
        }
    }

    BOOT_PRINTF("delete INIT\n")

    status = rtems_task_delete(RTEMS_SELF);

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
    Task_name[TASKID_SPIQ] = rtems_build_name( 'S', 'P', 'I', 'Q' );
    Task_name[TASKID_STAT] = rtems_build_name( 'S', 'T', 'A', 'T' );
    Task_name[TASKID_DUMB] = rtems_build_name( 'D', 'U', 'M', 'B' );
    Task_name[TASKID_SEND] = rtems_build_name( 'S', 'E', 'N', 'D' );
    Task_name[TASKID_WTDG] = rtems_build_name( 'W', 'T', 'D', 'G' );

    // TIMEGEN
    rtems_name_updt = rtems_build_name( 'U', 'P', 'D', 'T' );
    rtems_name_act_ = rtems_build_name( 'A', 'C', 'T', '_' );

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
    if (status == RTEMS_SUCCESSFUL) // ACT_
    {
        status = rtems_task_create(
            rtems_id_act_, TASK_PRIORITY_ACTN, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &rtems_id_act_
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
            rtems_name_updt, TASK_PRIORITY_UPDT, RTEMS_MINIMUM_STACK_SIZE,
            RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES, &rtems_id_updt
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

    if (status == RTEMS_SUCCESSFUL)     // ACT_
    {
        status = rtems_task_start( rtems_id_act_, act__task, 1 );
        if (status!=RTEMS_SUCCESSFUL) {
            BOOT_PRINTF("in INIT *** Error starting TASK_ACT_\n")
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
        status = rtems_task_start( rtems_id_updt, updt_task, 1 );
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

    // create the queue for handling valid TCs
    status_recv = rtems_message_queue_create( misc_name[QUEUE_RECV],
                                              MSG_QUEUE_COUNT_RECV, CCSDS_TC_PKT_MAX_SIZE,
                                         RTEMS_FIFO | RTEMS_LOCAL, &queue_id );
    if ( status_recv != RTEMS_SUCCESSFUL ) {
        PRINTF1("in create_message_queues *** ERR creating QUEU queue, %d\n", status_recv)
    }

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
    else
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

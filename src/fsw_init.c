/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2012-2018, Plasma Physics Laboratory - CNRS
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
-------------------------------------------------------------------------------*/
/*--                  Author : Paul Leroy
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/

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

#include <rtems.h>

/* configuration information */

#define CONFIGURE_INIT

#include <bsp.h> /* for device driver prototypes */

/* configuration information */

#include <fsw_params.h>

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
    #define CONFIGURE_DRIVER_AMBAPP_GAISLER_GRSPW /* GRSPW Driver */
    #include <drvmgr/drvmgr_confdefs.h>
#endif

#include "fsw_debug.h"
#include "GscMemoryLPP.hpp"
#include "fsw_config.c"
#include "fsw_init.h"
#include "processing/ASM/spectralmatrices.h"
#include "fsw_compile_warnings.h"
#include "hw/uart.h"
#include "hw/lfr_regs.h"

void initCache()
{
    // ASI 2 contains a few control registers that have not been assigned as ancillary state
    // registers. These should only be read and written using 32-bit LDA/STA instructions. All cache
    // registers are accessed through load/store operations to the alternate address space
    // (LDA/STA), using ASI = 2. The table below shows the register addresses:
    //      0x00 Cache control register
    //      0x04 Reserved
    //      0x08 Instruction cache configuration register
    //      0x0C Data cache configuration register

    // Cache Control Register Leon3 / Leon3FT
    // 31..30  29   28  27..24  23  22  21  20..19  18  17  16
    //         RFT  PS  TB      DS  FD  FI  FT          ST  IB
    // 15  14  13..12  11..10  9..8  7..6  5   4   3..2  1..0
    // IP  DP  ITE     IDE     DTE   DDE   DF  IF  DCS   ICS

    unsigned int cacheControlRegister;

    CCR_resetCacheControlRegister();
    ASR16_resetRegisterProtectionControlRegister();

    cacheControlRegister = CCR_getValue();
    LFR_PRINTF("(0) CCR - Cache Control Register = %x\n", cacheControlRegister);
    LFR_PRINTF("(0) ASR16                        = %x\n", *asr16Ptr);

    CCR_enableInstructionCache(); // ICS bits
    CCR_enableDataCache(); // DCS bits
    CCR_enableInstructionBurstFetch(); // IB  bit

    faultTolerantScheme();

    cacheControlRegister = CCR_getValue();
    LFR_PRINTF("(1) CCR - Cache Control Register = %x\n", cacheControlRegister);
    LFR_PRINTF("(1) ASR16 Register protection control register = %x\n", *asr16Ptr);

    LFR_PRINTF("\n");
}

rtems_task Init(rtems_task_argument ignored)
{
    /** This is the RTEMS INIT taks, it is the first task launched by the system.
     *
     * @param unused is the starting argument of the RTEMS task
     *
     * The INIT task create and run all other RTEMS tasks.
     *
     */
    IGNORE_UNUSED_PARAMETER(ignored);
    //***********
    // INIT CACHE

    unsigned char* vhdlVersion;

    reset_lfr();

    reset_local_time();

    rtems_cpu_usage_reset();

    rtems_status_code status;
    rtems_status_code status_spw;
    rtems_isr_entry old_isr_handler;

    old_isr_handler = NULL;

    // UART settings
    enable_apbuart_transmitter();
    set_apbuart_scaler_reload_register(APBUART_SCALER_RELOAD_VALUE);

    DEBUG_PRINTF("\n\n\n\n\nIn INIT *** Now the console is on port COM1\n");


    LFR_PRINTF("\n\n\n\n\n");

    initCache();

    LFR_PRINTF("*************************\n");
    LFR_PRINTF("** LFR Flight Software **\n");

    LFR_PRINTF("** %d-", SW_VERSION_N1);
    LFR_PRINTF("%d-", SW_VERSION_N2);
    LFR_PRINTF("%d-", SW_VERSION_N3);
    LFR_PRINTF("%d             **\n", SW_VERSION_N4);

    vhdlVersion = (unsigned char*)(REGS_ADDR_VHDL_VERSION);
    LFR_PRINTF("** VHDL                **\n");
    LFR_PRINTF("** %d-", vhdlVersion[1]);
    LFR_PRINTF("%d-", vhdlVersion[2]);
    LFR_PRINTF("%d              **\n", vhdlVersion[3]);
    LFR_PRINTF("*************************\n");
    LFR_PRINTF("\n\n");

    init_parameter_dump();
    init_kcoefficients_dump();
    init_local_mode_parameters();
    init_housekeeping_parameters();
    pa_bia_status_info = INIT_CHAR;

    // initialize all reaction wheels frequencies to NaN
    rw_f.cp_rpw_sc_rw1_f1 = NAN;
    rw_f.cp_rpw_sc_rw1_f2 = NAN;
    rw_f.cp_rpw_sc_rw1_f3 = NAN;
    rw_f.cp_rpw_sc_rw1_f4 = NAN;
    rw_f.cp_rpw_sc_rw2_f1 = NAN;
    rw_f.cp_rpw_sc_rw2_f2 = NAN;
    rw_f.cp_rpw_sc_rw2_f3 = NAN;
    rw_f.cp_rpw_sc_rw2_f4 = NAN;
    rw_f.cp_rpw_sc_rw3_f1 = NAN;
    rw_f.cp_rpw_sc_rw3_f2 = NAN;
    rw_f.cp_rpw_sc_rw3_f3 = NAN;
    rw_f.cp_rpw_sc_rw3_f4 = NAN;
    rw_f.cp_rpw_sc_rw4_f1 = NAN;
    rw_f.cp_rpw_sc_rw4_f2 = NAN;
    rw_f.cp_rpw_sc_rw4_f3 = NAN;
    rw_f.cp_rpw_sc_rw4_f4 = NAN;

    // initialize filtering parameters
    filterPar.spare_sy_lfr_pas_filter_enabled = DEFAULT_SY_LFR_PAS_FILTER_ENABLED;
    filterPar.sy_lfr_sc_rw_delta_f = DEFAULT_SY_LFR_SC_RW_DELTA_F;
    filterPar.sy_lfr_pas_filter_tbad = DEFAULT_SY_LFR_PAS_FILTER_TBAD;
    filterPar.sy_lfr_pas_filter_shift = DEFAULT_SY_LFR_PAS_FILTER_SHIFT;
    filterPar.modulus_in_finetime = DEFAULT_MODULUS;
    filterPar.tbad_in_finetime = DEFAULT_TBAD;
    filterPar.offset_in_finetime = DEFAULT_OFFSET;
    filterPar.shift_in_finetime = DEFAULT_SHIFT;
    update_last_valid_transition_date(DEFAULT_LAST_VALID_TRANSITION_DATE);

    // waveform picker initialization
    WFP_init_rings();
    LEON_Clear_interrupt(IRQ_SPARC_GPTIMER_WATCHDOG); // initialize the waveform rings
    WFP_reset_current_ring_nodes();
    reset_waveform_picker_regs();

    // spectral matrices initialization
    SM_init_rings(); // initialize spectral matrices rings
    SM_reset_current_ring_nodes();
    reset_spectral_matrix_regs();

    // configure calibration
    configureCalibration(false); // true means interleaved mode, false is for normal mode

    updateLFRCurrentMode(LFR_MODE_STANDBY);

    BOOT_PRINTF("in INIT *** lfrCurrentMode is %d\n", lfrCurrentMode);

    create_names(); // create all names

    status = create_timecode_timer(); // create the timer used by timecode_irq_handler
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR in create_timer_timecode, code %d", status);
    }

    status = create_message_queues(); // create message queues
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR in create_message_queues, code %d", status);
    }

    status = create_all_tasks(); // create all tasks
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR in create_all_tasks, code %d\n", status);
    }

    // **************************
    // <SPACEWIRE INITIALIZATION>
    status_spw = spacewire_open_link(); // (1) open the link
    if (status_spw != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR spacewire_open_link code %d\n", status_spw);
    }

    if (status_spw == RTEMS_SUCCESSFUL) // (2) configure the link
    {
        status_spw = spacewire_configure_link(fdSPW);
        if (status_spw != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("in INIT *** ERR spacewire_configure_link code %d\n", status_spw);
        }
    }

    if (status_spw == RTEMS_SUCCESSFUL) // (3) start the link
    {
        status_spw = spacewire_start_link(fdSPW);
        if (status_spw != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("in INIT *** ERR spacewire_start_link code %d\n", status_spw);
        }
    }
    // </SPACEWIRE INITIALIZATION>
    // ***************************

    status = start_all_tasks(); // start all tasks
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR in start_all_tasks, code %d", status);
    }

    // start RECV and SEND *AFTER* SpaceWire Initialization, due to the timeout of the start call
    // during the initialization
    status = start_recv_send_tasks();
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** ERR start_recv_send_tasks code %d\n", status);
    }

    // suspend science tasks, they will be restarted later depending on the mode
    status = suspend_science_tasks(); // suspend science tasks (not done in stop_current_mode if
                                      // current mode = STANDBY)
    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in INIT *** in suspend_science_tasks *** ERR code: %d\n", status);
    }

    // configure IRQ handling for the waveform picker unit
    status = rtems_interrupt_catch(waveforms_isr, IRQ_SPARC_WAVEFORM_PICKER, &old_isr_handler);
    // configure IRQ handling for the spectral matrices unit
    status
        = rtems_interrupt_catch(spectral_matrices_isr, IRQ_SPARC_SPECTRAL_MATRIX, &old_isr_handler);

    // if the spacewire link is not up then send an event to the SPIQ task for link recovery
    if (status_spw != RTEMS_SUCCESSFUL)
    {
        status = rtems_event_send(Task_id[TASKID_SPIQ], SPW_LINKERR_EVENT);
        if (status != RTEMS_SUCCESSFUL)
        {
            LFR_PRINTF("in INIT *** ERR rtems_event_send to SPIQ code %d\n", status);
        }
    }

    BOOT_PRINTF("delete INIT\n");

    set_hk_lfr_sc_potential_flag(true);

    // start the timer to detect a missing spacewire timecode
    // the timeout is larger because the spw IP needs to receive several valid timecodes before
    // generating a tickout if a tickout is generated, the timer is restarted
    status = rtems_timer_fire_after(
        timecode_timer_id, TIMECODE_TIMER_TIMEOUT_INIT, timecode_timer_routine, NULL);

    grspw_timecode_callback = &timecode_irq_handler;

    status = rtems_task_delete(RTEMS_SELF);
}

void init_local_mode_parameters(void)
{
    /** This function initialize the param_local global variable with default values.
     *
     */

    unsigned int i;

    // LOCAL PARAMETERS

    BOOT_PRINTF("local_sbm1_nb_cwf_max %d \n", param_local.local_sbm1_nb_cwf_max);
    BOOT_PRINTF("local_sbm2_nb_cwf_max %d \n", param_local.local_sbm2_nb_cwf_max);

    // init sequence counters

    for (i = 0; i < SEQ_CNT_NB_DEST_ID; i++)
    {
        sequenceCounters_TC_EXE[i] = INIT_CHAR;
        sequenceCounters_TM_DUMP[i] = INIT_CHAR;
    }
    sequenceCounters_SCIENCE_NORMAL_BURST = INIT_CHAR;
    sequenceCounters_SCIENCE_SBM1_SBM2 = INIT_CHAR;
    sequenceCounterHK = TM_PACKET_SEQ_CTRL_STANDALONE << TM_PACKET_SEQ_SHIFT;
}

void reset_local_time(void)
{
    time_management_regs->ctrl = time_management_regs->ctrl
        | VAL_SOFTWARE_RESET; // [0010] software reset, coarse time = 0x80000000
}

void create_names(void) // create all names for tasks and queues
{
    /** This function creates all RTEMS names used in the software for tasks and queues.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - successful completion
     *
     */

    // task names
    Task_name[TASKID_AVGV] = rtems_build_name('A', 'V', 'G', 'V');
    Task_name[TASKID_RECV] = rtems_build_name('R', 'E', 'C', 'V');
    Task_name[TASKID_ACTN] = rtems_build_name('A', 'C', 'T', 'N');
    Task_name[TASKID_SPIQ] = rtems_build_name('S', 'P', 'I', 'Q');
    Task_name[TASKID_LOAD] = rtems_build_name('L', 'O', 'A', 'D');
    Task_name[TASKID_AVF0] = rtems_build_name('A', 'V', 'F', '0');
    Task_name[TASKID_SWBD] = rtems_build_name('S', 'W', 'B', 'D');
    Task_name[TASKID_WFRM] = rtems_build_name('W', 'F', 'R', 'M');
    Task_name[TASKID_DUMB] = rtems_build_name('D', 'U', 'M', 'B');
    Task_name[TASKID_HOUS] = rtems_build_name('H', 'O', 'U', 'S');
    Task_name[TASKID_PRC0] = rtems_build_name('P', 'R', 'C', '0');
    Task_name[TASKID_CWF3] = rtems_build_name('C', 'W', 'F', '3');
    Task_name[TASKID_CWF2] = rtems_build_name('C', 'W', 'F', '2');
    Task_name[TASKID_CWF1] = rtems_build_name('C', 'W', 'F', '1');
    Task_name[TASKID_SEND] = rtems_build_name('S', 'E', 'N', 'D');
    Task_name[TASKID_LINK] = rtems_build_name('L', 'I', 'N', 'K');
    Task_name[TASKID_AVF1] = rtems_build_name('A', 'V', 'F', '1');
    Task_name[TASKID_PRC1] = rtems_build_name('P', 'R', 'C', '1');
    Task_name[TASKID_AVF2] = rtems_build_name('A', 'V', 'F', '2');
    Task_name[TASKID_PRC2] = rtems_build_name('P', 'R', 'C', '2');
    Task_name[TASKID_SCRB] = rtems_build_name('S', 'C', 'R', 'B');
    Task_name[TASKID_CALI] = rtems_build_name('C', 'A', 'L', 'I');

    // rate monotonic period names
    name_hk_rate_monotonic = rtems_build_name('H', 'O', 'U', 'S');
    name_avgv_rate_monotonic = rtems_build_name('A', 'V', 'G', 'V');

    misc_name[QUEUE_RECV] = rtems_build_name('Q', '_', 'R', 'V');
    misc_name[QUEUE_SEND] = rtems_build_name('Q', '_', 'S', 'D');
    misc_name[QUEUE_PRC0] = rtems_build_name('Q', '_', 'P', '0');
    misc_name[QUEUE_PRC1] = rtems_build_name('Q', '_', 'P', '1');
    misc_name[QUEUE_PRC2] = rtems_build_name('Q', '_', 'P', '2');

    timecode_timer_name = rtems_build_name('S', 'P', 'T', 'C');
}

int create_all_tasks(void) // create all tasks which run in the software
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
    status = rtems_task_create(Task_name[TASKID_RECV], TASK_PRIORITY_RECV, RTEMS_MINIMUM_STACK_SIZE,
        RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_RECV]);
    if (status == RTEMS_SUCCESSFUL) // SEND
    {
        status = rtems_task_create(Task_name[TASKID_SEND], TASK_PRIORITY_SEND,
            RTEMS_MINIMUM_STACK_SIZE * STACK_SIZE_MULT, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_SEND]);
    }
    if (status == RTEMS_SUCCESSFUL) // LINK
    {
        status = rtems_task_create(Task_name[TASKID_LINK], TASK_PRIORITY_LINK,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES,
            &Task_id[TASKID_LINK]);
    }
    if (status == RTEMS_SUCCESSFUL) // ACTN
    {
        status = rtems_task_create(Task_name[TASKID_ACTN], TASK_PRIORITY_ACTN,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_ACTN]);
    }
    if (status == RTEMS_SUCCESSFUL) // SPIQ
    {
        status = rtems_task_create(Task_name[TASKID_SPIQ], TASK_PRIORITY_SPIQ,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES, &Task_id[TASKID_SPIQ]);
    }

    //******************
    // SPECTRAL MATRICES
    if (status == RTEMS_SUCCESSFUL) // AVF0
    {
        status = rtems_task_create(Task_name[TASKID_AVF0], TASK_PRIORITY_AVF0,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVF0]);
    }
    if (status == RTEMS_SUCCESSFUL) // PRC0
    {
        status = rtems_task_create(Task_name[TASKID_PRC0], TASK_PRIORITY_PRC0,
            RTEMS_MINIMUM_STACK_SIZE * STACK_SIZE_MULT, RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_PRC0]);
    }
    if (status == RTEMS_SUCCESSFUL) // AVF1
    {
        status = rtems_task_create(Task_name[TASKID_AVF1], TASK_PRIORITY_AVF1,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVF1]);
    }
    if (status == RTEMS_SUCCESSFUL) // PRC1
    {
        status = rtems_task_create(Task_name[TASKID_PRC1], TASK_PRIORITY_PRC1,
            RTEMS_MINIMUM_STACK_SIZE * STACK_SIZE_MULT, RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_PRC1]);
    }
    if (status == RTEMS_SUCCESSFUL) // AVF2
    {
        status = rtems_task_create(Task_name[TASKID_AVF2], TASK_PRIORITY_AVF2,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVF2]);
    }
    if (status == RTEMS_SUCCESSFUL) // PRC2
    {
        status = rtems_task_create(Task_name[TASKID_PRC2], TASK_PRIORITY_PRC2,
            RTEMS_MINIMUM_STACK_SIZE * STACK_SIZE_MULT, RTEMS_DEFAULT_MODES | RTEMS_NO_PREEMPT,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_PRC2]);
    }

    //****************
    // WAVEFORM PICKER
    if (status == RTEMS_SUCCESSFUL) // WFRM
    {
        status = rtems_task_create(Task_name[TASKID_WFRM], TASK_PRIORITY_WFRM,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_WFRM]);
    }
    if (status == RTEMS_SUCCESSFUL) // CWF3
    {
        status = rtems_task_create(Task_name[TASKID_CWF3], TASK_PRIORITY_CWF3,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_CWF3]);
    }
    if (status == RTEMS_SUCCESSFUL) // CWF2
    {
        status = rtems_task_create(Task_name[TASKID_CWF2], TASK_PRIORITY_CWF2,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_CWF2]);
    }
    if (status == RTEMS_SUCCESSFUL) // CWF1
    {
        status = rtems_task_create(Task_name[TASKID_CWF1], TASK_PRIORITY_CWF1,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_CWF1]);
    }
    if (status == RTEMS_SUCCESSFUL) // SWBD
    {
        status = rtems_task_create(Task_name[TASKID_SWBD], TASK_PRIORITY_SWBD,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_SWBD]);
    }

    //*****
    // MISC
    if (status == RTEMS_SUCCESSFUL) // LOAD
    {
        status = rtems_task_create(Task_name[TASKID_LOAD], TASK_PRIORITY_LOAD,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES,
            &Task_id[TASKID_LOAD]);
    }
    if (status == RTEMS_SUCCESSFUL) // DUMB
    {
        status = rtems_task_create(Task_name[TASKID_DUMB], TASK_PRIORITY_DUMB,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES,
            &Task_id[TASKID_DUMB]);
    }
    if (status == RTEMS_SUCCESSFUL) // SCRUBBING TASK
    {
        status = rtems_task_create(Task_name[TASKID_SCRB], TASK_PRIORITY_SCRB,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_SCRB]);
    }
    if (status == RTEMS_SUCCESSFUL) // HOUS
    {
        status = rtems_task_create(Task_name[TASKID_HOUS], TASK_PRIORITY_HOUS,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_HOUS]);
    }
    if (status == RTEMS_SUCCESSFUL) // AVGV
    {
        status = rtems_task_create(Task_name[TASKID_AVGV], TASK_PRIORITY_AVGV,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_AVGV]);
    }
    if (status == RTEMS_SUCCESSFUL) // CALI
    {
        status = rtems_task_create(Task_name[TASKID_CALI], TASK_PRIORITY_CALI,
            RTEMS_MINIMUM_STACK_SIZE, RTEMS_DEFAULT_MODES,
            RTEMS_DEFAULT_ATTRIBUTES | RTEMS_FLOATING_POINT, &Task_id[TASKID_CALI]);
    }

    return status;
}

int start_recv_send_tasks(void)
{
    rtems_status_code status;

    status = rtems_task_start(Task_id[TASKID_RECV], recv_task, 1);
    if (status != RTEMS_SUCCESSFUL)
    {
        BOOT_PRINTF("in INIT *** Error starting TASK_RECV\n");
    }

    if (status == RTEMS_SUCCESSFUL) // SEND
    {
        status = rtems_task_start(Task_id[TASKID_SEND], send_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_SEND\n");
        }
    }

    return status;
}

int start_all_tasks(void) // start all tasks except SEND RECV and HOUS
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
    status = rtems_task_start(Task_id[TASKID_SPIQ], spiq_task, 1);
    if (status != RTEMS_SUCCESSFUL)
    {
        BOOT_PRINTF("in INIT *** Error starting TASK_SPIQ\n");
    }

    if (status == RTEMS_SUCCESSFUL) // LINK
    {
        status = rtems_task_start(Task_id[TASKID_LINK], link_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_LINK\n");
        }
    }

    if (status == RTEMS_SUCCESSFUL) // ACTN
    {
        status = rtems_task_start(Task_id[TASKID_ACTN], actn_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_ACTN\n");
        }
    }

    //******************
    // SPECTRAL MATRICES
    if (status == RTEMS_SUCCESSFUL) // AVF0
    {
        status = rtems_task_start(Task_id[TASKID_AVF0], avf0_task, LFR_MODE_STANDBY);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_AVF0\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // PRC0
    {
        status = rtems_task_start(Task_id[TASKID_PRC0], prc0_task, LFR_MODE_STANDBY);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_PRC0\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // AVF1
    {
        status = rtems_task_start(Task_id[TASKID_AVF1], avf1_task, LFR_MODE_STANDBY);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_AVF1\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // PRC1
    {
        status = rtems_task_start(Task_id[TASKID_PRC1], prc1_task, LFR_MODE_STANDBY);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_PRC1\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // AVF2
    {
        status = rtems_task_start(Task_id[TASKID_AVF2], avf2_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_AVF2\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // PRC2
    {
        status = rtems_task_start(Task_id[TASKID_PRC2], prc2_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_PRC2\n");
        }
    }

    //****************
    // WAVEFORM PICKER
    if (status == RTEMS_SUCCESSFUL) // WFRM
    {
        status = rtems_task_start(Task_id[TASKID_WFRM], wfrm_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_WFRM\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // CWF3
    {
        status = rtems_task_start(Task_id[TASKID_CWF3], cwf3_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_CWF3\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // CWF2
    {
        status = rtems_task_start(Task_id[TASKID_CWF2], cwf2_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_CWF2\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // CWF1
    {
        status = rtems_task_start(Task_id[TASKID_CWF1], cwf1_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_CWF1\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // SWBD
    {
        status = rtems_task_start(Task_id[TASKID_SWBD], swbd_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_SWBD\n");
        }
    }

    //*****
    // MISC
    if (status == RTEMS_SUCCESSFUL) // HOUS
    {
        status = rtems_task_start(Task_id[TASKID_HOUS], hous_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_HOUS\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // AVGV
    {
        status = rtems_task_start(Task_id[TASKID_AVGV], avgv_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_AVGV\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // DUMB
    {
        status = rtems_task_start(Task_id[TASKID_DUMB], dumb_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_DUMB\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // SCRUBBING
    {
        status = rtems_task_start(Task_id[TASKID_SCRB], scrubbing_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_DUMB\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // LOAD
    {
        status = rtems_task_start(Task_id[TASKID_LOAD], load_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_LOAD\n");
        }
    }
    if (status == RTEMS_SUCCESSFUL) // CALI
    {
        status = rtems_task_start(Task_id[TASKID_CALI], calibration_sweep_task, 1);
        if (status != RTEMS_SUCCESSFUL)
        {
            BOOT_PRINTF("in INIT *** Error starting TASK_LOAD\n");
        }
    }

    return status;
}

rtems_status_code create_message_queues(void) // create the five message queues used in the software
{
    rtems_status_code status_recv;
    rtems_status_code status_send;
    rtems_status_code status_q_p0;
    rtems_status_code status_q_p1;
    rtems_status_code status_q_p2;
    rtems_status_code ret;
    rtems_id queue_id;

    ret = RTEMS_SUCCESSFUL;
    queue_id = RTEMS_ID_NONE;

    //****************************************
    // create the queue for handling valid TCs
    status_recv = rtems_message_queue_create(misc_name[QUEUE_RECV], MSG_QUEUE_COUNT_RECV,
        CCSDS_TC_PKT_MAX_SIZE, RTEMS_FIFO | RTEMS_LOCAL, &queue_id);
    if (status_recv != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_message_queues *** ERR creating QUEU queue, %d\n", status_recv);
    }

    //************************************************
    // create the queue for handling TM packet sending
    status_send = rtems_message_queue_create(misc_name[QUEUE_SEND], MSG_QUEUE_COUNT_SEND,
        MSG_QUEUE_SIZE_SEND, RTEMS_FIFO | RTEMS_LOCAL, &queue_id);
    if (status_send != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_message_queues *** ERR creating PKTS queue, %d\n", status_send);
    }

    //*****************************************************************************
    // create the queue for handling averaged spectral matrices for processing @ f0
    status_q_p0 = rtems_message_queue_create(misc_name[QUEUE_PRC0], MSG_QUEUE_COUNT_PRC0,
        sizeof (asm_msg), RTEMS_FIFO | RTEMS_LOCAL, &queue_id);
    if (status_q_p0 != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_message_queues *** ERR creating Q_P0 queue, %d\n", status_q_p0);
    }

    //*****************************************************************************
    // create the queue for handling averaged spectral matrices for processing @ f1
    status_q_p1 = rtems_message_queue_create(misc_name[QUEUE_PRC1], MSG_QUEUE_COUNT_PRC1,
        sizeof (asm_msg), RTEMS_FIFO | RTEMS_LOCAL, &queue_id);
    if (status_q_p1 != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_message_queues *** ERR creating Q_P1 queue, %d\n", status_q_p1);
    }

    //*****************************************************************************
    // create the queue for handling averaged spectral matrices for processing @ f2
    status_q_p2 = rtems_message_queue_create(misc_name[QUEUE_PRC2], MSG_QUEUE_COUNT_PRC2,
        sizeof (asm_msg), RTEMS_FIFO | RTEMS_LOCAL, &queue_id);
    if (status_q_p2 != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_message_queues *** ERR creating Q_P2 queue, %d\n", status_q_p2);
    }

    if (status_recv != RTEMS_SUCCESSFUL)
    {
        ret = status_recv;
    }
    else if (status_send != RTEMS_SUCCESSFUL)
    {
        ret = status_send;
    }
    else if (status_q_p0 != RTEMS_SUCCESSFUL)
    {
        ret = status_q_p0;
    }
    else if (status_q_p1 != RTEMS_SUCCESSFUL)
    {
        ret = status_q_p1;
    }
    else
    {
        ret = status_q_p2;
    }

    return ret;
}

rtems_status_code create_timecode_timer(void)
{
    rtems_status_code status;

    status = rtems_timer_create(timecode_timer_name, &timecode_timer_id);

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in create_timer_timecode *** ERR creating SPTC timer, %d\n", status);
    }
    else
    {
        LFR_PRINTF("in create_timer_timecode *** OK creating SPTC timer\n");
    }

    return status;
}

rtems_status_code get_message_queue_id_send(rtems_id* queue_id)
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name('Q', '_', 'S', 'D');

    status = rtems_message_queue_ident(queue_name, 0, queue_id);

    return status;
}

rtems_status_code get_message_queue_id_recv(rtems_id* queue_id)
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name('Q', '_', 'R', 'V');

    status = rtems_message_queue_ident(queue_name, 0, queue_id);

    return status;
}

rtems_status_code get_message_queue_id_prc0(rtems_id* queue_id)
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name('Q', '_', 'P', '0');

    status = rtems_message_queue_ident(queue_name, 0, queue_id);

    return status;
}

rtems_status_code get_message_queue_id_prc1(rtems_id* queue_id)
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name('Q', '_', 'P', '1');

    status = rtems_message_queue_ident(queue_name, 0, queue_id);

    return status;
}

rtems_status_code get_message_queue_id_prc2(rtems_id* queue_id)
{
    rtems_status_code status;
    rtems_name queue_name;

    queue_name = rtems_build_name('Q', '_', 'P', '2');

    status = rtems_message_queue_ident(queue_name, 0, queue_id);

    return status;
}

/**
 * @brief update_queue_max_count returns max(fifo_size_max, pending_messages + 1)
 * @param queue_id
 * @param fifo_size_max
 */
void update_queue_max_count(rtems_id queue_id, unsigned char* fifo_size_max)
{
    DEBUG_CHECK_PTR(fifo_size_max);
    uint32_t count;
    rtems_status_code status;

    count = 0;

    status = rtems_message_queue_get_number_pending(queue_id, &count);
    DEBUG_CHECK_STATUS(status);

    count = count + 1;

    if (status != RTEMS_SUCCESSFUL)
    {
        LFR_PRINTF("in update_queue_max_count *** ERR = %d\n", status);
    }
    else
    {
        if (count > *fifo_size_max)
        {
            *fifo_size_max = count;
        }
    }
}

/**
 * @brief init_ring initializes given ring buffer
 * @param ring array of nodes to initialize
 * @param nbNodes number of node in the ring buffer
 * @param buffer memory space given to the ring buffer
 * @param bufferSize size of the whole ring buffer memory space
 *
 * @details This function creates a circular buffer from a given number of nodes and a given memory
 * space. It first sets all nodes attributes to thier defaults values and associates a portion of
 * the given memory space with each node. Then it connects each nodes to build a circular buffer.
 *
 * Each node capacity will be bufferSize/nbNodes.
 *
 * https://en.wikipedia.org/wiki/Circular_buffer
 */
void init_ring(
    ring_node ring[], unsigned char nbNodes, volatile int buffer[], unsigned int bufferSize)
{
    DEBUG_CHECK_PTR(ring);
    DEBUG_CHECK_PTR(buffer);
    unsigned char i;

    //***************
    // BUFFER ADDRESS
    for (i = 0; i < nbNodes; i++)
    {
        ring[i].coarseTime = INT32_ALL_F;
        ring[i].fineTime = INT32_ALL_F;
        ring[i].sid = INIT_CHAR;
        ring[i].status = INIT_CHAR;
        ring[i].buffer_address = (void*)&(buffer[i * bufferSize]);
    }

    //*****
    // NEXT
    ring[nbNodes - 1].next = (ring_node*)&ring[0];
    for (i = 0; i < nbNodes - 1; i++)
    {
        ring[i].next = (ring_node*)&ring[i + 1];
    }

    //*********
    // PREVIOUS
    ring[0].previous = (ring_node*)&ring[nbNodes - 1];
    for (i = 1; i < nbNodes; i++)
    {
        ring[i].previous = (ring_node*)&ring[i - 1];
    }
}

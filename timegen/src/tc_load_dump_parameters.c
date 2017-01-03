/** Functions to load and dump parameters in the LFR registers.
 *
 * @file
 * @author P. LEROY
 *
 * A group of functions to handle TC related to parameter loading and dumping.\n
 * TC_LFR_LOAD_COMMON_PAR\n
 * TC_LFR_LOAD_NORMAL_PAR\n
 * TC_LFR_LOAD_BURST_PAR\n
 * TC_LFR_LOAD_SBM1_PAR\n
 * TC_LFR_LOAD_SBM2_PAR\n
 *
 */

#include "tc_load_dump_parameters.h"

int action_load_common_par(ccsdsTelecommandPacket_t *TC)
{
    /** This function updates the LFR registers with the incoming common parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     *
     *
     */

    parameter_dump_packet.unused0 = TC->dataAndCRC[0];
    parameter_dump_packet.bw_sp0_sp1_r0_r1 = TC->dataAndCRC[1];
    set_wfp_data_shaping( );
    return LFR_SUCCESSFUL;
}

int action_load_normal_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming normal parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int result;
    int flag;
    rtems_status_code status;

    flag = LFR_SUCCESSFUL;

    if ( (lfrCurrentMode == LFR_MODE_NORMAL) ||
         (lfrCurrentMode == LFR_MODE_SBM1) || (lfrCurrentMode == LFR_MODE_SBM2) ) {
        status = send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        flag = LFR_DEFAULT;
    }

    // CHECK THE PARAMETERS SET CONSISTENCY
    if (flag == LFR_SUCCESSFUL)
    {
        flag = check_common_par_consistency( TC, queue_id );
    }

    // SET THE PARAMETERS IF THEY ARE CONSISTENT
    if (flag == LFR_SUCCESSFUL)
    {
        result = set_sy_lfr_n_swf_l( TC );
        result = set_sy_lfr_n_swf_p( TC );
        result = set_sy_lfr_n_bp_p0( TC );
        result = set_sy_lfr_n_bp_p1( TC );
        result = set_sy_lfr_n_asm_p( TC );
        result = set_sy_lfr_n_cwf_long_f3( TC );
    }

    return flag;
}

int action_load_burst_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming burst parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_b_bp_p0;
    unsigned char sy_lfr_b_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if ( lfrCurrentMode == LFR_MODE_BURST ) {
        status = send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        flag = LFR_DEFAULT;
    }

    sy_lfr_b_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P0 ];
    sy_lfr_b_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P1 ];

    // sy_lfr_b_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_b_bp_p0 < DEFAULT_SY_LFR_B_BP_P0 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0+10, sy_lfr_b_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_b_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_b_bp_p1 < DEFAULT_SY_LFR_B_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P1+10, sy_lfr_b_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    //****************************************************************
    // check the consistency between sy_lfr_b_bp_p0 and sy_lfr_b_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        sy_lfr_b_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P0 ];
        sy_lfr_b_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P1 ];
        aux = ( (float ) sy_lfr_b_bp_p1 / sy_lfr_b_bp_p0 ) - floor(sy_lfr_b_bp_p1 / sy_lfr_b_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0+10, sy_lfr_b_bp_p0 );
            flag = LFR_DEFAULT;
        }
    }

    // SET HTE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_b_bp_p0( TC );
        flag = set_sy_lfr_b_bp_p1( TC );
    }

    return flag;
}

int action_load_sbm1_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm1 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_s1_bp_p0;
    unsigned char sy_lfr_s1_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if ( lfrCurrentMode == LFR_MODE_SBM1 ) {
        status = send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        flag = LFR_DEFAULT;
    }

    sy_lfr_s1_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S1_BP_P0 ];
    sy_lfr_s1_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S1_BP_P1 ];

    // sy_lfr_s1_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s1_bp_p0 < DEFAULT_SY_LFR_S1_BP_P0 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0+10, sy_lfr_s1_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s1_bp_p1 < DEFAULT_SY_LFR_S1_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P1+10, sy_lfr_s1_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    //******************************************************************
    // check the consistency between sy_lfr_s1_bp_p0 and sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0*0.25) ) - floor(sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0*0.25));
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0+10, sy_lfr_s1_bp_p0 );
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_s1_bp_p0( TC );
        flag = set_sy_lfr_s1_bp_p1( TC );
    }

    return flag;
}

int action_load_sbm2_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_s2_bp_p0;
    unsigned char sy_lfr_s2_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if ( lfrCurrentMode == LFR_MODE_SBM2 ) {
        status = send_tm_lfr_tc_exe_not_executable( TC, queue_id );
        flag = LFR_DEFAULT;
    }

    sy_lfr_s2_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P0 ];
    sy_lfr_s2_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P1 ];

    // sy_lfr_s2_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s2_bp_p0 < DEFAULT_SY_LFR_S2_BP_P0 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0+10, sy_lfr_s2_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_s2_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s2_bp_p1 < DEFAULT_SY_LFR_S2_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P1+10, sy_lfr_s2_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    //******************************************************************
    // check the consistency between sy_lfr_s2_bp_p0 and sy_lfr_s2_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        sy_lfr_s2_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P0 ];
        sy_lfr_s2_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P1 ];
        aux = ( (float ) sy_lfr_s2_bp_p1 / sy_lfr_s2_bp_p0 ) - floor(sy_lfr_s2_bp_p1 / sy_lfr_s2_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0+10, sy_lfr_s2_bp_p0 );
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_s2_bp_p0( TC );
        flag = set_sy_lfr_s2_bp_p1( TC );
    }

    return flag;
}

int action_dump_par( rtems_id queue_id )
{
    /** This function dumps the LFR parameters by sending the appropriate TM packet to the dedicated RTEMS message queue.
     *
     * @param queue_id is the id of the queue which handles TM related to this execution step.
     *
     * @return RTEMS directive status codes:
     * - RTEMS_SUCCESSFUL - message sent successfully
     * - RTEMS_INVALID_ID - invalid queue id
     * - RTEMS_INVALID_SIZE - invalid message size
     * - RTEMS_INVALID_ADDRESS - buffer is NULL
     * - RTEMS_UNSATISFIED - out of message buffers
     * - RTEMS_TOO_MANY - queue s limit has been reached
     *
     */

    int status;

    // UPDATE TIME
    parameter_dump_packet.packetSequenceControl[0] = (unsigned char) (sequenceCounterParameterDump >> 8);
    parameter_dump_packet.packetSequenceControl[1] = (unsigned char) (sequenceCounterParameterDump     );
    increment_seq_counter( &sequenceCounterParameterDump );

    parameter_dump_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    parameter_dump_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    parameter_dump_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    parameter_dump_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
    parameter_dump_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    parameter_dump_packet.time[5] = (unsigned char) (time_management_regs->fine_time);
    // SEND DATA
    status =  rtems_message_queue_send( queue_id, &parameter_dump_packet,
                                        PACKET_LENGTH_PARAMETER_DUMP + CCSDS_TC_TM_PACKET_OFFSET + CCSDS_PROTOCOLE_EXTRA_BYTES);
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF1("in action_dump *** ERR sending packet, code %d", status)
    }

    return status;
}

//***********************
// NORMAL MODE PARAMETERS

int check_common_par_consistency( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    unsigned char msb;
    unsigned char lsb;
    int flag;
    float aux;
    rtems_status_code status;

    unsigned int sy_lfr_n_swf_l;
    unsigned int sy_lfr_n_swf_p;
    unsigned int sy_lfr_n_asm_p;
    unsigned char sy_lfr_n_bp_p0;
    unsigned char sy_lfr_n_bp_p1;
    unsigned char sy_lfr_n_cwf_long_f3;

    flag = LFR_SUCCESSFUL;

    //***************
    // get parameters
    msb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_L ];
    lsb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_L+1 ];
    sy_lfr_n_swf_l = msb * 256 + lsb;

    msb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P ];
    lsb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P+1 ];
    sy_lfr_n_swf_p = msb * 256  + lsb;

    msb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P ];
    lsb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P+1 ];
    sy_lfr_n_asm_p = msb * 256  + lsb;

    sy_lfr_n_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P0 ];

    sy_lfr_n_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P1 ];

    sy_lfr_n_cwf_long_f3 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3 ];

    //******************
    // check consistency
    // sy_lfr_n_swf_l
    if (sy_lfr_n_swf_l != 2048)
    {
        status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_L+10, sy_lfr_n_swf_l );
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_swf_p
    if (flag == LFR_SUCCESSFUL)
    {
        if ( sy_lfr_n_swf_p < 16 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_P+10, sy_lfr_n_swf_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_bp_p0 < DFLT_SY_LFR_N_BP_P0)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P0+10, sy_lfr_n_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_asm_p
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_asm_p == 0)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P+10, sy_lfr_n_asm_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_asm_p shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_n_asm_p / sy_lfr_n_bp_p0 ) - floor(sy_lfr_n_asm_p / sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P+10, sy_lfr_n_asm_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_bp_p1 < DFLT_SY_LFR_N_BP_P1)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1+10, sy_lfr_n_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p1 shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0 ) - floor(sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1+10, sy_lfr_n_bp_p1 );
            flag = LFR_DEFAULT;
        }
    }
    // sy_lfr_n_cwf_long_f3

    return flag;
}

int set_sy_lfr_n_swf_l( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the number of points of a snapshot (sy_lfr_n_swf_l).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int result;

    result = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_swf_l[0] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_L   ];
    parameter_dump_packet.sy_lfr_n_swf_l[1] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_L+1 ];

    return result;
}

int set_sy_lfr_n_swf_p(ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two snapshots, in s (sy_lfr_n_swf_p).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int result;

    result = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_swf_p[0] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P   ];
    parameter_dump_packet.sy_lfr_n_swf_p[1] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P+1 ];

    return result;
}

int set_sy_lfr_n_asm_p( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two full spectral matrices transmission, in s (SY_LFR_N_ASM_P).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int result;

    result = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_asm_p[0] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P   ];
    parameter_dump_packet.sy_lfr_n_asm_p[1] = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P+1 ];

    return result;
}

int set_sy_lfr_n_bp_p0( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets, in s (DFLT_SY_LFR_N_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P0 ];

    return status;
}

int set_sy_lfr_n_bp_p1(ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets (autocorrelation + crosscorrelation), in s (sy_lfr_n_bp_p1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P1 ];

    return status;
}

int set_sy_lfr_n_cwf_long_f3(ccsdsTelecommandPacket_t *TC )
{
    /** This function allows to switch from CWF_F3 packets to CWF_LONG_F3 packets.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_n_cwf_long_f3 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3 ];

    return status;
}

//**********************
// BURST MODE PARAMETERS
int set_sy_lfr_b_bp_p0(ccsdsTelecommandPacket_t *TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_B_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_b_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P0 ];

    return status;
}

int set_sy_lfr_b_bp_p1( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_B_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_b_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_B_BP_P1 ];

    return status;
}

//*********************
// SBM1 MODE PARAMETERS
int set_sy_lfr_s1_bp_p0( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S1_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_s1_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S1_BP_P0 ];

    return status;
}

int set_sy_lfr_s1_bp_p1( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S1_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_s1_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S1_BP_P1 ];

    return status;
}

//*********************
// SBM2 MODE PARAMETERS
int set_sy_lfr_s2_bp_p0(ccsdsTelecommandPacket_t *TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S2_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_s2_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P0 ];

    return status;
}

int set_sy_lfr_s2_bp_p1( ccsdsTelecommandPacket_t *TC )
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S2_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int status;

    status = LFR_SUCCESSFUL;

    parameter_dump_packet.sy_lfr_s2_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_S2_BP_P1 ];

    return status;
}


//*******************
// TC_LFR_UPDATE_INFO
unsigned int check_update_info_hk_lfr_mode( unsigned char mode )
{
    unsigned int status;

    if ( (mode == LFR_MODE_STANDBY) || (mode == LFR_MODE_NORMAL)
         || (mode == LFR_MODE_BURST)
         || (mode == LFR_MODE_SBM1) || (mode == LFR_MODE_SBM2))
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        status = LFR_DEFAULT;
    }

    return status;
}

unsigned int check_update_info_hk_tds_mode( unsigned char mode )
{
    unsigned int status;

    if ( (mode == TDS_MODE_STANDBY) || (mode == TDS_MODE_NORMAL)
         || (mode == TDS_MODE_BURST)
         || (mode == TDS_MODE_SBM1) || (mode == TDS_MODE_SBM2)
         || (mode == TDS_MODE_LFM))
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        status = LFR_DEFAULT;
    }

    return status;
}

unsigned int check_update_info_hk_thr_mode( unsigned char mode )
{
    unsigned int status;

    if ( (mode == THR_MODE_STANDBY) || (mode == THR_MODE_NORMAL)
         || (mode == THR_MODE_BURST))
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        status = LFR_DEFAULT;
    }

    return status;
}

//**********
// init dump

void init_parameter_dump( void )
{
    /** This function initialize the parameter_dump_packet global variable with default values.
     *
     */

    parameter_dump_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    parameter_dump_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    parameter_dump_packet.reserved = CCSDS_RESERVED;
    parameter_dump_packet.userApplication = CCSDS_USER_APP;
    parameter_dump_packet.packetID[0] = (unsigned char) (APID_TM_PARAMETER_DUMP >> 8);
    parameter_dump_packet.packetID[1] = (unsigned char) APID_TM_PARAMETER_DUMP;
    parameter_dump_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    parameter_dump_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    parameter_dump_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_PARAMETER_DUMP >> 8);
    parameter_dump_packet.packetLength[1] = (unsigned char) PACKET_LENGTH_PARAMETER_DUMP;
    // DATA FIELD HEADER
    parameter_dump_packet.spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    parameter_dump_packet.serviceType = TM_TYPE_PARAMETER_DUMP;
    parameter_dump_packet.serviceSubType = TM_SUBTYPE_PARAMETER_DUMP;
    parameter_dump_packet.destinationID = TM_DESTINATION_ID_GROUND;
    parameter_dump_packet.time[0] = (unsigned char) (time_management_regs->coarse_time>>24);
    parameter_dump_packet.time[1] = (unsigned char) (time_management_regs->coarse_time>>16);
    parameter_dump_packet.time[2] = (unsigned char) (time_management_regs->coarse_time>>8);
    parameter_dump_packet.time[3] = (unsigned char) (time_management_regs->coarse_time);
    parameter_dump_packet.time[4] = (unsigned char) (time_management_regs->fine_time>>8);
    parameter_dump_packet.time[5] = (unsigned char) (time_management_regs->fine_time);
    parameter_dump_packet.sid = SID_PARAMETER_DUMP;

    //******************
    // COMMON PARAMETERS
    parameter_dump_packet.unused0 = DEFAULT_SY_LFR_COMMON0;
    parameter_dump_packet.bw_sp0_sp1_r0_r1 = DEFAULT_SY_LFR_COMMON1;

    //******************
    // NORMAL PARAMETERS
    parameter_dump_packet.sy_lfr_n_swf_l[0] = (unsigned char) (DFLT_SY_LFR_N_SWF_L >> 8);
    parameter_dump_packet.sy_lfr_n_swf_l[1] = (unsigned char) (DFLT_SY_LFR_N_SWF_L     );
    parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char) (DFLT_SY_LFR_N_SWF_P >> 8);
    parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char) (DFLT_SY_LFR_N_SWF_P     );
    parameter_dump_packet.sy_lfr_n_asm_p[0] = (unsigned char) (DFLT_SY_LFR_N_ASM_P >> 8);
    parameter_dump_packet.sy_lfr_n_asm_p[1] = (unsigned char) (DFLT_SY_LFR_N_ASM_P     );
    parameter_dump_packet.sy_lfr_n_bp_p0 = (unsigned char) DFLT_SY_LFR_N_BP_P0;
    parameter_dump_packet.sy_lfr_n_bp_p1 = (unsigned char) DFLT_SY_LFR_N_BP_P1;
    parameter_dump_packet.sy_lfr_n_cwf_long_f3 = (unsigned char) DFLT_SY_LFR_N_CWF_LONG_F3;

    //*****************
    // BURST PARAMETERS
    parameter_dump_packet.sy_lfr_b_bp_p0 = (unsigned char) DEFAULT_SY_LFR_B_BP_P0;
    parameter_dump_packet.sy_lfr_b_bp_p1 = (unsigned char) DEFAULT_SY_LFR_B_BP_P1;

    //****************
    // SBM1 PARAMETERS
    parameter_dump_packet.sy_lfr_s1_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P0; // min value is 0.25 s for the period
    parameter_dump_packet.sy_lfr_s1_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S1_BP_P1;

    //****************
    // SBM2 PARAMETERS
    parameter_dump_packet.sy_lfr_s2_bp_p0 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P0;
    parameter_dump_packet.sy_lfr_s2_bp_p1 = (unsigned char) DEFAULT_SY_LFR_S2_BP_P1;
}








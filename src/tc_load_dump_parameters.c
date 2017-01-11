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

Packet_TM_LFR_KCOEFFICIENTS_DUMP_t kcoefficients_dump_1;
Packet_TM_LFR_KCOEFFICIENTS_DUMP_t kcoefficients_dump_2;
ring_node kcoefficient_node_1;
ring_node kcoefficient_node_2;

int action_load_common_par(ccsdsTelecommandPacket_t *TC)
{
    /** This function updates the LFR registers with the incoming common parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     *
     *
     */

    parameter_dump_packet.sy_lfr_common_parameters_spare    = TC->dataAndCRC[0];
    parameter_dump_packet.sy_lfr_common_parameters          = TC->dataAndCRC[1];
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
        flag = check_normal_par_consistency( TC, queue_id );
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

    // sy_lfr_b_bp_p0 shall not be lower than its default value
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_b_bp_p0 < DEFAULT_SY_LFR_B_BP_P0 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0 + DATAFIELD_OFFSET, sy_lfr_b_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_b_bp_p1 shall not be lower than its default value
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_b_bp_p1 < DEFAULT_SY_LFR_B_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P1 + DATAFIELD_OFFSET, sy_lfr_b_bp_p1 );
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
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0 + DATAFIELD_OFFSET, sy_lfr_b_bp_p0 );
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
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
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s1_bp_p1 < DEFAULT_SY_LFR_S1_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P1 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    //******************************************************************
    // check the consistency between sy_lfr_s1_bp_p0 and sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0 * S1_BP_P0_SCALE) )
                - floor(sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0 * S1_BP_P0_SCALE));
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p0 );
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
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_s2_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_s2_bp_p1 < DEFAULT_SY_LFR_S2_BP_P1 )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P1 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p1 );
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
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p0 );
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

int action_load_kcoefficients(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;

    flag = LFR_DEFAULT;

    flag = set_sy_lfr_kcoeff( TC, queue_id );

    return flag;
}

int action_load_fbins_mask(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;

    flag = LFR_DEFAULT;

    flag = set_sy_lfr_fbins( TC );

    // once the fbins masks have been stored, they have to be merged with the masks which handle the reaction wheels frequencies filtering
    merge_fbins_masks();

    return flag;
}

int action_load_filter_par(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag;

    flag = LFR_DEFAULT;

    flag = check_sy_lfr_filter_parameters( TC, queue_id );

    if (flag  == LFR_SUCCESSFUL)
    {
        parameter_dump_packet.spare_sy_lfr_pas_filter_enabled   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_ENABLED ];
        parameter_dump_packet.sy_lfr_pas_filter_modulus         = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS ];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_0]    = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_0 ];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_1]    = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_1 ];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_2]    = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_2 ];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_3]    = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_3 ];
        parameter_dump_packet.sy_lfr_pas_filter_offset          = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET ];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_0]   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_0 ];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_1]   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_1 ];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_2]   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_2 ];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_3]   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_3 ];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_0]      = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_0 ];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_1]      = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_1 ];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_2]      = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_2 ];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_3]      = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_3 ];

        //****************************
        // store PAS filter parameters
        // sy_lfr_pas_filter_enabled
        filterPar.spare_sy_lfr_pas_filter_enabled   = parameter_dump_packet.spare_sy_lfr_pas_filter_enabled;
        set_sy_lfr_pas_filter_enabled( parameter_dump_packet.spare_sy_lfr_pas_filter_enabled & BIT_PAS_FILTER_ENABLED );
        // sy_lfr_pas_filter_modulus
        filterPar.sy_lfr_pas_filter_modulus         = parameter_dump_packet.sy_lfr_pas_filter_modulus;
        // sy_lfr_pas_filter_tbad
        copyFloatByChar( (unsigned char*) &filterPar.sy_lfr_pas_filter_tbad,
                         parameter_dump_packet.sy_lfr_pas_filter_tbad );
        // sy_lfr_pas_filter_offset
        filterPar.sy_lfr_pas_filter_offset          = parameter_dump_packet.sy_lfr_pas_filter_offset;
        // sy_lfr_pas_filter_shift
        copyFloatByChar( (unsigned char*) &filterPar.sy_lfr_pas_filter_shift,
                         parameter_dump_packet.sy_lfr_pas_filter_shift );

        //****************************************************
        // store the parameter sy_lfr_sc_rw_delta_f as a float
        copyFloatByChar( (unsigned char*) &filterPar.sy_lfr_sc_rw_delta_f,
                         parameter_dump_packet.sy_lfr_sc_rw_delta_f );
    }

    return flag;
}

int action_dump_kcoefficients(ccsdsTelecommandPacket_t *TC, rtems_id queue_id, unsigned char *time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    unsigned int address;
    rtems_status_code status;
    unsigned int freq;
    unsigned int bin;
    unsigned int coeff;
    unsigned char *kCoeffPtr;
    unsigned char *kCoeffDumpPtr;

    // for each sy_lfr_kcoeff_frequency there is 32 kcoeff
    // F0 => 11 bins
    // F1 => 13 bins
    // F2 => 12 bins
    // 36 bins to dump in two packets (30 bins max per packet)

    //*********
    // PACKET 1
    // 11 F0 bins, 13 F1 bins and 6 F2 bins
    kcoefficients_dump_1.destinationID = TC->sourceID;
    increment_seq_counter_destination_id_dump( kcoefficients_dump_1.packetSequenceControl, TC->sourceID );
    for( freq = 0;
         freq < NB_BINS_COMPRESSED_SM_F0;
         freq++ )
    {
        kcoefficients_dump_1.kcoeff_blks[ (freq*KCOEFF_BLK_SIZE) + 1] = freq;
        bin = freq;
//        printKCoefficients( freq, bin, k_coeff_intercalib_f0_norm);
        for ( coeff=0; coeff<NB_K_COEFF_PER_BIN; coeff++ )
        {
            kCoeffDumpPtr = (unsigned char*) &kcoefficients_dump_1.kcoeff_blks[
                    (freq*KCOEFF_BLK_SIZE) + (coeff*NB_BYTES_PER_FLOAT) + KCOEFF_FREQ
                    ]; // 2 for the kcoeff_frequency
            kCoeffPtr     = (unsigned char*) &k_coeff_intercalib_f0_norm[ (bin*NB_K_COEFF_PER_BIN) + coeff ];
            copyFloatByChar( kCoeffDumpPtr, kCoeffPtr );
        }
    }
    for( freq = NB_BINS_COMPRESSED_SM_F0;
         freq < ( NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1 );
         freq++ )
    {
        kcoefficients_dump_1.kcoeff_blks[ (freq*KCOEFF_BLK_SIZE) + 1 ] = freq;
        bin = freq - NB_BINS_COMPRESSED_SM_F0;
//        printKCoefficients( freq, bin, k_coeff_intercalib_f1_norm);
        for ( coeff=0; coeff<NB_K_COEFF_PER_BIN; coeff++ )
        {
            kCoeffDumpPtr = (unsigned char*) &kcoefficients_dump_1.kcoeff_blks[
                    (freq*KCOEFF_BLK_SIZE) + (coeff*NB_BYTES_PER_FLOAT) + KCOEFF_FREQ
                    ]; // 2 for the kcoeff_frequency
            kCoeffPtr     = (unsigned char*) &k_coeff_intercalib_f1_norm[ (bin*NB_K_COEFF_PER_BIN) + coeff ];
            copyFloatByChar( kCoeffDumpPtr, kCoeffPtr );
        }
    }
    for( freq = ( NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1 );
         freq < KCOEFF_BLK_NR_PKT1 ;
         freq++ )
    {
        kcoefficients_dump_1.kcoeff_blks[ (freq * KCOEFF_BLK_SIZE) + 1 ] = freq;
        bin = freq - (NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1);
//        printKCoefficients( freq, bin, k_coeff_intercalib_f2);
        for ( coeff = 0; coeff <NB_K_COEFF_PER_BIN; coeff++ )
        {
            kCoeffDumpPtr = (unsigned char*) &kcoefficients_dump_1.kcoeff_blks[
                    (freq * KCOEFF_BLK_SIZE) + (coeff * NB_BYTES_PER_FLOAT) + KCOEFF_FREQ
                    ]; // 2 for the kcoeff_frequency
            kCoeffPtr     = (unsigned char*) &k_coeff_intercalib_f2[ (bin*NB_K_COEFF_PER_BIN) + coeff ];
            copyFloatByChar( kCoeffDumpPtr, kCoeffPtr );
        }
    }
    kcoefficients_dump_1.time[BYTE_0] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_3_BYTES);
    kcoefficients_dump_1.time[BYTE_1] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_2_BYTES);
    kcoefficients_dump_1.time[BYTE_2] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_1_BYTE);
    kcoefficients_dump_1.time[BYTE_3] = (unsigned char) (time_management_regs->coarse_time);
    kcoefficients_dump_1.time[BYTE_4] = (unsigned char) (time_management_regs->fine_time >> SHIFT_1_BYTE);
    kcoefficients_dump_1.time[BYTE_5] = (unsigned char) (time_management_regs->fine_time);
    // SEND DATA
    kcoefficient_node_1.status = 1;
    address = (unsigned int) &kcoefficient_node_1;
    status =  rtems_message_queue_send( queue_id, &address, sizeof( ring_node* ) );
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF1("in action_dump_kcoefficients *** ERR sending packet 1 , code %d", status)
    }

     //********
    // PACKET 2
    // 6 F2 bins
    kcoefficients_dump_2.destinationID = TC->sourceID;
    increment_seq_counter_destination_id_dump( kcoefficients_dump_2.packetSequenceControl, TC->sourceID );
    for( freq = 0;
         freq < KCOEFF_BLK_NR_PKT2;
         freq++ )
    {
        kcoefficients_dump_2.kcoeff_blks[ (freq*KCOEFF_BLK_SIZE) + 1 ] = KCOEFF_BLK_NR_PKT1 + freq;
        bin = freq + KCOEFF_BLK_NR_PKT2;
//        printKCoefficients( freq, bin, k_coeff_intercalib_f2);
        for ( coeff=0; coeff<NB_K_COEFF_PER_BIN; coeff++ )
        {
            kCoeffDumpPtr = (unsigned char*) &kcoefficients_dump_2.kcoeff_blks[
                    (freq*KCOEFF_BLK_SIZE) + (coeff*NB_BYTES_PER_FLOAT) + KCOEFF_FREQ ]; // 2 for the kcoeff_frequency
            kCoeffPtr     = (unsigned char*) &k_coeff_intercalib_f2[ (bin*NB_K_COEFF_PER_BIN) + coeff ];
            copyFloatByChar( kCoeffDumpPtr, kCoeffPtr );
        }
    }
    kcoefficients_dump_2.time[BYTE_0] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_3_BYTES);
    kcoefficients_dump_2.time[BYTE_1] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_2_BYTES);
    kcoefficients_dump_2.time[BYTE_2] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_1_BYTE);
    kcoefficients_dump_2.time[BYTE_3] = (unsigned char) (time_management_regs->coarse_time);
    kcoefficients_dump_2.time[BYTE_4] = (unsigned char) (time_management_regs->fine_time >> SHIFT_1_BYTE);
    kcoefficients_dump_2.time[BYTE_5] = (unsigned char) (time_management_regs->fine_time);
    // SEND DATA
    kcoefficient_node_2.status = 1;
    address = (unsigned int) &kcoefficient_node_2;
    status =  rtems_message_queue_send( queue_id, &address, sizeof( ring_node* ) );
    if (status != RTEMS_SUCCESSFUL) {
        PRINTF1("in action_dump_kcoefficients *** ERR sending packet 2, code %d", status)
    }

    return status;
}

int action_dump_par( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
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

    increment_seq_counter_destination_id_dump( parameter_dump_packet.packetSequenceControl, TC->sourceID );
    parameter_dump_packet.destinationID = TC->sourceID;

    // UPDATE TIME
    parameter_dump_packet.time[BYTE_0] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_3_BYTES);
    parameter_dump_packet.time[BYTE_1] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_2_BYTES);
    parameter_dump_packet.time[BYTE_2] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_3] = (unsigned char) (time_management_regs->coarse_time);
    parameter_dump_packet.time[BYTE_4] = (unsigned char) (time_management_regs->fine_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_5] = (unsigned char) (time_management_regs->fine_time);
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

int check_normal_par_consistency( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
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
    sy_lfr_n_swf_l = (msb * CONST_256) + lsb;

    msb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P ];
    lsb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_SWF_P+1 ];
    sy_lfr_n_swf_p = (msb * CONST_256)  + lsb;

    msb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P ];
    lsb = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_ASM_P+1 ];
    sy_lfr_n_asm_p = (msb * CONST_256)  + lsb;

    sy_lfr_n_bp_p0 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P0 ];

    sy_lfr_n_bp_p1 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_BP_P1 ];

    sy_lfr_n_cwf_long_f3 = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3 ];

    //******************
    // check consistency
    // sy_lfr_n_swf_l
    if (sy_lfr_n_swf_l != DFLT_SY_LFR_N_SWF_L)
    {
        status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_L + DATAFIELD_OFFSET, sy_lfr_n_swf_l );
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_swf_p
    if (flag == LFR_SUCCESSFUL)
    {
        if ( sy_lfr_n_swf_p < MIN_SY_LFR_N_SWF_P )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_P + DATAFIELD_OFFSET, sy_lfr_n_swf_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_bp_p0 < DFLT_SY_LFR_N_BP_P0)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P0 + DATAFIELD_OFFSET, sy_lfr_n_bp_p0 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_asm_p
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_asm_p == 0)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P + DATAFIELD_OFFSET, sy_lfr_n_asm_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_asm_p shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_n_asm_p / sy_lfr_n_bp_p0 ) - floor(sy_lfr_n_asm_p / sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P + DATAFIELD_OFFSET, sy_lfr_n_asm_p );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        if (sy_lfr_n_bp_p1 < DFLT_SY_LFR_N_BP_P1)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1 + DATAFIELD_OFFSET, sy_lfr_n_bp_p1 );
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p1 shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ( (float ) sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0 ) - floor(sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1 + DATAFIELD_OFFSET, sy_lfr_n_bp_p1 );
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
int set_sy_lfr_s2_bp_p0( ccsdsTelecommandPacket_t *TC )
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

void getReactionWheelsFrequencies( ccsdsTelecommandPacket_t *TC )
{
    /** This function get the reaction wheels frequencies in the incoming TC_LFR_UPDATE_INFO and copy the values locally.
     *
     * @param TC points to the TeleCommand packet that is being processed
     *
     */

    unsigned char * bytePosPtr; // pointer to the beginning of the incoming TC packet

    bytePosPtr = (unsigned char *) &TC->packetID;

    // cp_rpw_sc_rw1_f1
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw1_f1,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F1 ] );

    // cp_rpw_sc_rw1_f2
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw1_f2,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F2 ] );

    // cp_rpw_sc_rw2_f1
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw2_f1,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F1 ] );

    // cp_rpw_sc_rw2_f2
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw2_f2,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F2 ] );

    // cp_rpw_sc_rw3_f1
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw3_f1,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F1 ] );

    // cp_rpw_sc_rw3_f2
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw3_f2,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F2 ] );

    // cp_rpw_sc_rw4_f1
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw4_f1,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F1 ] );

    // cp_rpw_sc_rw4_f2
    copyFloatByChar( (unsigned char*) &cp_rpw_sc_rw4_f2,
                     (unsigned char*) &bytePosPtr[ BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F2 ] );
}

void setFBinMask( unsigned char *fbins_mask, float rw_f, unsigned char deltaFreq, unsigned char flag )
{
    /** This function executes specific actions when a TC_LFR_UPDATE_INFO TeleCommand has been received.
     *
     * @param fbins_mask
     * @param rw_f is the reaction wheel frequency to filter
     * @param delta_f is the frequency step between the frequency bins, it depends on the frequency channel
     * @param flag [true] filtering enabled [false] filtering disabled
     *
     * @return void
     *
     */

    float f_RW_min;
    float f_RW_MAX;
    float fi_min;
    float fi_MAX;
    float fi;
    float deltaBelow;
    float deltaAbove;
    int binBelow;
    int binAbove;
    int closestBin;
    unsigned int whichByte;
    int selectedByte;
    int bin;
    int binToRemove[NB_BINS_TO_REMOVE];
    int k;

    whichByte = 0;
    bin = 0;

    for (k = 0; k < NB_BINS_TO_REMOVE; k++)
    {
        binToRemove[k] = -1;
    }

    // compute the frequency range to filter [ rw_f - delta_f/2; rw_f + delta_f/2 ]
    f_RW_min = rw_f - (filterPar.sy_lfr_sc_rw_delta_f / 2.);
    f_RW_MAX = rw_f + (filterPar.sy_lfr_sc_rw_delta_f / 2.);

    // compute the index of the frequency bin immediately below rw_f
    binBelow = (int) ( floor( ((double) rw_f) / ((double) deltaFreq)) );
    deltaBelow = rw_f - binBelow * deltaFreq;

    // compute the index of the frequency bin immediately above rw_f
    binAbove = (int) ( ceil(  ((double) rw_f) / ((double) deltaFreq)) );
    deltaAbove = binAbove * deltaFreq - rw_f;

    // search the closest bin
    if (deltaAbove > deltaBelow)
    {
        closestBin = binBelow;
    }
    else
    {
        closestBin = binAbove;
    }

    // compute the fi interval [fi - deltaFreq * 0.285, fi + deltaFreq * 0.285]
    fi = closestBin * deltaFreq;
    fi_min = fi - (deltaFreq * FI_INTERVAL_COEFF);
    fi_MAX = fi + (deltaFreq * FI_INTERVAL_COEFF);

    //**************************************************************************************
    // be careful here, one shall take into account that the bin 0 IS DROPPED in the spectra
    // thus, the index 0 in a mask corresponds to the bin 1 of the spectrum
    //**************************************************************************************

    // 1. IF [ f_RW_min, f_RW_MAX] is included in [ fi_min; fi_MAX ]
    // => remove f_(i), f_(i-1) and f_(i+1)
    if ( ( f_RW_min > fi_min ) && ( f_RW_MAX < fi_MAX ) )
    {
        binToRemove[0] = (closestBin - 1) - 1;
        binToRemove[1] = (closestBin)     - 1;
        binToRemove[2] = (closestBin + 1) - 1;
    }
    // 2. ELSE
    // => remove the two f_(i) which are around f_RW
    else
    {
        binToRemove[0] = (binBelow) - 1;
        binToRemove[1] = (binAbove) - 1;
        binToRemove[2] = (-1);
    }

    for (k = 0; k < NB_BINS_TO_REMOVE; k++)
    {
        bin = binToRemove[k];
        if ( (bin >= BIN_MIN) && (bin <= BIN_MAX) )
        {
            if (flag == 1)
            {
                whichByte = (bin >> SHIFT_3_BITS);    // division by 8
                selectedByte = ( 1 << (bin - (whichByte * BITS_PER_BYTE)) );
                fbins_mask[BYTES_PER_MASK - 1 - whichByte] =
                        fbins_mask[BYTES_PER_MASK - 1 - whichByte] & ((unsigned char) (~selectedByte)); // bytes are ordered MSB first in the packets
            }
        }
    }
}

void build_sy_lfr_rw_mask( unsigned int channel )
{
    unsigned char local_rw_fbins_mask[BYTES_PER_MASK];
    unsigned char *maskPtr;
    double deltaF;
    unsigned k;

    k = 0;

    maskPtr = NULL;
    deltaF = DELTAF_F2;

    switch (channel)
    {
    case CHANNELF0:
        maskPtr = parameter_dump_packet.sy_lfr_rw_mask.fx.f0_word1;
        deltaF = DELTAF_F0;
        break;
    case CHANNELF1:
        maskPtr = parameter_dump_packet.sy_lfr_rw_mask.fx.f1_word1;
        deltaF = DELTAF_F1;
        break;
    case CHANNELF2:
        maskPtr = parameter_dump_packet.sy_lfr_rw_mask.fx.f2_word1;
        deltaF = DELTAF_F2;
        break;
    default:
        break;
    }

    for (k = 0; k < BYTES_PER_MASK; k++)
    {
        local_rw_fbins_mask[k] = INT8_ALL_F;
    }

    // RW1 F1
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw1_f1, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW1_F1) >> SHIFT_7_BITS );   // [1000 0000]

    // RW1 F2
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw1_f2, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW1_F2) >> SHIFT_6_BITS );   // [0100 0000]

    // RW2 F1
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw2_f1, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW2_F1) >> SHIFT_5_BITS );   // [0010 0000]

    // RW2 F2
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw2_f2, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW2_F2) >> SHIFT_4_BITS );   // [0001 0000]

    // RW3 F1
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw3_f1, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW3_F1) >> SHIFT_3_BITS );   // [0000 1000]

    // RW3 F2
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw3_f2, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW3_F2) >> SHIFT_2_BITS );   // [0000 0100]

    // RW4 F1
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw4_f1, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW4_F1) >> 1 );   // [0000 0010]

    // RW4 F2
    setFBinMask( local_rw_fbins_mask, cp_rpw_sc_rw4_f2, deltaF, (cp_rpw_sc_rw_f_flags & BIT_RW4_F2)       );  // [0000 0001]

    // update the value of the fbins related to reaction wheels frequency filtering
    if (maskPtr != NULL)
    {
        for (k = 0; k < BYTES_PER_MASK; k++)
        {
            maskPtr[k] = local_rw_fbins_mask[k];
        }
    }
}

void build_sy_lfr_rw_masks( void )
{
    build_sy_lfr_rw_mask( CHANNELF0 );
    build_sy_lfr_rw_mask( CHANNELF1 );
    build_sy_lfr_rw_mask( CHANNELF2 );
}

void merge_fbins_masks( void )
{
    unsigned char k;

    unsigned char *fbins_f0;
    unsigned char *fbins_f1;
    unsigned char *fbins_f2;
    unsigned char *rw_mask_f0;
    unsigned char *rw_mask_f1;
    unsigned char *rw_mask_f2;

    fbins_f0 = parameter_dump_packet.sy_lfr_fbins.fx.f0_word1;
    fbins_f1 = parameter_dump_packet.sy_lfr_fbins.fx.f1_word1;
    fbins_f2 = parameter_dump_packet.sy_lfr_fbins.fx.f2_word1;
    rw_mask_f0 = parameter_dump_packet.sy_lfr_rw_mask.fx.f0_word1;
    rw_mask_f1 = parameter_dump_packet.sy_lfr_rw_mask.fx.f1_word1;
    rw_mask_f2 = parameter_dump_packet.sy_lfr_rw_mask.fx.f2_word1;

    for( k=0; k < BYTES_PER_MASK; k++ )
    {
        fbins_masks.merged_fbins_mask_f0[k] = fbins_f0[k] & rw_mask_f0[k];
        fbins_masks.merged_fbins_mask_f1[k] = fbins_f1[k] & rw_mask_f1[k];
        fbins_masks.merged_fbins_mask_f2[k] = fbins_f2[k] & rw_mask_f2[k];
    }
}

//***********
// FBINS MASK

int set_sy_lfr_fbins( ccsdsTelecommandPacket_t *TC )
{
    int status;
    unsigned int k;
    unsigned char *fbins_mask_dump;
    unsigned char *fbins_mask_TC;

    status = LFR_SUCCESSFUL;

    fbins_mask_dump = parameter_dump_packet.sy_lfr_fbins.raw;
    fbins_mask_TC = TC->dataAndCRC;

    for (k=0; k < BYTES_PER_MASKS_SET; k++)
    {
        fbins_mask_dump[k] = fbins_mask_TC[k];
    }

    return status;
}

//***************************
// TC_LFR_LOAD_PAS_FILTER_PAR

int check_sy_lfr_filter_parameters( ccsdsTelecommandPacket_t *TC, rtems_id queue_id )
{
    int flag;
    rtems_status_code status;

    unsigned char sy_lfr_pas_filter_enabled;
    unsigned char sy_lfr_pas_filter_modulus;
    float sy_lfr_pas_filter_tbad;
    unsigned char sy_lfr_pas_filter_offset;
    float sy_lfr_pas_filter_shift;
    float sy_lfr_sc_rw_delta_f;
    char *parPtr;

    flag = LFR_SUCCESSFUL;
    sy_lfr_pas_filter_tbad  = INIT_FLOAT;
    sy_lfr_pas_filter_shift = INIT_FLOAT;
    sy_lfr_sc_rw_delta_f    = INIT_FLOAT;
    parPtr = NULL;

    //***************
    // get parameters
    sy_lfr_pas_filter_enabled   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_ENABLED ] & BIT_PAS_FILTER_ENABLED;   // [0000 0001]
    sy_lfr_pas_filter_modulus   = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS ];
    copyFloatByChar(
                (unsigned char*) &sy_lfr_pas_filter_tbad,
                (unsigned char*) &TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD ]
                );
    sy_lfr_pas_filter_offset    = TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET ];
    copyFloatByChar(
                (unsigned char*) &sy_lfr_pas_filter_shift,
                (unsigned char*) &TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT ]
                );
    copyFloatByChar(
                (unsigned char*) &sy_lfr_sc_rw_delta_f,
                (unsigned char*) &TC->dataAndCRC[ DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F ]
                );

    //******************
    // CHECK CONSISTENCY

    //**************************
    // sy_lfr_pas_filter_enabled
    // nothing to check, value is 0 or 1

    //**************************
    // sy_lfr_pas_filter_modulus
    if ( (sy_lfr_pas_filter_modulus < MIN_PAS_FILTER_MODULUS) || (sy_lfr_pas_filter_modulus > MAX_PAS_FILTER_MODULUS) )
    {
        status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS + DATAFIELD_OFFSET, sy_lfr_pas_filter_modulus );
        flag = WRONG_APP_DATA;
    }

    //***********************
    // sy_lfr_pas_filter_tbad
    if ( (sy_lfr_pas_filter_tbad < MIN_PAS_FILTER_TBAD) || (sy_lfr_pas_filter_tbad > MAX_PAS_FILTER_TBAD) )
    {
        parPtr = (char*) &sy_lfr_pas_filter_tbad;
        status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE] );
        flag = WRONG_APP_DATA;
    }

    //*************************
    // sy_lfr_pas_filter_offset
    if (flag == LFR_SUCCESSFUL)
    {
        if ( (sy_lfr_pas_filter_offset < MIN_PAS_FILTER_OFFSET) || (sy_lfr_pas_filter_offset > MAX_PAS_FILTER_OFFSET) )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET + DATAFIELD_OFFSET, sy_lfr_pas_filter_offset );
            flag = WRONG_APP_DATA;
        }
    }

    //************************
    // sy_lfr_pas_filter_shift
    if (flag == LFR_SUCCESSFUL)
    {
        if ( (sy_lfr_pas_filter_shift < MIN_PAS_FILTER_SHIFT) || (sy_lfr_pas_filter_shift > MAX_PAS_FILTER_SHIFT) )
        {
            parPtr = (char*) &sy_lfr_pas_filter_shift;
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE] );
            flag = WRONG_APP_DATA;
        }
    }

    //*************************************
    // check global coherency of the values
    if (flag == LFR_SUCCESSFUL)
    {
        if ( (sy_lfr_pas_filter_tbad + sy_lfr_pas_filter_offset + sy_lfr_pas_filter_shift) > sy_lfr_pas_filter_modulus )
        {
            status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS + DATAFIELD_OFFSET, sy_lfr_pas_filter_modulus );
            flag = WRONG_APP_DATA;
        }
    }

    //*********************
    // sy_lfr_sc_rw_delta_f
    // nothing to check, no default value in the ICD

    return flag;
}

//**************
// KCOEFFICIENTS
int set_sy_lfr_kcoeff( ccsdsTelecommandPacket_t *TC,rtems_id queue_id )
{
    unsigned int kcoeff;
    unsigned short sy_lfr_kcoeff_frequency;
    unsigned short bin;
    unsigned short *freqPtr;
    float *kcoeffPtr_norm;
    float *kcoeffPtr_sbm;
    int status;
    unsigned char *kcoeffLoadPtr;
    unsigned char *kcoeffNormPtr;
    unsigned char *kcoeffSbmPtr_a;
    unsigned char *kcoeffSbmPtr_b;

    status = LFR_SUCCESSFUL;

    kcoeffPtr_norm = NULL;
    kcoeffPtr_sbm  = NULL;
    bin = 0;

    freqPtr = (unsigned short *) &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY];
    sy_lfr_kcoeff_frequency = *freqPtr;

    if ( sy_lfr_kcoeff_frequency >= NB_BINS_COMPRESSED_SM )
    {
        PRINTF1("ERR *** in set_sy_lfr_kcoeff_frequency *** sy_lfr_kcoeff_frequency = %d\n", sy_lfr_kcoeff_frequency)
        status = send_tm_lfr_tc_exe_inconsistent( TC, queue_id, DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY + DATAFIELD_OFFSET + 1,
                                                  TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY + 1]  ); // +1 to get the LSB instead of the MSB
        status = LFR_DEFAULT;
    }
    else
    {
        if       ( ( sy_lfr_kcoeff_frequency >= 0 )
                && ( sy_lfr_kcoeff_frequency < NB_BINS_COMPRESSED_SM_F0 ) )
        {
            kcoeffPtr_norm = k_coeff_intercalib_f0_norm;
            kcoeffPtr_sbm  = k_coeff_intercalib_f0_sbm;
            bin   = sy_lfr_kcoeff_frequency;
        }
        else if   ( ( sy_lfr_kcoeff_frequency >= NB_BINS_COMPRESSED_SM_F0 )
                 && ( sy_lfr_kcoeff_frequency < (NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1) ) )
        {
            kcoeffPtr_norm = k_coeff_intercalib_f1_norm;
            kcoeffPtr_sbm  = k_coeff_intercalib_f1_sbm;
            bin   = sy_lfr_kcoeff_frequency -  NB_BINS_COMPRESSED_SM_F0;
        }
        else if   ( ( sy_lfr_kcoeff_frequency >= (NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1) )
                 && ( sy_lfr_kcoeff_frequency <  (NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1 + NB_BINS_COMPRESSED_SM_F2) ) )
        {
            kcoeffPtr_norm = k_coeff_intercalib_f2;
            kcoeffPtr_sbm  = NULL;
            bin   = sy_lfr_kcoeff_frequency - (NB_BINS_COMPRESSED_SM_F0 + NB_BINS_COMPRESSED_SM_F1);
        }
    }

    if (kcoeffPtr_norm != NULL )    // update K coefficient for NORMAL data products
    {
        for (kcoeff=0; kcoeff<NB_K_COEFF_PER_BIN; kcoeff++)
        {
            // destination
            kcoeffNormPtr = (unsigned char*) &kcoeffPtr_norm[ (bin * NB_K_COEFF_PER_BIN) + kcoeff ];
            // source
            kcoeffLoadPtr = (unsigned char*) &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_1 + (NB_BYTES_PER_FLOAT * kcoeff)];
            // copy source to destination
            copyFloatByChar( kcoeffNormPtr,  kcoeffLoadPtr );
        }
    }

    if (kcoeffPtr_sbm != NULL )     // update K coefficient for SBM data products
    {
        for (kcoeff=0; kcoeff<NB_K_COEFF_PER_BIN; kcoeff++)
        {
            // destination
            kcoeffSbmPtr_a= (unsigned char*) &kcoeffPtr_sbm[ ( (bin * NB_K_COEFF_PER_BIN) + kcoeff) * SBM_COEFF_PER_NORM_COEFF        ];
            kcoeffSbmPtr_b= (unsigned char*) &kcoeffPtr_sbm[ (((bin * NB_K_COEFF_PER_BIN) + kcoeff) * SBM_KCOEFF_PER_NORM_KCOEFF) + 1 ];
            // source
            kcoeffLoadPtr = (unsigned char*) &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_1 + (NB_BYTES_PER_FLOAT * kcoeff)];
            // copy source to destination
            copyFloatByChar( kcoeffSbmPtr_a, kcoeffLoadPtr );
            copyFloatByChar( kcoeffSbmPtr_b, kcoeffLoadPtr );
        }
    }

//    print_k_coeff();

    return status;
}

void copyFloatByChar( unsigned char *destination, unsigned char *source )
{
    destination[BYTE_0] = source[BYTE_0];
    destination[BYTE_1] = source[BYTE_1];
    destination[BYTE_2] = source[BYTE_2];
    destination[BYTE_3] = source[BYTE_3];
}

void floatToChar( float value, unsigned char* ptr)
{
    unsigned char* valuePtr;

    valuePtr = (unsigned char*) &value;
    ptr[BYTE_0] = valuePtr[BYTE_0];
    ptr[BYTE_1] = valuePtr[BYTE_1];
    ptr[BYTE_2] = valuePtr[BYTE_2];
    ptr[BYTE_3] = valuePtr[BYTE_3];
}

//**********
// init dump

void init_parameter_dump( void )
{
    /** This function initialize the parameter_dump_packet global variable with default values.
     *
     */

    unsigned int k;

    parameter_dump_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    parameter_dump_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    parameter_dump_packet.reserved = CCSDS_RESERVED;
    parameter_dump_packet.userApplication = CCSDS_USER_APP;
    parameter_dump_packet.packetID[0] = (unsigned char) (APID_TM_PARAMETER_DUMP >> SHIFT_1_BYTE);
    parameter_dump_packet.packetID[1] = (unsigned char) APID_TM_PARAMETER_DUMP;
    parameter_dump_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    parameter_dump_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    parameter_dump_packet.packetLength[0] = (unsigned char) (PACKET_LENGTH_PARAMETER_DUMP >> SHIFT_1_BYTE);
    parameter_dump_packet.packetLength[1] = (unsigned char) PACKET_LENGTH_PARAMETER_DUMP;
    // DATA FIELD HEADER
    parameter_dump_packet.spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    parameter_dump_packet.serviceType = TM_TYPE_PARAMETER_DUMP;
    parameter_dump_packet.serviceSubType = TM_SUBTYPE_PARAMETER_DUMP;
    parameter_dump_packet.destinationID = TM_DESTINATION_ID_GROUND;
    parameter_dump_packet.time[BYTE_0] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_3_BYTES);
    parameter_dump_packet.time[BYTE_1] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_2_BYTES);
    parameter_dump_packet.time[BYTE_2] = (unsigned char) (time_management_regs->coarse_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_3] = (unsigned char) (time_management_regs->coarse_time);
    parameter_dump_packet.time[BYTE_4] = (unsigned char) (time_management_regs->fine_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_5] = (unsigned char) (time_management_regs->fine_time);
    parameter_dump_packet.sid = SID_PARAMETER_DUMP;

    //******************
    // COMMON PARAMETERS
    parameter_dump_packet.sy_lfr_common_parameters_spare    = DEFAULT_SY_LFR_COMMON0;
    parameter_dump_packet.sy_lfr_common_parameters          = DEFAULT_SY_LFR_COMMON1;

    //******************
    // NORMAL PARAMETERS
    parameter_dump_packet.sy_lfr_n_swf_l[0] = (unsigned char) (DFLT_SY_LFR_N_SWF_L >> SHIFT_1_BYTE);
    parameter_dump_packet.sy_lfr_n_swf_l[1] = (unsigned char) (DFLT_SY_LFR_N_SWF_L     );
    parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char) (DFLT_SY_LFR_N_SWF_P >> SHIFT_1_BYTE);
    parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char) (DFLT_SY_LFR_N_SWF_P     );
    parameter_dump_packet.sy_lfr_n_asm_p[0] = (unsigned char) (DFLT_SY_LFR_N_ASM_P >> SHIFT_1_BYTE);
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

    //************
    // FBINS MASKS
    for (k=0; k < BYTES_PER_MASKS_SET; k++)
    {
        parameter_dump_packet.sy_lfr_fbins.raw[k] = INT8_ALL_F;
    }

    // PAS FILTER PARAMETERS
    parameter_dump_packet.pa_rpw_spare8_2                   = INIT_CHAR;
    parameter_dump_packet.spare_sy_lfr_pas_filter_enabled   = INIT_CHAR;
    parameter_dump_packet.sy_lfr_pas_filter_modulus         = DEFAULT_SY_LFR_PAS_FILTER_MODULUS;
    floatToChar( DEFAULT_SY_LFR_PAS_FILTER_TBAD,    parameter_dump_packet.sy_lfr_pas_filter_tbad );
    parameter_dump_packet.sy_lfr_pas_filter_offset          = DEFAULT_SY_LFR_PAS_FILTER_OFFSET;
    floatToChar( DEFAULT_SY_LFR_PAS_FILTER_SHIFT,   parameter_dump_packet.sy_lfr_pas_filter_shift );
    floatToChar( DEFAULT_SY_LFR_SC_RW_DELTA_F,      parameter_dump_packet.sy_lfr_sc_rw_delta_f );

    // LFR_RW_MASK
    for (k=0; k < BYTES_PER_MASKS_SET; k++)
    {
        parameter_dump_packet.sy_lfr_rw_mask.raw[k] = INT8_ALL_F;
    }

    // once the reaction wheels masks have been initialized, they have to be merged with the fbins masks
    merge_fbins_masks();
}

void init_kcoefficients_dump( void )
{   
    init_kcoefficients_dump_packet( &kcoefficients_dump_1, PKTNR_1, KCOEFF_BLK_NR_PKT1 );
    init_kcoefficients_dump_packet( &kcoefficients_dump_2, PKTNR_2, KCOEFF_BLK_NR_PKT2  );

    kcoefficient_node_1.previous = NULL;
    kcoefficient_node_1.next = NULL;
    kcoefficient_node_1.sid = TM_CODE_K_DUMP;
    kcoefficient_node_1.coarseTime = INIT_CHAR;
    kcoefficient_node_1.fineTime = INIT_CHAR;
    kcoefficient_node_1.buffer_address = (int) &kcoefficients_dump_1;
    kcoefficient_node_1.status = INIT_CHAR;

    kcoefficient_node_2.previous = NULL;
    kcoefficient_node_2.next = NULL;
    kcoefficient_node_2.sid = TM_CODE_K_DUMP;
    kcoefficient_node_2.coarseTime = INIT_CHAR;
    kcoefficient_node_2.fineTime = INIT_CHAR;
    kcoefficient_node_2.buffer_address = (int) &kcoefficients_dump_2;
    kcoefficient_node_2.status = INIT_CHAR;
}

void init_kcoefficients_dump_packet( Packet_TM_LFR_KCOEFFICIENTS_DUMP_t *kcoefficients_dump, unsigned char pkt_nr, unsigned char blk_nr )
{
    unsigned int k;
    unsigned int packetLength;

    packetLength =
            ((blk_nr * KCOEFF_BLK_SIZE) + BYTE_POS_KCOEFFICIENTS_PARAMETES) - CCSDS_TC_TM_PACKET_OFFSET; // 4 bytes for the CCSDS header

    kcoefficients_dump->targetLogicalAddress = CCSDS_DESTINATION_ID;
    kcoefficients_dump->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    kcoefficients_dump->reserved = CCSDS_RESERVED;
    kcoefficients_dump->userApplication = CCSDS_USER_APP;
    kcoefficients_dump->packetID[0] = (unsigned char) (APID_TM_PARAMETER_DUMP >> SHIFT_1_BYTE);
    kcoefficients_dump->packetID[1] = (unsigned char) APID_TM_PARAMETER_DUMP;
    kcoefficients_dump->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    kcoefficients_dump->packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    kcoefficients_dump->packetLength[0] = (unsigned char) (packetLength >> SHIFT_1_BYTE);
    kcoefficients_dump->packetLength[1] = (unsigned char) packetLength;
    // DATA FIELD HEADER
    kcoefficients_dump->spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    kcoefficients_dump->serviceType = TM_TYPE_K_DUMP;
    kcoefficients_dump->serviceSubType = TM_SUBTYPE_K_DUMP;
    kcoefficients_dump->destinationID= TM_DESTINATION_ID_GROUND;
    kcoefficients_dump->time[BYTE_0] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_1] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_2] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_3] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_4] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_5] = INIT_CHAR;
    kcoefficients_dump->sid = SID_K_DUMP;

    kcoefficients_dump->pkt_cnt = KCOEFF_PKTCNT;
    kcoefficients_dump->pkt_nr = PKTNR_1;
    kcoefficients_dump->blk_nr = blk_nr;

    //******************
    // SOURCE DATA repeated N times with N in [0 .. PA_LFR_KCOEFF_BLK_NR]
    // one blk is 2 + 4 * 32 = 130 bytes, 30 blks max in one packet (30 * 130 = 3900)
    for (k=0; k<(KCOEFF_BLK_NR_PKT1 * KCOEFF_BLK_SIZE); k++)
    {
        kcoefficients_dump->kcoeff_blks[k] = INIT_CHAR;
    }
}

void increment_seq_counter_destination_id_dump( unsigned char *packet_sequence_control, unsigned char destination_id )
{
    /** This function increment the packet sequence control parameter of a TC, depending on its destination ID.
     *
     * @param packet_sequence_control points to the packet sequence control which will be incremented
     * @param destination_id is the destination ID of the TM, there is one counter by destination ID
     *
     * If the destination ID is not known, a dedicated counter is incremented.
     *
     */

    unsigned short sequence_cnt;
    unsigned short segmentation_grouping_flag;
    unsigned short new_packet_sequence_control;
    unsigned char i;

    switch (destination_id)
    {
    case SID_TC_GROUND:
        i = GROUND;
        break;
    case SID_TC_MISSION_TIMELINE:
        i = MISSION_TIMELINE;
        break;
    case SID_TC_TC_SEQUENCES:
        i = TC_SEQUENCES;
        break;
    case SID_TC_RECOVERY_ACTION_CMD:
        i = RECOVERY_ACTION_CMD;
        break;
    case SID_TC_BACKUP_MISSION_TIMELINE:
        i = BACKUP_MISSION_TIMELINE;
        break;
    case SID_TC_DIRECT_CMD:
        i = DIRECT_CMD;
        break;
    case SID_TC_SPARE_GRD_SRC1:
        i = SPARE_GRD_SRC1;
        break;
    case SID_TC_SPARE_GRD_SRC2:
        i = SPARE_GRD_SRC2;
        break;
    case SID_TC_OBCP:
        i = OBCP;
        break;
    case SID_TC_SYSTEM_CONTROL:
        i = SYSTEM_CONTROL;
        break;
    case SID_TC_AOCS:
        i = AOCS;
        break;
    case SID_TC_RPW_INTERNAL:
        i = RPW_INTERNAL;
        break;
    default:
        i = GROUND;
        break;
    }

    segmentation_grouping_flag  = TM_PACKET_SEQ_CTRL_STANDALONE << SHIFT_1_BYTE;
    sequence_cnt                = sequenceCounters_TM_DUMP[ i ] & SEQ_CNT_MASK;

    new_packet_sequence_control = segmentation_grouping_flag | sequence_cnt ;

    packet_sequence_control[0] = (unsigned char) (new_packet_sequence_control >> SHIFT_1_BYTE);
    packet_sequence_control[1] = (unsigned char) (new_packet_sequence_control     );

    // increment the sequence counter
    if ( sequenceCounters_TM_DUMP[ i ] < SEQ_CNT_MAX )
    {
        sequenceCounters_TM_DUMP[ i ] = sequenceCounters_TM_DUMP[ i ] + 1;
    }
    else
    {
        sequenceCounters_TM_DUMP[ i ] = 0;
    }
}

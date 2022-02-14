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

#include <math.h>
#include <string.h>

#include "fsw_compile_warnings.h"
#include "fsw_debug.h"
#include "fsw_housekeeping.h"
#include "fsw_misc.h"
#include "hw/lfr_regs.h"
#include "processing/calibration_matrices.h"
#include "tc_tm/tc_load_dump_parameters.h"


DISABLE_MISSING_FIELD_INITIALIZER_WARNING
Packet_TM_LFR_KCOEFFICIENTS_DUMP_t kcoefficients_dump_1 = { 0 };
Packet_TM_LFR_KCOEFFICIENTS_DUMP_t kcoefficients_dump_2 = { 0 };
ring_node kcoefficient_node_1 = { 0 };
ring_node kcoefficient_node_2 = { 0 };
ENABLE_MISSING_FIELD_INITIALIZER_WARNING

int action_load_common_par(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function updates the LFR registers with the incoming common parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     *
     *
     */

    parameter_dump_packet.sy_lfr_common_parameters_spare = TC->dataAndCRC[0];
    parameter_dump_packet.sy_lfr_common_parameters = TC->dataAndCRC[1];
    set_wfp_data_shaping();
    return LFR_SUCCESSFUL;
}

int action_load_normal_par(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming normal parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */
    IGNORE_UNUSED_PARAMETER(time);

    int flag = LFR_SUCCESSFUL;

    if ((lfrCurrentMode == LFR_MODE_NORMAL) || (lfrCurrentMode == LFR_MODE_SBM1)
        || (lfrCurrentMode == LFR_MODE_SBM2))
    {
        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_not_executable(TC, queue_id));
        flag = LFR_DEFAULT;
    }

    // CHECK THE PARAMETERS SET CONSISTENCY
    if (flag == LFR_SUCCESSFUL)
    {
        flag = check_normal_par_consistency(TC, queue_id);
    }

    // SET THE PARAMETERS IF THEY ARE CONSISTENT
    if (flag == LFR_SUCCESSFUL)
    {
        flag |= set_sy_lfr_n_swf_l(TC);
        flag |= set_sy_lfr_n_swf_p(TC);
        flag |= set_sy_lfr_n_bp_p0(TC);
        flag |= set_sy_lfr_n_bp_p1(TC);
        flag |= set_sy_lfr_n_asm_p(TC);
        flag |= set_sy_lfr_n_cwf_long_f3(TC);
    }

    return flag;
}

int action_load_burst_par(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming burst parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */
    IGNORE_UNUSED_PARAMETER(time);
    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_b_bp_p0;
    unsigned char sy_lfr_b_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if (lfrCurrentMode == LFR_MODE_BURST)
    {
        status = send_tm_lfr_tc_exe_not_executable(TC, queue_id);
        DEBUG_CHECK_STATUS(status);
        flag = LFR_DEFAULT;
    }

    sy_lfr_b_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P0];
    sy_lfr_b_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P1];

    // sy_lfr_b_bp_p0 shall not be lower than its default value
    if (flag == LFR_SUCCESSFUL && sy_lfr_b_bp_p0 < DEFAULT_SY_LFR_B_BP_P0)
    {
        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0 + DATAFIELD_OFFSET, sy_lfr_b_bp_p0);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_b_bp_p1 shall not be lower than its default value
    if (flag == LFR_SUCCESSFUL && sy_lfr_b_bp_p1 < DEFAULT_SY_LFR_B_BP_P1)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P1 + DATAFIELD_OFFSET, sy_lfr_b_bp_p1);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    //****************************************************************
    // check the consistency between sy_lfr_b_bp_p0 and sy_lfr_b_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        sy_lfr_b_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P0];
        sy_lfr_b_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P1];
        aux = ((float)sy_lfr_b_bp_p1 / sy_lfr_b_bp_p0) - floorf(sy_lfr_b_bp_p1 / sy_lfr_b_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, DATAFIELD_POS_SY_LFR_B_BP_P0 + DATAFIELD_OFFSET, sy_lfr_b_bp_p0);
            DEBUG_CHECK_STATUS(status);
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_b_bp_p0(TC);
        flag |= set_sy_lfr_b_bp_p1(TC);
    }

    return flag;
}

int action_load_sbm1_par(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming sbm1 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    IGNORE_UNUSED_PARAMETER(time);
    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_s1_bp_p0;
    unsigned char sy_lfr_s1_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if (lfrCurrentMode == LFR_MODE_SBM1)
    {
        status = send_tm_lfr_tc_exe_not_executable(TC, queue_id);
        DEBUG_CHECK_STATUS(status);
        flag = LFR_DEFAULT;
    }

    sy_lfr_s1_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S1_BP_P0];
    sy_lfr_s1_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S1_BP_P1];

    // sy_lfr_s1_bp_p0
    if (flag == LFR_SUCCESSFUL && sy_lfr_s1_bp_p0 < DEFAULT_SY_LFR_S1_BP_P0)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p0);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL && sy_lfr_s1_bp_p1 < DEFAULT_SY_LFR_S1_BP_P1)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P1 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p1);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    //******************************************************************
    // check the consistency between sy_lfr_s1_bp_p0 and sy_lfr_s1_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ((float)sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0 * (float)S1_BP_P0_SCALE))
            - floorf(sy_lfr_s1_bp_p1 / (sy_lfr_s1_bp_p0 * (float)S1_BP_P0_SCALE));
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, DATAFIELD_POS_SY_LFR_S1_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s1_bp_p0);
            DEBUG_CHECK_STATUS(status);
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_s1_bp_p0(TC);
        flag |= set_sy_lfr_s1_bp_p1(TC);
    }

    return flag;
}

int action_load_sbm2_par(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    IGNORE_UNUSED_PARAMETER(time);
    int flag;
    rtems_status_code status;
    unsigned char sy_lfr_s2_bp_p0;
    unsigned char sy_lfr_s2_bp_p1;
    float aux;

    flag = LFR_SUCCESSFUL;

    if (lfrCurrentMode == LFR_MODE_SBM2)
    {
        status = send_tm_lfr_tc_exe_not_executable(TC, queue_id);
        DEBUG_CHECK_STATUS(status);
        flag = LFR_DEFAULT;
    }

    sy_lfr_s2_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P0];
    sy_lfr_s2_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P1];

    // sy_lfr_s2_bp_p0
    if (flag == LFR_SUCCESSFUL && sy_lfr_s2_bp_p0 < DEFAULT_SY_LFR_S2_BP_P0)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p0);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_s2_bp_p1
    if (flag == LFR_SUCCESSFUL && sy_lfr_s2_bp_p1 < DEFAULT_SY_LFR_S2_BP_P1)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P1 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p1);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    //******************************************************************
    // check the consistency between sy_lfr_s2_bp_p0 and sy_lfr_s2_bp_p1
    if (flag == LFR_SUCCESSFUL)
    {
        sy_lfr_s2_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P0];
        sy_lfr_s2_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P1];
        aux = ((float)sy_lfr_s2_bp_p1 / sy_lfr_s2_bp_p0)
            - floorf(sy_lfr_s2_bp_p1 / sy_lfr_s2_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, DATAFIELD_POS_SY_LFR_S2_BP_P0 + DATAFIELD_OFFSET, sy_lfr_s2_bp_p0);
            DEBUG_CHECK_STATUS(status);
            flag = LFR_DEFAULT;
        }
    }

    // SET THE PARAMETERS
    if (flag == LFR_SUCCESSFUL)
    {
        flag = set_sy_lfr_s2_bp_p0(TC);
        flag |= set_sy_lfr_s2_bp_p1(TC);
    }

    return flag;
}

int action_load_kcoefficients(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    IGNORE_UNUSED_PARAMETER(time);
    int flag = LFR_DEFAULT;
    rtems_status_code status;

    if (lfrCurrentMode != LFR_MODE_STANDBY)
    {
        status = send_tm_lfr_tc_exe_not_executable(TC, queue_id);
        DEBUG_CHECK_STATUS(status);
    }
    else
    {
        flag = set_sy_lfr_kcoeff(TC, queue_id);
    }

    return flag;
}

int action_load_fbins_mask(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    IGNORE_UNUSED_PARAMETER(time);
    IGNORE_UNUSED_PARAMETER(queue_id);
    int flag = set_sy_lfr_fbins(TC);

    // once the fbins masks have been stored, they have to be merged with the masks which handle the
    // reaction wheels frequencies filtering
    merge_fbins_masks();

    return flag;
}

int action_load_filter_par(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    int flag = check_sy_lfr_filter_parameters(TC, queue_id);

    if (flag == LFR_SUCCESSFUL)
    {
        parameter_dump_packet.spare_sy_lfr_pas_filter_enabled
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_ENABLED];
        parameter_dump_packet.sy_lfr_pas_filter_modulus
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_0]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_0];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_1]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_1];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_2]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_2];
        parameter_dump_packet.sy_lfr_pas_filter_tbad[BYTE_3]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + BYTE_3];
        parameter_dump_packet.sy_lfr_pas_filter_offset
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_0]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_0];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_1]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_1];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_2]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_2];
        parameter_dump_packet.sy_lfr_pas_filter_shift[BYTE_3]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + BYTE_3];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_0]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_0];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_1]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_1];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_2]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_2];
        parameter_dump_packet.sy_lfr_sc_rw_delta_f[BYTE_3]
            = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + BYTE_3];

        //****************************
        // store PAS filter parameters

        // sy_lfr_pas_filter_enabled
        filterPar.spare_sy_lfr_pas_filter_enabled
            = parameter_dump_packet.spare_sy_lfr_pas_filter_enabled;
        set_sy_lfr_pas_filter_enabled(
            parameter_dump_packet.spare_sy_lfr_pas_filter_enabled & BIT_PAS_FILTER_ENABLED);

        // sy_lfr_pas_filter_modulus
        filterPar.modulus_in_finetime
            = ((uint64_t)parameter_dump_packet.sy_lfr_pas_filter_modulus) * CONST_65536;

        // sy_lfr_pas_filter_tbad
        copyFloatByChar((unsigned char*)&filterPar.sy_lfr_pas_filter_tbad,
            parameter_dump_packet.sy_lfr_pas_filter_tbad);
        filterPar.tbad_in_finetime = (uint64_t)(filterPar.sy_lfr_pas_filter_tbad * CONST_65536);

        // sy_lfr_pas_filter_offset
        filterPar.offset_in_finetime
            = ((uint64_t)parameter_dump_packet.sy_lfr_pas_filter_offset) * CONST_65536;

        // sy_lfr_pas_filter_shift
        copyFloatByChar((unsigned char*)&filterPar.sy_lfr_pas_filter_shift,
            parameter_dump_packet.sy_lfr_pas_filter_shift);
        filterPar.shift_in_finetime = (uint64_t)(filterPar.sy_lfr_pas_filter_shift * CONST_65536);

        //****************************************************
        // store the parameter sy_lfr_sc_rw_delta_f as a float
        copyFloatByChar((unsigned char*)&filterPar.sy_lfr_sc_rw_delta_f,
            parameter_dump_packet.sy_lfr_sc_rw_delta_f);

        // copy rw.._k.. from the incoming TC to the local parameter_dump_packet
        for (unsigned char k = 0; k < NB_RW_K_COEFFS * NB_BYTES_PER_RW_K_COEFF; k++)
        {
            // TODO clean this, sy_lfr_rw1_k1 is a 4 bytes array, this is UB
            parameter_dump_packet.sy_lfr_rw1_k1[k]
                = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_RW1_K1 + k];
        }

        //***********************************************
        // store the parameter sy_lfr_rw.._k.. as a float
        // rw1_k
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw1_k1, parameter_dump_packet.sy_lfr_rw1_k1);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw1_k2, parameter_dump_packet.sy_lfr_rw1_k2);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw1_k3, parameter_dump_packet.sy_lfr_rw1_k3);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw1_k4, parameter_dump_packet.sy_lfr_rw1_k4);
        // rw2_k
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw2_k1, parameter_dump_packet.sy_lfr_rw2_k1);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw2_k2, parameter_dump_packet.sy_lfr_rw2_k2);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw2_k3, parameter_dump_packet.sy_lfr_rw2_k3);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw2_k4, parameter_dump_packet.sy_lfr_rw2_k4);
        // rw3_k
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw3_k1, parameter_dump_packet.sy_lfr_rw3_k1);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw3_k2, parameter_dump_packet.sy_lfr_rw3_k2);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw3_k3, parameter_dump_packet.sy_lfr_rw3_k3);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw3_k4, parameter_dump_packet.sy_lfr_rw3_k4);
        // rw4_k
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw4_k1, parameter_dump_packet.sy_lfr_rw4_k1);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw4_k2, parameter_dump_packet.sy_lfr_rw4_k2);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw4_k3, parameter_dump_packet.sy_lfr_rw4_k3);
        copyFloatByChar(
            (unsigned char*)&filterPar.sy_lfr_rw4_k4, parameter_dump_packet.sy_lfr_rw4_k4);
    }

    return flag;
}

int action_dump_kcoefficients(
    const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id, const unsigned char* const time)
{
    /** This function updates the LFR registers with the incoming sbm2 parameters.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    IGNORE_UNUSED_PARAMETER(time);
    void* address;
    rtems_status_code status;
    unsigned char* kCoeffDumpPtr;

    // Each packet is 3900 bytes
    // Packet1:
    // F0 needs 12 bins * 26 floats -> 1248 bytes
    // F1 needs 14 bins * 26 floats -> 1456 bytes
    // total -> 2704 bytes

    // Packet2:
    // F2 needs 13 bins * 26 floats -> 1352 bytes
    // total -> 1352 bytes


    //*********
    // PACKET 1
    // 12 F0 bins, 14 F1 bins
    kcoefficients_dump_1.destinationID = TC->sourceID;
    increment_seq_counter_destination_id_dump(
        kcoefficients_dump_1.packetSequenceControl, TC->sourceID);
    kCoeffDumpPtr = kcoefficients_dump_1.kcoeff_blks;
    for (int freq = 0; freq <= NB_BINS_COMPRESSED_SM_F0; freq++)
    {

        memcpy(kCoeffDumpPtr,
            mag_calibration_matrices_f0
                + (freq * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F0),
            NB_BYTES_MAG_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_MAG_CAL_MATRIX;
        memcpy(kCoeffDumpPtr,
            elec_calibration_matrices_f0
                + (freq * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F0),
            NB_BYTES_ELEC_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_ELEC_CAL_MATRIX;
    }
    for (int freq = 0; freq <= NB_BINS_COMPRESSED_SM_F1; freq++)
    {
        memcpy(kCoeffDumpPtr,
            mag_calibration_matrices_f1
                + (freq * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F1),
            NB_BYTES_MAG_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_MAG_CAL_MATRIX;
        memcpy(kCoeffDumpPtr,
            elec_calibration_matrices_f1
                + (freq * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F1),
            NB_BYTES_ELEC_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_ELEC_CAL_MATRIX;
    }

    kcoefficients_dump_1.time[BYTE_0]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
    kcoefficients_dump_1.time[BYTE_1]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
    kcoefficients_dump_1.time[BYTE_2]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
    kcoefficients_dump_1.time[BYTE_3] = (unsigned char)(time_management_regs->coarse_time);
    kcoefficients_dump_1.time[BYTE_4]
        = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
    kcoefficients_dump_1.time[BYTE_5] = (unsigned char)(time_management_regs->fine_time);
    // SEND DATA
    kcoefficient_node_1.status = 1;
    address = (void*)&kcoefficient_node_1;
    status = rtems_message_queue_send(queue_id, &address, sizeof(ring_node*));
    DEBUG_CHECK_STATUS(status);

    //********
    // PACKET 2
    // 13 F2 bins
    kcoefficients_dump_2.destinationID = TC->sourceID;
    increment_seq_counter_destination_id_dump(
        kcoefficients_dump_2.packetSequenceControl, TC->sourceID);

    kCoeffDumpPtr = kcoefficients_dump_2.kcoeff_blks;
    for (int freq = 0; freq <= NB_BINS_COMPRESSED_SM_F2; freq++)
    {

        memcpy(kCoeffDumpPtr,
            mag_calibration_matrices_f2
                + (freq * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F2),
            NB_BYTES_MAG_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_MAG_CAL_MATRIX;
        memcpy(kCoeffDumpPtr,
            elec_calibration_matrices_f2
                + (freq * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F2),
            NB_BYTES_ELEC_CAL_MATRIX);
        kCoeffDumpPtr += NB_BYTES_ELEC_CAL_MATRIX;
    }

    kcoefficients_dump_2.time[BYTE_0]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
    kcoefficients_dump_2.time[BYTE_1]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
    kcoefficients_dump_2.time[BYTE_2]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
    kcoefficients_dump_2.time[BYTE_3] = (unsigned char)(time_management_regs->coarse_time);
    kcoefficients_dump_2.time[BYTE_4]
        = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
    kcoefficients_dump_2.time[BYTE_5] = (unsigned char)(time_management_regs->fine_time);
    // SEND DATA
    kcoefficient_node_2.status = 1;
    address = (void*)&kcoefficient_node_2;
    status = rtems_message_queue_send(queue_id, &address, sizeof(ring_node*));
    DEBUG_CHECK_STATUS(status);

    return status;
}

int action_dump_par(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
{
    /** This function dumps the LFR parameters by sending the appropriate TM packet to the dedicated
     * RTEMS message queue.
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

    increment_seq_counter_destination_id_dump(
        parameter_dump_packet.packetSequenceControl, TC->sourceID);
    parameter_dump_packet.destinationID = TC->sourceID;

    // UPDATE TIME
    parameter_dump_packet.time[BYTE_0]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
    parameter_dump_packet.time[BYTE_1]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
    parameter_dump_packet.time[BYTE_2]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_3] = (unsigned char)(time_management_regs->coarse_time);
    parameter_dump_packet.time[BYTE_4]
        = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_5] = (unsigned char)(time_management_regs->fine_time);
    // SEND DATA
    status
        = rtems_message_queue_send(queue_id, &parameter_dump_packet, sizeof(parameter_dump_packet));
    DEBUG_CHECK_STATUS(status);

    return status;
}

//***********************
// NORMAL MODE PARAMETERS
// See https://hephaistos.lpp.polytechnique.fr/redmine/issues/481
// and https://hephaistos.lpp.polytechnique.fr/redmine/issues/482
int check_normal_par_consistency(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
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

    flag = LFR_SUCCESSFUL;

    //***************
    // get parameters
    msb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_L];
    lsb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_L + 1];
    sy_lfr_n_swf_l = (msb * CONST_256) + lsb;

    msb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_P];
    lsb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_P + 1];
    sy_lfr_n_swf_p = (msb * CONST_256) + lsb;

    msb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_ASM_P];
    lsb = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_ASM_P + 1];
    sy_lfr_n_asm_p = (msb * CONST_256) + lsb;

    sy_lfr_n_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_BP_P0];

    sy_lfr_n_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_BP_P1];

    //******************
    // check consistency
    // sy_lfr_n_swf_l
    if (sy_lfr_n_swf_l != DFLT_SY_LFR_N_SWF_L)
    {
        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_L + DATAFIELD_OFFSET, (char)sy_lfr_n_swf_l);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_swf_p
    if (flag == LFR_SUCCESSFUL && sy_lfr_n_swf_p < MIN_SY_LFR_N_SWF_P)
    {
        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_N_SWF_P + DATAFIELD_OFFSET, sy_lfr_n_swf_p);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL && sy_lfr_n_bp_p0 < DFLT_SY_LFR_N_BP_P0)
    {
        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P0 + DATAFIELD_OFFSET, sy_lfr_n_bp_p0);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_asm_p
    if (flag == LFR_SUCCESSFUL && sy_lfr_n_asm_p == 0)
    {
        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P + DATAFIELD_OFFSET, sy_lfr_n_asm_p);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_asm_p shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ((float)sy_lfr_n_asm_p / sy_lfr_n_bp_p0)
            - floorf((float)sy_lfr_n_asm_p / (float)sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, DATAFIELD_POS_SY_LFR_N_ASM_P + DATAFIELD_OFFSET, sy_lfr_n_asm_p);
            DEBUG_CHECK_STATUS(status);
            flag = WRONG_APP_DATA;
        }
    }
    // sy_lfr_n_bp_p1
    if (flag == LFR_SUCCESSFUL && sy_lfr_n_bp_p1 < DFLT_SY_LFR_N_BP_P1)
    {

        status = send_tm_lfr_tc_exe_inconsistent(
            TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1 + DATAFIELD_OFFSET, sy_lfr_n_bp_p1);
        DEBUG_CHECK_STATUS(status);
        flag = WRONG_APP_DATA;
    }
    // sy_lfr_n_bp_p1 shall be a whole multiple of sy_lfr_n_bp_p0
    if (flag == LFR_SUCCESSFUL)
    {
        aux = ((float)sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0) - floorf(sy_lfr_n_bp_p1 / sy_lfr_n_bp_p0);
        if (aux > FLOAT_EQUAL_ZERO)
        {
            status = send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, DATAFIELD_POS_SY_LFR_N_BP_P1 + DATAFIELD_OFFSET, sy_lfr_n_bp_p1);
            DEBUG_CHECK_STATUS(status);
            flag = LFR_DEFAULT;
        }
    }

    return flag;
}

int set_sy_lfr_n_swf_l(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the number of points of a snapshot (sy_lfr_n_swf_l).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_swf_l[0] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_L];
    parameter_dump_packet.sy_lfr_n_swf_l[1] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_L + 1];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_n_swf_p(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two snapshots, in s (sy_lfr_n_swf_p).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_swf_p[0] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_P];
    parameter_dump_packet.sy_lfr_n_swf_p[1] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_SWF_P + 1];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_n_asm_p(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two full spectral matrices transmission, in s
     * (SY_LFR_N_ASM_P).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_asm_p[0] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_ASM_P];
    parameter_dump_packet.sy_lfr_n_asm_p[1] = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_ASM_P + 1];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_n_bp_p0(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (DFLT_SY_LFR_N_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_BP_P0];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_n_bp_p1(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets (autocorrelation +
     * crosscorrelation), in s (sy_lfr_n_bp_p1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_BP_P1];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_n_cwf_long_f3(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function allows to switch from CWF_F3 packets to CWF_LONG_F3 packets.
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_n_cwf_long_f3
        = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_N_CWF_LONG_F3] & BIT_CWF_LONG_F3;

    return LFR_SUCCESSFUL;
}

//**********************
// BURST MODE PARAMETERS

int set_sy_lfr_b_bp_p0(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_B_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_b_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P0];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_b_bp_p1(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_B_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_b_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_B_BP_P1];

    return LFR_SUCCESSFUL;
}

//*********************
// SBM1 MODE PARAMETERS

int set_sy_lfr_s1_bp_p0(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S1_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_s1_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S1_BP_P0];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_s1_bp_p1(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S1_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_s1_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S1_BP_P1];

    return LFR_SUCCESSFUL;
}

//*********************
// SBM2 MODE PARAMETERS

int set_sy_lfr_s2_bp_p0(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S2_BP_P0).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_s2_bp_p0 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P0];

    return LFR_SUCCESSFUL;
}

int set_sy_lfr_s2_bp_p1(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function sets the time between two basic parameter sets, in s (SY_LFR_S2_BP_P1).
     *
     * @param TC points to the TeleCommand packet that is being processed
     * @param queue_id is the id of the queue which handles TM related to this execution step
     *
     */

    parameter_dump_packet.sy_lfr_s2_bp_p1 = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_S2_BP_P1];

    return LFR_SUCCESSFUL;
}

//*******************
// TC_LFR_UPDATE_INFO

unsigned int check_update_info_hk_lfr_mode(unsigned char mode)
{
    unsigned int status = LFR_DEFAULT;

    if ((mode == LFR_MODE_STANDBY) || (mode == LFR_MODE_NORMAL) || (mode == LFR_MODE_BURST)
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

unsigned int check_update_info_hk_tds_mode(unsigned char mode)
{
    unsigned int status = LFR_DEFAULT;

    if ((mode == TDS_MODE_STANDBY) || (mode == TDS_MODE_NORMAL) || (mode == TDS_MODE_BURST)
        || (mode == TDS_MODE_SBM1) || (mode == TDS_MODE_SBM2) || (mode == TDS_MODE_LFM))
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        status = LFR_DEFAULT;
    }

    return status;
}

unsigned int check_update_info_hk_thr_mode(unsigned char mode)
{
    unsigned int status = LFR_DEFAULT;

    if ((mode == THR_MODE_STANDBY) || (mode == THR_MODE_NORMAL) || (mode == THR_MODE_BURST))
    {
        status = LFR_SUCCESSFUL;
    }
    else
    {
        status = LFR_DEFAULT;
    }

    return status;
}

void set_hk_lfr_sc_rw_f_flag(unsigned char wheel, unsigned char freq, float value)
{
    unsigned char flag;
    unsigned char flagPosInByte;
    unsigned char newFlag;
    unsigned char flagMask;

    // if the frequency value is not a number, the flag is set to 0 and the frequency RWx_Fy is not
    // filtered
    if (isnan(value))
    {
        flag = FLAG_NAN;
    }
    else
    {
        flag = FLAG_IAN;
    }

    switch (wheel)
    {
        case WHEEL_1:
            flagPosInByte = FLAG_OFFSET_WHEELS_1_3 - freq;
            flagMask = (unsigned char)(~(1 << flagPosInByte));
            newFlag = (unsigned char)(flag << flagPosInByte);
            housekeeping_packet.hk_lfr_sc_rw1_rw2_f_flags
                = (housekeeping_packet.hk_lfr_sc_rw1_rw2_f_flags & flagMask) | newFlag;
            break;
        case WHEEL_2:
            flagPosInByte = FLAG_OFFSET_WHEELS_2_4 - freq;
            flagMask = (unsigned char)(~(1 << flagPosInByte));
            newFlag = (unsigned char)(flag << flagPosInByte);
            housekeeping_packet.hk_lfr_sc_rw1_rw2_f_flags
                = (housekeeping_packet.hk_lfr_sc_rw1_rw2_f_flags & flagMask) | newFlag;
            break;
        case WHEEL_3:
            flagPosInByte = FLAG_OFFSET_WHEELS_1_3 - freq;
            flagMask = (unsigned char)(~(1 << flagPosInByte));
            newFlag = (unsigned char)(flag << flagPosInByte);
            housekeeping_packet.hk_lfr_sc_rw3_rw4_f_flags
                = (housekeeping_packet.hk_lfr_sc_rw3_rw4_f_flags & flagMask) | newFlag;
            break;
        case WHEEL_4:
            flagPosInByte = FLAG_OFFSET_WHEELS_2_4 - freq;
            flagMask = (unsigned char)(~(1 << flagPosInByte));
            newFlag = (unsigned char)(flag << flagPosInByte);
            housekeeping_packet.hk_lfr_sc_rw3_rw4_f_flags
                = (housekeeping_packet.hk_lfr_sc_rw3_rw4_f_flags & flagMask) | newFlag;
            break;
        default:
            break;
    }
}

void set_hk_lfr_sc_rw_f_flags(void)
{
    // RW1
    set_hk_lfr_sc_rw_f_flag(WHEEL_1, FREQ_1, rw_f.cp_rpw_sc_rw1_f1);
    set_hk_lfr_sc_rw_f_flag(WHEEL_1, FREQ_2, rw_f.cp_rpw_sc_rw1_f2);
    set_hk_lfr_sc_rw_f_flag(WHEEL_1, FREQ_3, rw_f.cp_rpw_sc_rw1_f3);
    set_hk_lfr_sc_rw_f_flag(WHEEL_1, FREQ_4, rw_f.cp_rpw_sc_rw1_f4);

    // RW2
    set_hk_lfr_sc_rw_f_flag(WHEEL_2, FREQ_1, rw_f.cp_rpw_sc_rw2_f1);
    set_hk_lfr_sc_rw_f_flag(WHEEL_2, FREQ_2, rw_f.cp_rpw_sc_rw2_f2);
    set_hk_lfr_sc_rw_f_flag(WHEEL_2, FREQ_3, rw_f.cp_rpw_sc_rw2_f3);
    set_hk_lfr_sc_rw_f_flag(WHEEL_2, FREQ_4, rw_f.cp_rpw_sc_rw2_f4);

    // RW3
    set_hk_lfr_sc_rw_f_flag(WHEEL_3, FREQ_1, rw_f.cp_rpw_sc_rw3_f1);
    set_hk_lfr_sc_rw_f_flag(WHEEL_3, FREQ_2, rw_f.cp_rpw_sc_rw3_f2);
    set_hk_lfr_sc_rw_f_flag(WHEEL_3, FREQ_3, rw_f.cp_rpw_sc_rw3_f3);
    set_hk_lfr_sc_rw_f_flag(WHEEL_3, FREQ_4, rw_f.cp_rpw_sc_rw3_f4);

    // RW4
    set_hk_lfr_sc_rw_f_flag(WHEEL_4, FREQ_1, rw_f.cp_rpw_sc_rw4_f1);
    set_hk_lfr_sc_rw_f_flag(WHEEL_4, FREQ_2, rw_f.cp_rpw_sc_rw4_f2);
    set_hk_lfr_sc_rw_f_flag(WHEEL_4, FREQ_3, rw_f.cp_rpw_sc_rw4_f3);
    set_hk_lfr_sc_rw_f_flag(WHEEL_4, FREQ_4, rw_f.cp_rpw_sc_rw4_f4);
}

int check_sy_lfr_rw_f(const ccsdsTelecommandPacket_t* const TC, int offset, int* pos, float* value)
{
    float rw_k;
    int ret;

    ret = LFR_SUCCESSFUL;
    rw_k = INIT_FLOAT;

    copyFloatByChar((unsigned char*)&rw_k, &TC->packetID[offset]);

    *pos = offset;
    *value = rw_k;

    if (rw_k < MIN_SY_LFR_RW_F)
    {
        ret = WRONG_APP_DATA;
    }

    return ret;
}

int check_all_sy_lfr_rw_f(const ccsdsTelecommandPacket_t* const TC, int* pos, float* value)
{
    int ret = LFR_SUCCESSFUL;

    //****
    //****
    // RW1
    ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F1, pos, value); // F1
    if (ret == LFR_SUCCESSFUL) // F2
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F3
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F4
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F4, pos, value);
    }

    //****
    //****
    // RW2
    if (ret == LFR_SUCCESSFUL) // F1
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F2
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F3
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F4
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F4, pos, value);
    }

    //****
    //****
    // RW3
    if (ret == LFR_SUCCESSFUL) // F1
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F2
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F3
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F4
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F4, pos, value);
    }

    //****
    //****
    // RW4
    if (ret == LFR_SUCCESSFUL) // F1
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F2
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F3
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // F4
    {
        ret = check_sy_lfr_rw_f(TC, BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F4, pos, value);
    }

    return ret;
}

void getReactionWheelsFrequencies(const ccsdsTelecommandPacket_t* const TC)
{
    /** This function get the reaction wheels frequencies in the incoming TC_LFR_UPDATE_INFO and
     * copy the values locally.
     *
     * @param TC points to the TeleCommand packet that is being processed
     *
     */
    // pointer to the beginning of the incoming TC packet
    const unsigned char* const bytePosPtr = (const unsigned char* const)&TC->packetID;

    // rw1_f
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw1_f1, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F1]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw1_f2, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F2]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw1_f3, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F3]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw1_f4, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW1_F4]);

    // rw2_f
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw2_f1, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F1]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw2_f2, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F2]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw2_f3, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F3]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw2_f4, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW2_F4]);

    // rw3_f
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw3_f1, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F1]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw3_f2, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F2]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw3_f3, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F3]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw3_f4, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW3_F4]);

    // rw4_f
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw4_f1, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F1]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw4_f2, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F2]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw4_f3, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F3]);
    copyFloatByChar(
        (unsigned char*)&rw_f.cp_rpw_sc_rw4_f4, &bytePosPtr[BYTE_POS_UPDATE_INFO_CP_RPW_SC_RW4_F4]);

    // test each reaction wheel frequency value. NaN means that the frequency is not filtered
}

void setFBinMask(
    unsigned char* fbins_mask, float rw_f_comp, unsigned char deltaFreq, float sy_lfr_rw_k)
{
    /** This function executes specific actions when a TC_LFR_UPDATE_INFO TeleCommand has been
     * received.
     *
     * @param fbins_mask
     * @param rw_f is the reaction wheel frequency to filter
     * @param delta_f is the frequency step between the frequency bins, it depends on the frequency
     * channel
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
    float freqToFilterOut;
    int binBelow;
    int binAbove = 0;
    int closestBin;
    unsigned int whichByte = 0;
    int selectedByte;
    int bin = 0;
    int binToRemove[NB_BINS_TO_REMOVE];
    int k;
    bool filteringSet = false;

    for (k = 0; k < NB_BINS_TO_REMOVE; k++)
    {
        binToRemove[k] = -1;
    }

    if (!isnan(rw_f_comp))
    {
        // compute the frequency range to filter [ rw_f - delta_f; rw_f + delta_f ]
        f_RW_min = rw_f_comp - ((filterPar.sy_lfr_sc_rw_delta_f) * sy_lfr_rw_k);
        f_RW_MAX = rw_f_comp + ((filterPar.sy_lfr_sc_rw_delta_f) * sy_lfr_rw_k);

        freqToFilterOut = f_RW_min;
        while (filteringSet == false)
        {
            // compute the index of the frequency bin immediately below rw_f
            binBelow = (int)(floor(((double)freqToFilterOut) / ((double)deltaFreq)));
            deltaBelow = freqToFilterOut - (float)(binBelow * deltaFreq);

            // compute the index of the frequency bin immediately above rw_f
            binAbove = (int)(ceil(((double)freqToFilterOut) / ((double)deltaFreq)));
            deltaAbove = (float)(binAbove * deltaFreq) - freqToFilterOut;

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
            fi = (float)closestBin * deltaFreq;
            fi_min = fi - ((float)deltaFreq * (float)FI_INTERVAL_COEFF);
            fi_MAX = fi + ((float)deltaFreq * (float)FI_INTERVAL_COEFF);

            //**************************************************************************************
            // be careful here, one shall take into account that the bin 0 IS DROPPED in the spectra
            // thus, the index 0 in a mask corresponds to the bin 1 of the spectrum
            //**************************************************************************************

            // 1. IF freqToFilterOut is included in [ fi_min; fi_MAX ]
            // => remove f_(i), f_(i-1) and f_(i+1)
            if ((freqToFilterOut > fi_min) && (freqToFilterOut < fi_MAX))
            {
                binToRemove[0] = (closestBin - 1) - 1;
                binToRemove[1] = closestBin - 1;
                binToRemove[2] = (closestBin + 1) - 1;
            }
            // 2. ELSE
            // => remove the two f_(i) which are around f_RW
            else
            {
                binToRemove[0] = binBelow - 1;
                binToRemove[1] = binAbove - 1;
                binToRemove[2] = -1;
            }

            for (k = 0; k < NB_BINS_TO_REMOVE; k++)
            {
                bin = binToRemove[k];
                if ((bin >= BIN_MIN) && (bin <= BIN_MAX))
                {
                    whichByte = (bin >> SHIFT_3_BITS); // division by 8
                    selectedByte = (1 << (bin - (whichByte * BITS_PER_BYTE)));
                    fbins_mask[BYTES_PER_MASK - 1 - whichByte]
                        = fbins_mask[BYTES_PER_MASK - 1 - whichByte]
                        & ((unsigned char)(~selectedByte)); // bytes are ordered MSB first in the
                                                            // packets
                }
            }

            // update freqToFilterOut
            if (freqToFilterOut == f_RW_MAX)
            {
                filteringSet = true; // end of the loop
            }
            else
            {
                freqToFilterOut = freqToFilterOut + deltaFreq;
            }

            if (freqToFilterOut > f_RW_MAX)
            {
                freqToFilterOut = f_RW_MAX;
            }
        }
    }
}

void build_sy_lfr_rw_mask(unsigned int channel)
{
    unsigned char local_rw_fbins_mask[BYTES_PER_MASK];
    unsigned char* maskPtr = NULL;
    unsigned char deltaF = (unsigned char)DELTAF_F2;
    unsigned k;

    switch (channel)
    {
        case CHANNELF0:
            maskPtr = parameter_dump_packet.sy_lfr_rw_mask_f0_word1;
            deltaF = (unsigned char)DELTAF_F0;
            break;
        case CHANNELF1:
            maskPtr = parameter_dump_packet.sy_lfr_rw_mask_f1_word1;
            deltaF = (unsigned char)DELTAF_F1;
            break;
        case CHANNELF2:
            maskPtr = parameter_dump_packet.sy_lfr_rw_mask_f2_word1;
            deltaF = (unsigned char)DELTAF_F2;
            break;
        default:
            break;
    }

    for (k = 0; k < BYTES_PER_MASK; k++)
    {
        local_rw_fbins_mask[k] = INT8_ALL_F;
    }

    // RW1
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw1_f1, deltaF, filterPar.sy_lfr_rw1_k1);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw1_f2, deltaF, filterPar.sy_lfr_rw1_k2);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw1_f3, deltaF, filterPar.sy_lfr_rw1_k3);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw1_f4, deltaF, filterPar.sy_lfr_rw1_k4);

    // RW2
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw2_f1, deltaF, filterPar.sy_lfr_rw2_k1);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw2_f2, deltaF, filterPar.sy_lfr_rw2_k2);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw2_f3, deltaF, filterPar.sy_lfr_rw2_k3);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw2_f4, deltaF, filterPar.sy_lfr_rw2_k4);

    // RW3
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw3_f1, deltaF, filterPar.sy_lfr_rw3_k1);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw3_f2, deltaF, filterPar.sy_lfr_rw3_k2);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw3_f3, deltaF, filterPar.sy_lfr_rw3_k3);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw3_f4, deltaF, filterPar.sy_lfr_rw3_k4);

    // RW4
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw4_f1, deltaF, filterPar.sy_lfr_rw4_k1);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw4_f2, deltaF, filterPar.sy_lfr_rw4_k2);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw4_f3, deltaF, filterPar.sy_lfr_rw4_k3);
    setFBinMask(local_rw_fbins_mask, rw_f.cp_rpw_sc_rw4_f4, deltaF, filterPar.sy_lfr_rw4_k4);

    // update the value of the fbins related to reaction wheels frequency filtering
    if (maskPtr != NULL)
    {
        for (k = 0; k < BYTES_PER_MASK; k++)
        {
            maskPtr[k] = local_rw_fbins_mask[k];
        }
    }
}

void build_sy_lfr_rw_masks(void)
{
    build_sy_lfr_rw_mask(CHANNELF0);
    build_sy_lfr_rw_mask(CHANNELF1);
    build_sy_lfr_rw_mask(CHANNELF2);
}

void merge_fbins_masks(void)
{
    const unsigned char* const fbins_f0 = parameter_dump_packet.sy_lfr_fbins_f0_word1;
    const unsigned char* const fbins_f1 = parameter_dump_packet.sy_lfr_fbins_f1_word1;
    const unsigned char* const fbins_f2 = parameter_dump_packet.sy_lfr_fbins_f2_word1;
    const unsigned char* const rw_mask_f0 = parameter_dump_packet.sy_lfr_rw_mask_f0_word1;
    const unsigned char* const rw_mask_f1 = parameter_dump_packet.sy_lfr_rw_mask_f1_word1;
    const unsigned char* const rw_mask_f2 = parameter_dump_packet.sy_lfr_rw_mask_f2_word1;


    for (unsigned char k = 0; k < BYTES_PER_MASK; k++)
    {
        fbins_masks.merged_fbins_mask_f0[k] = fbins_f0[k] & rw_mask_f0[k];
        fbins_masks.merged_fbins_mask_f1[k] = fbins_f1[k] & rw_mask_f1[k];
        fbins_masks.merged_fbins_mask_f2[k] = fbins_f2[k] & rw_mask_f2[k];
    }
}

//***********
// FBINS MASK

int set_sy_lfr_fbins(const ccsdsTelecommandPacket_t* const TC)
{
    unsigned char* const fbins_mask_dump = parameter_dump_packet.sy_lfr_fbins_f0_word1;
    const unsigned char* const fbins_mask_TC = TC->dataAndCRC;

    for (unsigned int k = 0; k < BYTES_PER_MASKS_SET; k++)
    {
        fbins_mask_dump[k] = fbins_mask_TC[k];
    }

    return LFR_SUCCESSFUL;
}

//***************************
// TC_LFR_LOAD_PAS_FILTER_PAR

int check_sy_lfr_rw_k(const ccsdsTelecommandPacket_t* const TC, int offset, int* pos, float* value)
{
    float rw_k;
    int ret;

    ret = LFR_SUCCESSFUL;
    rw_k = INIT_FLOAT;

    copyFloatByChar((unsigned char*)&rw_k, &TC->dataAndCRC[offset]);

    *pos = offset;
    *value = rw_k;

    if (rw_k < MIN_SY_LFR_RW_F)
    {
        ret = WRONG_APP_DATA;
    }

    return ret;
}

int check_all_sy_lfr_rw_k(const ccsdsTelecommandPacket_t* const TC, int* pos, float* value)
{
    int ret = LFR_SUCCESSFUL;

    //****
    //****
    // RW1
    ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW1_K1, pos, value); // K1
    if (ret == LFR_SUCCESSFUL) // K2
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW1_K2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K3
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW1_K3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K4
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW1_K4, pos, value);
    }

    //****
    //****
    // RW2
    if (ret == LFR_SUCCESSFUL) // K1
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW2_K1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K2
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW2_K2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K3
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW2_K3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K4
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW2_K4, pos, value);
    }

    //****
    //****
    // RW3
    if (ret == LFR_SUCCESSFUL) // K1
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW3_K1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K2
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW3_K2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K3
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW3_K3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K4
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW3_K4, pos, value);
    }

    //****
    //****
    // RW4
    if (ret == LFR_SUCCESSFUL) // K1
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW4_K1, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K2
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW4_K2, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K3
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW4_K3, pos, value);
    }
    if (ret == LFR_SUCCESSFUL) // K4
    {
        ret = check_sy_lfr_rw_k(TC, DATAFIELD_POS_SY_LFR_RW4_K4, pos, value);
    }

    return ret;
}

int check_sy_lfr_filter_parameters(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
{
    int flag = LFR_SUCCESSFUL;

    unsigned char sy_lfr_pas_filter_modulus;
    float sy_lfr_pas_filter_tbad = 0.f;
    unsigned char sy_lfr_pas_filter_offset;
    float sy_lfr_pas_filter_shift = 0.f;
    float sy_lfr_sc_rw_delta_f = 0.f;
    const char* parPtr = NULL;
    int datafield_pos = 0;
    float rw_k = 0.f;


    //***************
    // get parameters
    sy_lfr_pas_filter_modulus = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS];
    copyFloatByChar((unsigned char*)&sy_lfr_pas_filter_tbad,
        &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD]);
    sy_lfr_pas_filter_offset = TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET];
    copyFloatByChar((unsigned char*)&sy_lfr_pas_filter_shift,
        &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT]);
    copyFloatByChar(
        (unsigned char*)&sy_lfr_sc_rw_delta_f, &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F]);

    //******************
    // CHECK CONSISTENCY

    //**************************
    // sy_lfr_pas_filter_enabled
    // nothing to check, value is 0 or 1

    //**************************
    // sy_lfr_pas_filter_modulus
    if ((sy_lfr_pas_filter_modulus < MIN_PAS_FILTER_MODULUS)
        || (sy_lfr_pas_filter_modulus > MAX_PAS_FILTER_MODULUS))
    {
        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
            DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS + DATAFIELD_OFFSET, sy_lfr_pas_filter_modulus));
        flag = WRONG_APP_DATA;
    }

    //***********************
    // sy_lfr_pas_filter_tbad
    if (flag == LFR_SUCCESSFUL)
    {
        if ((sy_lfr_pas_filter_tbad < MIN_PAS_FILTER_TBAD)
            || (sy_lfr_pas_filter_tbad > MAX_PAS_FILTER_TBAD))
        {
            parPtr = (const char*)&sy_lfr_pas_filter_tbad;
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
                DATAFIELD_POS_SY_LFR_PAS_FILTER_TBAD + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE]));
            flag = WRONG_APP_DATA;
        }
    }

    //*************************
    // sy_lfr_pas_filter_offset
    if (flag == LFR_SUCCESSFUL && sy_lfr_pas_filter_offset > MAX_PAS_FILTER_OFFSET)
    {
        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
            DATAFIELD_POS_SY_LFR_PAS_FILTER_OFFSET + DATAFIELD_OFFSET, sy_lfr_pas_filter_offset));
        flag = WRONG_APP_DATA;
    }

    //************************
    // sy_lfr_pas_filter_shift
    if (flag == LFR_SUCCESSFUL)
    {
        if ((sy_lfr_pas_filter_shift < MIN_PAS_FILTER_SHIFT)
            || (sy_lfr_pas_filter_shift > MAX_PAS_FILTER_SHIFT))
        {
            parPtr = (const char*)&sy_lfr_pas_filter_shift;
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
                DATAFIELD_POS_SY_LFR_PAS_FILTER_SHIFT + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE]));
            flag = WRONG_APP_DATA;
        }
    }

    //*************************************
    // check global coherency of the values
    if (flag == LFR_SUCCESSFUL)
    {
        if ((sy_lfr_pas_filter_offset + sy_lfr_pas_filter_shift) >= sy_lfr_pas_filter_modulus)
        {
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
                DATAFIELD_POS_SY_LFR_PAS_FILTER_MODULUS + DATAFIELD_OFFSET,
                sy_lfr_pas_filter_modulus));
            flag = WRONG_APP_DATA;
        }
    }

    //*********************
    // sy_lfr_sc_rw_delta_f
    if (flag == LFR_SUCCESSFUL && sy_lfr_sc_rw_delta_f < MIN_SY_LFR_SC_RW_DELTA_F)
    {
        parPtr = (const char*)&sy_lfr_sc_rw_delta_f;
        DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
            DATAFIELD_POS_SY_LFR_SC_RW_DELTA_F + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE]));
        flag = WRONG_APP_DATA;
    }

    //************
    // sy_lfr_rw_k
    if (flag == LFR_SUCCESSFUL)
    {
        flag = check_all_sy_lfr_rw_k(TC, &datafield_pos, &rw_k);
        if (flag != LFR_SUCCESSFUL)
        {
            parPtr = (const char*)&sy_lfr_pas_filter_shift;
            DEBUG_CHECK_STATUS(send_tm_lfr_tc_exe_inconsistent(
                TC, queue_id, datafield_pos + DATAFIELD_OFFSET, parPtr[FLOAT_LSBYTE]));
        }
    }

    return flag;
}


void interpolate_calibration_matrix(
    float* matrix, unsigned int floats_per_matrix, unsigned int matrices_count, unsigned int gap)
{
    unsigned int elements_gap = gap*floats_per_matrix;
    for (unsigned int major_bin = 0; major_bin < matrices_count; major_bin++)
    {
        for (unsigned int element = 0; element < floats_per_matrix; element++)
        {
            float alpha = (matrix[elements_gap] - matrix[0]) / (float)gap;
            float previous_value=matrix[0];
            for (unsigned int minor_bin = elements_gap; minor_bin < (gap*elements_gap); minor_bin+=elements_gap)
            {
                matrix[minor_bin] = previous_value + alpha;
                previous_value = matrix[minor_bin];
            }
            matrix++;
        }
        matrix += floats_per_matrix * (gap - 1);
    }
}

//**************
// KCOEFFICIENTS
int set_sy_lfr_kcoeff(const ccsdsTelecommandPacket_t* const TC, rtems_id queue_id)
{

#define F0_COMPRESSED_BIN_OFFSET            0
#define F1_COMPRESSED_BIN_OFFSET            NB_BINS_COMPRESSED_SM_F0
#define F2_COMPRESSED_BIN_OFFSET            (NB_BINS_COMPRESSED_SM_F1 + F1_COMPRESSED_BIN_OFFSET)
#define TOTAL_COMPRESSED_BIN_COUNT          (F2_COMPRESSED_BIN_OFFSET + NB_BINS_COMPRESSED_SM_F2)
#define DATAFIELD_POS_SY_LFR_MAG_CAL_MATRIX DATAFIELD_POS_SY_LFR_KCOEFF_1
#define DATAFIELD_POS_SY_LFR_ELEC_CAL_MATRIX                                                       \
    (DATAFIELD_POS_SY_LFR_MAG_CAL_MATRIX + NB_BYTES_MAG_CAL_MATRIX)
#define DATAFIELD_POS_SY_LFR_EXTRA_CAL_MATRIX                                                      \
    (DATAFIELD_POS_SY_LFR_ELEC_CAL_MATRIX + NB_BYTES_ELEC_CAL_MATRIX)
#if TOTAL_COMPRESSED_BIN_COUNT != NB_BINS_COMPRESSED_SM
    #error "TOTAL_COMPRESSED_BIN_COUNT must match NB_BINS_COMPRESSED_SM"
#endif

    unsigned short sy_lfr_kcoeff_frequency = 255;
    int status = LFR_SUCCESSFUL;

    unsigned int matrix_index = 0;
    float* mag_matrix_ptr = NULL;
    float* elec_matrix_ptr = NULL;
    float* extra_mag_matrix_ptr = NULL;
    float* extra_elec_matrix_ptr = NULL;
    enum
    {
        no = 0,
        yes = 1
    } interpolate
        = 0;

    // copy the value of the frequency byte by byte DO NOT USE A SHORT* POINTER
    copyInt16ByChar((unsigned char*)&sy_lfr_kcoeff_frequency,
        &TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY]);


    if (sy_lfr_kcoeff_frequency >= NB_BINS_COMPRESSED_SM)
    {
        LFR_PRINTF("ERR *** in set_sy_lfr_kcoeff_frequency *** sy_lfr_kcoeff_frequency = %d\n",
            sy_lfr_kcoeff_frequency);
        status = send_tm_lfr_tc_exe_inconsistent(TC, queue_id,
            DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY + DATAFIELD_OFFSET,
            TC->dataAndCRC[DATAFIELD_POS_SY_LFR_KCOEFF_FREQUENCY
                + 1]); // +1 to get the LSB instead of the MSB
        DEBUG_CHECK_STATUS(status);
        status = LFR_DEFAULT;
    }
    else
    {
        if (sy_lfr_kcoeff_frequency >= F2_COMPRESSED_BIN_OFFSET)
        {
            matrix_index = (sy_lfr_kcoeff_frequency - F2_COMPRESSED_BIN_OFFSET);
            mag_matrix_ptr = mag_calibration_matrices_f2
                + (matrix_index * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F2);
            elec_matrix_ptr = elec_calibration_matrices_f2
                + (matrix_index * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F2);
            extra_mag_matrix_ptr
                = mag_calibration_matrices_f2 + (ASM_F2_KEEP_BINS * NB_FLOATS_MAG_CAL_MATRIX);
            extra_elec_matrix_ptr
                = elec_calibration_matrices_f2 + (ASM_F2_KEEP_BINS * NB_FLOATS_ELEC_CAL_MATRIX);

            if (sy_lfr_kcoeff_frequency == NB_BINS_COMPRESSED_SM - 1)
                interpolate = yes;
        }
        else if (sy_lfr_kcoeff_frequency >= F1_COMPRESSED_BIN_OFFSET)
        {
            matrix_index = (sy_lfr_kcoeff_frequency - NB_BINS_COMPRESSED_SM_F0);
            mag_matrix_ptr = mag_calibration_matrices_f1
                + (matrix_index * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F1);
            elec_matrix_ptr = elec_calibration_matrices_f1
                + (matrix_index * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F1);
            extra_mag_matrix_ptr
                = mag_calibration_matrices_f1 + (ASM_F1_KEEP_BINS * NB_FLOATS_MAG_CAL_MATRIX);
            extra_elec_matrix_ptr
                = elec_calibration_matrices_f1 + (ASM_F1_KEEP_BINS * NB_FLOATS_ELEC_CAL_MATRIX);
        }
        else
        {
            matrix_index = sy_lfr_kcoeff_frequency;
            mag_matrix_ptr = mag_calibration_matrices_f0
                + (matrix_index * NB_FLOATS_MAG_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F0);
            elec_matrix_ptr = elec_calibration_matrices_f0
                + (matrix_index * NB_FLOATS_ELEC_CAL_MATRIX * NB_BINS_TO_AVERAGE_ASM_F0);
            extra_mag_matrix_ptr
                = mag_calibration_matrices_f0 + (ASM_F0_KEEP_BINS * NB_FLOATS_MAG_CAL_MATRIX);
            extra_elec_matrix_ptr
                = elec_calibration_matrices_f0 + (ASM_F0_KEEP_BINS * NB_FLOATS_ELEC_CAL_MATRIX);
        }
    }

    if (mag_matrix_ptr != NULL && elec_matrix_ptr != NULL)
    {
        memcpy((void*)mag_matrix_ptr, TC->dataAndCRC + DATAFIELD_POS_SY_LFR_MAG_CAL_MATRIX,
            NB_BYTES_MAG_CAL_MATRIX);
        memcpy((void*)elec_matrix_ptr, TC->dataAndCRC + DATAFIELD_POS_SY_LFR_ELEC_CAL_MATRIX,
            NB_BYTES_ELEC_CAL_MATRIX);
    }

    // The 3 first packets contains one line of last MAG CAL MATRIX
    if (extra_mag_matrix_ptr != NULL && matrix_index < 3)
    {
        memcpy(extra_mag_matrix_ptr + (matrix_index * NB_MAG_COMPONENT_PER_SM * FLOATS_PER_COMPLEX),
            TC->dataAndCRC + DATAFIELD_POS_SY_LFR_EXTRA_CAL_MATRIX,
            NB_MAG_COMPONENT_PER_SM * FLOATS_PER_COMPLEX * NB_BYTES_PER_FLOAT);
    }
    else if (extra_elec_matrix_ptr != NULL
        && matrix_index < 5) // The 2 following packets contains one line of last ELEC CAL MATRIX
    {
        memcpy(extra_elec_matrix_ptr
                + ((matrix_index - 3) * NB_ELEC_COMPONENT_PER_SM * FLOATS_PER_COMPLEX),
            TC->dataAndCRC + DATAFIELD_POS_SY_LFR_EXTRA_CAL_MATRIX,
            NB_ELEC_COMPONENT_PER_SM * FLOATS_PER_COMPLEX * NB_BYTES_PER_FLOAT);
    }

    if (interpolate == yes)
    {
        interpolate_calibration_matrix(mag_calibration_matrices_f0, NB_FLOATS_MAG_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0);
        interpolate_calibration_matrix(elec_calibration_matrices_f0, NB_FLOATS_ELEC_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F0, NB_BINS_TO_AVERAGE_ASM_F0);

        interpolate_calibration_matrix(mag_calibration_matrices_f1, NB_FLOATS_MAG_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F1, NB_BINS_TO_AVERAGE_ASM_F1);
        interpolate_calibration_matrix(elec_calibration_matrices_f1, NB_FLOATS_ELEC_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F1, NB_BINS_TO_AVERAGE_ASM_F1);


        interpolate_calibration_matrix(mag_calibration_matrices_f2, NB_FLOATS_MAG_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F2, NB_BINS_TO_AVERAGE_ASM_F2);
        interpolate_calibration_matrix(elec_calibration_matrices_f2, NB_FLOATS_ELEC_CAL_MATRIX,
            NB_BINS_COMPRESSED_SM_F2, NB_BINS_TO_AVERAGE_ASM_F2);
    }

    return status;
}

void copyFloatByChar(unsigned char* destination, const unsigned char* const source)
{
    destination[BYTE_0] = source[BYTE_0];
    destination[BYTE_1] = source[BYTE_1];
    destination[BYTE_2] = source[BYTE_2];
    destination[BYTE_3] = source[BYTE_3];
}

void copyInt32ByChar(unsigned char* destination, const unsigned char* const source)
{
    destination[BYTE_0] = source[BYTE_0];
    destination[BYTE_1] = source[BYTE_1];
    destination[BYTE_2] = source[BYTE_2];
    destination[BYTE_3] = source[BYTE_3];
}

void copyInt16ByChar(unsigned char* destination, const unsigned char* const source)
{
    destination[BYTE_0] = source[BYTE_0];
    destination[BYTE_1] = source[BYTE_1];
}

void floatToChar(float value, unsigned char* ptr)
{
    const unsigned char* const valuePtr = (const unsigned char*)&value;

    ptr[BYTE_0] = valuePtr[BYTE_0];
    ptr[BYTE_1] = valuePtr[BYTE_1];
    ptr[BYTE_2] = valuePtr[BYTE_2];
    ptr[BYTE_3] = valuePtr[BYTE_3];
}

//**********
// init dump

void init_parameter_dump(void)
{
    /** This function initialize the parameter_dump_packet global variable with default values.
     *
     */

    unsigned int k;

    parameter_dump_packet.targetLogicalAddress = CCSDS_DESTINATION_ID;
    parameter_dump_packet.protocolIdentifier = CCSDS_PROTOCOLE_ID;
    parameter_dump_packet.reserved = CCSDS_RESERVED;
    parameter_dump_packet.userApplication = CCSDS_USER_APP;
    parameter_dump_packet.packetID[0] = (unsigned char)(APID_TM_PARAMETER_DUMP >> SHIFT_1_BYTE);
    parameter_dump_packet.packetID[1] = (unsigned char)APID_TM_PARAMETER_DUMP;
    parameter_dump_packet.packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    parameter_dump_packet.packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    parameter_dump_packet.packetLength[0]
        = (unsigned char)(PACKET_LENGTH_PARAMETER_DUMP >> SHIFT_1_BYTE);
    parameter_dump_packet.packetLength[1] = (unsigned char)PACKET_LENGTH_PARAMETER_DUMP;
    // DATA FIELD HEADER
    parameter_dump_packet.spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    parameter_dump_packet.serviceType = TM_TYPE_PARAMETER_DUMP;
    parameter_dump_packet.serviceSubType = TM_SUBTYPE_PARAMETER_DUMP;
    parameter_dump_packet.destinationID = TM_DESTINATION_ID_GROUND;
    parameter_dump_packet.time[BYTE_0]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_3_BYTES);
    parameter_dump_packet.time[BYTE_1]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_2_BYTES);
    parameter_dump_packet.time[BYTE_2]
        = (unsigned char)(time_management_regs->coarse_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_3] = (unsigned char)(time_management_regs->coarse_time);
    parameter_dump_packet.time[BYTE_4]
        = (unsigned char)(time_management_regs->fine_time >> SHIFT_1_BYTE);
    parameter_dump_packet.time[BYTE_5] = (unsigned char)(time_management_regs->fine_time);
    parameter_dump_packet.sid = SID_PARAMETER_DUMP;

    //******************
    // COMMON PARAMETERS
    parameter_dump_packet.sy_lfr_common_parameters_spare = DEFAULT_SY_LFR_COMMON0;
    parameter_dump_packet.sy_lfr_common_parameters = DEFAULT_SY_LFR_COMMON1;

    //******************
    // NORMAL PARAMETERS
    parameter_dump_packet.sy_lfr_n_swf_l[0] = (unsigned char)(DFLT_SY_LFR_N_SWF_L >> SHIFT_1_BYTE);
    parameter_dump_packet.sy_lfr_n_swf_l[1] = (unsigned char)(DFLT_SY_LFR_N_SWF_L);
    parameter_dump_packet.sy_lfr_n_swf_p[0] = (unsigned char)(DFLT_SY_LFR_N_SWF_P >> SHIFT_1_BYTE);
    parameter_dump_packet.sy_lfr_n_swf_p[1] = (unsigned char)(DFLT_SY_LFR_N_SWF_P);
    parameter_dump_packet.sy_lfr_n_asm_p[0] = (unsigned char)(DFLT_SY_LFR_N_ASM_P >> SHIFT_1_BYTE);
    parameter_dump_packet.sy_lfr_n_asm_p[1] = (unsigned char)(DFLT_SY_LFR_N_ASM_P);
    parameter_dump_packet.sy_lfr_n_bp_p0 = (unsigned char)DFLT_SY_LFR_N_BP_P0;
    parameter_dump_packet.sy_lfr_n_bp_p1 = (unsigned char)DFLT_SY_LFR_N_BP_P1;
    parameter_dump_packet.sy_lfr_n_cwf_long_f3 = (unsigned char)DFLT_SY_LFR_N_CWF_LONG_F3;

    //*****************
    // BURST PARAMETERS
    parameter_dump_packet.sy_lfr_b_bp_p0 = (unsigned char)DEFAULT_SY_LFR_B_BP_P0;
    parameter_dump_packet.sy_lfr_b_bp_p1 = (unsigned char)DEFAULT_SY_LFR_B_BP_P1;

    //****************
    // SBM1 PARAMETERS
    parameter_dump_packet.sy_lfr_s1_bp_p0
        = (unsigned char)DEFAULT_SY_LFR_S1_BP_P0; // min value is 0.25 s for the period
    parameter_dump_packet.sy_lfr_s1_bp_p1 = (unsigned char)DEFAULT_SY_LFR_S1_BP_P1;

    //****************
    // SBM2 PARAMETERS
    parameter_dump_packet.sy_lfr_s2_bp_p0 = (unsigned char)DEFAULT_SY_LFR_S2_BP_P0;
    parameter_dump_packet.sy_lfr_s2_bp_p1 = (unsigned char)DEFAULT_SY_LFR_S2_BP_P1;

    //************
    // FBINS MASKS
    for (k = 0; k < BYTES_PER_MASKS_SET; k++)
    {
        parameter_dump_packet.sy_lfr_fbins_f0_word1[k] = INT8_ALL_F;
    }

    // PAS FILTER PARAMETERS
    parameter_dump_packet.pa_rpw_spare8_2 = INIT_CHAR;
    parameter_dump_packet.spare_sy_lfr_pas_filter_enabled = INIT_CHAR;
    parameter_dump_packet.sy_lfr_pas_filter_modulus = DEFAULT_SY_LFR_PAS_FILTER_MODULUS;
    floatToChar(DEFAULT_SY_LFR_PAS_FILTER_TBAD, parameter_dump_packet.sy_lfr_pas_filter_tbad);
    parameter_dump_packet.sy_lfr_pas_filter_offset = DEFAULT_SY_LFR_PAS_FILTER_OFFSET;
    floatToChar(DEFAULT_SY_LFR_PAS_FILTER_SHIFT, parameter_dump_packet.sy_lfr_pas_filter_shift);
    floatToChar(DEFAULT_SY_LFR_SC_RW_DELTA_F, parameter_dump_packet.sy_lfr_sc_rw_delta_f);

    // RW1_K
    floatToChar(DEFAULT_SY_LFR_RW_K1, parameter_dump_packet.sy_lfr_rw1_k1);
    floatToChar(DEFAULT_SY_LFR_RW_K2, parameter_dump_packet.sy_lfr_rw1_k2);
    floatToChar(DEFAULT_SY_LFR_RW_K3, parameter_dump_packet.sy_lfr_rw1_k3);
    floatToChar(DEFAULT_SY_LFR_RW_K4, parameter_dump_packet.sy_lfr_rw1_k4);
    // RW2_K
    floatToChar(DEFAULT_SY_LFR_RW_K1, parameter_dump_packet.sy_lfr_rw2_k1);
    floatToChar(DEFAULT_SY_LFR_RW_K2, parameter_dump_packet.sy_lfr_rw2_k2);
    floatToChar(DEFAULT_SY_LFR_RW_K3, parameter_dump_packet.sy_lfr_rw2_k3);
    floatToChar(DEFAULT_SY_LFR_RW_K4, parameter_dump_packet.sy_lfr_rw2_k4);
    // RW3_K
    floatToChar(DEFAULT_SY_LFR_RW_K1, parameter_dump_packet.sy_lfr_rw3_k1);
    floatToChar(DEFAULT_SY_LFR_RW_K2, parameter_dump_packet.sy_lfr_rw3_k2);
    floatToChar(DEFAULT_SY_LFR_RW_K3, parameter_dump_packet.sy_lfr_rw3_k3);
    floatToChar(DEFAULT_SY_LFR_RW_K4, parameter_dump_packet.sy_lfr_rw3_k4);
    // RW4_K
    floatToChar(DEFAULT_SY_LFR_RW_K1, parameter_dump_packet.sy_lfr_rw4_k1);
    floatToChar(DEFAULT_SY_LFR_RW_K2, parameter_dump_packet.sy_lfr_rw4_k2);
    floatToChar(DEFAULT_SY_LFR_RW_K3, parameter_dump_packet.sy_lfr_rw4_k3);
    floatToChar(DEFAULT_SY_LFR_RW_K4, parameter_dump_packet.sy_lfr_rw4_k4);

    // LFR_RW_MASK
    for (k = 0; k < BYTES_PER_MASKS_SET; k++)
    {
        parameter_dump_packet.sy_lfr_rw_mask_f0_word1[k] = INT8_ALL_F;
    }

    // once the reaction wheels masks have been initialized, they have to be merged with the fbins
    // masks
    merge_fbins_masks();
}

void init_kcoefficients_dump(void)
{
    init_kcoefficients_dump_packet(&kcoefficients_dump_1, PKTNR_1, KCOEFF_BLK_NR_PKT1);
    init_kcoefficients_dump_packet(&kcoefficients_dump_2, PKTNR_2, KCOEFF_BLK_NR_PKT2);

    kcoefficient_node_1.previous = NULL;
    kcoefficient_node_1.next = NULL;
    kcoefficient_node_1.packet_id = TM_K_DUMP_PKT_ID;
    kcoefficient_node_1.coarseTime = INIT_CHAR;
    kcoefficient_node_1.fineTime = INIT_CHAR;
    kcoefficient_node_1.buffer_address = &kcoefficients_dump_1;
    kcoefficient_node_1.status = INIT_CHAR;

    kcoefficient_node_2.previous = NULL;
    kcoefficient_node_2.next = NULL;
    kcoefficient_node_2.packet_id = TM_K_DUMP_PKT_ID;
    kcoefficient_node_2.coarseTime = INIT_CHAR;
    kcoefficient_node_2.fineTime = INIT_CHAR;
    kcoefficient_node_2.buffer_address = &kcoefficients_dump_2;
    kcoefficient_node_2.status = INIT_CHAR;
}

void init_kcoefficients_dump_packet(Packet_TM_LFR_KCOEFFICIENTS_DUMP_t* const kcoefficients_dump,
    unsigned char pkt_nr, unsigned char blk_nr)
{
    unsigned int packetLength;

    packetLength = ((blk_nr * (NB_BYTES_MAG_CAL_MATRIX + NB_BYTES_ELEC_CAL_MATRIX))
                       + BYTE_POS_KCOEFFICIENTS_PARAMETES)
        - CCSDS_TC_TM_PACKET_OFFSET; // 4 bytes for the CCSDS header

    kcoefficients_dump->targetLogicalAddress = CCSDS_DESTINATION_ID;
    kcoefficients_dump->protocolIdentifier = CCSDS_PROTOCOLE_ID;
    kcoefficients_dump->reserved = CCSDS_RESERVED;
    kcoefficients_dump->userApplication = CCSDS_USER_APP;
    kcoefficients_dump->packetID[0] = (unsigned char)(APID_TM_PARAMETER_DUMP >> SHIFT_1_BYTE);
    kcoefficients_dump->packetID[1] = (unsigned char)APID_TM_PARAMETER_DUMP;
    kcoefficients_dump->packetSequenceControl[0] = TM_PACKET_SEQ_CTRL_STANDALONE;
    kcoefficients_dump->packetSequenceControl[1] = TM_PACKET_SEQ_CNT_DEFAULT;
    kcoefficients_dump->packetLength[0] = (unsigned char)(packetLength >> SHIFT_1_BYTE);
    kcoefficients_dump->packetLength[1] = (unsigned char)packetLength;
    // DATA FIELD HEADER
    kcoefficients_dump->spare1_pusVersion_spare2 = SPARE1_PUSVERSION_SPARE2;
    kcoefficients_dump->serviceType = TM_TYPE_K_DUMP;
    kcoefficients_dump->serviceSubType = TM_SUBTYPE_K_DUMP;
    kcoefficients_dump->destinationID = TM_DESTINATION_ID_GROUND;
    kcoefficients_dump->time[BYTE_0] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_1] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_2] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_3] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_4] = INIT_CHAR;
    kcoefficients_dump->time[BYTE_5] = INIT_CHAR;
    kcoefficients_dump->sid = SID_K_DUMP;

    kcoefficients_dump->pkt_cnt = KCOEFF_PKTCNT;
    kcoefficients_dump->pkt_nr = pkt_nr;
    kcoefficients_dump->blk_nr = blk_nr;

    //******************
    // SOURCE DATA repeated N times with N in [0 .. PA_LFR_KCOEFF_BLK_NR]
    // one blk is 2 + 4 * 32 = 130 bytes, 30 blks max in one packet (30 * 130 = 3900)
    memset(kcoefficients_dump->kcoeff_blks, 0, KCOEFF_BLK_MAX_SZ);
}

void increment_seq_counter_destination_id_dump(
    unsigned char* const packet_sequence_control, unsigned char destination_id)
{
    /** This function increment the packet sequence control parameter of a TC, depending on its
     * destination ID.
     *
     * @param packet_sequence_control points to the packet sequence control which will be
     * incremented
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

    segmentation_grouping_flag = TM_PACKET_SEQ_CTRL_STANDALONE << SHIFT_1_BYTE;
    sequence_cnt = sequenceCounters_TM_DUMP[i] & SEQ_CNT_MASK;

    new_packet_sequence_control = segmentation_grouping_flag | sequence_cnt;

    packet_sequence_control[0] = (unsigned char)(new_packet_sequence_control >> SHIFT_1_BYTE);
    packet_sequence_control[1] = (unsigned char)(new_packet_sequence_control);

    // increment the sequence counter
    if (sequenceCounters_TM_DUMP[i] < SEQ_CNT_MAX)
    {
        sequenceCounters_TM_DUMP[i] = sequenceCounters_TM_DUMP[i] + 1;
    }
    else
    {
        sequenceCounters_TM_DUMP[i] = 0;
    }
}

/*------------------------------------------------------------------------------
--  Solar Orbiter's Low Frequency Receiver Flight Software (LFR FSW),
--  This file is a part of the LFR FSW
--  Copyright (C) 2021, Plasma Physics Laboratory - CNRS
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
/*--                  Author : Alexis Jeandet
--                   Contact : Alexis Jeandet
--                      Mail : alexis.jeandet@lpp.polytechnique.fr
----------------------------------------------------------------------------*/
#include "mitigations/PAS_filtering.h"
#include "lfr_common_headers/fsw_params.h"
#include "fsw_globals.h"

/**
 * @brief isPolluted returns MATRIX_IS_POLLUTED if there is any overlap between t0:t1 and tbad0:tbad1 ranges
 * @param t0 Start acquisition time
 * @param t1 End of acquisition time
 * @param tbad0 Start time of poluting signal
 * @param tbad1 End time of poluting signal
 * @return
 */
unsigned char isPolluted( u_int64_t t0, u_int64_t t1, u_int64_t tbad0, u_int64_t tbad1 ) {
  unsigned char polluted;

  polluted = MATRIX_IS_NOT_POLLUTED;

  if ( ((tbad0 < t0) && (t0 < tbad1))     // t0 is inside the polluted range
      || ((tbad0 < t1) && (t1 < tbad1))  // t1 is inside the polluted range
      || ((t0 < tbad0) && (tbad1 < t1))  // the polluted range is inside the signal range
      || ((tbad0 < t0) && (t1 < tbad1))) // the signal range is inside the polluted range
  {
    polluted = MATRIX_IS_POLLUTED;
  }

  return polluted;
}

/**
 * @brief acquisitionTimeIsValid checks if the given acquisition time is poluted by PAS
 * @param coarseTime Coarse acquisition time of the given SM
 * @param fineTime Fine acquisition time of the given ASM
 * @param channel Frequency channel to check, will impact SM time footprint
 * @return MATRIX_IS_POLLUTED if there is any time overlap between SM and PAS poluting signal
 */
unsigned char acquisitionTimeIsValid( unsigned int coarseTime, unsigned int fineTime, unsigned char channel) {
  u_int64_t t0;
  u_int64_t t1;
  u_int64_t tc;
  u_int64_t tbad0;
  u_int64_t tbad1;

  u_int64_t modulusInFineTime;
  u_int64_t offsetInFineTime;
  u_int64_t shiftInFineTime;
  u_int64_t tbadInFineTime;

  u_int64_t timecodeReference;

  unsigned char pasFilteringIsEnabled;
  unsigned char ret;

  // compute acquisition time from caoarseTime and fineTime
  t0 = ( ((u_int64_t)coarseTime) <<  SHIFT_2_BYTES ) + (u_int64_t) fineTime;
  t1 = t0;
  tc = t0;
  tbad0 = t0;
  tbad1 = t0;

  switch(channel) {
  case CHANNELF0:
    t1 = t0 + ACQUISITION_DURATION_F0;
    tc = t0 + HALF_ACQUISITION_DURATION_F0;
    break;
  case CHANNELF1:
    t1 = t0 + ACQUISITION_DURATION_F1;
    tc = t0 + HALF_ACQUISITION_DURATION_F1;
    break;
  case CHANNELF2:
    t1 = t0 + ACQUISITION_DURATION_F2;
    tc = t0 + HALF_ACQUISITION_DURATION_F2;
    break;
  default:
    break;
  }

  // compute the acquitionTime range
  modulusInFineTime   = filterPar.modulus_in_finetime;
  offsetInFineTime    = filterPar.offset_in_finetime;
  shiftInFineTime     = filterPar.shift_in_finetime;
  tbadInFineTime      = filterPar.tbad_in_finetime;
  timecodeReference   = INIT_INT;

  pasFilteringIsEnabled = (filterPar.spare_sy_lfr_pas_filter_enabled & 1); // [0000 0001]
  ret = MATRIX_IS_NOT_POLLUTED;

  if ( (tbadInFineTime == 0) || (pasFilteringIsEnabled == 0) ) {
    ret = MATRIX_IS_NOT_POLLUTED;
  } else {
    // INTERSECTION TEST #1
    timecodeReference = (tc - (tc % modulusInFineTime)) - modulusInFineTime ;
    tbad0 = timecodeReference + offsetInFineTime + shiftInFineTime;
    tbad1 = timecodeReference + offsetInFineTime + shiftInFineTime + tbadInFineTime;
    ret = isPolluted( t0, t1, tbad0, tbad1 );

    // INTERSECTION TEST #2
    if (ret == MATRIX_IS_NOT_POLLUTED) {
      timecodeReference = (tc - (tc % modulusInFineTime)) ;
      tbad0 = timecodeReference + offsetInFineTime + shiftInFineTime;
      tbad1 = timecodeReference + offsetInFineTime + shiftInFineTime + tbadInFineTime;
      ret = isPolluted( t0, t1, tbad0, tbad1 );
    }

    // INTERSECTION TEST #3
    if (ret == MATRIX_IS_NOT_POLLUTED) {
      timecodeReference = (tc - (tc % modulusInFineTime)) + modulusInFineTime ;
      tbad0 = timecodeReference + offsetInFineTime + shiftInFineTime;
      tbad1 = timecodeReference + offsetInFineTime + shiftInFineTime + tbadInFineTime;
      ret = isPolluted( t0, t1, tbad0, tbad1 );
    }
  }

  return ret;
}

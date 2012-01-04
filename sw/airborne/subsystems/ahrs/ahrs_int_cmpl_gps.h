/*
 * Copyright (C) 2011 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#ifndef AHRS_INT_CMPL_H
#define AHRS_INT_CMPL_H

//parameters for the 2nd order low-pass filter
//calculated in MATLAB using
//     [num,den] = butter(2,(cutoff_freq/(sample_freq/2)))*
#define ACCEL_BUTTER_NUM_1 = +0.00014802198653
#define ACCEL_BUTTER_NUM_2 = +0.00029604397306
#define ACCEL_BUTTER_NUM_3 = +0.14802198653185
//warning, ACCEL_BUTTER_DEN_1 is always one for this filter, so it is omitted here.
#define ACCEL_BUTTER_DEN_2 = -1.96529337262269
#define ACCEL_BUTTER_DEN_3 = +0.96588546056881

#define FILTER_IC_ACCEL

#include "subsystems/ahrs.h"
#include "std.h"
#include "math/pprz_algebra_int.h"

struct AhrsIntCmpl {
  struct Int32Rates  gyro_bias;
  struct Int32Rates  rate_correction;
  struct Int64Quat   high_rez_quat;
  struct Int64Rates  high_rez_bias;
#ifdef AHRS_GRAVITY_UPDATE_COORDINATED_TURN
  int32_t ltp_vel_norm;
#endif
};

extern struct AhrsIntCmpl ahrs_impl;




#ifdef USE_GPS
#include "subsystems/gps.h"
#endif

struct Int32Rates omega;

#ifdef FILTER_IC_ACCEL
struct FloatVect3 accel_float_prev = {0,0,0};
struct FloatVect3 accel_float_prev_prev = {0,0,0};
struct FloatVect3 accel_filter_val_prev = {0,0,0};
struct FloatVect3 accel_filter_val_prev_prev = {0,0,0};
#endif

#ifdef AHRS_UPDATE_FW_ESTIMATOR
// TODO copy ahrs to state instead of estimator
void ahrs_update_fw_estimator(void);
extern float ins_roll_neutral;
extern float ins_pitch_neutral;
#endif


#endif /* AHRS_INT_CMPL_H */

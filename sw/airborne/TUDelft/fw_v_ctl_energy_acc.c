/*
 * $Id: fw_v_ctl.c 4304 2009-10-31 04:13:12Z vassilis $
 *
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin, Michel Gorraz
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
 *
 */

/**
 *  \file v_ctl_ctl
 *  \brief Vertical control for fixed wing vehicles.
 *
 */

#include "fw_v_ctl.h"
#include "fw_v_ctl_energy.h"
#include "estimator.h"
#include "nav.h"
#include "airframe.h"
#include "autopilot.h"
#include "downlink.h"

/* Flight Envelope */
float v_ctl_vmin;
float v_ctl_vmax;
float v_ctl_vz_max;
float v_ctl_pot_energy_bounds;
float v_ctl_kin_energy_bounds;
float v_ctl_pitch_min;
float v_ctl_pitch_max;
float v_ctl_throttle_min;
float v_ctl_throttle_max;

/* Gains */
float v_ctl_energy_tot_i;
float v_ctl_energy_dis_i;
float v_ctl_energy_tot_p;
float v_ctl_energy_dis_p;
float v_ctl_energy_tot_d;
float v_ctl_energy_dis_d;

/* mode */
uint8_t v_ctl_mode;
uint8_t v_ctl_climb_mode;
uint8_t v_ctl_auto_throttle_submode;

/* outer loop */
float v_ctl_altitude_error;

/* inner loop */
float v_ctl_climb_setpoint;




float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;

/* Setpoint */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb;
float v_ctl_auto_airspeed_setpoint;

/* Slewing */
pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;
float v_ctl_auto_throttle_sum_err;


void v_ctl_init( void )
{
  /* flight envelope */
  v_ctl_pot_energy_bounds = 20.;
  v_ctl_kin_energy_bounds = 20.;
  v_ctl_pitch_min         = H_CTL_PITCH_MIN_SETPOINT;
  v_ctl_pitch_max         = H_CTL_PITCH_MAX_SETPOINT;
  v_ctl_throttle_min      = 0.;
  v_ctl_throttle_max      = 1.;

  // Gains
  v_ctl_energy_tot_i      = 0.09;
  v_ctl_energy_dis_i      = 0.06;
  v_ctl_energy_tot_p      = 0.36;
  v_ctl_energy_dis_p      = 0.36;
  v_ctl_energy_tot_d      = 0.0;
  v_ctl_energy_dis_d      = 0.0;

  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;
  v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
  v_ctl_auto_throttle_submode = V_CTL_MODE_AUTO_ALT;

  /* setpoints */
  v_ctl_altitude_setpoint = 70.;
  v_ctl_altitude_pre_climb = 0.;
  v_ctl_auto_airspeed_setpoint = 12.;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;

  /* energy */
  v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;

  v_ctl_auto_throttle_sum_err = 0.;

  v_ctl_throttle_setpoint = 0;
}

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */

float v_ctl_altitude_pgain = 0.05;
float v_ctl_speed_pgain = 0.18;

void v_ctl_altitude_loop( void )
{
}

void v_ctl_climb_loop ( void )
{
  // Integrators
  static float v_ctl_throttle_trim = 0;
  static float v_ctl_pitch_trim = 0;

  static float v_ctl_last_speed = 0;

  // Local
  float v_ctl_gamma_setpoint = 0;
  float v_ctl_gamma_error = 0.;
  float v_ctl_gamma = 0;
  float v_ctl_speed_error = 0;
  float v_ctl_acceleration = 0;
  float v_ctl_acceleration_error = 0;
  float v_ctl_acceleration_setpoint = 0;
  float throttle = 0.;

  // Energy: Total and distribution
  float v_ctl_energy_error_tot = 0;
  float v_ctl_energy_error_dis = 0;

  float speed = estimator_hspeed_mod;

  // Altitude Error
  v_ctl_altitude_error = v_ctl_altitude_setpoint - estimator_z;

  // Commanded Climb Rate
  v_ctl_climb_setpoint = v_ctl_altitude_pgain * v_ctl_altitude_error
                       + v_ctl_altitude_pre_climb;
  BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);

  // Commanded Flight Path
  if (speed > 1) {
    v_ctl_gamma_setpoint = v_ctl_climb_setpoint / speed;
  } else {
    v_ctl_gamma_setpoint = v_ctl_climb_setpoint;
  }
  BoundAbs(v_ctl_gamma_error, 0.7);

  // Actual Flight Path
  if (speed > 1) {
    v_ctl_gamma = estimator_z_dot / speed;
  } else {
    v_ctl_gamma = estimator_z_dot;
  }
  BoundAbs(v_ctl_gamma, 0.7);

  // Flight Path Error
  v_ctl_gamma_error = v_ctl_gamma_setpoint - v_ctl_gamma;

  // Speed Error
  v_ctl_speed_error = v_ctl_auto_airspeed_setpoint - speed;

  // Acceleration Command
  v_ctl_acceleration_setpoint = v_ctl_speed_error * v_ctl_speed_pgain;

  // Actual Acceleration (1 sec filter)
  v_ctl_acceleration += ((speed - v_ctl_last_speed) - v_ctl_acceleration) / 2.;
  v_ctl_last_speed = speed;

  // Energy Distribution
  v_ctl_acceleration_error = (v_ctl_acceleration_setpoint - v_ctl_acceleration) / 9.81;

  //////////////////////////////////////////////////

  // Massless Energy
  // Total and Distribution
  v_ctl_energy_error_tot = v_ctl_gamma_error + v_ctl_acceleration_error;
  v_ctl_energy_error_dis = v_ctl_acceleration_error - v_ctl_gamma_error;

  // Actual Control
  // integrators
  v_ctl_throttle_trim += v_ctl_energy_error_tot * v_ctl_energy_tot_i;
  if (v_ctl_throttle_trim >= v_ctl_throttle_max)
  {
    v_ctl_throttle_trim = v_ctl_throttle_max;
  }
  else if (v_ctl_throttle_trim < v_ctl_throttle_min)
  {
    v_ctl_throttle_trim = v_ctl_throttle_min;
  }

  v_ctl_pitch_trim += v_ctl_energy_error_dis * v_ctl_energy_dis_i;
  if (v_ctl_pitch_trim >= v_ctl_pitch_max)
  {
    v_ctl_pitch_trim = v_ctl_pitch_max;
  }
  else if (v_ctl_pitch_trim < v_ctl_pitch_min)
  {
    v_ctl_pitch_trim = v_ctl_pitch_min;
  }

  // proportional
  throttle  =  v_ctl_throttle_trim - (v_ctl_gamma + v_ctl_acceleration/9.81) * v_ctl_energy_tot_p;
  nav_pitch = -v_ctl_pitch_trim    + (v_ctl_acceleration/9.81 - v_ctl_gamma) * v_ctl_energy_dis_p;

  if (nav_pitch >= v_ctl_pitch_max)
  {
    nav_pitch = v_ctl_pitch_max;
  }
  else if (nav_pitch < v_ctl_pitch_min)
  {
    nav_pitch = v_ctl_pitch_min;
  }

  if (throttle >= v_ctl_throttle_max)
  {
    throttle = v_ctl_throttle_max;
  }
  else if (throttle < v_ctl_throttle_min)
  {
    throttle = v_ctl_throttle_min;
  }

  v_ctl_throttle_setpoint = TRIM_PPRZ(throttle * MAX_PPRZ);

  DOWNLINK_SEND_VERTICAL_ENERGY(DefaultChannel,
    &v_ctl_gamma_setpoint, &v_ctl_acceleration_setpoint,
    &throttle, &nav_pitch,
    &throttle, &nav_pitch,
    &v_ctl_auto_airspeed_setpoint);

}


#ifdef V_CTL_THROTTLE_SLEW_LIMITER
#define V_CTL_THROTTLE_SLEW (1./CONTROL_RATE/(V_CTL_THROTTLE_SLEW_LIMITER))
#endif

#ifndef V_CTL_THROTTLE_SLEW
#define V_CTL_THROTTLE_SLEW 1.
#endif
/** \brief Computes slewed throttle from throttle setpoint
    called at 20Hz
 */
void v_ctl_throttle_slew( void ) {
  pprz_t diff_throttle = v_ctl_throttle_setpoint - v_ctl_throttle_slewed;
  BoundAbs(diff_throttle, TRIM_PPRZ(V_CTL_THROTTLE_SLEW*MAX_PPRZ));
  v_ctl_throttle_slewed += diff_throttle;
}

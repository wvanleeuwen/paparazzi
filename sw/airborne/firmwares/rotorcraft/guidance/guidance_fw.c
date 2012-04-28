/*
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
 */

/**
 *  @file firmwares/fixedwing/guidance/guidance_v.c
 *  Vertical control for fixed wing vehicles.
 *
 */

#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "estimator.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"

/* mode */
uint8_t v_ctl_mode;

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;
float v_ctl_altitude_max_climb;

/* inner loop */
float v_ctl_climb_setpoint;

#ifndef V_CTL_AUTO_THROTTLE_DGAIN
#define V_CTL_AUTO_THROTTLE_DGAIN 0.
#endif

/* "auto throttle" inner loop parameters */
float v_ctl_auto_throttle_cruise_throttle;
float v_ctl_auto_throttle_nominal_cruise_throttle;
float v_ctl_auto_throttle_min_cruise_throttle;
float v_ctl_auto_throttle_max_cruise_throttle;
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pgain;
float v_ctl_auto_throttle_igain;
float v_ctl_auto_throttle_dgain;
float v_ctl_auto_throttle_sum_err;
#define V_CTL_AUTO_THROTTLE_MAX_SUM_ERR 150
float v_ctl_auto_throttle_pitch_of_vz_pgain;
float v_ctl_auto_throttle_pitch_of_vz_dgain;

#ifndef V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN
#define V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN 0.
#endif

pprz_t v_ctl_throttle_setpoint;

inline static void v_ctl_climb_auto_throttle_loop( void );

void v_ctl_init( void ) {
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_altitude_error = 0.;
  v_ctl_altitude_max_climb = V_CTL_ALTITUDE_MAX_CLIMB;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_min_cruise_throttle = V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE;
  v_ctl_auto_throttle_max_cruise_throttle = V_CTL_AUTO_THROTTLE_MAX_CRUISE_THROTTLE;
  v_ctl_auto_throttle_cruise_throttle = v_ctl_auto_throttle_nominal_cruise_throttle;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pgain = V_CTL_AUTO_THROTTLE_PGAIN;
  v_ctl_auto_throttle_igain = V_CTL_AUTO_THROTTLE_IGAIN;
  v_ctl_auto_throttle_dgain = V_CTL_AUTO_THROTTLE_DGAIN;
  v_ctl_auto_throttle_sum_err = 0.;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;
  v_ctl_auto_throttle_pitch_of_vz_dgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_DGAIN;

  v_ctl_throttle_setpoint = 0;
}

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */

void v_ctl_altitude_loop( void ) {
  v_ctl_altitude_error = v_ctl_altitude_setpoint - estimator_z;
  v_ctl_climb_setpoint = altitude_pgain_boost * v_ctl_altitude_pgain * v_ctl_altitude_error;
  BoundAbs(v_ctl_climb_setpoint, v_ctl_altitude_max_climb);
}


/**
 * auto throttle inner loop
 * \brief
 */

inline static void v_ctl_climb_auto_throttle_loop(void) {
  static float last_err;

  float f_throttle = 0;
  float err  = estimator_z_dot - v_ctl_climb_setpoint;
  float d_err = err - last_err;
  last_err = err;
  float controlled_throttle = v_ctl_auto_throttle_cruise_throttle
    + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
    - v_ctl_auto_throttle_pgain *
    (err + v_ctl_auto_throttle_igain * v_ctl_auto_throttle_sum_err
     + v_ctl_auto_throttle_dgain * d_err);

  /* pitch pre-command */
  float v_ctl_pitch_of_vz = (v_ctl_climb_setpoint + d_err * v_ctl_auto_throttle_pitch_of_vz_dgain) * v_ctl_auto_throttle_pitch_of_vz_pgain;

  f_throttle = controlled_throttle;
  v_ctl_auto_throttle_sum_err += err;
  BoundAbs(v_ctl_auto_throttle_sum_err, V_CTL_AUTO_THROTTLE_MAX_SUM_ERR);
  nav_pitch += v_ctl_pitch_of_vz;

  v_ctl_throttle_setpoint = TRIM_UPPRZ(f_throttle * MAX_PPRZ);
}




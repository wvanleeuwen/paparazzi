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
 *  Vertical control using total energy control for fixed wing vehicles.
 *


	=================================================
	Energy:
	------
	E 		= mgh + 1/2mV^2
	Edot / V 	= (gamma + Vdot/g) * W

	equilibrium

	Vdot / g = Thrust/W - Drag/W - sin(gamma)
	with: Drag/Weight = (Cl/Cd)^-1

	-glide angle: Vdot = 0, T=0 ==> gamma = Cd/Cl
	-level flight: Vdot = 0, gamma=0 ==> W/T = Cl/Cd
	=================================================
	
	Strategy:  thrust = path + acceleration[g] (total energy)
		   pitch = path - acceleration[g]  (energy balance)

	Pseudo-Control Unit = dimensionless acceleration [g]

		- pitch <-> pseudocontrol:    sin(Theta) steers Vdot in [g]
		- throttle <-> pseudocontrol: motor characteristic as function of V x throttle steeds VDot

 */

#pragma message "CAUTION! Using TOTAL ENERGY CONTROLLER: Experimental!"

#include "firmwares/fixedwing/guidance/energy_ctrl.h"
#include "estimator.h"
#include "subsystems/nav.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/autopilot.h"
#include "subsystems/ahrs.h"

/////// DEFAULT GUIDANCE_V NECESSITIES //////

/* mode */
uint8_t v_ctl_mode = V_CTL_MODE_MANUAL;
uint8_t v_ctl_climb_mode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
uint8_t v_ctl_auto_throttle_submode = V_CTL_CLIMB_MODE_AUTO_THROTTLE;
float v_ctl_auto_throttle_sum_err = 0;
float v_ctl_auto_airspeed_controlled = 0;
float v_ctl_auto_groundspeed_setpoint = 0;

#ifdef LOITER_TRIM
#error "Energy Controller can not accep Loiter Trim"
#endif
//#ifdef V_CTL_AUTO_THROTTLE_MIN_CRUISE_THROTTLE
//#error

/////// ACTUALLY USED STUFF //////

/* outer loop */
float v_ctl_altitude_setpoint;
float v_ctl_altitude_pre_climb; ///< Path Angle
float v_ctl_altitude_pgain;
float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low

float v_ctl_auto_airspeed_setpoint; ///< in meters per second

/* inner loop */
float v_ctl_climb_setpoint;

/* "auto throttle" inner loop parameters */
float v_ctl_auto_throttle_nominal_cruise_throttle;
float v_ctl_auto_throttle_climb_throttle_increment;
float v_ctl_auto_throttle_pitch_of_vz_pgain;

float v_ctl_auto_throttle_of_airspeed_pgain;
float v_ctl_auto_throttle_of_airspeed_igain;
float v_ctl_auto_pitch_of_airspeed_pgain;
float v_ctl_auto_pitch_of_airspeed_igain;

pprz_t v_ctl_throttle_setpoint;
pprz_t v_ctl_throttle_slewed;

void v_ctl_init( void ) {
  /* mode */
  v_ctl_mode = V_CTL_MODE_MANUAL;

  /* outer loop */
  v_ctl_altitude_setpoint = 0.;
  v_ctl_altitude_pgain = V_CTL_ALTITUDE_PGAIN;
  v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;

  /* inner loops */
  v_ctl_climb_setpoint = 0.;

  /* "auto throttle" inner loop parameters */
  v_ctl_auto_throttle_nominal_cruise_throttle = V_CTL_AUTO_THROTTLE_NOMINAL_CRUISE_THROTTLE;
  v_ctl_auto_throttle_climb_throttle_increment = V_CTL_AUTO_THROTTLE_CLIMB_THROTTLE_INCREMENT;
  v_ctl_auto_throttle_pitch_of_vz_pgain = V_CTL_AUTO_THROTTLE_PITCH_OF_VZ_PGAIN;

  v_ctl_throttle_setpoint = 0;
}

/**
 * outer loop
 * \brief Computes v_ctl_climb_setpoint and sets v_ctl_auto_throttle_submode
 */

float v_ctl_gamma_setpoint = 0;
float v_ctl_vdot_setpoint = 0;

void v_ctl_altitude_loop( void ) 
{
  // Imput Checks
  if (v_ctl_auto_airspeed_setpoint <= 0.0f) v_ctl_auto_airspeed_setpoint = NOMINAL_AIRSPEED;

  // Altitude Controller
  v_ctl_altitude_error = v_ctl_altitude_setpoint - estimator_z;
  v_ctl_climb_setpoint = v_ctl_altitude_pgain * v_ctl_altitude_error;
  BoundAbs(v_ctl_climb_setpoint, V_CTL_ALTITUDE_MAX_CLIMB);
}


/**
 * auto throttle inner loop
 * \brief
 */

const float dt = 0.01f;

void v_ctl_climb_loop( void ) 
{
  float gamma_err  = (estimator_z_dot - v_ctl_climb_setpoint) / v_ctl_auto_airspeed_setpoint;

  float serr = v_ctl_auto_airspeed_setpoint - estimator_airspeed;

  // Speed Controller to PseudoControl
  float desired_acceleration = (v_ctl_auto_airspeed_setpoint - estimator_airspeed) * v_ctl_auto_pitch_of_airspeed_pgain;

  // Outerloop Energy Commands
  v_ctl_gamma_setpoint = v_ctl_climb_setpoint / v_ctl_auto_airspeed_setpoint;
  v_ctl_vdot_setpoint = desired_acceleration / 9.81f;

  // Auto Cruise Throttle
  if (v_ctl_mode >= V_CTL_MODE_AUTO_CLIMB)
  {
    v_ctl_auto_throttle_nominal_cruise_throttle += v_ctl_auto_throttle_of_airspeed_igain * serr * dt;
    if (v_ctl_auto_throttle_nominal_cruise_throttle < 0.0f) v_ctl_auto_throttle_nominal_cruise_throttle = 0.0f;
    else if (v_ctl_auto_throttle_nominal_cruise_throttle > 1.0f) v_ctl_auto_throttle_nominal_cruise_throttle = 1.0f;
  }

  // Total Controller
  float controlled_throttle = v_ctl_auto_throttle_nominal_cruise_throttle
    + v_ctl_auto_throttle_climb_throttle_increment * v_ctl_climb_setpoint
    + v_ctl_auto_throttle_of_airspeed_pgain * serr;

  /* pitch pre-command */
  ins_pitch_neutral -=  v_ctl_auto_pitch_of_airspeed_igain * (serr) * dt;
  float v_ctl_pitch_of_vz = - v_ctl_auto_pitch_of_airspeed_pgain * serr + (v_ctl_climb_setpoint /*+ d_err * v_ctl_auto_throttle_pitch_of_vz_dgain*/) * v_ctl_auto_throttle_pitch_of_vz_pgain;

  nav_pitch = v_ctl_pitch_of_vz;

  v_ctl_throttle_setpoint = TRIM_UPPRZ(controlled_throttle * MAX_PPRZ);
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

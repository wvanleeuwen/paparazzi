/*
 * Copyright (C) 2012 TUDelft
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
 * @file firmwares/fixedwing/guidance/energy_ctrl.h
 * Vertical control using total energy control for fixed wing vehicles.
 *
 */

#ifndef FW_V_CTL_ENERGY_H
#define FW_V_CTL_ENERGY_H

#include <inttypes.h>
#include "paparazzi.h"

/////// DEFAULT GUIDANCE_V NECESSITIES //////

/* Vertical mode */
#define V_CTL_MODE_MANUAL        0
#define V_CTL_MODE_AUTO_THROTTLE 1
#define V_CTL_MODE_AUTO_CLIMB    2
#define V_CTL_MODE_AUTO_ALT      3
#define V_CTL_MODE_NB            4
extern uint8_t v_ctl_mode;

extern uint8_t v_ctl_climb_mode;
extern uint8_t v_ctl_auto_throttle_submode;

#define V_CTL_CLIMB_MODE_AUTO_ENERGY 	   0
#define V_CTL_CLIMB_MODE_AUTO_PITCHONLY    1

// Needed for telemetry
extern float v_ctl_auto_throttle_sum_err;

// Needed for course loop gain 
extern float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low

// Old airspeed code wants:
extern float v_ctl_auto_airspeed_controlled;
extern float v_ctl_auto_groundspeed_setpoint;

/////// ACTUALLY USED STUFF //////

/* outer loop */
// extern float v_ctl_altitude_error;    ///< in meters, (setpoint - alt) -> positive = too low
extern float v_ctl_altitude_setpoint; ///< in meters above MSL
extern float v_ctl_altitude_pre_climb; ///< Path Angle
extern float v_ctl_altitude_pgain;

extern float v_ctl_auto_airspeed_setpoint; ///< in meters per second


/* "auto throttle" inner loop parameters */
extern float v_ctl_auto_throttle_nominal_cruise_throttle;
extern float v_ctl_auto_throttle_climb_throttle_increment;
extern float v_ctl_auto_throttle_pitch_of_vz_pgain;

extern pprz_t v_ctl_throttle_setpoint;
extern pprz_t v_ctl_throttle_slewed;

extern void v_ctl_init( void );
extern void v_ctl_altitude_loop( void );
extern void v_ctl_climb_loop ( void );

/** Computes throttle_slewed from throttle_setpoint */
extern void v_ctl_throttle_slew( void );

#define guidance_v_SetCruiseThrottle(_v) { \
  v_ctl_auto_throttle_cruise_throttle = (_v ? _v : v_ctl_auto_throttle_nominal_cruise_throttle); \
  Bound(v_ctl_auto_throttle_cruise_throttle, v_ctl_auto_throttle_min_cruise_throttle, v_ctl_auto_throttle_max_cruise_throttle); \
}

#define guidance_v_SetAutoThrottleIgain(_v) {	\
    v_ctl_auto_throttle_igain = _v;		\
    v_ctl_auto_throttle_sum_err = 0;		\
  }

#endif /* FW_V_CTL_H */

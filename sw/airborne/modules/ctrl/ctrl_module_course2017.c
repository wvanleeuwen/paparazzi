/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/ctrl/ctrl_module_course.h
 * @brief example empty controller
 *
 */

#include <stdbool.h>
#include "modules/ctrl/ctrl_module_course2017.h"
#include "state.h"
#include "waypoints.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "generated/flight_plan.h"

#define BOUND_ANGLE 15.0
#define Bound(_x, _min, _max ) ( ( _x ) > ( _max ) ? ( _max ) : ( ( _x ) < ( _min ) ? ( _min ) : ( _x ) ) )

struct ctrl_module_course_struct {
  int rc_x;
  int rc_y;
  int rc_z;
  int rc_t;

} ctrl_module_course;

double rc_yaw_gain      = 1.0;
double rc_pitch_gain    = 1.0;
double rc_roll_gain     = 1.0;
double position_gain    = 1.0;

void ctrl_module_init(void);
void ctrl_module_run(bool in_flight);

void ctrl_module_init(void)
{
  ctrl_module_course.rc_x = 0;
  ctrl_module_course.rc_y = 0;
  ctrl_module_course.rc_z = 0;
  ctrl_module_course.rc_t = 0;
}

// simple rate control without reference model nor attitude
void ctrl_module_run(bool in_flight)
{
  struct Int32Eulers setpoint;
  if (!in_flight) {
      // We aren't flying so reset the stabilization
      setpoint.phi      = ANGLE_BFP_OF_REAL( RAD_OF_DEG( 0.0 ) );
      setpoint.theta    = ANGLE_BFP_OF_REAL( RAD_OF_DEG( 0.0 ) );
      setpoint.psi      = ANGLE_BFP_OF_REAL( RAD_OF_DEG( 0.0 ) );
  } else {
      // Get our position and orientation
      struct EnuCoor_i*     position    = stateGetPositionEnu_i();
      struct Int32Eulers*   orientation = stateGetNedToBodyEulers_i();
      // Get the position of the waypoint we want to center at
      struct EnuCoor_i*     center      = &waypoints[WP_CENTER].enu_i;
      // Calculate the radial coordinates of the waypoint
      double centerAngle    = atan2(POS_FLOAT_OF_BFP(position.y - center.y), POS_FLOAT_OF_BFP(position.x - center.x));
      double centerRadius   = hypot(POS_FLOAT_OF_BFP(position.x - center.x), POS_FLOAT_OF_BFP(position.y - center.y));
      // Determine the body angle to the waypoint
      double relativeAngle  = centerAngle - ANGLE_FLOAT_OF_BFP( orientation->psi );
      if(relativeAngle < -M_PI)     relativeAngle += 2*M_PI;
      if(relativeAngle >  M_PI)     relativeAngle -= 2*M_PI;
      // Now we put gains on the distances from the waypoint
      // Euler angles are derived from NED whilst waypoints are in ENU so we switch X and Y to derive our angles
      double xGain      = position_gain * POS_FLOAT_OF_BFP(position->y - center->y);
      double yGain      = position_gain * POS_FLOAT_OF_BFP(position->x - center->x);
      // And rotate these according to our relative waypoint
      // Our NED x and y gains should now be converted to body angles
      setpoint.theta    = Bound(ANGLE_BFP_OF_REAL( cos(relativeAngle) * xGain - sin(relativeAngle) * yGain ), -BOUND_ANGLE, BOUND_ANGLE);
      setpoint.phi      = Bound(ANGLE_BFP_OF_REAL( sin(relativeAngle) * xGain + cos(relativeAngle) * yGain ), -BOUND_ANGLE, BOUND_ANGLE);
      // Allow the user to steer the drone using RC
      setpoint.theta   += (int32_t) round(ctrl_module_course.rc_y * rc_pitch_gain);
      setpoint.phi     += (int32_t) round(ctrl_module_course.rc_x * rc_roll_gain);
      setpoint.psi      = orientation->psi + ((int32_t) round(ctrl_module_course.rc_z * rc_yaw_gain));
  }
  stabilization_attitude_set_rpy_setpoint_i(&setpoint);
  stabilization_attitude_run(in_flight);
}


////////////////////////////////////////////////////////////////////
// Call our controller
// Implement own Horizontal loops
void guidance_h_module_init(void)
{
  ctrl_module_init();
}

void guidance_h_module_enter(void)
{
  ctrl_module_init();
}

void guidance_h_module_read_rc(void)
{
  // -MAX_PPRZ to MAX_PPRZ
  ctrl_module_course.rc_t = radio_control.values[RADIO_THROTTLE];
  ctrl_module_course.rc_x = radio_control.values[RADIO_ROLL];
  ctrl_module_course.rc_y = radio_control.values[RADIO_PITCH];
  ctrl_module_course.rc_z = radio_control.values[RADIO_YAW];
}

void guidance_h_module_run(bool in_flight)
{
  // Call full inner-/outerloop / horizontal-/vertical controller:
  ctrl_module_run(in_flight);
}

void guidance_v_module_init(void)
{
  // initialization of your custom vertical controller goes here
}

// Implement own Vertical loops
void guidance_v_module_enter(void)
{
  // your code that should be executed when entering this vertical mode goes here
}

void guidance_v_module_run(__attribute__((UNUSED)) bool in_flight)
{
  // your vertical controller goes here
}

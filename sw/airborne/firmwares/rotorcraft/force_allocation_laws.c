/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

/** @file firmwares/rotorcraft/guidance/force_allocation.c
 *  Distribute Outerloop Acceleration Commands To Lifting Surfaces
 *
 */
#include "std.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/force_allocation_laws.h"
#include "modules/ATMOS/multiGain.h"
#include "modules/ATMOS/newTransition.h"

uint8_t transition_percentage;

struct PprzLiftDevice lift_devices[LIFT_GENERATION_NR_OF_LIFT_DEVICES] =
{
  {
    ROTOR_LIFTING_DEVICE,
    100,
    0,
    {0, 0, 0, 0}
  },
  {
    WING_LIFTING_DEVICE,
    0,
    0,
    {0, 0, 0, 0}
  }

};

uint8_t percent_from_rc(int channel)
{
  int per = (MAX_PPRZ + (int32_t)radio_control.values[channel]) * 50 / MAX_PPRZ;
  if (per < 0)
    per = 0;
  else if (per > 100)
    per = 100;
  return per;
}



/**   Force_Allocation_Laws
 *
 *    @param input1 = stabilization_cmd[COMMAND_THRUST]:  updated every loop in guidance_v.
 *    @param input2 = stab_att_sp_euler: in attitude mode only updated on RC-frame = 1 out of 10 times.
 *
 *    @param output = stab_att_sp_quat
 */

int32_t outerloop_throttle_command = 0;

void Force_Allocation_Laws(void)
{
  // Blended Output Commands
  int32_t cmd_thrust = 0;
  struct Int32Eulers command_euler;

  INT_EULERS_ZERO(command_euler);

  float cmd_trim   = 0;

  /////////////////////////////////////////////////////
  // Hard Configure (should come from airframe file
  //transition_percentage=percent_from_rc(RADIO_EXTRA1);

  transition_percentage=percent_from_rc(RADIO_EXTRA1);

  lift_devices[0].activation = transition_percentage;
  lift_devices[1].activation = 100-transition_percentage;

  lift_devices[0].lift_type = ROTOR_LIFTING_DEVICE;
  lift_devices[1].lift_type = WING_LIFTING_DEVICE;

  lift_devices[0].trim_pitch = 0;
  lift_devices[1].trim_pitch = -90;
  /////////////////////////////////////////////////////


  if (transition_percentage < 30) {
    //fixedwing
    atmos_pitch_factor = 1;
    SetGainSetB();
  }
  else if (transition_percentage >= 30 && transition_percentage < 60) {
    //fixedwing+rotor
    atmos_pitch_factor = 0.5;
    SetGainSetA();
  }
  else if (transition_percentage >= 60) {
    //rotor
    SetGainSetA();
    atmos_pitch_factor = 0;
  }

  for (int i=0; i < LIFT_GENERATION_NR_OF_LIFT_DEVICES; i++)
  {

    struct PprzLiftDevice *wing = &(lift_devices[i]);
    float percent = ((float)wing->activation) / 100.0f;

    if (wing->lift_type == ROTOR_LIFTING_DEVICE)
    {
      // Rotorcraft Mode
      // ---------------
      // lift command (vertical acceleration/) -> thrust
      // forward acceleration (command) -> pitch
      // lateral acceleration (command) -> roll
      // heading ANGLE -> yaw

      wing->commands[COMMAND_THRUST] = outerloop_throttle_command;
      wing->commands[COMMAND_ROLL]   = stab_att_sp_euler.phi;
      wing->commands[COMMAND_PITCH]  = stab_att_sp_euler.theta;
      wing->commands[COMMAND_YAW]    = stab_att_sp_euler.psi;
    }
    else
    {
      // Plane Mode
      // ----------
      // lift command (verical acceleration) -> pitch + thrust
      // forward acceleration (neglected)
      // lateral acceleration (command) -> roll
      // heading ANGLE -> integrated

      const float MAX_CLIMB = 3.0f; // m/s
      const float PITCH_OF_VZ = 0.4f;
      const float THROTTLE_INCREMENT = 0.1f;
      float CRUISE_THROTTLE = guidance_v_nominal_throttle;
      const float PITCH_TRIM = 0.0f;

      float climb_speed = ((outerloop_throttle_command - (MAX_PPRZ / 2)) * 2 * MAX_CLIMB);  // MAX_PPRZ

      // Lateral Plane Motion
      wing->commands[COMMAND_ROLL]    = stab_att_sp_euler.phi;

      // Longitudinal Plane Motion
      wing->commands[COMMAND_THRUST]  = (CRUISE_THROTTLE)
                                      + climb_speed * THROTTLE_INCREMENT
                                      + (-(stab_att_sp_euler.theta * MAX_PPRZ) >> INT32_ANGLE_FRAC ); // MAX_PPRZ
                                      //+ ((stab_att_sp_euler.theta * MAX_PPRZ) >> INT32_ANGLE_FRAC ); // MAX_PPRZ

      wing->commands[COMMAND_PITCH]   = ANGLE_BFP_OF_REAL(PITCH_TRIM + climb_speed * PITCH_OF_VZ / MAX_PPRZ);

      // Coordinated Turn
#ifdef FREE_FLOATING_HEADING
      const float function_of_speed = 1.0f;
      const int loop_rate = 512;
      wing->commands[COMMAND_YAW]    += wing->commands[COMMAND_ROLL] * function_of_speed / loop_rate;
#else
      //wing->commands[COMMAND_YAW]    = ahrs.ltp_to_body_euler.psi;
      wing->commands[COMMAND_YAW]    =  stab_att_sp_euler.psi;
#endif
    }

    cmd_thrust           += wing->commands[COMMAND_THRUST]     * percent;
    command_euler.phi    += wing->commands[COMMAND_ROLL]       * percent;
    command_euler.theta  += wing->commands[COMMAND_PITCH]      * percent;
    command_euler.psi    += wing->commands[COMMAND_YAW]        * percent;     // Hmmm this would benefit from some more thinking...
    cmd_trim             += RadOfDeg((float)wing->trim_pitch)  * percent;
  }

  stabilization_cmd[COMMAND_THRUST] = cmd_thrust;

  struct Int32Quat command_att;
  INT32_QUAT_OF_EULERS(command_att, command_euler);
  INT32_QUAT_WRAP_SHORTEST(command_att);

  // Post Multiply with the pitch trim...
  struct Int32Quat trim_quat;
  QUAT_ASSIGN(trim_quat,
	QUAT1_BFP_OF_REAL(1),
	QUAT1_BFP_OF_REAL(0),
	QUAT1_BFP_OF_REAL(cmd_trim) / 2,
	QUAT1_BFP_OF_REAL(0) );
  INT32_QUAT_NORMALIZE(trim_quat);

  // INT32_QUAT_OF_AXIS_ANGLE(trim_quat, axis, cmd_trim)
  INT32_QUAT_COMP(stab_att_sp_quat, command_att, trim_quat);
  //INT32_QUAT_COMP(stab_att_sp_quat, trim_quat, command_att);
}


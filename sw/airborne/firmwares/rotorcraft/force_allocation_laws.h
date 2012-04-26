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

/** @file firmwares/rotorcraft/guidance/force_allocation.h
 *  Distribute Outerloop Acceleration Commands To Lifting Surfaces
 *
 */

#define FORCE_ALLOCATION_H

#include "std.h"

#include "generated/airframe.h"

#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "subsystems/radio_control.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "subsystems/ahrs.h"
#include "subsystems/ins.h"
#include "math/pprz_algebra_int.h"


struct PprzLiftDevice {
  // Type and Activation
  enum lift_type_enum {ROTOR_LIFTING_DEVICE = 0, WING_LIFTING_DEVICE = 1} lift_type;
  int activation;   // 0 to 100 percent
  
  int trim_pitch;
  
  // Output
  int32_t commands[COMMANDS_NB];
};

#ifndef LIFT_GENERATION_NR_OF_LIFT_DEVICES
#error Please Define a WING_LIFTING_DEVICE or ROTOR_LIFTING_DEVICE in your airframe file
#endif

struct PprzLiftDevice lift_devices[LIFT_GENERATION_NR_OF_LIFT_DEVICES] = 
{
  {
    ROTOR_LIFTING_DEVICE,
    100,
    0,
    0, 0, 0, 0
  }
};


__attribute__ ((always_inline)) static inline void Force_Allocation_Laws(void) 
{
  stabilization_cmd[COMMAND_ROLL]   = 0; // actuator 0 - MAX_PPRZ
  stabilization_cmd[COMMAND_PITCH]  = 0; // command: linear angle
  stabilization_cmd[COMMAND_THRUST] = 0; // command
  stabilization_cmd[COMMAND_YAW]    = 0; // abolute psi
  

  for (int i=0; i < LIFT_GENERATION_NR_OF_LIFT_DEVICES; i++)
  {
    struct PprzLiftDevice *wing = &(lift_devices[i]);

    if (wing->lift_type == ROTOR_LIFTING_DEVICE)
    {
      // Rotorcraft Mode
      // ---------------
      // lift command (vertical acceleration/) -> thrust
      // forward acceleration (command) -> pitch
      // lateral acceleration (command) -> roll
      // heading ANGLE -> yaw
      
      wing->commands[COMMAND_THRUST] = stabilization_cmd[COMMAND_THRUST];
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
      const float PITCH_OF_VZ = 0.1f;
      const float THROTTLE_INCREMENT = 0.1f;
      const float CRUISE_THROTTLE = 0.5f;
      const float PITCH_TRIM = 0.0f;
    
      int32_t climb_speed = ((stabilization_cmd[COMMAND_THRUST] - (MAX_PPRZ / 2)) * 2 * MAX_CLIMB);  // FRAC_COMMAND
    
      // Lateral Motion
      wing->commands[COMMAND_ROLL]    = stab_att_sp_euler.phi;

      // Vertical Motion
      wing->commands[COMMAND_THRUST]  = (CRUISE_THROTTLE * MAX_PPRZ)
                                      + climb_speed * THROTTLE_INCREMENT
                                      + (stab_att_sp_euler.theta * MAX_PPRZ); // FRAC_COMMAND
      wing->commands[COMMAND_PITCH]   = PITCH_TRIM + climb_speed * PITCH_OF_VZ;
      
      // Longitudinal Motion
      
      // Coordinated Turn
      const float function_of_speed = 1.0f;
      const int loop_rate = 512;
      wing->commands[COMMAND_YAW]    += wing->commands[COMMAND_ROLL] * function_of_speed / loop_rate;
    }
    
    stabilization_cmd[COMMAND_THRUST]  += wing->commands[COMMAND_THRUST] * wing->activation / 100;
    stabilization_cmd[COMMAND_ROLL]    += wing->commands[COMMAND_ROLL]   * wing->activation / 100;
    stabilization_cmd[COMMAND_PITCH]   += wing->commands[COMMAND_PITCH]  * wing->activation / 100;
    stabilization_cmd[COMMAND_YAW]     += wing->commands[COMMAND_ROLL]   * wing->activation / 100;
    
  }

  stabilization_cmd[COMMAND_ROLL] /= LIFT_GENERATION_NR_OF_LIFT_DEVICES;
  stabilization_cmd[COMMAND_PITCH] /= LIFT_GENERATION_NR_OF_LIFT_DEVICES;
  stabilization_cmd[COMMAND_THRUST] /= LIFT_GENERATION_NR_OF_LIFT_DEVICES;
  stabilization_cmd[COMMAND_YAW] /= LIFT_GENERATION_NR_OF_LIFT_DEVICES;
  
  
}

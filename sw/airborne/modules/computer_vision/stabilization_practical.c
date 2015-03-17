/*
 * Copyright (C) 2014 Hann Woei Ho
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/stabilization_practical.c
 * @brief Optical-flow based control for Linux based systems
 *
 */

// Own Header
#include "stabilization_practical.h"

// Stabilization
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "autopilot.h"
#include "subsystems/datalink/downlink.h"

static struct Int32Eulers cmd;

static int8_t turning = 0;
int32_t adding_theta = -250;
int32_t adding_psi = 10;
int32_t adding_phi = 350;

/**
 * Horizontal guidance mode enter resets the errors
 * and starts the controller.
 */
void guidance_h_module_enter(void)
{
  // Set the euler command to 0
  INT_EULERS_ZERO(cmd);
  cmd.theta = adding_theta;
  cmd.psi = stateGetNedToBodyEulers_i()->psi;
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
  // TODO: change the desired vx/vy
}

/**
 * Main guidance loop
 * @param[in] in_flight Whether we are in flight or not
 */
void guidance_h_module_run(bool_t in_flight)
{
  // GUIDANCE: Set Hover-z-hold (0.5 meter)
  guidance_v_z_sp = -1 << 7;

  // If turning left
  if(turning == -1) {
    cmd.psi += adding_psi;
    cmd.phi = adding_phi;
  }
  // If not turning
  else if(turning == 0) {
    cmd.phi = 0;
  }
  // If turning right
  else {
    cmd.psi -= adding_psi;
    cmd.phi = -adding_phi;
  }

  INT32_COURSE_NORMALIZE(cmd.psi);
  stabilization_attitude_set_rpy_setpoint_i(&cmd);

  // Run the default attitude stabilization
  stabilization_attitude_run(in_flight);
}

/**
 * Update the controls based on a vision result
 * @param[in] left Turn left or right
 */
void stabilization_practical_turn(int8_t turn)
{
  turning = turn;
}

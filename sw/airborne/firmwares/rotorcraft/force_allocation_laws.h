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
#ifndef FORCE_ALLOCATION_H
#define FORCE_ALLOCATION_H
#include "std.h"
#include "generated/airframe.h"

extern uint8_t transition_percentage,transition_percentage_nav;
extern int32_t outerloop_throttle_command;

extern float force_allocation_fixedwing_max_climb; // m/s
extern float force_allocation_fixedwing_pitch_of_vz; // VZ = vertical speed
extern float force_allocation_fixedwing_throttle_of_vz;
extern float force_allocation_fixedwing_pitch_trim;
extern float force_allocation_fixedwing_throttle_of_xdd; // forward acceleration
extern float force_allocation_fixedwing_yawrate_of_ydd;  // lateral acceleration

extern int32_t dbg1;
extern uint32_t dbg2;
extern uint32_t dbg3;
extern int32_t dbg4;


struct PprzLiftDevice {
  // Type and Activation
  enum lift_type_enum {ROTOR_LIFTING_DEVICE = 0, WING_LIFTING_DEVICE = 1} lift_type;
  int activation;   // 0 to 100 percent

  int orientation_pitch;

  // Output
  int32_t commands[COMMANDS_NB];
};


extern uint8_t percent_from_rc(int channel);
extern void Force_Allocation_Laws(void);

// TODO
// HARD CODE FOR NOW
#ifdef LIFT_GENERATION_NR_OF_LIFT_DEVICES
#undef LIFT_GENERATION_NR_OF_LIFT_DEVICES
//#error Please Define a WING_LIFTING_DEVICE or ROTOR_LIFTING_DEVICE in your airframe file
#endif

#define LIFT_GENERATION_NR_OF_LIFT_DEVICES 2

#endif

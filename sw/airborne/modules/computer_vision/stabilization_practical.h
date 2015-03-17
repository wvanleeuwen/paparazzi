/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file modules/computer_vision/stabilization_practical.h
 * @brief Optical-flow based control for Linux based systems
 *
 */

#ifndef CV_STABILIZATION_PRACTICAL_H_
#define CV_STABILIZATION_PRACTICAL_H_

#include "std.h"
#include "math/pprz_algebra_int.h"

extern int32_t adding_theta;
extern int32_t adding_psi;
extern int32_t adding_phi;

// Implement own Horizontal loops
extern void guidance_h_module_enter(void);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_run(bool_t in_flight);

// Update the stabiliztion commands based on a vision result
void stabilization_practical_turn(int8_t turn);

#endif /* CV_STABILIZATION_PRACTICAL_H_ */

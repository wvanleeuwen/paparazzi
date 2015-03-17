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
 * @file modules/computer_vision/practical_module.h
 * @brief Module for the Practical assignment
 *
 * Avoiding obstacles in the arena
 */

#ifndef PRACTICAL_MODULE_H
#define PRACTICAL_MODULE_H

#include "std.h"

struct practical_t {
  uint8_t y_m;
  uint8_t y_M;
  uint8_t u_m;
  uint8_t u_M;
  uint8_t v_m;
  uint8_t v_M;
};
extern struct practical_t practical;

// Module functions
extern void practical_module_init(void);
extern void practical_module_run(void);
extern void practical_module_start(void);
extern void practical_module_stop(void);

#endif /* PRACTICAL_MODULE_H */

/*
 *
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
 *
 */

/**
 * @file boards/bebop/electrical.c
 * Dummy electrical status readings for the bebop.
 * Because the voltage measurements is done trough the motor controllers.
 */

#include "subsystems/electrical.h"
#include "subsystems/commands.h"
#include <stdlib.h>

struct Electrical electrical;
static float nonlin_factor;

#if defined COMMAND_THROTTLE
#define COMMAND_CURRENT_ESTIMATION COMMAND_THROTTLE
#elif defined COMMAND_THRUST
#define COMMAND_CURRENT_ESTIMATION COMMAND_THRUST
#endif

#ifndef CURRENT_ESTIMATION_NONLINEARITY
#define CURRENT_ESTIMATION_NONLINEARITY 1.2
#endif

void electrical_init(void)
{
  // First we try to kill the dragon-prog and its respawner if it is running (done here because initializes first)
  int ret = system("killall -9 watchdog.sh; killall -9 dragon-prog");
  (void) ret;

  electrical.current = 0;
#if defined MILLIAMP_AT_FULL_THROTTLE
  PRINT_CONFIG_VAR(CURRENT_ESTIMATION_NONLINEARITY)
  nonlin_factor = CURRENT_ESTIMATION_NONLINEARITY;
#endif
}

void electrical_periodic(void) {
#if defined MILLIAMP_AT_FULL_THROTTLE && defined COMMAND_CURRENT_ESTIMATION
  /*
   * Superellipse: abs(x/a)^n + abs(y/b)^n = 1
   * with a = 1
   * b = mA at full throttle
   * n = 1.2     This defines nonlinearity (1 = linear)
   * x = throttle
   * y = current
   *
   * define CURRENT_ESTIMATION_NONLINEARITY in your airframe file to change the default nonlinearity factor of 1.2
   */
  float b = (float)MILLIAMP_AT_FULL_THROTTLE;
  float x = ((float)commands[COMMAND_CURRENT_ESTIMATION]) / ((float)MAX_PPRZ);
  /* electrical.current y = ( b^n - (b* x/a)^n )^1/n
   * a=1, n = electrical_priv.nonlin_factor
   */
  electrical.current = b - pow((pow(b, nonlin_factor) - pow((b * x), nonlin_factor)),
                               (1. / nonlin_factor));
#endif
}

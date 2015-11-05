/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/helicopter/swashplate_mixing.h"
 * @author C. De Wagter
 * Helicopter Swashplate Mixing
 */

#ifndef SWASHPLATE_MIXING_H
#define SWASHPLATE_MIXING_H

#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

/**
 * MECH, H120 (front/rightback/leftback), HR120 (back/leftfront/rightfront)
 */

#if SWASHPLATE_MIXING_TYPE == MECH

#define SW_NB 2
#define SW_FRONT     0
#define SW_RIGHT     1
#define SW_MIXING_ROLL_COEF   {    0, -256 }
#define SW_MIXING_PITCH_COEF  {  256,    0 }
#define SW_MIXING_THRUST_COEF {    0,    0 }


#elif SWASHPLATE_MIXING_TYPE == H120

#define SW_NB 3
#define SW_FRONT     0
#define SW_RIGHTBACK 1
#define SW_LEFTBACK  2
#define SW_MIXING_ROLL_COEF   {    0, -256,  256 }
#define SW_MIXING_PITCH_COEF  {  256, -128, -128 }
#define SW_MIXING_THRUST_COEF {  256,  256,  256 }

#elif SWASHPLATE_MIXING_TYPE == HR120

#define SW_NB 3
#define SW_BACK       0
#define SW_LEFTFRONT  1
#define SW_RIGHTFRONT 2
#define SW_MIXING_ROLL_COEF   {    0,  256, -256 }
#define SW_MIXING_PITCH_COEF  { -256,  128,  128 }
#define SW_MIXING_THRUST_COEF {  256,  256,  256 }

#endif


struct SwashplateMixing {
  int32_t commands[SW_NB];
  int32_t trim[SW_NB];
};

extern struct SwashplateMixing swashplate_mixing;



extern void swashplate_mixing_init(void);
extern void swashplate_mixing_periodic(void);

extern void swashplate_mixing_run(bool_t motors_on, bool_t override_on, pprz_t in_cmd[]);

#endif


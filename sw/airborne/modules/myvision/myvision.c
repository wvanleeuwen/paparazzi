/*
 * Copyright (C) cdw
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
 * @file "modules/myvision/myvision.c"
 * @author cdw
 * bla
 */

#include "modules/myvision/myvision.h"

#include "generated/flight_plan.h"
#include "firmwares/rotorcraft/navigation.h"

void myvision_run()
{
  static double time = 0;
  struct EnuCoor_f enu;
  time += 0.02;
  enu.x = cos(time)*20.0;
  enu.y = sin(time)*20.0;
  enu.z = 2.0;
  nav_set_waypoint_enu_f(WP_STDBY, &enu);
}

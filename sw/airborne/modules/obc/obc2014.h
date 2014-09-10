/*
 * Copyright (C) 2014 OpenUAS
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

/** \file obc2014.h
 *
 * OUTBACK Challenge Stuff
 */

#ifndef SAFETY_TIMERS_H
#define SAFETY_TIMERS_H

#include "std.h"
#include "generated/airframe.h"

#define SCRUTENEERING_FLIGHT  0
#define SEARCH_FLIGHT 1
#define DROP_FLIGHT 2
extern int obc_flight_mode;

extern float set_airspeed_nominal;
extern float set_airspeed_tracking;
extern float set_airspeed_glide;
extern float set_takeoff_pitch;
extern float set_flare_pitch;


#define SetAltitudeForFinalFromTo(X,Y) (waypoints[Y].a=(waypoints[X].a+35))

#include "inter_mcu.h"

#define ThrottleHigh() MoreThan(fbw_state->channels[RADIO_THROTTLE],4000)

extern void periodic_obc(void);
extern bool_t gps_has_been_good(void);

#endif

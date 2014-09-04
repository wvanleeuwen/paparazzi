/*
 * Copyright (C) 2009  Gautier Hattenberger
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

/** \file safety_timers.h
 *
 * demo module with blinking LEDs
 */

#ifndef SAFETY_TIMERS_H
#define SAFETY_TIMERS_H

#include "std.h"
#include "generated/airframe.h"

#define SCRUTENEERING_FLIGHT  0
#define SEARCH_FLIGHT 1
#define DROP_FLIGHT 2
extern int obc_flight_mode;

#define AirbrakesOff() {ap_state->commands[COMMAND_BRAKE]=0;}
#define AirbrakesOn() {ap_state->commands[COMMAND_BRAKE]=SERVO_BRAKE_FULL;}

#define AircraftIsBooting() ((nav_block <= 3))



#endif

/*
 * Copyright (C) 2013 Freek van Tienen
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

#ifdef ARDRONE2_RAW
#include <stdio.h>
#endif

#include "std.h"
#include "generated/settings.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "imav2013.h"
#define EXPLAIN_NAME        0
#define EXPLAIN_IMAV2013    4

// Flight plans
// 1
#define FLIGHT_PLAN_188             {BLOCK_Land, BLOCK_Race, BLOCK_VTOL}
// 2
#define FLIGHT_PLAN_186             {BLOCK_Land, BLOCK_Race, BLOCK_VTOL}
// 3
#define FLIGHT_PLAN_196             {BLOCK_Land, BLOCK_Race, BLOCK_VTOL}
// 4
#define FLIGHT_PLAN_193             {BLOCK_Land, BLOCK_Race, BLOCK_VTOL}
// 5
#define FLIGHT_PLAN_184             {BLOCK_VTOL, BLOCK_Race, BLOCK_Land}
// 6
#define FLIGHT_PLAN_195             {BLOCK_Search, BLOCK_VTOL, BLOCK_Land}
// 7
#define FLIGHT_PLAN_194             {BLOCK_Race, BLOCK_Race, BLOCK_Land}
// 8 -> Errors
#define FLIGHT_PLAN_189             {BLOCK_Race, BLOCK_Race, BLOCK_Land}

// Visio
#define FLIGHT_PLAN_191             {BLOCK_Search, BLOCK_VTOL}
// Pyro
#define FLIGHT_PLAN_185             {BLOCK_Dropzone_1, BLOCK_Dropzone_2, BLOCK_Dropzone_3, BLOCK_Dropzone_4, BLOCK_Land}


#define _FLIGHT_PLAN(id)    FLIGHT_PLAN_ ## id
#define IMAV2013_FLIGHT_PLAN(id)    _FLIGHT_PLAN(id)
static uint8_t flight_plan[] = IMAV2013_FLIGHT_PLAN( AC_ID );
static uint8_t flight_plan_idx = 0;

static inline void imav2013_send_imav2013(void);
static inline void imav2013_send_name(void);

void imav2013_periodic(void) {
  RunOnceEvery(3, imav2013_send_imav2013());
  RunOnceEvery(10, imav2013_send_name());
}

static inline void imav2013_send_imav2013(void) {
  uint8_t type = EXPLAIN_IMAV2013;
  uint8_t id = 1;
  uint8_t string[15];

  string[0]     = SETTINGS_kill_throttle;
  string[1]     = SETTINGS_flight_altitude;
  string[2]     = SETTINGS_time_until_land;
  string[4]     = BLOCK_Start_Engine;
  string[5]     = BLOCK_Safety_Land_Here;
  string[6]     = BLOCK_Land;
  string[7]     = BLOCK_Safety_Standby;
  string[8]     = WP_TD;
  string[9]     = WP_STDBY;
  string[10]    = WP_HOME;
  string[11]    = WP__FA1;
  string[12]    = WP__FA2;
  string[13]    = WP__FA3;
  string[14]    = WP__FA4;

  DOWNLINK_SEND_EXPLAIN(DefaultChannel, DefaultDevice, &type, &id, 15, string);
}

static inline void imav2013_send_name(void) {
  uint8_t type = EXPLAIN_NAME;
  uint8_t id = 1;
  uint8_t string[] = AIRFRAME_NAME;
  uint8_t size = sizeof(string);

  DOWNLINK_SEND_EXPLAIN(DefaultChannel, DefaultDevice, &type, &id, size, string);
}

bool_t imav2013_dropball(void) {
#ifdef ARDRONE2_RAW
 printf("<Drop_Paintball_Now>");
#endif
 return FALSE;
}

bool_t imav2013_goto_mission(void) {
  nav_goto_block(flight_plan[flight_plan_idx]);
  flight_plan_idx = (flight_plan_idx + 1) % sizeof(flight_plan);
  return TRUE;
}

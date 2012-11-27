/*
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
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

/** @file ins_ardrone.c
 * Parser for the ardrone protocol.
 * Based on ins_xsens.c
 */

#include "ins_module.h"
#include "subsystems/ins.h"
#include "ins_ardrone.h"

#include <inttypes.h>

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "messages.h"


#include "subsystems/gps.h"
#include "math/pprz_geodetic_wgs84.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h" /* needed for nav_utm_zone0 */


// positions
INS_FORMAT ins_x;
INS_FORMAT ins_y;
INS_FORMAT ins_z;

// velocities
INS_FORMAT ins_vx;
INS_FORMAT ins_vy;
INS_FORMAT ins_vz;

// body angles
INS_FORMAT ins_phi;
INS_FORMAT ins_theta;
INS_FORMAT ins_psi;

// angle rates
INS_FORMAT ins_p;
INS_FORMAT ins_q;
INS_FORMAT ins_r;

// accelerations
INS_FORMAT ins_ax;
INS_FORMAT ins_ay;
INS_FORMAT ins_az;

// magnetic
INS_FORMAT ins_mx;
INS_FORMAT ins_my;
INS_FORMAT ins_mz;

#if USE_INS_MODULE
float ins_pitch_neutral;
float ins_roll_neutral;
#endif

ins_x = 0; /* gps */
ins_y = 0; /* gps */
ins_z = 0;

ins_vx = 0;
ins_vy = 0;
ins_vz = 0;

ins_phi = 0;
ins_theta = 0;
ins_psi = 0;

ins_p = 0;
ins_q = 0;
ins_r = 0;

ins_ax = 0;
ins_ay = 0;
ins_az = 0;

ins_mx = 0;
ins_my = 0;
ins_mz = 0;

ins_roll_neutral = 0; /* leave at 0 */
ins_pitch_neutral = 0; /* leave at 0 */


volatile uint8_t ins_msg_received;
volatile uint8_t new_ins_attitude;

void handle_ins_msg(void);
void parse_ins_msg(void);
void parse_ins_buffer(uint8_t);

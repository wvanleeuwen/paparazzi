/*
 * Copyright (C) 2014 CDW
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

#include "qnh.h"
#include "state.h"
#include "subsystems/sensors/baro.h"
#include "generated/airframe.h"

float qnh = 0;
float amsl = 0;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void send_amsl(void)
{
  DOWNLINK_SEND_AMSL(DefaultChannel, DefaultDevice, &amsl, &qnh);
}
#endif


void init_qnh(void) {
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(&telemetry_Ap, "AMSL", send_amsl);
#endif
  qnh = 1013.00;
}

void periodic_qnh(void) {
/*
  1200Pa per 100m
  8.333cm per Pa

  (1 -   (p/p0) ^ ((R*L)/(g*M))    ) * T0/L
*/

  const float L = 0.0065; // [K/m]
  const float p0 = 101325; // [Pa]
  const float T0 = 288.15; // [K]
  const float g = 9.80665; // [m/s^2]
  const float M = 0.0289644; // [kg/mol]
  const float R = 8.31447; // [J/(mol*K)]
  const float MeterPerFeet = 0.3048;

  float h = stateGetPositionLla_f()->alt;

  float Trel = 1 - L*h/T0;
  float Expo = g * M / R / L;
  float p = p0 * pow(Trel,Expo);
  qnh = p / 100.0f;

  amsl = h / MeterPerFeet;
}


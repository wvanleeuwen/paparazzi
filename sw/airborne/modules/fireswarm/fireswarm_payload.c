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

#include "fireswarm_payload.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "AutoPilotProt.h"

#define FIRESWARM_PAYLOAD_POWER_LED  5

AutoPilotMsgSensorData FireSwarmData;

void fireswarm_payload_init(void)
{
  LED_INIT(FIRESWARM_PAYLOAD_POWER_LED);

  FireSwarmData.FlyState = 1;
  FireSwarmData.GPSState = 3;
}

const char* hello_world = "Hello World\n";

void fireswarm_periodic(void)
{
  static uint8_t distribute = 0;
  LED_TOGGLE(FIRESWARM_PAYLOAD_POWER_LED);

  distribute++;
  if (distribute >= 10)
  {
    char* c = (char*) hello_world;
    while (*c != 0)
      Uart3Transmit(*c++);

    distribute = 0;
  }
}

void fireswarm_event(void)
{
}


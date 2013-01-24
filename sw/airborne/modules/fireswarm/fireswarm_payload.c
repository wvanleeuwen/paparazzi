/*
 * Copyright (C) 2013  Christophe De Wagter
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
#include "fireswarm_communication.h"

#define FIRESWARM_PAYLOAD_POWER_LED  5

AutoPilotMsgHeader FireSwarmHeader;
AutoPilotMsgSensorData FireSwarmData;

void fireswarm_payload_init(void)
{
  LED_INIT(FIRESWARM_PAYLOAD_POWER_LED);

  FireSwarmData.FlyState = 1;
  FireSwarmData.GPSState = 3;
 
  FireSwarmHeader.Header = 0x1234;
  FireSwarmHeader.MsgType = AP_PROT_REQ_SENSORDATA;
  FireSwarmHeader.TimeStamp = 0;
  FireSwarmHeader.DataSize = sizeof(FireSwarmData);
  
  fireswarm_payload_link_init();
 
}

const char* hello_world = "Hello World\n";

void fireswarm_periodic(void)
{
  LED_TOGGLE(FIRESWARM_PAYLOAD_POWER_LED);

  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));

}

void fireswarm_event(void)
{
}


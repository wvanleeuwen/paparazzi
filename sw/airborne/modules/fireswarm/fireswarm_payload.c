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
 
  FireSwarmHeader.Header = 0x1234;
  FireSwarmHeader.MsgType = AP_PROT_REQ_SENSORDATA;
  FireSwarmHeader.TimeStamp = 0;
  FireSwarmHeader.DataSize = sizeof(FireSwarmData);
  
  fireswarm_payload_link_init();
 
}

void fireswarm_periodic(void)
{
  LED_TOGGLE(FIRESWARM_PAYLOAD_POWER_LED);

  FireSwarmData.FlyState = AP_PROT_FLY_STATE_FLYING;
  int gps_quality = 255 - (gps.pacc-200) / 20;
  if (gps_quality < 0) gps_quality = 0;
  if (gps_quality > 255) gps_quality = 255;
  FireSwarmData.GPSState = gps_quality;
  FireSwarmData.BatteryLeft = 255;
  FireSwarmData.ServoState = AP_PROT_STATE_SERVO_PROP | AP_PROT_STATE_SERVO_WING_LEFT | AP_PROT_STATE_SERVO_WING_RIGHT | AP_PROT_STATE_SERVO_TAIL;
  FireSwarmData.AutoPilotState = AP_PROT_STATE_AP_OUTER_LOOP | AP_PROT_STATE_AP_INNER_LOOP;
  FireSwarmData.SensorState = AP_PROT_STATE_SENSOR_COMPASS | AP_PROT_STATE_SENSOR_ACCELERO | AP_PROT_STATE_SENSOR_GPS | AP_PROT_STATE_SENSOR_WIND | AP_PROT_STATE_SENSOR_PRESSURE;
  
  FireSwarmData.Position.X = stateGetPositionUtm_f()->alt;
  FireSwarmData.Position.Y = 3;
  FireSwarmData.Position.Z = 3;

  FireSwarmData.GroundSpeed = 3;
  FireSwarmData.VerticalSpeed = 3;
  FireSwarmData.Heading = 3;
  FireSwarmData.Yaw = 3;
  FireSwarmData.Pitch = 3;
  FireSwarmData.Roll = 3;
  FireSwarmData.WindHeading = 3;
  FireSwarmData.WindSpeed = 3;
    
  
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));

}

void fireswarm_event(void)
{
}


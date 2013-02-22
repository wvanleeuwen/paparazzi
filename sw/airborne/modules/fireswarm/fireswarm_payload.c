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
#include "subsystems/gps.h"
#include "subsystems/nav.h"
#include "state.h"
#include "led.h"

#include "fireswarm_communication.h"
#include "subsystems/navigation/common_flight_plan.h"

#define FIRESWARM_PAYLOAD_POWER_LED  5

AutoPilotMsgHeader     FireSwarmHeader;

AutoPilotMsgSensorData FireSwarmData;      // out
AutoPilotMsgWpStatus   FireSwarmStatus;    // out
AutoPilotMsgLanding    FireSwarmLanding;   // in
AutoPilotMsgMode       FireSwarmMode;      // in
AutoPilotMsgWayPoints  FireSwarmWaypoints; // in

#define FIRESWARM_LED_ON(X) LED_OFF(X)
#define FIRESWARM_LED_OFF(X) LED_ON(X)

void fireswarm_payload_init(void)
{
  LED_INIT(FIRESWARM_PAYLOAD_POWER_LED);
  LED_INIT(FIRESWARM_READY_LED);

  FireSwarmHeader.Header = AP_PROT_HEADER;
  FireSwarmHeader.MsgType = AP_PROT_SENSORDATA;
  FireSwarmHeader.TimeStamp = 0;
  FireSwarmHeader.DataSize = sizeof(FireSwarmData);

  fireswarm_payload_link_init();

  FIRESWARM_LED_ON(FIRESWARM_PAYLOAD_POWER_LED);
  FIRESWARM_LED_OFF(FIRESWARM_READY_LED);

}

void fireswarm_periodic(void)
{
  static uint8_t dispatch = 0;

  FireSwarmData.FlyState = AP_PROT_FLY_STATE_FLYING;
  int gps_quality = 255 - (gps.pacc-200) / 20;
  if (gps_quality < 0) gps_quality = 0;
  if (gps_quality > 255) gps_quality = 255;

  dispatch++;
  if (dispatch >= 16)
  {
    dispatch = 0;
//     LED_TOGGLE(FIRESWARM_READY_LED);
  }


  FireSwarmHeader.MsgType = AP_PROT_SENSORDATA;
  FireSwarmHeader.DataSize = sizeof(FireSwarmData);

  FireSwarmData.GPSState = gps_quality;
  FireSwarmData.BatteryLeft = 255;
  FireSwarmData.ServoState = AP_PROT_STATE_SERVO_PROP | AP_PROT_STATE_SERVO_WING_LEFT | AP_PROT_STATE_SERVO_WING_RIGHT | AP_PROT_STATE_SERVO_TAIL;
  FireSwarmData.AutoPilotState = AP_PROT_STATE_AP_OUTER_LOOP | AP_PROT_STATE_AP_INNER_LOOP;
  FireSwarmData.SensorState = AP_PROT_STATE_SENSOR_COMPASS | AP_PROT_STATE_SENSOR_ACCELERO | AP_PROT_STATE_SENSOR_GPS | AP_PROT_STATE_SENSOR_WIND | AP_PROT_STATE_SENSOR_PRESSURE;

  FireSwarmData.Position.X = stateGetPositionUtm_f()->east;
  FireSwarmData.Position.Y = stateGetPositionUtm_f()->north;
  FireSwarmData.Position.Z = stateGetPositionUtm_f()->alt;

  FireSwarmData.GroundSpeed = 0;
  FireSwarmData.VerticalSpeed = 0;
  FireSwarmData.Heading = 0;
  FireSwarmData.Yaw = 0;
  FireSwarmData.Pitch = 0;
  FireSwarmData.Roll = 0;
  FireSwarmData.WindHeading = 0;
  FireSwarmData.WindSpeed = 0;


  fireswarm_payload_link_start();
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmHeader, sizeof(FireSwarmHeader));
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));
  fireswarm_payload_link_crc();

  //fprintf(stderr,"Bytes: %d \n", fireswarm_payload_link_has_data());
  //fprintf(stderr,".");
}

void fireswarm_periodic_nav(void)
{
  FireSwarmHeader.MsgType = AP_PROT_WP_STATUS;
  FireSwarmHeader.DataSize = sizeof(FireSwarmStatus);

  FireSwarmStatus.NumWaypoints = 0;

  fireswarm_payload_link_start();
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmHeader, sizeof(FireSwarmHeader));
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmStatus, sizeof(FireSwarmStatus));
  fireswarm_payload_link_crc();

  //fprintf(stderr,"Bytes: %d \n", fireswarm_payload_link_has_data());
  //fprintf(stderr,".");
}

/* parser status */
#define FIRESWARM_UNINIT        0
#define FIRESWARM_GOT_SYNC1     1
#define FIRESWARM_GOT_SYNC2     2
#define FIRESWARM_GOT_ID        3
#define FIRESWARM_GOT_T0        4
#define FIRESWARM_GOT_T1        5
#define FIRESWARM_GOT_T2        6
#define FIRESWARM_GOT_T3        7
#define FIRESWARM_GOT_LEN       8
#define FIRESWARM_GOT_PAYLOAD   9

#define FIRESWARM_MAX_PAYLOAD 255
struct FireSwarmMessage {
  bool_t msg_available;
  uint8_t msg_buf[FIRESWARM_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t crc;

  uint8_t status;
  uint8_t len;
  uint8_t msg_idx;
  uint8_t error_cnt;
  uint8_t error_last;
};

struct FireSwarmMessage fsw_msg;

/* parsing */
void fireswarm_parse( uint8_t c );
void fireswarm_parse( uint8_t c ) {
  switch (fsw_msg.status) {
  case FIRESWARM_UNINIT:
    fsw_msg.crc = c;
    if (c == 0xee)
      fsw_msg.status++;
    break;
  case FIRESWARM_GOT_SYNC1:
    fsw_msg.crc += c;
    if (c != 0xfe) {
      fsw_msg.error_last = 0x01;
      goto error;
    }
    fsw_msg.status++;
    break;
  case FIRESWARM_GOT_SYNC2:
    if (fsw_msg.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      fsw_msg.error_last = 2;
      goto error;
    }
    fsw_msg.msg_id = c;
    fsw_msg.status++;
    fsw_msg.crc += c;
    break;
  case FIRESWARM_GOT_ID:
  case FIRESWARM_GOT_T0:
  case FIRESWARM_GOT_T1:
  case FIRESWARM_GOT_T2:
    fsw_msg.crc += c;
    fsw_msg.status++;
    break;
  case FIRESWARM_GOT_T3:
    fsw_msg.crc += c;
    fsw_msg.len = c;
    fsw_msg.msg_idx = 0;
    fsw_msg.status++;
    if (fsw_msg.msg_idx >= fsw_msg.len)
      fsw_msg.status++;
    break;
  case FIRESWARM_GOT_LEN:
    fsw_msg.crc += c;
    fsw_msg.msg_buf[fsw_msg.msg_idx] = c;
    fsw_msg.msg_idx++;
    if (fsw_msg.msg_idx >= fsw_msg.len) {
      fsw_msg.status++;
    }
    break;
  case FIRESWARM_GOT_PAYLOAD:
    if (fsw_msg.crc == c)
    {
      // fprintf(stderr,"OK %d %d %d <> %d \n", fsw_msg.msg_id, fsw_msg.len, fsw_msg.crc, c);

      fsw_msg.msg_available = TRUE;
      goto restart;
    }
    else
    {
      fsw_msg.error_last = 3;
      goto error;
    }
    break;
  default:
    fsw_msg.error_last = 4;
    goto error;
  }
  return;
 error:
#ifdef SITL
   fprintf(stderr,"Error %d %d %d %d <> %d \n", fsw_msg.error_last, fsw_msg.msg_id, fsw_msg.len, fsw_msg.crc, c);
#endif
  fsw_msg.error_cnt++;
 restart:
  fsw_msg.status = FIRESWARM_UNINIT;
  return;
}



void fireswarm_event(void)
{
  while (fireswarm_payload_link_has_data())
  {
    //printf("read ->");
    fireswarm_parse(fireswarm_payload_link_get());
    if (fsw_msg.msg_available)
    {
#ifdef SITL
      fprintf(stderr,"MSG %d %d \n",fsw_msg.msg_id, fsw_msg.len );
#endif
      fsw_msg.msg_available = 0;

      switch (fsw_msg.msg_id)
      {
      case AP_PROT_SET_MODE:
        memcpy(&FireSwarmMode, fsw_msg.msg_buf, sizeof(FireSwarmMode));
        //nav_block = FireSwarmMode.Mode + 3;
        nav_pitch = ((float)FireSwarmMode.Mode) / 10;
        break;
      case AP_PROT_SET_WAYPOINTS:
        LED_TOGGLE(FIRESWARM_PAYLOAD_POWER_LED);
        break;
      case AP_PROT_SET_LAND:
        LED_TOGGLE(FIRESWARM_READY_LED);
        break;
      }
    }
  }
}


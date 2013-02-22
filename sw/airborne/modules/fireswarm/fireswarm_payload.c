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
#include "subsystems/ahrs.h"
#include "generated/flight_plan.h"
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

#define FIRESWARM_HEARTBEAT_DELAY 20
uint8_t fireswarm_heartbeat = 0;

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

void fireswarm_status_led(uint8_t ahrs, uint8_t gps, uint8_t payload)
{
  static uint8_t dispatch = 0;
  dispatch++;

  if (ahrs)
  {
    if (gps && payload)
    {
      FIRESWARM_LED_ON(FIRESWARM_READY_LED);
    }
    else
    {
      uint8_t rate = 15;

      if (gps)
        rate = 6;

      if (dispatch >= rate)
      {
        dispatch = 0;
        LED_TOGGLE(FIRESWARM_READY_LED);
      }

    }
  }
  else
  {
    // Flash Error FAST
    LED_TOGGLE(FIRESWARM_READY_LED);
  }
}


void fireswarm_periodic(void)
{
  struct FloatEulers* att = stateGetNedToBodyEulers_f();

  if (fireswarm_heartbeat > 0)
    fireswarm_heartbeat--;

  FireSwarmData.FlyState = AP_PROT_FLY_STATE_FLYING;
  int gps_quality = 255 - (gps.pacc-200) / 20;
  if (gps_quality < 0) gps_quality = 0;
  if (gps_quality > 255) gps_quality = 255;

  fireswarm_status_led(ahrs.status == AHRS_RUNNING, gps.fix == GPS_FIX_3D, fireswarm_heartbeat > 0);

  FireSwarmHeader.MsgType = AP_PROT_SENSORDATA;
  FireSwarmHeader.DataSize = sizeof(FireSwarmData);

  FireSwarmData.GPSState = gps_quality;
  FireSwarmData.BatteryLeft = 255;
  FireSwarmData.ServoState = AP_PROT_STATE_SERVO_PROP | AP_PROT_STATE_SERVO_WING_LEFT | AP_PROT_STATE_SERVO_WING_RIGHT | AP_PROT_STATE_SERVO_TAIL;
  FireSwarmData.AutoPilotState |= AP_PROT_STATE_AP_OUTER_LOOP | AP_PROT_STATE_AP_INNER_LOOP;
  FireSwarmData.SensorState = AP_PROT_STATE_SENSOR_COMPASS | AP_PROT_STATE_SENSOR_ACCELERO | AP_PROT_STATE_SENSOR_GPS | AP_PROT_STATE_SENSOR_WIND | AP_PROT_STATE_SENSOR_PRESSURE;

  FireSwarmData.Position.X = stateGetPositionUtm_f()->east;
  FireSwarmData.Position.Y = stateGetPositionUtm_f()->north;
  FireSwarmData.Position.Z = stateGetPositionUtm_f()->alt;

  FireSwarmData.GroundSpeed = 0;
  FireSwarmData.VerticalSpeed = 0;
  FireSwarmData.Heading = att->psi;
  FireSwarmData.Yaw = att->psi;
  FireSwarmData.Pitch = att->theta;
  FireSwarmData.Roll = att->phi;
  FireSwarmData.WindHeading = 0;
  FireSwarmData.WindSpeed = 0;


  fireswarm_payload_link_start();
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmHeader, sizeof(FireSwarmHeader));
  fireswarm_payload_link_transmit((uint8_t*)&FireSwarmData, sizeof(FireSwarmData));
  fireswarm_payload_link_crc();

  //fprintf(stderr,"Bytes: %d \n", fireswarm_payload_link_has_data());
  //fprintf(stderr,".");
}


static uint8_t slowdown = 0;
bool_t fireswarm_periodic_nav_init(void)
{
  FireSwarmData.AutoPilotState |= AP_PROT_STATE_AP_TAKEOVER;
  slowdown = 36;

  return FALSE;
}


bool_t fireswarm_periodic_nav(void)
{

  // Stop listening to payload if to becomes quite
  if (fireswarm_heartbeat == 0)
    return FALSE;


    for (int i=0; i<FireSwarmWaypoints.NumWayPoints; i++)
    {
      //fprintf(stderr,"Bytes: %d \n", fireswarm_payload_link_has_data());
      if (FireSwarmWaypoints.WayPoints[i].WpType == AP_PROT_WP_LINE)
      {
        WaypointX(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Line.To.X - nav_utm_east0;
        WaypointY(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Line.To.Y - nav_utm_north0;
        WaypointAlt(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Line.To.Z;
        fprintf(stderr,"LINE %f %f ",FireSwarmWaypoints.WayPoints[i].Line.To.X,WaypointX(0));

//        DOWNLINK_SEND_CIRCLE(_trans, _dev, &nav_circle_x, &nav_circle_y, &nav_circle_radius);

//#define PERIODIC_SEND_SEGMENT(_trans, _dev) if (nav_in_segment) { DOWNLINK_SEND_SEGMENT(_trans, _dev, &nav_segment_x_1, &nav_segment_y_1, &nav_segment_x_2, &nav_segment_y_2); }
      }
      else if (FireSwarmWaypoints.WayPoints[i].WpType == AP_PROT_WP_ARC)
      {
        float arm = FireSwarmWaypoints.WayPoints[i].Arc.Radius;
        float ang = - FireSwarmWaypoints.WayPoints[i].Arc.AngleStart - FireSwarmWaypoints.WayPoints[i].Arc.AngleArc;
        float dn = cos(ang)*arm;
        float de = sin(ang)*arm;

        fprintf(stderr,"ARC ");
        WaypointX(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Arc.Center.X - nav_utm_east0 + de;
        WaypointY(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Arc.Center.Y - nav_utm_north0 + dn;
        WaypointAlt(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Arc.Center.Z;
      }
      else
      {
        fprintf(stderr,"CIRCLE ");
        WaypointX(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Circle.Center.X - nav_utm_east0;
        WaypointY(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Circle.Center.Y - nav_utm_north0;
        WaypointAlt(WP_FS1+i) = FireSwarmWaypoints.WayPoints[i].Circle.Center.Z;
      }
    }
    if (FireSwarmWaypoints.NumWayPoints > 0)
      fprintf(stderr,"\n");


  if (slowdown == 36)
  {
    slowdown = 0;

    FireSwarmHeader.MsgType = AP_PROT_WP_STATUS;
    FireSwarmHeader.DataSize = sizeof(FireSwarmStatus);

    FireSwarmStatus.NumWaypoints = 0;

    fireswarm_payload_link_start();
    fireswarm_payload_link_transmit((uint8_t*)&FireSwarmHeader, sizeof(FireSwarmHeader));
    fireswarm_payload_link_transmit((uint8_t*)&FireSwarmStatus, sizeof(FireSwarmStatus));
    fireswarm_payload_link_crc();
  }
  slowdown++;

  NavVerticalAltitudeMode(WaypointAlt(WP_FS3), 0);    // vertical mode (folow glideslope)
  NavVerticalAutoThrottleMode(0);     // throttle mode
  NavGotoWaypoint(WP_FS1);            // horizontal mode (stay on localiser)

  return TRUE;
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
      fireswarm_heartbeat = FIRESWARM_HEARTBEAT_DELAY;
      switch (fsw_msg.msg_id)
      {
      case AP_PROT_SET_MODE:
        memcpy(&FireSwarmMode, fsw_msg.msg_buf, sizeof(FireSwarmMode));
        break;
      case AP_PROT_SET_WAYPOINTS:
        memcpy(&FireSwarmWaypoints, fsw_msg.msg_buf, sizeof(FireSwarmWaypoints));
        break;
      case AP_PROT_SET_LAND:
        memcpy(&FireSwarmLanding, fsw_msg.msg_buf, sizeof(FireSwarmLanding));
        break;
      }
    }
  }
}


/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/stereocam/stereocam_range2roll.c"
 * @author C. DW
 * Stereocam Range 2 Pitch/Roll angles
 */

#include "modules/stereocam/stereocam_range2roll.h"

#include "firmwares/rotorcraft/navigation.h"

// Serial Port
#include "mcu_periph/uart.h"
PRINT_CONFIG_VAR(STEREO_UART)

// define coms link for stereocam
struct link_device *xdev = &((STEREO_UART).device);



// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "led.h"





// Module data
struct RangeStruct {
  uint8_t parser;
  uint8_t buff[8];
  uint8_t sensors[5];
  uint8_t left;
  uint8_t right;
  uint8_t front;
  uint8_t up;
  uint8_t timeout;
};

struct RangeStruct range_data;



static inline uint8_t val_from_hex(uint8_t h) {
/*  if (h >= 'A') {
    return h - 'A' + 10;
  }
  else
  {
*/    return h - '0';
//  }
}



static inline void range_parse(uint8_t c)
{
  if (c == 13)
  {
    if (range_data.parser >= 4) {
      // Decode
      range_data.left = val_from_hex(range_data.buff[0]);
      range_data.right = val_from_hex(range_data.buff[1]);
      range_data.front = val_from_hex(range_data.buff[2]);
      range_data.up = val_from_hex(range_data.buff[3]);

      range_data.sensors[0] = range_data.left;
      range_data.sensors[1] = range_data.right;
      range_data.sensors[2] = range_data.front;
      range_data.sensors[3] = range_data.up;

    }
    range_data.parser = 0;
  }
  else if (c == 10) {
    range_data.parser = 0;
  }
  else {
    range_data.buff[range_data.parser] = c;
    range_data.parser ++;
    if (range_data.parser > 6) {
      range_data.parser = 0;
    }
  }

  // Timeout counter
  range_data.timeout = 20;
}


void range2roll_init() {
  range_data.parser = 0;
}

void range2roll_periodic(void) {

  // Read Serial
  while (xdev->char_available(xdev->periph)) {
    range_parse(xdev->get_byte(xdev->periph));
  }

  // Handle Timeout
  if (range_data.timeout <= 0)
    return;
  range_data.timeout --;

  // Telemetry
  RunOnceEvery(25,   DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 4, range_data.sensors));

}

float trim_phi = 0;
float trim_theta = 0;

static inline void go(float roll, float pitch, float yaw, float height)
{
  nav_set_heading_rad(RadOfDeg(yaw));
  NavAttitude(RadOfDeg(roll+trim_phi));
  NavVerticalAutoThrottleMode(RadOfDeg(pitch+trim_theta));
  NavVerticalAltitudeMode(height, 0.);
}

void range2roll_flightplan(void) {
  float roll = 0;
  float pitch = -3;
  float lateral = range_data.left - range_data.right;
  roll = lateral * 0.5;
  pitch += range_data.front * 0.5;
  float yaw = 0;
  float height = 2.0;
  go(roll,pitch,yaw,height);
}





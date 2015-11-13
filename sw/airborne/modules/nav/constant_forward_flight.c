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
 * @file "modules/nav/constant_forward_flight.c"
 * @author C. DW
 * 
 */

#include "modules/nav/constant_forward_flight.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

#include "navigation.h"

#include "std.h"

// Serial Port
#include "mcu_periph/uart.h"
PRINT_CONFIG_VAR(STEREO_UART)

// define coms link for stereocam
#define STEREO_PORT   (&((STEREO_UART).device))
struct link_device *xdev = STEREO_PORT;

#define StereoGetch() STEREO_PORT ->get_byte(STEREO_PORT->periph)
#define StereoSend1(c) STEREO_PORT->put_byte(STEREO_PORT->periph, c)
#define StereoUartSend1(c) StereoSend1(c)
#define StereoSend(_dat,_len) { for (uint8_t i = 0; i< (_len); i++) StereoSend1(_dat[i]); };
#define StereoUartSetBaudrate(_b) uart_periph_set_baudrate(STEREO_PORT, _b);
#define StereoChAvailable()(xdev->char_available(xdev->periph))






// Downlink
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "subsystems/datalink/downlink.h"

#include "led.h"





// Module data
struct AvoidNavigationStruct {
  uint8_t mode; ///< 0 = straight, 1 =  right, 2 = left, ...
  uint8_t stereo_bin[8];
};

struct AvoidNavigationStruct avoid_navigation_data;




const int start[20] = {
    -15, -5, -5, -5, -3,
    -3,-3,-3,-3,-3,
    -3,-3,-3,-3,-3,
    -3,-3,-3,-3,-3
};

const int stop[20] = {
    20, 18, 16, 14, 2,
    0,0,0,0,0,
    0,0,0,0,0,
    0,0,0,0,0,
};


float trim_phi = -1.5;
float trim_theta = 4.5;

static void stereo_parse(uint8_t c);
static void stereo_parse(uint8_t c)
{
  // Protocol is one byte only: store last instance
  avoid_navigation_data.stereo_bin[0] = c;
}


void forward_flight_init(void) {
  // Do nothing
  avoid_navigation_data.mode = 0;

}

uint8_t wp_nr = 0;
int mod_state = 20;

// start, move, brake, turn
int mod_phase = 0;

/** FP functions */
bool_t mod_avoid_init(uint8_t _wp)
{
  //height = GetPosAlt();
  wp_nr = _wp;
  mod_state = 0;
  mod_phase = 0;
  return FALSE;
}

void go(float roll, float pitch, float yaw)
{
  nav_set_heading_rad(RadOfDeg(yaw));
  NavAttitude(RadOfDeg(roll+trim_phi));
  NavVerticalAutoThrottleMode(RadOfDeg(pitch+trim_theta));
  NavVerticalAltitudeMode(WaypointAlt(wp_nr), 0.);
}

void play(mod_state)
{
  mod_state++;
  if (mod_state >= 20)
  {
    mod_state = 0;
    mod_phase++;
  }
}

bool_t mod_avoid_run(void)
{
  if (mod_state >= 20)
  {
    return FALSE;
  }

  switch (mod_phase)
  {
  case 0:
    go(0,start[mod_state],180);
    break;
  case 1:
  case 2:
    go(0,-3,180);
    break;
  case 3:
    go(0,stop[mod_state],180);
    break;
  case 4:
  case 5:
    go(0,0,180);
    break;
  }
  mod_state++;
  if (mod_state >= 20)
  {
    mod_state = 0;
    mod_phase++;
    if (mod_phase >= 6)
      return FALSE;
  }

  return TRUE;
}


void forward_flight_periodic(void) {

  static float heading = 0;

  static int speed = 0;
  static int action = 0;

  //////////////////////////////////////////////
  // Read Serial
  while (StereoChAvailable()) {
    stereo_parse(StereoGetch());
  }

  /////////////////////////////////////////////
  // Downlink
  DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, avoid_navigation_data.stereo_bin);





  volatile bool_t once = TRUE;
  // Move waypoint with constant speed in current direction
  if (
        (avoid_navigation_data.stereo_bin[0] == 97) ||
        (avoid_navigation_data.stereo_bin[0] == 100)
      )
  {
    // FORWARD!!!
    action = 1;
    once = TRUE;
  }
  else if (avoid_navigation_data.stereo_bin[0] == 98)
  {
    // STOP!!!
    if (once)
    {
      action = 0;
      once = FALSE;
    }
  }
  else
  {
    once = TRUE;
  }

  if (avoid_navigation_data.stereo_bin[0] == 99)
  {
    /*
    // TURN!!!
      heading += 4;
      if (heading > 360) heading = 0;
      nav_set_heading_rad(RadOfDeg(heading));
      */
  }


#if STEREO_LED
  if (obstacle_detected) {
    LED_ON(STEREO_LED);
  } else {
    LED_OFF(STEREO_LED);
  }
#endif

}




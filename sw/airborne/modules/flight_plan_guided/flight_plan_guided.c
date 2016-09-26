/*
 * Copyright (C) 2016 - IMAV 2016
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */
/**
 * @file modules/computer_vision/flight_plan_guided.c
 * @author IMAV 2016
 */

#include "modules/flight_plan_guided/flight_plan_guided.h"
#include "subsystems/ins.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "modules/sonar/sonar_bebop.h"
#include "generated/flight_plan.h"
#include "autopilot.h"
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/marker/detector.h"


void flight_plan_guided_init(void) {} // Dummy


/* Kill throttle */
uint8_t KillEngines(void) {
    autopilot_set_motors_on(FALSE);

    return false;
}


/* Start throttle */
uint8_t StartEngines(void) {
    autopilot_set_motors_on(TRUE);

    return false;
}


/* Reset the altitude reference to the current GPS alt if GPS is used */
uint8_t ResetAlt(void) {if (autopilot_mode == AP_MODE_GUIDED) { ins_reset_altitude_ref(); } return false;}


bool TakeOff(float climb_rate) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    guidance_v_set_guided_vz(-climb_rate);

    return false;
}

bool WaitUntilAltitude(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (stateGetPositionEnu_f()->z < altitude) { return true; }

    return false;
}

bool RotateToHeading(float heading) {
    guidance_h_set_guided_heading(heading);

    return false;
}

uint8_t Hover(float altitude) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }
    // Horizontal velocities are set to zero
    guidance_h_set_guided_body_vel(0, 0);

    // Vertical velocity increases until certain altitude is reached
    guidance_v_set_guided_z(-altitude);

    return false;
}

/* Move forward */
uint8_t MoveForward(float vx) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(vx, 0);
    }
    return false;
}

/* Move Right */
uint8_t MoveRight(float vy) {
    if (autopilot_mode != AP_MODE_GUIDED) { return true; }

    if (autopilot_mode == AP_MODE_GUIDED) {
        guidance_h_set_guided_body_vel(0, vy);
    }
    return false;
}



bool Land(float end_altitude) {
    // return true if not completed

    //For bucket
//    guidance_v_set_guided_vz(0.2);
    //For landing pad
    guidance_v_set_guided_vz(1.5);
    guidance_h_set_guided_body_vel(0, 0);


    if (stateGetPositionEnu_f()->z > end_altitude) {
        return true;
    }

    return false;
}


void marker_detection_periodic(void) {

    if (marker1.detected) {
        guidance_h_set_guided_pos(marker1.geo_location.x, marker1.geo_location.y);
    }

}

static int BUCKET_HEADING_MARGIN = 60;
static int BUCKET_HEADING_RATE = 1;

bool bucket_heading_change(void) {
  guidance_h_set_guided_body_vel(0, 0);

  if (marker2.detected) {
    if (!marker2.processed) {
      // Marker detected and not processed
      marker2.processed = true;
      int relative_heading = marker2.pixel.y - 320;

      if (relative_heading > BUCKET_HEADING_MARGIN) {
        // Marker is to the right
        guidance_h_set_guided_heading_rate(BUCKET_HEADING_RATE);
      } else if (relative_heading < -BUCKET_HEADING_MARGIN) {
        // Marker is to the left
        guidance_h_set_guided_heading_rate(-BUCKET_HEADING_RATE);
      } else {
        // Marker is more or less centered
        guidance_h_set_guided_heading_rate(0);
        return false;
//            guidance_h_set_guided_body_vel(0.3, 0);
      }
    } else {
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    // Marker not detected
    guidance_h_set_guided_heading_rate(0);
  }

  return true;
}

static int BUCKET_POSITION_MARGIN = 30;
static int BUCKET_DRIFT_CORRECTION_RATE = 0.1;

bool bucket_approach(void) {
  if (marker2.detected) {
    if (!marker2.processed) {
      int relative_pos = marker2.pixel.y - 320;

      if (relative_pos > BUCKET_POSITION_MARGIN) {
        guidance_h_set_guided_body_vel(0, BUCKET_DRIFT_CORRECTION_RATE);
      } else if (relative_pos < -BUCKET_POSITION_MARGIN) {
        guidance_h_set_guided_body_vel(0, -BUCKET_DRIFT_CORRECTION_RATE);
      } else {
        guidance_h_set_guided_body_vel(0, 0);
      }
    } else {
      // Marker detected but already processed
      // ** just wait **
    }
  } else {
    // Marker not detected
    // TODO: go back to search mode
  }

  return true;
}

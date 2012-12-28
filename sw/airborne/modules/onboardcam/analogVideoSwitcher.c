/*
 * $Id: $
 *
 * Copyright (C) 2012 Dirk Dokter
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
 */

#include "onboardcam/analogVideoSwitcher.h"
#include "generated/airframe.h"
#include "generated/airframe.h"
#include "actuators.h"
#include "firmwares/rotorcraft/telemetry.h"
#include "firmwares/rotorcraft/commands.h"
uint8_t active_cam;
int32_t prev_rc_val;
int32_t advance_direction;

void analogVideoSwitcher_init(void) {
  active_cam = 1;
  advance_direction = 1;
  prev_rc_val = radio_control.values[ANALOG_VIDEO_SWITCHER_RC_CHANNEL];
}

void analogVideoSwitcher_periodic(void) {
  if ((radio_control.values[ANALOG_VIDEO_SWITCHER_RC_CHANNEL] - prev_rc_val) >  ANALOG_VIDEO_SWITCHER_RC_MINDIFF ||
      (radio_control.values[ANALOG_VIDEO_SWITCHER_RC_CHANNEL] - prev_rc_val) < -ANALOG_VIDEO_SWITCHER_RC_MINDIFF) {
    analogVideoSwitcher_advanceCam();
    analogVideoSwitcher_setCam();
  }
  prev_rc_val = radio_control.values[ANALOG_VIDEO_SWITCHER_RC_CHANNEL];
}

void analogVideoSwitcher_setCam(void) {
  active_cam = (active_cam < ANALOG_VIDEO_SWITCHER_NUM_CAMS) ? active_cam : (ANALOG_VIDEO_SWITCHER_NUM_CAMS-1);
  commands[COMMAND_AVS] = Chop((MIN_PPRZ + ANALOG_VIDEO_SWITCHER_CMD_STEP*active_cam),MIN_PPRZ,MAX_PPRZ);
}

void analogVideoSwitcher_setCamFromGCS(uint8_t cam_nr) {
  if (cam_nr < active_cam)
    advance_direction = -1;
  else if (cam_nr > active_cam)
      advance_direction = 1;

  active_cam = cam_nr;
  analogVideoSwitcher_setCam();
}

void analogVideoSwitcher_advanceCam(void) {
  if (active_cam == ANALOG_VIDEO_SWITCHER_NUM_CAMS-1) {
    active_cam--;
    advance_direction = -1;
  }
  else if (active_cam == 0) {
    active_cam++;
    advance_direction = 1;
  }
  else {
      active_cam += advance_direction;
  }
}

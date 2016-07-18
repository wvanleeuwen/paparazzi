/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/div_ctrl/div_ctrl.c"
 * @author Kirk Scheper
 *
 */

#include "modules/div_ctrl/div_ctrl.h"

#include "math/pprz_algebra_int.h"
#include "math/pprz_algebra_float.h"

//#include "computer_vision/opticflow_module.h"
#include "modules/stereocam/stereocam2state/stereocam2state.h"

#include "generated/flight_plan.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"

#include "subsystems/datalink/downlink.h"

#include "led.h"

#ifndef FOE_GAIN
#define FOE_GAIN 0.05
#endif

#ifndef FOE_CMD
#define FOE_CMD 0
#endif

static struct FloatVect2 FOE;       // focus of expansion measured from the center of the image
static struct FloatVect2 FOE_filtered;       // focus of expansion measured from the center of the image
static const uint8_t max_filter_points = 4;
static uint8_t filter_points = 0;
static struct FloatVect2 vel_sp;    // velocity set-point

float gain;                         // control gain for FOE controller
float foe_cmd;                      // reference FOE x-location command

void div_ctrl_init(void)
{
  FLOAT_VECT2_ZERO(FOE);
  FLOAT_VECT2_ZERO(FOE_filtered);

  vel_sp.x = 0.5; // fixed forward speed
  vel_sp.y = 0.;

  gain = FOE_GAIN;
  foe_cmd = FOE_CMD;
}

void div_ctrl_run(void)
{
/*  // FOE is x intercept of flow field, rotate camera to x positive right, y positive up
  FOE.x = -(float)opticflow_result.flow_x / (opticflow_result.divergence * (float)opticflow.img_gray.w *
          (float)opticflow.subpixel_factor);
  FOE.y = -(float)opticflow_result.flow_y / (opticflow_result.divergence * (float)opticflow.img_gray.h *
          (float)opticflow.subpixel_factor);

  vel_sp.y = gain * (foe_cmd - FOE.x);*/

  // FOE is x intercept of flow field, rotate camera to x positive right, y positive up
  if(stereo_motion.div.x != 0) {
    FOE.x = 64 - stereo_motion.flow.x / stereo_motion.div.x ; // 128/2
  } else {FOE.x = 64;}
  if(stereo_motion.div.y != 0) {
    FOE.y = 48 - stereo_motion.flow.y / stereo_motion.div.y; // 96/2
  } else {FOE.y = 48;}

  FOE_filtered.x *= filter_points;
  FOE_filtered.y *= filter_points++;

  FOE_filtered.x += FOE.x;
  FOE_filtered.y += FOE.y;

  FOE_filtered.x /= filter_points;
  FOE_filtered.y /= filter_points;

  if (filter_points > max_filter_points) {
    filter_points = max_filter_points;
  }

  vel_sp.y = gain * (foe_cmd - FOE_filtered.x);

  BoundAbs(vel_sp.y, 1);

  // set x,y velocity set-point with fixed alt
  if (stateGetSpeedEnu_f()->x > 0.3) {
    guidance_h_set_guided_body_vel(vel_sp.x, vel_sp.y);
  } else {
    guidance_h_set_guided_body_vel(vel_sp.x, 0.);
  }
  guidance_v_set_guided_z(-1.5);

  uint8_t tempx = (uint8_t)FOE.x;
  uint8_t tempy = (uint8_t)FOE.y;
  uint8_t tempx2 = (uint8_t)FOE_filtered.x;
  uint8_t tempy2 = (uint8_t)FOE_filtered.y;

  uint8_t msg[] = {tempx, tempy, tempx2, tempy2};
  DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 4, msg);
}

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

#include "pprz_algebra_int.h"
#include "pprz_algebra_float.h"

#include "opticflow_module.h"

#include "generated/flight_plan.h"
#include "guidance_h.h"
#include "guidance_v.h"

#include "downlink.h"

#ifndef FOE_GAIN
#define FOE_GAIN 0.1
#endif

#ifndef FOE_CMD
#define FOE_CMD 0
#endif

static struct FloatVect2 FOE;       // focus of expansion measured from the center of the image
static struct FloatVect2 vel_sp;    // velocity set-point

float gain;                         // control gain for FOE controller
float foe_cmd;                      // reference FOE x-location command

void div_ctrl_init(void) {
  FOE.x = 0;
  FOE.y = 0;

  vel_sp.x = 0.;
  vel_sp.y = 0.5;   // fixed foward speed

  gain = FOE_GAIN;
  foe_cmd = FOE_CMD;
}

void div_ctrl_run(void) {
  // FOE is x intercept of flow field
  FOE.x = -(float)opticflow_result.flow_der_x / (opticflow_result.divergence * opticflow.img_gray.w);
  FOE.y = -(float)opticflow_result.flow_der_y / (opticflow_result.divergence * opticflow.img_gray.h);

  vel_sp.x = gain*(foe_cmd - FOE.x);

  // set x,y velocity set-point with fixed alt
  guidance_h_set_guided_body_vel(vel_sp.x, vel_sp.y);
  guidance_v_set_guided_z(1.5);

  uint8_t msg[] = {(uint8_t)(FOE.x*opticflow.img_gray.w), (uint8_t)(FOE.y*opticflow.img_gray.h), vel_sp.x*100, vel_sp.y*100};
  DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 4, msg);
}

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
#include "modules/stereocam/stereocam.h"

#include "generated/flight_plan.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"
#include "autopilot.h"

#include "subsystems/datalink/downlink.h"

#include "led.h"

#ifndef FOE_GAIN
#define FOE_GAIN 0.05
#endif

#ifndef FOE_CMD
#define FOE_CMD 48
#endif

enum {
  WINDOW_TRACKING,
  WINDOW_FLY_THROUGH,
  WINDOW_FLY_OPTITRACK,
  WINDOW_STANDBY
};

#ifndef WINDOW_MODE
#define WINDOW_MODE WINDOW_TRACKING
#endif

static struct FloatVect2 FOE;       // focus of expansion measured from the center of the image
static struct FloatVect2 FOE_filtered;       // focus of expansion measured from the center of the image
static const uint8_t max_filter_points = 4;
static uint8_t filter_points = 0;
static struct FloatVect2 vel_sp;    // velocity set-point

float gain;                         // control gain for FOE controller
float foe_cmd;                      // reference FOE x-location command
uint8_t window_mode;

struct georeference_t {
    struct Int32Vect3 target_p;   ///< Target in pixels, with z being the focal length in pixels, in camera frame x=up,y=right,out
    struct Int32Vect3 target_rel;    ///< Relative position to target
    struct NedCoor_i target_abs;    ///< Absolute position to target NED frame
};
struct georeference_t geo;

void div_ctrl_init(void)
{
  FLOAT_VECT2_ZERO(FOE);
  FLOAT_VECT2_ZERO(FOE_filtered);

  vel_sp.x = 0;//0.2; // fixed forward speed
  vel_sp.y = 0.;

  gain = FOE_GAIN;
  foe_cmd = FOE_CMD;
  window_mode = WINDOW_MODE;
}


static void georeference_project_target(void)
{
  // set target pixel location from center of the image not top left (still with same signs)
  //scale position to later computations
  geo.target_p.x = ((int32_t)win_x - 64) << 4;
  geo.target_p.y = ((int32_t)win_y - 48) << 4;
  geo.target_p.z = 400;

  // target_l is now a scale-less [pix<<POS_FRAC] vector in LTP from the drone to the target
  // Divide by z-component to normalize the projection vector

  // Multiply with height above ground
  int32_t zb = POS_BFP_OF_REAL(win_dist)/100;
  geo.target_p.x *= zb;
  geo.target_p.y *= zb;

  // Divide by z-component
  geo.target_p.x /= geo.target_p.z;
  geo.target_p.y /= geo.target_p.z;
  geo.target_p.z = zb;

  // Rotate Camera <-> Body
  struct Int32Vect3 target_b;
  struct Int32RMat body_to_stereocam_i;
  RMAT_BFP_OF_REAL(body_to_stereocam_i, body_to_stereocam);
  int32_rmat_transp_vmult(&target_b, &body_to_stereocam_i, &geo.target_p);

  // Body <-> LTP
  int32_rmat_transp_vmult(&geo.target_rel, stateGetNedToBodyRMat_i(), &target_b);

  // NED
  VECT3_SUM(geo.target_abs, *stateGetPositionNed_i(), geo.target_rel);
}

uint32_t time;

void div_ctrl_run(void)
{
  if(autopilot_mode == AP_MODE_GUIDED) {
    switch (window_mode) {
      case WINDOW_FLY_THROUGH:
        /*  // FOE is x intercept of flow field, rotate camera to x positive right, y positive up
        FOE.x = -(float)opticflow_result.flow_x / (opticflow_result.divergence * (float)opticflow.img_gray.w *
                (float)opticflow.subpixel_factor);
        FOE.y = -(float)opticflow_result.flow_y / (opticflow_result.divergence * (float)opticflow.img_gray.h *
                (float)opticflow.subpixel_factor);

        vel_sp.y = gain * (foe_cmd - FOE.x);*/

        /*
        // FOE is x intercept of flow field, rotate camera to x positive right, y positive up
        // TODO find a nice min value for divergence
        if(abs(stereo_motion.div.x) >= 1) {
          FOE.x = 48 - stereo_motion.ventral_flow.x / stereo_motion.div.x; // 128/2
        } else {FOE.x = 48;}
        if(abs(stereo_motion.div.y) >= 1) {
          FOE.y = 64 - stereo_motion.ventral_flow.y / stereo_motion.div.y; // 96/2
        } else {FOE.y = 64;}

        FOE_filtered.x *= filter_points;
        FOE_filtered.y *= filter_points++;

        FOE_filtered.x += FOE.x;
        FOE_filtered.y += FOE.y;

        FOE_filtered.x /= filter_points;
        FOE_filtered.y /= filter_points;

        if (filter_points > max_filter_points) {
          filter_points = max_filter_points;
        }

        //tracked_x, tracked_y;

        vel_sp.y = gain * (foe_cmd - FOE_filtered.x);

        BoundAbs(vel_sp.y, 0.3);

        // set x,y velocity set-point with fixed alt
        if (stateGetHorizontalSpeedNorm_f() > 0.1) {
          guidance_h_set_guided_body_vel(vel_sp.x, vel_sp.y);
        } else {
          guidance_h_set_guided_body_vel(vel_sp.x, 0.);
        }

        uint8_t tempx = (uint8_t)FOE.x;
        uint8_t tempy = (uint8_t)FOE.y;
        uint8_t tempx2 = (uint8_t)FOE_filtered.x;
        uint8_t tempy2 = (uint8_t)FOE_filtered.y;*/

        if (sys_time.nb_sec > time + 5) {
          guidance_h_set_guided_body_vel(0., 0.);
          window_mode = WINDOW_STANDBY;
        }
        break;
      case WINDOW_FLY_OPTITRACK:    // fly to a geolocation using optitrack
        if ( win_cert < 50  && win_size > 40) {   // if window in sight and sure its not noise (noise usually small window)
          georeference_project_target();
          guidance_h_set_guided_pos(POS_FLOAT_OF_BFP(geo.target_abs.x), POS_FLOAT_OF_BFP(geo.target_abs.y));
          window_mode = WINDOW_STANDBY;
        }
        break;
      case WINDOW_TRACKING:   // actuate to place window near window center
        if ( win_cert < 50 ) {  // threshold for positive window detection, lower is more likely window
          if (win_x > 70) {   // place window near center of frame
            guidance_h_set_guided_body_vel(0., 0.2);
          } else if (win_x < 58) {
            guidance_h_set_guided_body_vel(0., -0.2);
          } else {
            guidance_h_set_guided_body_vel(0.5, 0.);
            if (win_size > 40) {  // when window is large enough in frame fly thrugh
              window_mode = WINDOW_FLY_OPTITRACK;
              /*
               window_mode = WINDOW_FLY_THROUGH;
               time = sys_time.nb_sec;
               */
            }
          }
        }
        break;
      case WINDOW_STANDBY:  // wait for user instruction
        break;
      default:
        break;
    }
  }
}

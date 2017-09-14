/*
 * Copyright (C) 2015 Kirk Scheper
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

/** @file modules/stereocam/stereocam.c
 *  @brief interface to TU Delft serial stereocam
 *  Include stereocam.xml to your airframe file.
 *  Parameters STEREO_PORT, STEREO_BAUD, SEND_STEREO should be configured with stereocam.xml.
 */

#include "modules/stereocam/stereocam.h"

#include "mcu_periph/uart.h"
#include "subsystems/datalink/telemetry.h"
#include "pprzlink/messages.h"
#include "pprzlink/intermcu_msg.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "state.h"

#include "stereocam_follow_me/follow_me.h"
#include "modules/imav2017/imav2017.h"


// forward received image to ground station
#ifndef FORWARD_IMAGE_DATA
#define FORWARD_IMAGE_DATA FALSE
#endif

#ifndef STEREO_ARRAY_DATA
#define STEREO_ARRAY_DATA TRUE
#endif


/* This defines the location of the stereocamera with respect to the body fixed coordinates.
 *
 *    Coordinate system stereocam (image coordinates)
 *    z      x
 * (( * ))----->
 *    |                       * = arrow pointed into the frame away from you
 *    | y
 *    V
 *
 * The conversion order in euler angles is psi (yaw) -> theta (pitch) -> phi (roll)
 *
 * Standard rotations: MAV NED body to stereocam in Deg:
 * - facing forward:   90 -> 0 -> 90
 * - facing backward: -90 -> 0 -> 90
 * - facing downward:  90 -> 0 -> 0
 */

// general stereocam definitions
#if !defined(STEREO_BODY_TO_STEREO_PHI) || !defined(STEREO_BODY_TO_STEREO_THETA) || !defined(STEREO_BODY_TO_STEREO_PSI)
#warning "STEREO_BODY_TO_STEREO_XXX not defined. Using default Euler rotation angles (0,0,0)"
#endif

#ifndef STEREO_BODY_TO_STEREO_PHI
#define STEREO_BODY_TO_STEREO_PHI 0
#endif

#ifndef STEREO_BODY_TO_STEREO_THETA
#define STEREO_BODY_TO_STEREO_THETA 0
#endif

#ifndef STEREO_BODY_TO_STEREO_PSI
#define STEREO_BODY_TO_STEREO_PSI 0
#endif

struct stereocam_t stereocam = {
  .device = (&((UART_LINK).device)),
  .msg_available = false
};
static uint8_t stereocam_msg_buf[256]  __attribute__((aligned));   ///< The message buffer for the stereocamera

// incoming messages definitions
#ifndef STEREOCAM2STATE_SENDER_ID
#define STEREOCAM2STATE_SENDER_ID ABI_BROADCAST
#endif

#ifndef STEREOCAM_USE_MEDIAN_FILTER
#define STEREOCAM_USE_MEDIAN_FILTER 1
#endif

#include "filters/median_filter.h"
struct MedianFilter3Float medianfilter_vel;
struct MedianFilterFloat medianfilter_noise;

void stereocam_init(void)
{
  struct FloatEulers euler;
  if (STEREO_BODY_TO_STEREO_PHI > 4)
  {
    euler.phi = RadOfDeg(STEREO_BODY_TO_STEREO_PHI);
    euler.theta = RadOfDeg(STEREO_BODY_TO_STEREO_THETA);
    euler.psi = RadOfDeg(STEREO_BODY_TO_STEREO_PSI);
  } else {
    euler.phi = STEREO_BODY_TO_STEREO_PHI;
    euler.theta = STEREO_BODY_TO_STEREO_THETA;
    euler.psi = STEREO_BODY_TO_STEREO_PSI;
  }

  float_rmat_of_eulers(&stereocam.body_to_cam, &euler);

  // Initialize transport protocol
  pprz_transport_init(&stereocam.transport);

  InitMedianFilterVect3Float(medianfilter_vel, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&medianfilter_noise, MEDIAN_DEFAULT_SIZE);
}

void stereocam_parse_vel(struct FloatVect3 camera_vel, float R2)
{
  uint32_t now_ts = get_sys_time_usec();
  float noise = 0.5*(1.1 - R2);
  //float noise = 1.5*(1.1 - R2);

  // Rotate camera frame to body frame
  static struct FloatVect3 body_vel;
  float_rmat_transp_vmult(&body_vel, &stereocam.body_to_cam, &camera_vel);

  //todo make setting
  if (STEREOCAM_USE_MEDIAN_FILTER) {
    // Use a slight median filter to filter out the large outliers before sending it to state
    UpdateMedianFilterVect3Float(medianfilter_vel, body_vel);
    update_median_filter_f(&medianfilter_noise, noise);
  }

  //DOWNLINK_SEND_IMU_MAG(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &body_vel.x, &body_vel.y, &body_vel.z);

  if(stateGetPositionEnu_f()->z > 0.4 && body_vel.x < 2.f && body_vel.y < 2.f)
  {
    //Send velocities to state
    AbiSendMsgVELOCITY_ESTIMATE(STEREOCAM2STATE_SENDER_ID, now_ts,
                                body_vel.x,
                                body_vel.y,
                                body_vel.z,
                                noise
                               );
  }

  // todo activate this after changing optical flow message to be dimentionless instead of in pixels
  /*
  static struct FloatVect3 camera_flow;

  float avg_dist = (float)DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf)/res;

  camera_flow.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);
  camera_flow.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);
  camera_flow.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/DL_STEREOCAM_VELOCITY_avg_dist(stereocam_msg_buf);

  struct FloatVect3 body_flow;
  float_rmat_transp_vmult(&body_flow, &body_to_stereocam, &camera_flow);

  AbiSendMsgOPTICAL_FLOW(STEREOCAM2STATE_SENDER_ID, now_ts,
                              body_flow.x,
                              body_flow.y,
                              body_flow.z,
                              quality,
                              body_flow.z,
                              avg_dist
                             );
  */
}

/* Parse the InterMCU message */
static void stereocam_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the mag-pitot message */
  uint8_t msg_id = stereocam_msg_buf[1];
  switch (msg_id) {

  case DL_STEREOCAM_VELOCITY: {
    static struct FloatVect3 camera_vel;

    float res = (float)DL_STEREOCAM_VELOCITY_resolution(stereocam_msg_buf);

    camera_vel.x = (float)DL_STEREOCAM_VELOCITY_velx(stereocam_msg_buf)/res;
    camera_vel.y = (float)DL_STEREOCAM_VELOCITY_vely(stereocam_msg_buf)/res;
    camera_vel.z = (float)DL_STEREOCAM_VELOCITY_velz(stereocam_msg_buf)/res;

    float R2 = (float)DL_STEREOCAM_VELOCITY_vRMS(stereocam_msg_buf)/res;

    stereocam_parse_vel(camera_vel, R2);
    break;
  }

  case DL_STEREOCAM_ARRAY: {
	    uint8_t type = DL_STEREOCAM_ARRAY_type(stereocam_msg_buf);
	    uint8_t w = DL_STEREOCAM_ARRAY_width(stereocam_msg_buf);
	    uint8_t h = DL_STEREOCAM_ARRAY_height(stereocam_msg_buf);
	    uint8_t nb = DL_STEREOCAM_ARRAY_package_nb(stereocam_msg_buf);
	    uint8_t l = DL_STEREOCAM_ARRAY_image_data_length(stereocam_msg_buf);
#if FORWARD_IMAGE_DATA
    // forward image to ground station
    DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &type, &w, &h, &nb,
        l, DL_STEREOCAM_ARRAY_image_data(stereocam_msg_buf));
#endif
#if   STEREO_ARRAY_DATA
    uint8_t stereo_distance_filtered[128] ={0};
    //memset(stereo_distance_filtered, 0, width*sizeof(uint8_t) );
    uint8_t closest_average_distance = 255;
    uint8_t pixel_location_of_closest_object = 0;

    //TODO: do this for each stereo image column
    imav2017_histogram_obstacle_detection(DL_STEREOCAM_ARRAY_image_data(stereocam_msg_buf), stereo_distance_filtered,
    		&closest_average_distance, &pixel_location_of_closest_object, 128);

    /*DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &type, &w, &h, &nb,
         100, DL_STEREOCAM_ARRAY_image_data(stereocam_msg_buf));*/


    //TODO: automatically get FOV
    float pxtorad=(float)RadOfDeg(59) / 128;
    float heading = (float)(pixel_location_of_closest_object)*pxtorad;
    float distance = (float)(closest_average_distance)/100;

    DOWNLINK_SEND_SETTINGS(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &distance, &heading);


    AbiSendMsgOBSTACLE_DETECTION(AGL_RANGE_SENSORS_GAZEBO_ID, distance, heading);

#endif
    break;
  }

#ifdef STEREOCAM_FOLLOWME
  // todo is follow me still used?
  case DL_STEREOCAM_FOLLOW_ME: {
    follow_me( DL_STEREOCAM_FOLLOW_ME_headingToFollow(stereocam_msg_buf),
               DL_STEREOCAM_FOLLOW_ME_heightObject(stereocam_msg_buf),
               DL_STEREOCAM_FOLLOW_ME_distanceToObject(stereocam_msg_buf));
    break;
  }
#endif

  case DL_STEREOCAM_GATE: {
    uint8_t q = DL_STEREOCAM_GATE_quality(stereocam_msg_buf);
    float w = DL_STEREOCAM_GATE_width(stereocam_msg_buf);
    float h = DL_STEREOCAM_GATE_hieght(stereocam_msg_buf);
    float d = DL_STEREOCAM_GATE_depth(stereocam_msg_buf);

    // rotate angles to body frame
    static struct FloatEulers camera_bearing;
    camera_bearing.phi = DL_STEREOCAM_GATE_phi(stereocam_msg_buf);
    camera_bearing.theta = DL_STEREOCAM_GATE_theta(stereocam_msg_buf);
    camera_bearing.psi = 0;

    static struct FloatEulers body_bearing;
    float_rmat_transp_mult(&body_bearing, &stereocam.body_to_cam, &camera_bearing);

    uint8_t gate_detected=0;
    if(q>15)
    	gate_detected = 1;
    imav2017_set_gate(q, w, h, body_bearing.psi, body_bearing.theta, d,gate_detected);
    break;
  }

    default:
      break;
  }
}

/* We need to wait for incoming messages */
void stereocam_event(void) {
  // Check if we got some message from the stereocamera
  pprz_check_and_parse(stereocam.device, &stereocam.transport, stereocam_msg_buf, &stereocam.msg_available);

  // If we have a message we should parse it
  if (stereocam.msg_available) {
    stereocam_parse_msg();
    stereocam.msg_available = false;
  }
}

/* Send state to camera to facilitate derotation
 *
 */
void state2stereocam(void)
{
  // rotate body angles to camera reference frame
  static struct FloatEulers cam_angles;
  float_rmat_mult(&cam_angles, &stereocam.body_to_cam, stateGetNedToBodyEulers_f());

  float agl = 0;//stateGetAgl);
  pprz_msg_send_STEREOCAM_STATE(&(stereocam.transport.trans_tx), stereocam.device,
      AC_ID, &(cam_angles.phi), &(cam_angles.theta), &(cam_angles.psi), &agl);
}

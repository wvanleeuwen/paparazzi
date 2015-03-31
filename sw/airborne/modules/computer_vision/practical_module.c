/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * along with Paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/**
 * @file modules/computer_vision/practical_module.c
 * @brief Module for the Practical assignment
 *
 * Avoiding obstacles in the arena
 */


#include "practical_module.h"

#include <stdio.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>
#include "state.h"
#include "subsystems/abi.h"
#include "stabilization_practical.h"
#include "subsystems/datalink/downlink.h"

#include "lib/v4l/v4l2.h"
#include "lib/encoding/jpeg.h"
#include "lib/encoding/rtp.h"

/* default sonar/agl to use in practical visual_estimator */
#ifndef PRACTICAL_AGL_ID
#define PRACTICAL_AGL_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(PRACTICAL_AGL_ID);

/* The main variables */
struct practical_t practical;
static struct v4l2_device *practical_video_dev;     //< The main video device
static abi_event practical_agl_ev;                  //< The altitude ABI event
static pthread_t practical_calc_thread;             //< The practical calculation thread

/* Static functions */
static void *practical_module_calc(void *data);                   //< The main calculation thread
static void practical_agl_cb(uint8_t sender_id, float distance);  //< Callback function of the ground altitude
static void practical_tx_img(struct image_t *img, bool_t rtp);  //< Trnsmit an image

static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size, struct image_t* int_y, struct image_t* int_u, struct image_t* int_v);

uint8_t point_in_sector(struct image_t *img, struct point_t point);
uint16_t num_features_in_sector[4] = {0, 0, 0, 0};

uint32_t last_second;
static uint32_t counter;

/**
 * Initialize the practical module
 */
void practical_module_init(void)
{
  last_second = get_sys_time_msec();
  counter = 0;

  // Subscribe to the altitude above ground level ABI messages
  AbiBindMsgAGL(PRACTICAL_AGL_ID, &practical_agl_ev, practical_agl_cb);

  /* Default settings */
#ifdef PRACTICAL_Y_m
  practical.y_m = PRACTICAL_Y_m;
#endif
#ifdef PRACTICAL_Y_M
  practical.y_M = PRACTICAL_Y_M;
#endif
#ifdef PRACTICAL_U_m
  practical.u_m = PRACTICAL_U_m;
#endif
#ifdef PRACTICAL_U_M
  practical.u_M = PRACTICAL_U_M;
#endif
#ifdef PRACTICAL_V_m
  practical.v_m = PRACTICAL_V_m;
#endif
#ifdef PRACTICAL_V_M
  practical.v_M = PRACTICAL_V_M;
#endif

  /* Try to initialize the video device */
  practical_video_dev = v4l2_init("/dev/video1", 1280, 720, 10); //TODO: Fix defines
  if (practical_video_dev == NULL) {
    printf("[practical_module] Could not initialize the video device\n");
  }
}

/**
 * Main practical run
 */
void practical_module_run(void)
{

}

/**
 * Start the practical calculation
 */
void practical_module_start(void)
{
  // Check if we are not already running
  if (practical_calc_thread != 0) {
    printf("[practical_module] Calculation of practical already started!\n");
    return;
  }

  // Create the practical calculation thread
  int rc = pthread_create(&practical_calc_thread, NULL, practical_module_calc, NULL);
  if (rc) {
    printf("[practical_module] Could not initialize calculation thread (return code: %d)\n", rc);
  }
}

/**
 * Stop the optical flow calculation
 */
void practical_module_stop(void)
{
  // Stop the capturing
  v4l2_stop_capture(practical_video_dev);

  // TODO: fix thread stop
}



/**
 * Do the main calculation
 */
static void *practical_module_calc(void *data __attribute__((unused)))
{
  // Start the streaming on the V4L2 device
  if (!v4l2_start_capture(practical_video_dev)) {
    printf("[practical_module] Could not start capture of the camera\n");
    return 0;
  }

#if PRACTICAL_DEBUG
  // Create a new JPEG image
  struct image_t img_jpeg, img_small;
  image_create(&img_jpeg, practical_video_dev->w, practical_video_dev->h, IMAGE_JPEG);
  image_create(&img_small,
    practical_video_dev->w/4,
    practical_video_dev->h/4,
    IMAGE_YUV422);
#endif

  struct image_t int_y, int_u, int_v;
  struct image_t img;

  uint8_t first_time = 1;
  uint16_t img_height = 200;

  /* Main loop of the optical flow calculation */
  while (TRUE) {
<<<<<<< Updated upstream
    // Try to fetch an image    
=======
    counter++;
    printf("%d, %d\n", get_sys_time_msec(), last_second);
    if ((get_sys_time_msec()-last_second) > 1000) {
      printf("Count: %d\n", counter);
      counter = 0;
      last_second = get_sys_time_msec();
    }

    // Try to fetch an image
    struct image_t img;
>>>>>>> Stashed changes
    v4l2_image_get(practical_video_dev, &img);

    if(first_time){
      image_create(&int_y, img.w, img_height, IMAGE_INTEGRAL);
      image_create(&int_u, img.w/2, img_height, IMAGE_INTEGRAL);
      image_create(&int_v, img.w/2, img_height, IMAGE_INTEGRAL);
      first_time = 0;
    }

    // Calculate the colours in 2 bins (left/right)
    // uint32_t bins[2];
    // memset(bins, 0, sizeof(uint32_t) * 2);
    // image_yuv422_colorfilt(&img, &img_copy, bins, 2, practical.y_m, practical.y_M, practical.u_m, practical.u_M, practical.v_m, practical.v_M);
    // RunOnceEvery(10, printf("Bins: %d\t%d\n", bins[0], bins[1]));

    // // Update the heading
    // if(bins[0] > 50000 || bins[1] > 50000) {
    //   if(bins[1] < bins[0]) {
    //     // Turn left
    //     printf("Turn left.. %d\t%d\n", bins[0], bins[1]);
    //     stabilization_practical_turn(-1);
    //   }
    //   else {
    //     // Turn left
    //     printf("Turn right.. %d\t%d\n", bins[0], bins[1]);
    //     stabilization_practical_turn(1);
    //   }
    // } else {
    //   stabilization_practical_turn(0);
    // }

    // window_h = f(height,pitch, target obstacle avoidacne distance)
    practical_integral_img_detect(&img, img_height /*window_h*/, 25 /*box size*/, &int_y, &int_u, &int_v);

#if PRACTICAL_DEBUG
    //RunOnceEvery(10, {
    image_yuv422_downsample(&img, &img_small, 4);
    jpeg_encode_image(&img_small, &img_jpeg, 60, FALSE);
    practical_tx_img(&img_jpeg, FALSE);
    //});
#endif

    
  }

  // Free the image
  v4l2_image_free(practical_video_dev, &img);

  image_free(&int_y);
  image_free(&int_u);
  image_free(&int_v);

#if PRACTICAL_DEBUG
  image_free(&img_jpeg);
  image_free(&img_small);
#endif
}

/**
 * Get the altitude above ground of the drone
 * @param[in] sender_id The id that send the ABI message (unused)
 * @param[in] distance The distance above ground level in meters
 */
static void practical_agl_cb(uint8_t sender_id __attribute__((unused)), float distance)
{
  // Update the distance if we got a valid measurement
  if (distance > 0) {
    //opticflow_state.agl = distance; TODO
  }
}

/**
 * Transmit a JPEG image trough RTP or netcat
 * @param[in] *img The image to transmit
 * @param[in] use_netcat If we want to transmit over use netcat or RTP
 */
#if PRACTICAL_DEBUG
static void practical_tx_img(struct image_t *img, bool_t use_netcat)
{
  if (!use_netcat) {
    rtp_frame_send(
      &PRACTICAL_UDP_DEV,       // UDP device
      img,
      0,                        // Format 422
      20,                       // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );
    return;
  }

  // Open process to send using netcat (in a fork because sometimes kills itself???)
  pid_t pid = fork();

  if (pid < 0) {
    printf("[practical_module] Could not create netcat fork.\n");
  } else if (pid == 0) {
    // We are the child and want to send the image
    FILE *netcat = popen("nc 192.168.1.3 5000 2>/dev/null", "w");
    if (netcat != NULL) {
      fwrite(img->buf, sizeof(uint8_t), img->buf_size, netcat);
      pclose(netcat); // Ignore output, because it is too much when not connected
    } else {
      printf("[practical_module] Failed to open netcat process.\n");
    }

    // Exit the program since we don't want to continue after transmitting
    exit(0);
  } else {
    // We want to wait until the child is finished
    wait(NULL);
  }
}
#endif

/**
 * detect object based on color difference with floor
 * @param[in] *img The image to transmit
 * @param[in] sub_img_h The height of the bottom of the frame to search
 * @param[in] feature_size The bin size to average for object detection
 */
static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size, struct image_t* int_y, struct image_t* int_u, struct image_t* int_v)
{
  // note numbering of channels 0,1,2 -> Y,U,V
  // TODO: optimise computation of median and integral images in image.c
  // TODO: Remove placing point in image in final version, maybe place in a debug define...
  // TODO: Optimse thresholds for each channel (a bit)
  // TODO: Don't predefine intergral image? Done now to not initialise the memory every loop, maybe can be optimsed


  // Calculate the median values
  uint16_t sub_img_start = img->buf_size - img->w * 2 * sub_img_h;     // location of the start of sub image
  // uint8_t median_y = median(img, 0, sub_img_start, sub_img_h);
  uint8_t median_u = median(img, 1, sub_img_start, sub_img_h);
  uint8_t median_v = median(img, 2, sub_img_start, sub_img_h);
  //printf("Median values: %d %d %d\n", median_y, median_u, median_v);

  // Get the integral image
  struct point_t start_point = {
    .x = 0,
    .y = img->h - sub_img_h
  };
  image_get_integral(img, int_y, int_u, int_v, &start_point);

  // Show boxes above thresholds
  struct point_t from, to;
  struct point_t midpoint_feature;
  int16_t diff_u, diff_v, y_value;
  uint8_t  sector;
  uint16_t feature_s2 = feature_size * feature_size;
  num_features_in_sector[0] = num_features_in_sector[1]  = num_features_in_sector[2] = num_features_in_sector[3] = 0;
  for (uint16_t x = 0; x <= img->w - feature_size; x += feature_size) {
    for (uint16_t y = 0; y <= sub_img_h - feature_size; y += feature_size) {
      // Set the from and to
      from.x = x;
      from.y = y;
      to.x = x + feature_size - 1;
      to.y = y + feature_size - 1;

//      int16_t diff_y = image_get_integral_sum(&int_y, &from, &to) / feature_s2 - median_y;
      y_value = image_get_integral_sum(int_y, &from, &to) / feature_s2;
      //int16_t diff_y = y_value - median_y;

      // Update the x for the U and V values (since we have 2 times less pixels)
      from.x /= 2;
      to.x /= 2;

      diff_u = 2*image_get_integral_sum(int_u, &from, &to) / feature_s2 - median_u;
      diff_v = 2*image_get_integral_sum(int_v, &from, &to) / feature_s2 - median_v;

      //printf("Point(%d, %d): %dY %dU %dV\n", x, y, avg_y - median_y, avg_u - median_u, avg_v - median_v);

      midpoint_feature.x = x + start_point.x + feature_size/2;
      midpoint_feature.y = y + start_point.y + feature_size/2;

      sector = point_in_sector(img, midpoint_feature);
#if PRACTICAL_DEBUG
      // Show points
      from.x = x + start_point.x;
      from.y = y + start_point.y;
      to.x = from.x + feature_size;
      to.y = from.y + feature_size;

      if((practical.y_m < y_value) && (y_value < practical.y_M) && (sector != 0)) {
        image_draw_line(img, &from, &to);
      }

      from.x = x + start_point.x + feature_size;
      from.y = y + start_point.y;
      to.x = x + start_point.x;
      to.y = y + start_point.y + feature_size;
      if((practical.u_m > diff_u || diff_u > practical.u_M) && sector != 0) {
        image_draw_line(img, &from, &to);
      }
      if((practical.v_m > diff_v || diff_v > practical.v_M) && sector != 0) {
        image_draw_line(img, &from, &to);
      }
#endif

      // compute number of features per sector
      if((practical.y_m < y_value && y_value < practical.y_M) || (practical.u_m > diff_u || diff_u > practical.u_M) || (practical.v_m > diff_v || diff_v > practical.v_M)) {
        num_features_in_sector[sector] += 1;
      }

// Display sector
//       if(sector == 2) {
//         image_draw_line(img, &from, &to);
//       }
    }
  }

  //do avoidance
  if(num_features_in_sector[2] == 0) {
    //go straight
    yaw_rate = 0;
  }
  else {
    //fly with zero velocity
    //turn
//     if(num_features_in_sector[1] > num_features_in_sector[3])
//       yaw_rate = 2;//turn right
//     else
//       yaw_rate = -2;
    //always turn right
    yaw_rate = 2;//turn right
  }

  DOWNLINK_SEND_OPTIC_AVOID(DefaultChannel, DefaultDevice,
                            &num_features_in_sector[0],
                            &num_features_in_sector[1],
                            &num_features_in_sector[2],
                            &num_features_in_sector[3]);

}

uint8_t point_in_sector(struct image_t *img, struct point_t point) {

  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();
  uint8_t sector = 0;
  float path_angle = 0.8;//0.6981;
  struct point_t center_point;

  center_point.x = img->w/2;
  center_point.y = img->h/2 + img->h*eulers->theta*1.5; // the '1.5' is the conversion factor

  // look at a fourth under the center point
  float y_line_at_point = center_point.y + img->h/4 + eulers->phi*(img->w/2 - point.x);

  if(point.y > y_line_at_point) {
    float x_line_left_at_point = img->w/2 - tan(path_angle - eulers->phi) * (point.y - center_point.y);
    float x_line_right_at_point = img->w/2 + tan(path_angle + eulers->phi) * (point.y - center_point.y);
    if(point.x < x_line_left_at_point)
      sector = 1;
    else if(point.x < x_line_right_at_point)
      sector = 2;
    else
      sector = 3;
  }

  return sector;
}

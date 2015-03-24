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
#include "opticflow/stabilization_opticflow.h"

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

static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size);

/**
 * Initialize the practical module
 */
void practical_module_init(void)
{
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

#ifdef PRACTICAL_DEBUG
  // Create a new JPEG image
  struct image_t img_jpeg, img_small;
  image_create(&img_jpeg, practical_video_dev->w, practical_video_dev->h, IMAGE_JPEG);
  image_create(&img_small,
    practical_video_dev->w/4,
    practical_video_dev->h/4,
    IMAGE_YUV422);
#endif

  /* Main loop of the optical flow calculation */
  while (TRUE) {
    // Try to fetch an image
    struct image_t img;
    v4l2_image_get(practical_video_dev, &img);

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
    practical_integral_img_detect(&img, 200 /*window_h*/, 100 /*box size*/);

#ifdef PRACTICAL_DEBUG
    //RunOnceEvery(10, {
    image_yuv422_downsample(&img, &img_small, 4);
    jpeg_encode_image(&img_small, &img_jpeg, 60, FALSE);
    practical_tx_img(&img_jpeg, FALSE);
    //});
#endif

    // Free the image
    v4l2_image_free(practical_video_dev, &img);
  }

#ifdef PRACTICAL_DEBUG
  image_free(&img_jpeg);
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
static void practical_tx_img(struct image_t *img, bool_t use_netcat)
{
  if (!use_netcat) {
    rtp_frame_send(
      &PRACTICAL_UDP_DEV,       // UDP device
      img,
      0,                        // Format 422
      60,                       // Jpeg-Quality
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
    FILE *netcat = popen("nc 192.168.1.2 5000 2>/dev/null", "w");
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

/**
 * detect object based on color difference with floor
 * @param[in] *img The image to transmit
 * @param[in] sub_img_h The height of the bottom of the frame to search
 * @param[in] feature_size The bin size to average for object detection
 */
uint32_t integral_image0[460800] = {0}; // max size = 1280x720/2 looking at every UYVY pair
uint32_t integral_image1[460800] = {0};
uint32_t integral_image2[460800] = {0};
static void practical_integral_img_detect(struct image_t *img, uint16_t sub_img_h, uint16_t feature_size)
{
  // note numbering of channels 0,1,2 -> Y,U,V
  // TODO: optimise computation of median and integral images in image.c
  // TODO: Remove placing point in image in final version, maybe place in a debug define...
  // TODO: Optimse thresholds for each channel (a bit)
  // TODO: Don't predefine intergral image? Done now to not initialise the memory every loop, maybe can be optimsed

  uint8_t *img_buf = (uint8_t *)img->buf;
  const uint8_t intergral_img_w = img->w / 2;
  uint16_t sub_img_start = img->buf_size - img->w * 2 * sub_img_h; // location of the start of sub image

  uint8_t median0, median1, median2;
  int8_t response0, response1, response2;

  uint32_t px_inner = feature_size * feature_size;  // number of pixels in box

  median0 = median(img, 0, sub_img_start, sub_img_h);          // calculate median value in subimage Y channel
  median1 = median(img, 1, sub_img_start, sub_img_h);          // calculate median value in subimage U channel
  median2 = median(img, 2, sub_img_start, sub_img_h);          // calculate median value in subimage V channel

  get_integral_image(img, integral_image0, integral_image1, integral_image2, sub_img_start,
                     sub_img_h);  // calucalte the intergral image

  uint16_t x_response, y_response;
  uint8_t feature_size2 = feature_size / 2;
  for (y_response = 0; y_response < sub_img_h - feature_size; y_response += feature_size) {
    for (x_response = 0; x_response < intergral_img_w - feature_size; x_response += feature_size) {
      // NOTE: placing color in image not needed during real flight!
      response0 = get_obs_response(integral_image0, intergral_img_w, x_response, y_response, feature_size, px_inner,
                                   median0);
      if (response0 < -16) {
        img_buf[sub_img_start + intergral_img_w * 4 * (y_response + feature_size2) + (x_response + feature_size2) * 4 + 1] =
          255;
      }
      response1 = get_obs_response(integral_image1, intergral_img_w, x_response, y_response, feature_size, px_inner,
                                   median1);
      if (abs(response1) > 9) {
        img_buf[sub_img_start + intergral_img_w * 4 * (y_response + feature_size2) + (x_response + feature_size2) * 4] = 255;
      }
      response2 = get_obs_response(integral_image2, intergral_img_w, x_response, y_response, feature_size, px_inner,
                                   median2);
      if (abs(response2) > 11) {
        img_buf[sub_img_start + intergral_img_w * 4 * (y_response + feature_size2) + (x_response + feature_size2) * 4 + 2] =
          255;
      }

      // printf("found box %d %d %d %d %d\n", x_response, y_response, response, median_val, img_buf[sub_img_start + img->w*2*y_response + x_response*2 + 1] );

    }
  }
}

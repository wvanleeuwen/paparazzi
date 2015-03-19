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

static void practical_integral_img_detect(struct image_t *img);

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
  if(practical_calc_thread != 0) {
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
static void *practical_module_calc(void *data __attribute__((unused))) {
  // Start the streaming on the V4L2 device
  if(!v4l2_start_capture(practical_video_dev)) {
    printf("[practical_module] Could not start capture of the camera\n");
    return 0;
  }

#ifdef PRACTICAL_DEBUG
  // Create a new JPEG image
  struct image_t img_jpeg, img_copy;
  image_create(&img_copy, practical_video_dev->w, practical_video_dev->h, IMAGE_YUV422);
  image_create(&img_jpeg, practical_video_dev->w, practical_video_dev->h, IMAGE_JPEG);
#endif

  /* Main loop of the optical flow calculation */
  while(TRUE) {
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

    practical_integral_img_detect(&img);

#ifdef PRACTICAL_DEBUG
    //RunOnceEvery(30, {
      jpeg_encode_image(&img, &img_jpeg, 60, FALSE);
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
 * @param[in] rtp If we want to transmit over RTP, if false it uses netcat
 */
static void practical_tx_img(struct image_t *img, bool_t rtp)
{
  if(rtp) {
    rtp_frame_send(
      &PRACTICAL_UDP_DEV,       // UDP device
      img,
      0,                        // Format 422
      60,                       // Jpeg-Quality
      0,                        // DRI Header
      0                         // 90kHz time increment
    );
  }

  // Open process to send using netcat (in a fork because sometimes kills itself???)
  pid_t pid = fork();

  if(pid < 0) {
    printf("[practical_module] Could not create netcat fork.\n");
  }
  else if(pid ==0) {
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
  }
  else {
    // We want to wait until the child is finished
    wait(NULL);
  }
}

/**
 * detect object based on color difference with floor
 * @param[in] *img The image to transmit
 * @param[in] 
 */
uint32_t integral_image[921600] = {0};
static void practical_integral_img_detect(struct image_t *img)
{
	uint16_t sub_img_h = 300;
	uint16_t feature_size = 50;
	uint8_t median_val;
    // uint16_tborder = feature_size*2;

    uint32_t px_inner = feature_size * feature_size;
    //px_whole = (feature_size+2*border)*(feature_size);
    //px_border = px_whole - px_inner;
	    
	struct image_t sub_img;
	image_create(&sub_img, img->w, sub_img_h, IMAGE_YUV422);
  if(sub_img.buf == NULL)
    printf("Too much memory is used\n");
	// get subimage
  uint8_t *img_buf = (uint8_t *)img->buf;
	memcpy(sub_img.buf, &img_buf[img->buf_size - sub_img.buf_size], sub_img.buf_size);
	// calculate median value in subimage
    median_val = median(&sub_img);

    //printf("median! %d\n", median_val);

    get_integral_image( &sub_img, integral_image);

//    printf("whole_area = %d width: %d, height: %d\n", whole_area, integral_image.cols, integral_image.rows);

    uint16_t x_response, y_response, response;

    for (y_response = 0; y_response < sub_img.h - feature_size; y_response+=feature_size){
      for (x_response = 0; x_response < sub_img.w - feature_size; x_response+=feature_size){
        response = get_obs_response( integral_image, sub_img.w, x_response, y_response, feature_size, px_inner, median_val);
        if ( response > 12){
          printf("found box %d %d %d %d %d\n", x_response, y_response, response, median_val, img_buf[img->buf_size - sub_img.buf_size + sub_img.w*2*y_response + x_response + 1] );
        }
			img_buf[img->buf_size - sub_img.buf_size + img->w*2*(y_response + feature_size/2) + x_response + feature_size/2 ] = 255;
      }
    }
    image_free(&sub_img);
}

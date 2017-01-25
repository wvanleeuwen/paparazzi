/*
 * Copyright (C) Wilco Vlenterie
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
 * @file "modules/computer_vision/autoswarm//autoswarm.c"
 * @author Wilco Vlenterie
 * Autonomous bebop swarming module based on vision
 */

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/autoswarm/autoswarm.h"
#include "modules/computer_vision/autoswarm/autoswarm_opencv.h"

#ifndef AUTOSWARM_CAMERA
#define AUTOSWARM_CAMERA front_camera
#endif

#ifndef AUTOSWARM_GLOBAL_ATTRACTOR
#define AUTOSWARM_GLOBAL_ATTRACTOR AUTOSWARM_CIRCLE_CW
#endif

// Function
struct image_t* autoswarm_func(struct image_t* img);
struct image_t* autoswarm_func(struct image_t* img)
{
  if (img->type == IMAGE_YUV422)
  {
	  // Call OpenCV (C++ from paparazzi C function)
	  autoswarm_opencv_run();
  }
  return img;
}

void autoswarm_init(void)
{
	autoswarm_opencv_init(AUTOSWARM_GLOBAL_ATTRACTOR);
	cv_add_to_device(&AUTOSWARM_CAMERA, autoswarm_func);
}

// void swarm_init {}

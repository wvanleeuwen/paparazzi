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
 * @file "modules/computer_vision/wv_swarm/cv_swarm.c"
 * @author Wilco Vlenterie
 * @brief Swarming module for bebop
 *
 * Swarming with bebop for thesis
 */

#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/wv_swarm/cv_swarm.h"
#include "modules/computer_vision/wv_swarm/wv_swarm.h"

#if !(WV_INIT_GLOBAL_ATTRACTOR)
#define WV_INIT_GLOBAL_ATTRACTOR 1
#endif

bool WV_SWARM_RUNNING = false;

// Function
struct image_t* opencv_func(struct image_t* img);
struct image_t* opencv_func(struct image_t* img)
{
	if(!WV_SWARM_RUNNING)
	{
		WV_SWARM_RUNNING = true;
		// Call OpenCV (C++ from paparazzi C function)
		img->buf = wv_swarm_run((char*) img->buf, img->w, img->h);
		WV_SWARM_RUNNING = false;
	}else{
		printf("[SKIP] Not yet finished with previous frame, skipping this frame.\n");
	}
	return img;
}

void cv_swarm_init(void)
{
	wv_swarm_init(WV_INIT_GLOBAL_ATTRACTOR);
	cv_add(&opencv_func);
}

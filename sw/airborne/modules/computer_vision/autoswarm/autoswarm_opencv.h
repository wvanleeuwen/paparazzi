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
 * @file "modules/computer_vision/autoswarm//autoswarm.h"
 * @author Wilco Vlenterie
 * Autonomous bebop swarming module based on vision
 */

#ifndef OPENCV_EXAMPLE_H
#define OPENCV_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _trackResults {
    int     x_p;
    int     y_p;
    double  area_p;
    double 	x_c;
    double 	y_c;
    double 	r_c;
    double  x_b;
    double  y_b;
    double  z_b;
    double  x_w;
    double  y_w;
    double  z_w;
} trackResults;

typedef struct _memBlock {
	int lastSeen;
	int id;
	int     x_p;
	int     y_p;
	double x_w;
	double y_w;
	double z_w;
} memoryBlock;

extern double 	WV_GLOBAL_CIRCLE_R;
extern double 	WV_SWARM_SEPERATION;
extern int 		WV_GLOBAL_ATTRACTOR;
extern double 	WV_SWARM_AMAX;
extern double 	WV_SWARM_VMAX;
extern double	WV_SWARM_YAWRATEMAX;
extern double 	WV_SWARM_GLOBAL;
extern int 		WV_SWARM_MODE;
extern double 	WV_SWARM_E;
extern double 	WV_SWARM_EPS;
extern double  	WV_FILTER_CR;
extern double  	WV_FILTER_CG;
extern double  	WV_FILTER_CB;
extern int 		WV_TRACK_GREY_THRESHOLD;
extern int 		WV_TRACK_IMAGE_CROP_FOVY;
extern int 		WV_TRACK_RND_PIX_SAMPLE;
extern int 		WV_FILTER_Y_MIN;
extern int 		WV_FILTER_Y_MAX;
extern int 		WV_FILTER_CB_MIN;
extern int 		WV_FILTER_CB_MAX;
extern int 		WV_FILTER_CR_MIN;
extern int 		WV_FILTER_CR_MAX;
extern int		FILTER_SAMPLE_STYLE;
extern int 		FILTER_FLOOD_STYLE;

// Initialize global attractor
struct originPoint { double cx; double cy; double cz;};
struct originPoint globalOrigin;
static inline void setGlobalOrigin(double x, double y, double z){ globalOrigin.cx = x; globalOrigin.cy = y; globalOrigin.cz = z;};
static inline bool setGlobalMode(int mode){ WV_GLOBAL_ATTRACTOR = mode; return false; };
static inline bool setSwarmMode(int mode){ 	WV_SWARM_MODE = mode; return false; };

void autoswarm_opencv_init(int globalMode);
void autoswarm_opencv_run(char* img, int width, int height);
bool amIhome(void);

#ifdef __cplusplus
}
#endif

#endif

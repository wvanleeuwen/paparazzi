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

#ifndef AUTOSWARM_OPENCV_H
#define AUTOSWARM_OPENCV_H

#ifdef __cplusplus
extern "C" {
#endif

// Global options definitions
#define AUTOSWARM_POINT 			0
#define AUTOSWARM_BUCKET 			1
#define AUTOSWARM_CIRCLE_CW 		2
#define AUTOSWARM_CIRCLE_CC 		3
//// Filter sample styles
//#define FILTER_STYLE_FULL			0
//#define FILTER_STYLE_GRID			1
//#define FILTER_STYLE_RANDOM			2
//// Filter flood styles
//#define FILTER_FLOOD_OMNI 			0
//#define FILTER_FLOOD_CW				1

/*
#define WP__TD 2
#define WP__GOAL 4
#define WP__CAM 5
#define WP_GLOBAL 6
#define FP_BLOCKS { \
 "Wait GPS" , \
 "Geo init" , \
 "Holding point" , \
 "Start Engine" , \
 "Takeoff" , \
 "Standby" , \
 "Swarm" , \
 "Swarm Home" , \
 "Land here" , \
 "Land" , \
 "Flare" , \
 "Landed" , \
 "circle_cw" , \
 "circle_ccw" , \
 "bucket" , \
 "spread_out" , \
 "spread_in" , \
 "inc_circle" , \
 "dec_circle" , \
 "HOME" , \
}
*/

extern double 	AUTOSWARM_CIRCLE_R;
extern double 	AUTOSWARM_SEPERATION;
extern int 		AUTOSWARM_ATTRACTOR;
extern double 	AUTOSWARM_AMAX;
extern double 	AUTOSWARM_VMAX;
extern double	AUTOSWARM_YAWRATEMAX;
extern double 	AUTOSWARM_GLOBAL;
extern int 		AUTOSWARM_MODE;
extern double 	AUTOSWARM_E;
extern double 	AUTOSWARM_EPS;

// Initialize global attractor
struct originPoint { double cx; double cy; double cz;};
struct originPoint globalOrigin;
static inline void setGlobalOrigin  (double x, double y, double z){ globalOrigin.cx = x; globalOrigin.cy = y; globalOrigin.cz = z;};
static inline bool setGlobalMode    (int mode){ AUTOSWARM_ATTRACTOR = mode; return false; };
static inline bool setSwarmMode     (int mode){ 	AUTOSWARM_MODE = mode; return false; };

void autoswarm_opencv_init          (int globalMode);
void autoswarm_opencv_run           (void);
bool amIhome                        (void);

#ifdef __cplusplus
}
#endif

#endif

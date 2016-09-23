/*
 * Copyright (C) IMAV 2016
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
 * @file "modules/computer_vision/marker/detector.h"
 */

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

#include "math/pprz_geodetic_float.h"
#include "lib/vision/image.h"

struct Marker {
    volatile bool detected;
    struct point_t pixel;
    struct NedCoor_f geo_location;
    struct FloatVect3 geo_relative;
    float found_time;
    float mid;
    float mid_time;

};

extern struct Marker marker1;
extern struct Marker marker2;

void detector_init(void);

void detector_locate_bucket(void);
void detector_locate_helipad(void);

#endif

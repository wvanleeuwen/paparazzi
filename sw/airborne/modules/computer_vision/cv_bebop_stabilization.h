/*
 * Copyright (C) W Vlenterie
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
 * @file "modules/computer_vision/cv_bebop_stabilization.h"
 * @author W Vlenterie
 * Image stabilization for bebop front camera
 */

#ifndef CV_BEBOP_STABILIZATION_H
#define CV_BEBOP_STABILIZATION_H

extern float k_fisheye;
extern float focalLength;
extern float scale_x;
extern float scale_y;
extern float near;
extern float far;
extern float angleOfView;
extern int   noroll;
extern int   nopitch;
extern int   noprojection;
extern float imgRot;
extern float frameRot;

typedef float mat4[16];
typedef float vec4[4];

void setPerspectiveMat(mat4);
void setRotationMat(float, float, float, mat4);
void setIdentityMatrix(mat4 m);
void matrixMultiply(mat4 m1, mat4 m2, mat4 result);
struct image_t* cv_bebop_stabilization_func(struct image_t * img);
extern void cv_bebop_stabilization_init(void);
void view_set_lookat(mat4 result, vec4 eye, vec4 center, vec4 up);
float vector_length(const float x, const float y, const float z);
void translate_xyz(float* result, const float translatex, const float translatey, const float translatez);
void rotateVector(float x, float y, float z, vec4 vector);
void setTranslationMat(float x, float y, float z, mat4 outputMat);
#endif


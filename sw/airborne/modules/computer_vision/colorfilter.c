/*
 * Copyright (C) 2015
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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

struct video_listener *listener = NULL;

// Filter Settings

uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

uint8_t orange_color_lum_min = 20;
uint8_t orange_color_lum_max = 255;
uint8_t orange_color_cb_min  = 75;
uint8_t orange_color_cb_max  = 145;
uint8_t orange_color_cr_min  = 158;
uint8_t orange_color_cr_max  = 255;

uint8_t black_color_lum_min = 0;
uint8_t black_color_lum_max = 80;
uint8_t black_color_cb_min  = 105;
uint8_t black_color_cb_max  = 135;
uint8_t black_color_cr_min  = 115;
uint8_t black_color_cr_max  = 145;

uint8_t green_color_lum_min = 0;
uint8_t green_color_lum_max = 142;
uint8_t green_color_cb_min  = 0;
uint8_t green_color_cb_max  = 110;
uint8_t green_color_cr_min  = 0;
uint8_t green_color_cr_max  = 143;

// Result
int orange_color_count = 0;
int black_color_count = 0;
int green_color_count = 0;

// Function
struct image_t *colorfilter_func(struct image_t *img);

struct image_t *colorfilter_func(struct image_t *img)
{
 /*orange_color_lum_min = color_lum_min;
 orange_color_lum_max = color_lum_max;
 orange_color_cb_min  = color_cb_min;
 orange_color_cb_max  = color_cb_max;
 orange_color_cr_min  = color_cr_min;
 orange_color_cr_max  = color_cr_max;
*/
  // Filter
  orange_color_count = image_yuv422_colorfilt(img, img,
		  orange_color_lum_min, orange_color_lum_max,
		  orange_color_cb_min, orange_color_cb_max,
		  orange_color_cr_min, orange_color_cr_max
                                      ); 
  /*
black_color_count = image_yuv422_colorfilt(img, img,
		  black_color_lum_min, black_color_lum_max,
		  black_color_cb_min, black_color_cb_max,
		  black_color_cr_min, black_color_cr_max
                                      );
*/
  green_color_count = image_yuv422_colorfilt(img, img,
		  green_color_lum_min, green_color_lum_max,
		  green_color_cb_min, green_color_cb_max,
		  green_color_cr_min, green_color_cr_max
                                      );
  return img; // Colorfilter did not make a new image
}

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func);
}

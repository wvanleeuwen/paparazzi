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
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/lib/vision/image.c
 * Image helper functions, like resizing, color filter, converters...
 */

#include <stdio.h>
#include "image.h"
#include <stdlib.h>
#include <string.h>

typedef uint8_t elem_type;
elem_type torben(elem_type m[], uint16_t n, uint8_t pixel_width, uint8_t channel);
int get_sum(uint32_t *integral_image, uint16_t width, uint16_t x_min, uint16_t y_min, uint16_t x_max, uint16_t y_max);

/**
 * Create a new image
 * @param[out] *img The output image
 * @param[in] width The width of the image
 * @param[in] height The height of the image
 * @param[in] type The type of image (YUV422 or grayscale)
 */
void image_create(struct image_t *img, uint16_t width, uint16_t height, enum image_type type)
{
  // Set the variables
  img->type = type;
  img->w = width;
  img->h = height;

  // Depending on the type the size differs
  if (type == IMAGE_YUV422) {
    img->buf_size = sizeof(uint8_t) * 2 * width * height;
  } else if (type == IMAGE_JPEG) {
    img->buf_size = sizeof(uint8_t) * 1.1 * width * height;  // At maximum quality this is enough
  } else if (type == IMAGE_GRADIENT) {
    img->buf_size = sizeof(int16_t) * width * height;
  } else {
    img->buf_size = sizeof(uint8_t) * width * height;
  }

  img->buf = malloc(img->buf_size);
}

/**
 * Free the image
 * @param[in] *img The image to free
 */
void image_free(struct image_t *img)
{
  free(img->buf);
}

/**
 * Copy an image from inut to output
 * This will only work if the formats are the same
 * @param[in] *input The input image to copy from
 * @param[out] *output The out image to copy to
 */
void image_copy(struct image_t *input, struct image_t *output)
{
  if (input->type != output->type) {
    return;
  }

  output->w = input->w;
  output->h = input->h;
  output->buf_size = input->buf_size;
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));
  memcpy(output->buf, input->buf, input->buf_size);
}

/**
 * Convert an image to grayscale.
 * Depending on the output type the U/V bytes are removed
 * @param[in] *input The input image (Needs to be YUV422)
 * @param[out] *output The output image
 */
void image_to_grayscale(struct image_t *input, struct image_t *output)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  source++;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Copy the pixels
  for (int y = 0; y < output->h; y++) {
    for (int x = 0; x < output->w; x++) {
      if (output->type == IMAGE_YUV422) {
        *dest++ = 127;  // U / V
      }
      *dest++ = *source;    // Y
      source += 2;
    }
  }
}

/**
 * Filter colors in an YUV422 image
 * @param[in] *input The input image to filter
 * @param[out] *output The filtered output image, where pixels inside are set to maximum values (can be null)
 * @param[out] *bins The output bin counts
 * @param[in] bins_cnt The amount of bins to count
 * @param[in] y_m, y_M The Y minimum and maximum value
 * @param[in] u_m, u_M The U minimum and maximum value
 * @param[in] v_m, v_M The V minimum and maximum value
 * @return The amount of filtered pixels
 */
uint32_t image_yuv422_colorfilt(struct image_t *input, struct image_t *output, uint32_t *bins, uint16_t bins_cnt,
                                uint8_t y_m, uint8_t y_M, uint8_t u_m, uint8_t u_M, uint8_t v_m, uint8_t v_M)
{
  uint32_t cnt = 0;
  uint8_t *source = input->buf;
  uint8_t *dest = NULL;

  // Check if the ouput is not NULL
  if (output != NULL) {
    dest = output->buf;
    // Copy the creation timestamp (stays the same)
    memcpy(&output->ts, &input->ts, sizeof(struct timeval));
  }

  // Go trough all the pixels
  for (uint16_t y = 0; y < input->h; y++) {
    for (uint16_t x = 0; x < input->w; x += 2) {
      // Check if the color is inside the specified values
      if (
        (source[1] >= y_m)
        && (source[1] <= y_M)
        && (source[0] >= u_m)
        && (source[0] <= u_M)
        && (source[2] >= v_m)
        && (source[2] <= v_M)
      ) {
        cnt++; // Update the total count

        // Update the bins
        if (bins_cnt > 0) {
          bins[x / (input->w / bins_cnt)]++;
        }

        // Update the pixel
        if (dest != NULL) {
          dest[0] = 255;  // U
          dest[1] = 255;  // Y
          dest[2] = 255;  // V
          dest[3] = 255;  // Y
        }
      } else if (dest != NULL) {
        // Update the pixel
        dest[0] = source[0];  // U
        dest[1] = source[1];  // Y
        dest[2] = source[2];  // V
        dest[3] = source[3];  // Y
      }

      // Go to the next 2 pixels
      dest += 4;
      source += 4;
    }
  }
  return cnt;
}

/**
* Simplified high-speed low CPU downsample function without averaging
*  downsample factor must be 1, 2, 4, 8 ... 2^X
*  image of typ UYVY expected. Only one color UV per 2 pixels
*
*  we keep the UV color of the first pixel pair
*  and sample the intensity evenly 1-3-5-7-... or 1-5-9-...
*
*  input:         u1y1 v1y2 u3y3 v3y4 u5y5 v5y6 u7y7 v7y8 ...
*  downsample=1   u1y1 v1y2 u3y3 v3y4 u5y5 v5y6 u7y7 v7y8 ...
*  downsample=2   u1y1v1 (skip2) y3 (skip2) u5y5v5 (skip2 y7 (skip2) ...
*  downsample=4   u1y1v1 (skip6) y5 (skip6) ...
* @param[in] *input The input YUV422 image
* @param[out] *output The downscaled YUV422 image
* @param[in] downsample The downsampel facter (must be downsample=2^X)
*/
void image_yuv422_downsample(struct image_t *input, struct image_t *output, uint16_t downsample)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;
  uint16_t pixelskip = (downsample - 1) * 2;

  // Copy the creation timestamp (stays the same)
  memcpy(&output->ts, &input->ts, sizeof(struct timeval));

  // Go trough all the pixels
  for (uint16_t y = 0; y < output->h; y++) {
    for (uint16_t x = 0; x < output->w; x += 2) {
      // YUYV
      *dest++ = *source++; // U
      *dest++ = *source++; // Y
      *dest++ = *source++; // V
      source += pixelskip;
      *dest++ = *source++; // Y
      source += pixelskip;
    }
    // read 1 in every 'downsample' rows, so skip (downsample-1) rows after reading the first
    source += (downsample - 1) * input->w * 2;
  }
}

/**
 * This outputs a subpixel window image in grayscale
 * Currently only works with Grayscale images as input but could be upgraded to
 * also support YUV422 images.
 * @param[in] *input Input image (grayscale only)
 * @param[out] *output Window output (width and height is used to calculate the window size)
 * @param[in] *center Center point in subpixel coordinates
 * @param[in] subpixel_factor The subpixel factor per pixel
 */
void image_subpixel_window(struct image_t *input, struct image_t *output, struct point_t *center,
                           uint16_t subpixel_factor)
{
  uint8_t *input_buf = (uint8_t *)input->buf;
  uint8_t *output_buf = (uint8_t *)output->buf;

  // Calculate the window size
  uint16_t half_window = output->w / 2;
  uint16_t subpixel_w = input->w * subpixel_factor;
  uint16_t subpixel_h = input->h * subpixel_factor;

  // Go through the whole window size in normal coordinates
  for (uint16_t i = 0; i < output->w; i++) {
    for (uint16_t j = 0; j < output->h; j++) {
      // Calculate the subpixel coordinate
      uint16_t x = center->x + (i - half_window) * subpixel_factor;
      uint16_t y = center->y + (j - half_window) * subpixel_factor;
      Bound(x, 0, subpixel_w);
      Bound(y, 0, subpixel_h);

      // Calculate the original pixel coordinate
      uint16_t orig_x = x / subpixel_factor;
      uint16_t orig_y = y / subpixel_factor;

      // Calculate top left (in subpixel coordinates)
      uint16_t tl_x = orig_x * subpixel_factor;
      uint16_t tl_y = orig_y * subpixel_factor;

      // Check if it is the top left pixel
      if (tl_x == x &&  tl_y == y) {
        output_buf[output->w * j + i] = input_buf[input->w * orig_y + orig_x];
      } else {
        // Calculate the difference from the top left
        uint16_t alpha_x = (x - tl_x);
        uint16_t alpha_y = (y - tl_y);

        // Blend from the 4 surrounding pixels
        uint32_t blend = (subpixel_factor - alpha_x) * (subpixel_factor - alpha_y) * input_buf[input->w * orig_y + orig_x];
        blend += alpha_x * (subpixel_factor - alpha_y) * input_buf[input->w * orig_y + (orig_x + 1)];
        blend += (subpixel_factor - alpha_x) * alpha_y * input_buf[input->w * (orig_y + 1) + orig_x];
        blend += alpha_x * alpha_y * input_buf[input->w * (orig_y + 1) + (orig_x + 1)];

        // Set the normalized pixel blend
        output_buf[output->w * j + i] = blend / (subpixel_factor * subpixel_factor);
      }
    }
  }
}

/**
 * Calculate the  gradients using the following matrix:
 * [0 -1 0; -1 0 1; 0 1 0]
 * @param[in] *input Input grayscale image
 * @param[out] *dx Output gradient in the X direction (dx->w = input->w-2, dx->h = input->h-2)
 * @param[out] *dy Output gradient in the Y direction (dx->w = input->w-2, dx->h = input->h-2)
 */
void image_gradients(struct image_t *input, struct image_t *dx, struct image_t *dy)
{
  // Fetch the buffers in the correct format
  uint8_t *input_buf = (uint8_t *)input->buf;
  int16_t *dx_buf = (int16_t *)dx->buf;
  int16_t *dy_buf = (int16_t *)dy->buf;

  // Go trough all pixels except the borders
  for (uint16_t x = 1; x < input->w - 1; x++) {
    for (uint16_t y = 1; y < input->h - 1; y++) {
      dx_buf[(y - 1)*dx->w + (x - 1)] = (int16_t)input_buf[y * input->w + x + 1] - (int16_t)input_buf[y * input->w + x - 1];
      dy_buf[(y - 1)*dy->w + (x - 1)] = (int16_t)input_buf[(y + 1) * input->w + x] - (int16_t)
                                        input_buf[(y - 1) * input->w + x];
    }
  }
}

/**
 * Calculate the G vector of an image gradient
 * This is used for optical flow calculation.
 * @param[in] *dx The gradient in the X direction
 * @param[in] *dy The gradient in the Y direction
 * @param[out] *g The G[4] vector devided by 255 to keep in range
 */
void image_calculate_g(struct image_t *dx, struct image_t *dy, int32_t *g)
{
  int32_t sum_dxx = 0, sum_dxy = 0, sum_dyy = 0;

  // Fetch the buffers in the correct format
  int16_t *dx_buf = (int16_t *)dx->buf;
  int16_t *dy_buf = (int16_t *)dy->buf;

  // Calculate the different sums
  for (uint16_t x = 0; x < dx->w; x++) {
    for (uint16_t y = 0; y < dy->h; y++) {
      sum_dxx += ((int32_t)dx_buf[y * dx->w + x] * dx_buf[y * dx->w + x]);
      sum_dxy += ((int32_t)dx_buf[y * dx->w + x] * dy_buf[y * dy->w + x]);
      sum_dyy += ((int32_t)dy_buf[y * dy->w + x] * dy_buf[y * dy->w + x]);
    }
  }

  // ouput the G vector
  g[0] = sum_dxx / 255;
  g[1] = sum_dxy / 255;
  g[2] = g[1];
  g[3] = sum_dyy / 255;
}

/**
 * Calculate the difference between two images and return the error
 * This will only work with grayscale images
 * @param[in] *img_a The image to substract from
 * @param[in] *img_b The image to substract from img_a
 * @param[out] *diff The image difference (if not needed can be NULL)
 * @return The squared difference summed
 */
uint32_t image_difference(struct image_t *img_a, struct image_t *img_b, struct image_t *diff)
{
  uint32_t sum_diff2 = 0;
  int16_t *diff_buf = NULL;

  // Fetch the buffers in the correct format
  uint8_t *img_a_buf = (uint8_t *)img_a->buf;
  uint8_t *img_b_buf = (uint8_t *)img_b->buf;

  // If we want the difference image back
  if (diff != NULL) {
    diff_buf = (int16_t *)diff->buf;
  }

  // Go trough the imagge pixels and calculate the difference
  for (uint16_t x = 0; x < img_b->w; x++) {
    for (uint16_t y = 0; y < img_b->h; y++) {
      int16_t diff_c = img_a_buf[(y + 1) * img_a->w + (x + 1)] - img_b_buf[y * img_b->w + x];
      sum_diff2 += diff_c * diff_c;

      // Set the difference image
      if (diff_buf != NULL) {
        diff_buf[y * diff->w + x] = diff_c;
      }
    }
  }

  return sum_diff2;
}

/**
 * Calculate the multiplication between two images and return the error
 * This will only work with image gradients
 * @param[in] *img_a The image to multiply
 * @param[in] *img_b The image to multiply with
 * @param[out] *mult The image multiplication (if not needed can be NULL)
 * @return The sum of the multiplcation
 */
int32_t image_multiply(struct image_t *img_a, struct image_t *img_b, struct image_t *mult)
{
  int32_t sum = 0;
  int16_t *img_a_buf = (int16_t *)img_a->buf;
  int16_t *img_b_buf = (int16_t *)img_b->buf;
  int16_t *mult_buf = NULL;

  // When we want an output
  if (mult != NULL) {
    mult_buf = (int16_t *)mult->buf;
  }

  // Calculate the multiplication
  for (uint16_t x = 0; x < img_a->w; x++) {
    for (uint16_t y = 0; y < img_a->h; y++) {
      int16_t mult_c = img_a_buf[y * img_a->w + x] * img_b_buf[y * img_b->w + x];
      sum += mult_c;

      // Set the difference image
      if (mult_buf != NULL) {
        mult_buf[y * mult->w + x] = mult_c;
      }
    }
  }

  return sum;
}

/**
 * Show points in an image by coloring them through giving
 * the pixels the maximum value.
 * This works with YUV422 and grayscale images
 * @param[in,out] *img The image to place the points on
 * @param[in] *points The points to sohw
 * @param[in] *points_cnt The amount of points to show
 */
void image_show_points(struct image_t *img, struct point_t *points, uint16_t points_cnt)
{
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;

  // Go trough all points and color them
  for (int i = 0; i < points_cnt; i++) {
    uint32_t idx = pixel_width * points[i].y * img->w + points[i].x * pixel_width;
    img_buf[idx] = 255;

    // YUV422 consists of 2 pixels
    if (img->type == IMAGE_YUV422) {
      idx++;
      img_buf[idx] = 255;
    }
  }
}

/**
 * Shows the flow from a specific point to a new point
 * This works on YUV422 and Grayscale images
 * @param[in,out] *img The image to show the flow on
 * @param[in] *vectors The flow vectors to show
 * @param[in] *points_cnt The amount of points and vectors to show
 */
void image_show_flow(struct image_t *img, struct flow_t *vectors, uint16_t points_cnt, uint8_t subpixel_factor)
{
  // Go through all the points
  for(uint16_t i = 0; i < points_cnt; i++) {
    // Draw a line from the original position with the flow vector
    struct point_t from = {
      vectors[i].pos.x / subpixel_factor,
      vectors[i].pos.y / subpixel_factor
    };
    struct point_t to = {
      (vectors[i].pos.x + vectors[i].flow_x) / subpixel_factor,
      (vectors[i].pos.y + vectors[i].flow_y) / subpixel_factor
    };
    image_draw_line(img, &from, &to);
  }
}

/**
 * Draw a line on the image
 * @param[in,out] *img The image to show the line on
 * @param[in] *from The point to draw from
 * @param[in] *to The point to draw to
 */
void image_draw_line(struct image_t *img, struct point_t *from, struct point_t *to)
{
  int xerr = 0, yerr = 0;
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;
  uint16_t startx = from->x;
  uint16_t starty = from->y;

  /* compute the distances in both directions */
  int32_t delta_x = from->x - to->x;
  int32_t delta_y = from->y - to->y;

  /* Compute the direction of the increment,
     an increment of 0 means either a horizontal or vertical
     line.
  */
  int8_t incx, incy;
  if (delta_x > 0) { incx = 1; }
  else if (delta_x == 0) { incx = 0; }
  else { incx = -1; }

  if (delta_y > 0) { incy = 1; }
  else if (delta_y == 0) { incy = 0; }
  else { incy = -1; }

  /* determine which distance is greater */
  uint16_t distance = 0;
  delta_x = abs(delta_x);
  delta_y = abs(delta_y);
  if (delta_x > delta_y) { distance = delta_x * 20; }
  else { distance = delta_y * 20; }

  /* draw the line */
  for(uint16_t t = 0; starty >= 0 && starty < img->h && startx >= 0 && startx < img->w && t <= distance+1; t++) {
      img_buf[img->w*pixel_width*starty + startx*pixel_width] = (t <= 3)? 0 : 255;

      if(img->type == IMAGE_YUV422) {
        img_buf[img->w*pixel_width*starty + startx*pixel_width +1] = 255;

        if(startx+1 < img->w) {
          img_buf[img->w*pixel_width*starty + startx*pixel_width +2] = (t <= 3)? 0 : 255;
          img_buf[img->w*pixel_width*starty + startx*pixel_width +3] = 255;
        }
      }

    xerr += delta_x;
    yerr += delta_y;
    if (xerr > distance) {
      xerr -= distance;
      startx += incx;
    }
    if (yerr > distance) {
      yerr -= distance;
      starty += incy;
    }
  }
}

/**
 * Draw a line on the image according to the Bresenham algorithm, 
 * YUV422 images may display wider lines due to pixel rounding.
 * @param[in,out] *img The image to show the line on
 * @param[in] *from The point to draw from
 * @param[in] *to The point to draw to
 */
void image_draw_line_bresenham(struct image_t *img, struct point_t *from, struct point_t *to)
{
  int16_t x = 0, y = 0;
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = (img->type == IMAGE_YUV422) ? 2 : 1;

  // define the starting point
  uint16_t start_x = from->x;
  uint16_t start_y = from->y;

  // compute the distances in both directions
  int32_t delta_x = from->x - to->x;
  int32_t delta_y = from->y - to->y;

  // Compute the direction of the increment,
  int8_t incr_x = (delta_x > 0) ? 1 : ((delta_x < 0) ? -1 : 0);
  int8_t incr_y = (delta_y > 0) ? 1 : ((delta_y < 0) ? -1 : 0);

  // Draw the line segment
  uint32_t abs_delta_x = abs(delta_x);
  uint32_t abs_delta_y = abs(delta_y);
  int32_t cnt_x = abs_delta_y>>1, cnt_y = abs_delta_y>>1;
  if (abs_delta_x >= abs_delta_y) // the line segment is directed towards the horizontal
  {
    for(i = 0; i < abs_delta_x; i++)
    {
      abs_delta_y += abs_delta_y;
      if (cnt_y >= abs_delta_x)
      {
        cnt_y -= abs_delta_x;
        y += incr_y;
      }
      x += incr_x;
      // DO IMAGE PROCESSING
    }
  }
  else // the line segment is directed towards the vertical
  {
    for(i=0; i < abs_delta_y; i++)
    {
      x += abs_delta_x;
      if (x >= abs_delta_y)
      {
        cnt_x -= abs_delta_y;
        x += incr_x;
      }
      y += incr_y;
      // DO IMAGE PROCESSING
    }
  }
}

/**
 * Draw rectangle
 * @param[in,out] *img The image to show the rectangle on
 * @param[in] *from The vertex of the rectangle
 * @param[in] *to The vertex opposite to the 'from' vertex
 */
void image_draw_rectangle(struct image_t *img, struct point_t *from, struct point_t *to) 
{

}


/**
 * Compute Integral Image
 * @param[in] *img The image to be summed
 * @param[in, out] *int_img Resultant integral image
 * @param[in] px_start The start location in the input image to sum
 * @param[in] img_h The hieght of th esub image
 */
void get_integral_image(struct image_t *img, uint32_t *int_img0, uint32_t *int_img1, uint32_t *int_img2,
                        uint16_t px_start, uint16_t img_h)
{
  uint8_t *img_buf = (uint8_t *)img->buf;
  uint8_t pixel_width = 4; // (img->type == IMAGE_YUV422) ? 2 : 1;
  uint16_t w = img->w / 2, h = img_h;
  uint16_t x, y;

  for (x = 0; x < w; x++) {
    for (y = 0; y < h; y++) {
      if (x >= 1 && y >= 1) {
        int_img0[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 1] + int_img0[w * y + x - 1] +
                              int_img0[w * (y - 1) + x] - int_img0[w * (y - 1) + x - 1];
        int_img1[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width    ] + int_img1[w * y + x - 1] +
                              int_img1[w * (y - 1) + x] - int_img1[w * (y - 1) + x - 1];
        int_img2[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 2] + int_img2[w * y + x - 1] +
                              int_img2[w * (y - 1) + x] - int_img2[w * (y - 1) + x - 1];
      } else if (x >= 1) {
        int_img0[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 1] + int_img0[w * y + x - 1];
        int_img1[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width    ] + int_img1[w * y + x - 1];
        int_img2[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 2] + int_img2[w * y + x - 1];
      } else if (y >= 1) {
        int_img0[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 1] + int_img0[w * (y - 1) + x];
        int_img1[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width    ] + int_img1[w * (y - 1) + x];
        int_img2[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 2] + int_img2[w * (y - 1) + x];
      } else {
        int_img0[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 1];
        int_img1[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width    ];
        int_img2[w * y + x] = img_buf[px_start + w * pixel_width * y + x * pixel_width + 2];
      }
    }
  }
}

/*
 * The following code is public domain.
 * Algorithm by Torben Mogensen, implementation by N. Devillard.
 * This code in public domain.
Compute Median of Image
 * @param[in] *m The image
 * @param[in] n total number of pixels in image
 * @param[in] pixel_width Distance between recurring channel
 * @param[in] channel Channel number
 */
elem_type torben(elem_type m[], uint16_t n, uint8_t pixel_width, uint8_t channel)
{
  int  i, less, greater, equal;
  elem_type  min, max, guess, maxltguess, mingtguess;

  min = max = m[0] ;
  for (i = pixel_width + channel; i < n; i += pixel_width) {
    if (m[i] < min) { min = m[i]; }
    if (m[i] > max) { max = m[i]; }
  }

  while (1) {
    guess = (min + max) / 2;
    less = 0; greater = 0; equal = 0;
    maxltguess = min ;
    mingtguess = max ;
    for (i = channel; i < n; i += pixel_width) {
      if (m[i] < guess) {
        less++;
        if (m[i] > maxltguess) { maxltguess = m[i] ; }
      } else if (m[i] > guess) {
        greater++;
        if (m[i] < mingtguess) { mingtguess = m[i] ; }
      } else { equal++; }
    }
    if (less <= (n + 1) / 2 / pixel_width && greater <= (n + 1) / 2 / pixel_width) { break ; }
    else if (less > greater) { max = maxltguess ; }
    else { min = mingtguess; }
  }
  if (less >= (n + 1) / 2 / pixel_width) { return maxltguess; }
  else if (less + equal >= (n + 1) / 2 / pixel_width) { return guess; }
  else { return mingtguess; }
}

/**
 * Compute Median of Image
 * @param[in] *img The image
 * @param[in] px_start The start location to find median
 * @param[in] img_h The hieght of the sub image
 */
int median(struct image_t *img, uint8_t channel, uint16_t px_start, uint16_t img_h)
{
  uint8_t pixel_width = 4; // distance between repeating channel values // = (img->type == IMAGE_YUV422) ? 2 : 1;
  uint8_t *img_buf = (uint8_t *)img->buf;

  uint8_t offset = 0;
  switch (channel) {  // YUV UY VY
    case 0:
      offset = 1; break;
    case 1:
      offset = 0; break;
    case 2:
      offset = 2; break;
    default:
      perror("Input channel out of range for YUV!");
  }

  return torben(&img_buf[px_start], img->w * pixel_width * img_h, pixel_width, offset);
}

int get_sum(uint32_t *integral_image, uint16_t width, uint16_t x_min, uint16_t y_min, uint16_t x_max, uint16_t y_max)
{
  return (integral_image[width * y_min + x_min] + integral_image[width * y_max + x_max] -
          integral_image[width * y_min + x_max] - integral_image[width * y_max + x_min]);
}

/**
 * Get obstacle response, computes percentage difference from median image value
 * @param[in] *img The image
 * @param[in] width Width of the integral image
 * @param[in] x The x location in of the top left corner of the box
 * @param[in] y The y location in of the top left corner of the box
 * @param[in] feature_size The length of one side of the box
 * @param[in] px_inner The number of pixels in the box
 * @param[in] median_val The median value of the original sub-image
 */
int8_t get_obs_response(uint32_t *integral_image, uint16_t width, uint16_t x, uint16_t y,
                        uint16_t feature_size, uint32_t px_inner, uint8_t median_val)
{
  uint32_t sub_area = get_sum(integral_image, width, x, y, x + feature_size, y + feature_size);

  return ((100 * ((sub_area / px_inner) - median_val)) / 256);
}

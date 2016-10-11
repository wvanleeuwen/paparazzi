/*
 * Copyright (C) 2016
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
 * @file modules/computer_vision/snake_gate_detection.c
 */

// Own header
#include "modules/computer_vision/snake_gate_detection.h"
#include <stdio.h>
#include <stdlib.h>

#include "modules/computer_vision/lib/vision/image.h"
#include "modules/computer_vision/lib/vision/gate_detection.h"

#include "std.h"
#include "state.h"
#include "mcu_periph/sys_time.h"


/* Gate structure */
struct gate_img {
  int x;             ///< The image x coordinate of the gate centre
  int y;             ///< The image y coordinate of the gate centre
  int sz;            ///< Half the image size of the gate
  float q;           //gate quality
  int n_sides;       ///< How many sides are orange (to prevent detecting a small gate in the corner of a big one partially out of view).
  int sz_left;     ///< Half the image size of the left side
  int sz_right;    ///< Half the image size of the right side
};

#ifndef SGD_CAMERA
#define SGD_CAMERA front_camera
#endif

#define DRAW_GATE

//initial position after gate pass
// this initialises the filter location before positive identification
#define INITIAL_X 1.5
#define INITIAL_Y 0
#define INITIAL_Z 0

//initial position and speed safety margins
#define X_POS_MARGIN 0.15 //m
#define Y_POS_MARGIN 0.5 //m
#define Z_POS_MARGIN 0.15 //m
#define X_SPEED_MARGIN 0.15 //m/s
#define Y_SPEED_MARGIN 0.15 //m/s

struct video_listener *listener = NULL;

#define RED 0
#define BLUE 1

#define WINDOW_COLOR RED

// Filter Settings
#if WINDOW_COLOR == RED
uint8_t color_lum_min = 60;// 60;//105;
uint8_t color_lum_max = 100;//228;//205;
uint8_t color_cb_min  = 90;//66;//52;
uint8_t color_cb_max  = 130;//194;//140;
uint8_t color_cr_min  = 145;//140;//180;
uint8_t color_cr_max  = 200;//230;//255;
#else
// TODO find color scheme for blue
uint8_t color_lum_min = 60;// 60;//105;
uint8_t color_lum_max = 255;//228;//205;
uint8_t color_cb_min  = 100;//140;//180;
uint8_t color_cb_max  = 255;//230;//255;
uint8_t color_cr_min  = 20;//66;//52;
uint8_t color_cr_max  = 100;//194;//140;
#endif

// Gate detection settings:
int n_samples = 2500; //1000;//500;
int min_pixel_size = 40;  //100;
float min_gate_quality = 0.15;
float gate_thickness = 0; //0.05;//0.10;//
float angle_to_gate = 0;

// TODO KIRK find correct
const float gate_size_m = 1.1;   // gate size in meters

// Result
#define MAX_GATES 50
struct gate_img gates[MAX_GATES];
struct gate_img best_gate = {0};
struct gate_img previous_gate = {0};
struct gate_img gen_gate = {0};
struct image_t img_result;
int n_gates = 0;

//color picker
uint8_t y_center_picker  = 0;
uint8_t cb_center  = 0;
uint8_t cr_center  = 0;

//camera parameters
// TODO KIRK find correct
#define radians_per_pix_w 0.003333334 //2.1 rad(60deg)/315
#define radians_per_pix_h 0.00328125   //1.05rad / 160

// gate location relative to drone in body frame (assume forward camera)
float gate_x_dist = 0;
float gate_y_dist = 0;
float gate_z_dist = 0;

//state filter
float body_vx = 0;
float body_vy = 0;

float predicted_x_gate = 0;
float predicted_y_gate = 0;
float predicted_z_gate = 0;

float filtered_x_gate = 0;
float filtered_y_gate = 0;
float filtered_z_gate = 0;

float previous_x_gate = 0;
float previous_y_gate = 0;
float previous_z_gate = 0;

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
int gate_detected = 0;
int gate_processed = 0;
int counter_gate_detected = 0;
int init_pos_filter = 0;
int ready_pass_through;

// timers
float last_processed, time_gate_detected, time_tracked;

/*
static void check_color_center(struct image_t *im, uint8_t *y_c, uint8_t *cb_c, uint8_t *cr_c)
{
  uint8_t *buf = im->buf;
  int x = (im->w) / 2;
  int y = (im->h) / 2;
  buf += y * (im->w) * 2 + x * 2;

  *y_c = buf[1];
  *cb_c = buf[0];
  *cr_c = buf[2];
}


//set color pixel
static void image_yuv422_set_color(struct image_t *input, struct image_t *output, int x, int y)
{
  uint8_t *source = input->buf;
  uint8_t *dest = output->buf;

  // Copy the creation timestamp (stays the same)
  output->ts = input->ts;
  if (x % 2 == 1) { x--; }
  if (x < 0 || x >= input->w || y < 0 || y >= input->h) {
    return;
  }

  source += y * (input->w) * 2 + x * 2;
  dest += y * (output->w) * 2 + x * 2;
  // UYVY
  dest[0] = 65;//211;        // U//was 65
  dest[1] = source[1];  // Y
  dest[2] = 255;//60;        // V//was 255
  dest[3] = source[3];  // Y
}
*/

static void calculate_gate_position(int x_pix, int y_pix, int sz_pix, struct image_t *img, struct gate_img gate)
{
  // pixel distance conversion
  static float hor_angle = 0., vert_angle = 0.;

  // calculate angles here, rotate camera pixels 90 deg
  vert_angle = -(x_pix - img->h / 2) * radians_per_pix_h - stateGetNedToBodyEulers_f()->theta;
  hor_angle = (y_pix - img->w / 2) * radians_per_pix_w;

  // in body frame
  gate_x_dist = gate_size_m / (gate.sz * 2 * radians_per_pix_w);
  gate_y_dist = gate_x_dist * sin(hor_angle);
  gate_z_dist = gate_x_dist * sin(vert_angle);
}

static void draw_gate(struct image_t *im, struct gate_img gate)
{
  // draw four lines on the image:
  struct point_t from, to;
  from.x = (gate.x - gate.sz);
  from.y = gate.y - gate.sz;
  to.x = (gate.x - gate.sz);
  to.y = gate.y + gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x - gate.sz);
  from.y = gate.y + gate.sz;
  to.x = (gate.x + gate.sz);
  to.y = gate.y + gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x + gate.sz);
  from.y = gate.y + gate.sz;
  to.x = (gate.x + gate.sz);
  to.y = gate.y - gate.sz;
  image_draw_line(im, &from, &to);
  from.x = (gate.x + gate.sz);
  from.y = gate.y - gate.sz;
  to.x = (gate.x - gate.sz);
  to.y = gate.y - gate.sz;
  image_draw_line(im, &from, &to);
}

static void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points)
{
  (*n_points) = 0;
  (*n_colored_points) = 0;

  float t_step = 0.05;
  int x, y;
  float t;
  // go from Q1 to Q2 in 1/t_step steps:
  for (t = 0.0f; t < 1.0f; t += t_step) {
    // determine integer coordinate on the line:
    x = (int)(t * Q1.x + (1.0f - t) * Q2.x);
    y = (int)(t * Q1.y + (1.0f - t) * Q2.y);

    if (x >= 0 && x < im->w && y >= 0 && y < im->h) {
      // augment number of checked points:
      (*n_points)++;

      if (check_color(im, x, y)) {
        // the point is of the right color:
        (*n_colored_points)++;
      }
    }
  }
}

static void check_gate(struct image_t *im, struct gate_img gate, float *quality, int *n_sides)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;
  // how much of the side should be visible to count as a detected side?
  float min_ratio_side = 0.30;
  (*n_sides) = 0;

  // check the four lines of which the gate consists:
  struct point_t from, to;

  from.x = gate.x - gate.sz;
  from.y = gate.y - gate.sz_left;
  to.x = gate.x - gate.sz;
  to.y = gate.y + gate.sz_left;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x - gate.sz;
  from.y = gate.y + gate.sz_left;
  to.x = gate.x + gate.sz;
  to.y = gate.y + gate.sz_right;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y + gate.sz_right;
  to.x = gate.x + gate.sz;
  to.y = gate.y - gate.sz_right;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y - gate.sz_right;
  to.x = gate.x - gate.sz;
  to.y = gate.y - gate.sz_left;
  check_line(im, from, to, &np, &nc);
  if ((float) nc / (float) np >= min_ratio_side) {
    (*n_sides)++;
  }
  n_points += np;
  n_colored_points += nc;

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}

/* y direction is left right */
static void snake_left_right(struct image_t *im, int x, int y, int *y_low, int *y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;

  // snake towards negative y (down?)
  while ((*y_low) > 0 && !done) {
    if (check_color(im, x, (*y_low) - 1)) {
      (*y_low)--;
    } else if (x < im->h - 1 && check_color(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
    } else if (x > 0 && check_color(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
    } else if (x < im->h - 2 && check_color(im, x + 2, (*y_low) - 1)) {
      x+=2;
      (*y_low)--;
    } else if (x > 1 && check_color(im, x - 2, (*y_low) - 1)) {
      x-=2;
      (*y_low)--;
    } else {
      done = 1;
    }
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  while ((*y_high) < im->w - 1 && !done) {
    if (check_color(im, x, (*y_high) + 1)) {
      (*y_high)++;
    } else if (x < im->h - 1 && check_color(im, x + 1, (*y_high) + 1)) {
      x++;
      (*y_high)++;
    } else if (x > 0 && check_color(im, x - 1, (*y_high) + 1)) {
      x--;
      (*y_high)++;
    } else if (x < im->h - 2 && check_color(im, x + 2, (*y_high) + 1)) {
      x+=2;
      (*y_high)++;
    } else if (x > 1 && check_color(im, x - 2, (*y_high) + 1)) {
      x-=2;
      (*y_high)++;
    } else {
      done = 1;
    }
  }
}

/* x direction is up down */
static void snake_up_down(struct image_t *im, int x, int y, int *x_low, int *x_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color(im, (*x_low) - 1, y)) {
      (*x_low)--;
    } else if (y < im->w - 1 && check_color(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
    } else if (y > 0 && check_color(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
    } else if (y < im->w - 2 && check_color(im, (*x_low) - 1, y + 2)) {
      y+=2;
      (*x_low)--;
    } else if (y > 1 && check_color(im, (*x_low) - 1, y - 2)) {
      y-=2;
      (*x_low)--;
    }else {
      done = 1;
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  while ((*x_high) < im->h - 1 && !done) {
    if (check_color(im, (*x_high) + 1, y)) {
      (*x_high)++;
    } else if (y < im->w - 1 && check_color(im, (*x_high) + 1, y + 1)) {
      y++;
      (*x_high)++;
    } else if (y > 0 && check_color(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
    } else if (y < im->w - 2 && check_color(im, (*x_high) + 1, y + 2)) {
      y+=2;
      (*x_high)++;
    } else if (y > 1 && check_color(im, (*x_high) + 1, y - 2)) {
      y-=2;
      (*x_high)++;
    } else {
      done = 1;
    }
  }
}

// Function
// Samples from the image and checks if the pixel is the right color.
// If yes, it "snakes" up and down to see if it is the side of a gate.
// If this stretch is long enough, it "snakes" also left and right.
// If the left/right stretch is also long enough, add the coords as a
// candidate square, optionally drawing it on the image.
static struct image_t *snake_gate_detection_func(struct image_t *img)
{
  int gen_alg = 1;
  uint16_t i;
  int x, y;
  int x_low = 0, x_high = 0;
  int y_low1 = 0, y_high1 = 0, y_low2 = 0, y_high2 = 0;
  int sz = 0, szy1 = 0, szy2 = 0;

  float best_quality = 0.;
  n_gates = 0;

  //color picker
  //check_color_center(img,&y_center_picker,&cb_center,&cr_center);

  if (n_samples > img->w * img->h) {n_samples = img->w * img->h;}
  for (i = 0; i < n_samples; i++) {
    if (i == 0){
      // use previous estimate
      x = best_gate.x - best_gate.sz;
      y = best_gate.y;
    } else {
      // get a random coordinate:
      x = rand() % img->h;
      y = rand() % img->w;
    }

	// NOTE: image is rotated
    //check_color(img, 1, 1);
    // check if it has the right color
    if (check_color(img, x, y)) {
      // snake up and down:
      snake_up_down(img, x, y, &x_low, &x_high);
      sz = x_high - x_low;

      x_low = x_low + (sz * gate_thickness);
      x_high = x_high - (sz * gate_thickness);

      x = (x_high + x_low) / 2;

      // if the stretch is long enough
      if (sz > min_pixel_size) {
        // snake left and right:
        snake_left_right(img, x_low, y, &y_low1, &y_high1);
        snake_left_right(img, x_high, y, &y_low2, &y_high2);

        y_low1 = y_low1 + (sz * gate_thickness);
        y_high1 = y_high1 - (sz * gate_thickness);
        y_low2 = y_low2 + (sz * gate_thickness);
        y_high2 = y_high2 - (sz * gate_thickness);

        // sizes of the left-right stretches: in y pixel coordinates
        szy1 = (y_high1 - y_low1);
        szy2 = (y_high2 - y_low2);

        // if the size is big enough:
        if (szy1 > min_pixel_size) {
          // draw four lines on the image:
          y = (y_high1 + y_low1) / 2;
          // set the size to the largest line found:
          //sz = (sz > szy1) ? sz : szy1;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &gates[n_gates].q, &gates[n_gates].n_sides);
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (gates[n_gates].q > min_gate_quality && gates[n_gates].q > best_quality) {
            best_quality = gates[n_gates].q;
            n_gates++;
          }
        } else if (szy2 > min_pixel_size) {
          y = (y_high2 + y_low2) / 2;
          // set the size to the largest line found:
          //sz = (sz > szy2) ? sz : szy2;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &gates[n_gates].q, &gates[n_gates].n_sides);
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (gates[n_gates].q > min_gate_quality && gates[n_gates].q > best_quality) {
            best_quality = gates[n_gates].q;
            n_gates++;
          }
        }
        if (n_gates >= MAX_GATES) {
          printf("TOO MANY GATES!!\n\n");
          break;
        }
      }
    }
  }

  if (n_gates > 0) {
    best_gate = gates[n_gates - 1];

    // do an additional fit to improve the gate detection:
    if (gen_alg) {
      // temporary variables:
      float fitness, angle_1, angle_2;
      int clock_arms = 0;

      // prepare the Region of Interest (ROI), which is larger than the gate:
      float size_factor = 1.25;//2;//1.25;

      int max_candidate_gates = 10;
      int start = 0;
      if (n_gates > max_candidate_gates) {
        // only check max_candidate_gate gates, last gates are best quality
        start = n_gates - max_candidate_gates;
      }

      for (int gate_nr = start; gate_nr < n_gates; gate_nr++) {
        int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
        int16_t min_x = gates[gate_nr].x - ROI_size;
        min_x = (min_x < 0) ? 0 : min_x;
        int16_t max_x = gates[gate_nr].x + ROI_size;
        max_x = (max_x < img->h) ? max_x : img->h;
        int16_t min_y = gates[gate_nr].y - ROI_size;
        min_y = (min_y < 0) ? 0 : min_y;
        int16_t max_y = gates[gate_nr].y + ROI_size;
        max_y = (max_y < img->w) ? max_y : img->w;

        // use best gate a seed for elite population
        gen_gate = gates[gate_nr];
        // detect the gate:
        gate_detection(img, &gen_gate.x, &gen_gate.y, &gen_gate.sz, &fitness, min_x, min_y, max_x, max_y, clock_arms, &angle_1, &angle_2, &angle_to_gate, &gates[gate_nr].sz_left, &gates[gate_nr].sz_right);
        check_gate(img, gen_gate, &gen_gate.q, &gen_gate.n_sides);

        if(gen_gate.n_sides > 2 && gen_gate.q > best_gate.q) {
          // store the information in the gate:
          best_gate = gen_gate;
        }
      }
    }
    calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);
    time_gate_detected = get_sys_time_float();

#ifdef DRAW_GATE
    draw_gate(img, best_gate);
#endif
  } else {
    printf("NO GATES!!\n\n");
    gate_detected = 0;
  }

#ifdef DRAW_GATE
  // color filtered version of image for overlay and debugging
  image_yuv422_colorfilt(img, img,
                    color_lum_min, color_lum_max,
                    color_cb_min, color_cb_max,
                    color_cr_min, color_cr_max);
#endif

  return img; // snake_gate_detection did not make a new image
}

void snake_gate_detection_start(void){
  time_gate_detected = last_processed = get_sys_time_float();
  gate_detected = 0;
  ready_pass_through = 0;
  listener->active = true;
}

void snake_gate_detection_stop(void){
  gate_detected = 0;
  ready_pass_through = 0;
  listener->active = false;
}

void snake_gate_detection_init(void)
{
  listener = cv_add_to_device(&SGD_CAMERA, snake_gate_detection_func);
  listener->active = false;
}

// state filter in periodic loop
void snake_gate_detection_periodic(void)
{
  // Reinitialization after gate is cleared and turn is made(called from velocity guidance module)
  if (init_pos_filter == 1) {
    init_pos_filter = 0;
    //assumed initial position at other end of the gate
    predicted_x_gate = INITIAL_X;
    predicted_y_gate = INITIAL_Y;
    predicted_z_gate = INITIAL_Z;
  }

  if (time_gate_detected > last_processed) {
    // SAFETY gate_detected
    if (gate_x_dist > 0.6 && gate_x_dist < 5) {
      gate_detected = 1;
      counter_gate_detected = 0;
      gate_processed = 0;
    } else {
      gate_detected = 0;
      counter_gate_detected = 0;
    }

    //convert earth velocity to body x y velocity
    float psi = stateGetNedToBodyEulers_f()->psi;
    body_vx = cosf(psi)*stateGetSpeedNed_f()->x + sinf(psi)*stateGetSpeedNed_f()->y;
    body_vy = -sinf(psi)*stateGetSpeedNed_f()->x + cosf(psi)*stateGetSpeedNed_f()->y;

    //State filter
    float dt = time_gate_detected - last_processed;

    // predict the new location:
    float dx_gate = dt * body_vx; //(cos(current_angle_gate) * gate_turn_rate * current_distance);
    float dy_gate = dt * body_vy; //(velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
    predicted_x_gate = previous_x_gate + dx_gate;
    predicted_y_gate = previous_y_gate + dy_gate;
    predicted_z_gate = previous_z_gate;

    if (gate_detected == 1) {
      // Mix the measurement with the prediction:
      float weight_measurement;
      if (uncertainty_gate > 150) {
        weight_measurement = 1.0f;
        uncertainty_gate = 151;//max
      } else {
        weight_measurement = 0.7;  //(GOOD_FIT-best_quality)/GOOD_FIT;//check constant weight
      }

      filtered_x_gate = weight_measurement * gate_x_dist + (1.0f - weight_measurement) * predicted_x_gate;
      filtered_y_gate = weight_measurement * gate_y_dist + (1.0f - weight_measurement) * predicted_y_gate;
      filtered_z_gate = weight_measurement * gate_z_dist + (1.0f - weight_measurement) * predicted_z_gate;

      // reset uncertainty:
      uncertainty_gate = 0;
    } else {
      // just the prediction
      filtered_x_gate = predicted_x_gate;
      filtered_y_gate = predicted_y_gate;
      filtered_z_gate = predicted_z_gate;

      // increase uncertainty
      uncertainty_gate++;
    }
    // set the previous state for the next time:
    previous_x_gate = filtered_x_gate;
    previous_y_gate = filtered_y_gate;
    previous_z_gate = filtered_z_gate;

    last_processed = time_gate_detected;

    //SAFETY ready_pass_trough
    printf("gate at %f %d %f %f %f %f\n", time_gate_detected - time_tracked, gate_detected, filtered_x_gate, filtered_y_gate, body_vx, body_vy);
    if (gate_detected && fabs(filtered_x_gate - INITIAL_X) < X_POS_MARGIN && fabs(filtered_y_gate - INITIAL_Y) < Y_POS_MARGIN
        && fabs(body_vx) < X_SPEED_MARGIN && fabs(body_vy) < Y_SPEED_MARGIN) {
      if (time_gate_detected - time_tracked > 1.5) {
        ready_pass_through = 1;
      }
    } else {
      time_tracked = time_gate_detected;
      ready_pass_through = 0;
    }
  }
}

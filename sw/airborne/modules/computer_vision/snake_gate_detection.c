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

#include "state.h"
#include "mcu_periph/sys_time.h"

//#include "modules/state_autonomous_race/state_autonomous_race.h"
//#include "modules/flight_plan_in_guided_mode/flight_plan_clock.h"
//#include "modules/state_autonomous_race/state_autonomous_race.h"

#ifndef SGD_CAMERA
#define SGD_CAMERA front_camera
#endif

#define PI 3.1415926

//initial position after gate pass
// this initialises the filter location before positive identification
#define INITIAL_X 2
#define INITIAL_Y 0
#define INITIAL_Z 0

//initial position and speed safety margins
#define X_POS_MARGIN 0.15//m
#define Y_POS_MARGIN 0.5//m
#define Z_POS_MARGIN 0.15//m
#define X_SPEED_MARGIN 0.15//m/s
#define Y_SPEED_MARGIN 0.2//m/s

struct video_listener *listener = NULL;

#define RED 0
#define BLUE 1

#define WINDOW_COLOR RED

// Filter Settings
#if WINDOW_COLOR == RED
uint8_t color_lum_min = 60;// 60;//105;
uint8_t color_lum_max = 255;//228;//205;
uint8_t color_cb_min  = 20;//66;//52;
uint8_t color_cb_max  = 180;//194;//140;
uint8_t color_cr_min  = 150;//140;//180;
uint8_t color_cr_max  = 255;//230;//255;
#else
// TODO find color scheme for blue
uint8_t color_lum_min = 60;// 60;//105;
uint8_t color_lum_max = 255;//228;//205;
uint8_t color_cb_min  = 20;//66;//52;
uint8_t color_cb_max  = 180;//194;//140;
uint8_t color_cr_min  = 150;//140;//180;
uint8_t color_cr_max  = 255;//230;//255;
#endif

// Gate detection settings:
int n_samples = 1000;//1000;//500;
int min_pixel_size = 40;//100;
float min_gate_quality = 0.15;
float gate_thickness = 0;//0.05;//0.10;//

// TODO KIRK find correct
const float gate_size_m = 1.;   // gate size in meters

// Result
#define MAX_GATES 50
struct gate_img gates[MAX_GATES];
struct gate_img best_gate;
struct image_t img_result;
int n_gates = 0;
float best_quality = 0;
float current_quality = 0;
float best_fitness = 100000;

//color picker
uint8_t y_center_picker  = 0;
uint8_t cb_center  = 0;
uint8_t cr_center  = 0;

//camera parameters
// TODO KIRK find correct
#define radians_per_pix_w 0.006666667 //2.1 rad(60deg)/315
#define radians_per_pix_h 0.0065625   //1.05rad / 160

//original settings
/*<define name="MT9F002_OUTPUT_WIDTH" value="160"/><!--128-->
<define name="MT9F002_OUTPUT_HEIGHT" value="315"/>

<define name="MT9F002_OUTPUT_SCALER" value="0.125"/>
<define name="MT9F002_INITIAL_OFFSET_X" value="2050"/>      <!--2034--><!--2050-->

<define name="MT9F002_INITIAL_OFFSET_Y" value="476"/><!--700-->*/

// gate location relative to drone in body frame (assume forward camera)
float x_dist = 0;
float y_dist = 0;
float z_dist = 0;

//state filter
float body_vx = 0;
float body_vy = 0;

float predicted_x_gate = 0;
float predicted_y_gate = 0;
float predicted_z_gate = 0;

float filtered_x_gate = 0;
float filtered_y_gate = 0;
float filtered_z_gate = 0;
float delta_z_gate   = 0;

float previous_x_gate = 0;
float previous_y_gate = 0;
float previous_z_gate = 0;

//SAFETY AND RESET FLAGS
int uncertainty_gate = 0;
int gate_detected = 0;
int counter_gate_detected = 0;
int init_pos_filter = 0;
int safe_pass_counter = 0;
int ready_pass_through;

float gate_quality = 0;

float fps_filter = 0;

// timers
float last_processed, time_gate_detected;

//Debug messages

// Checks for a single pixel if it is the right color
// 1 means that it passes the filter
int check_color(struct image_t *im, int x, int y)
{
  if (x % 2 == 1) { x--; }

  if (x < 0 || x >= im->w || y < 0 || y >= im->h) {
    return 0;
  }

  uint8_t *buf = im->buf;
  buf += 2 * (y * (im->w) + x); // each pixel has two bytes
  // odd ones are uy
  // even ones are vy


  if (
    (buf[1] >= color_lum_min)
    && (buf[1] <= color_lum_max)
    && (buf[0] >= color_cb_min)
    && (buf[0] <= color_cb_max)
    && (buf[2] >= color_cr_min)
    && (buf[2] <= color_cr_max)
  ) {
    // the pixel passes:
    return 1;
  } else {
    // the pixel does not:
    return 0;
  }
}

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
  //pixel distance conversion
  static float hor_angle = 0, vert_angle = 0;

  // calculate angles here, rotate camera pixels 90 deg
  vert_angle = (-(((float)x_pix * 1.0) - ((float)(img->w) / 2.0)) * radians_per_pix_w) -
               (stateGetNedToBodyEulers_f()->theta);
  hor_angle = (((float)y_pix * 1.0) - ((float)(img->h) / 2.0)) * radians_per_pix_h;

  current_quality = gate.gate_q;

  printf("gate size %d %f\n", gate.sz, gate.sz * radians_per_pix_w / 2);

  // in body frame
  x_dist = gate_size_m * 2 / (gate.sz * radians_per_pix_w);
  y_dist = x_dist * sin(hor_angle);
  z_dist = x_dist * sin(vert_angle);
}

//state filter in periodic loop
void snake_gate_periodic(void)
{
  if (time_gate_detected > last_processed) {

    //SAFETY gate_detected
    if (x_dist > 0.6 && x_dist < 5) {
      gate_detected = 1;
      counter_gate_detected = 0;
    } else {
      gate_detected = 0;
      counter_gate_detected = 0;
    }

    if (gate_detected == 1){
      printf("gate at %f %f %f\n", x_dist, y_dist, z_dist);
    }

    //convert earth velocity to body x y velocity
    float psi = stateGetNedToBodyEulers_f()->psi;
    //When using optitrack
    body_vx = cosf(psi)*stateGetSpeedNed_f()->x + sinf(psi)*stateGetSpeedNed_f()->y;
    body_vy = -sinf(psi)*stateGetSpeedNed_f()->x + cosf(psi)*stateGetSpeedNed_f()->y;

    //SAFETY ready_pass_trough
    if (gate_detected == 1 && fabs(x_dist - INITIAL_X) < X_POS_MARGIN && fabs(y_dist - INITIAL_Y) < Y_POS_MARGIN
        && fabs(z_dist - INITIAL_Z) < Z_POS_MARGIN && fabs(body_vx) < X_SPEED_MARGIN
        && fabs(body_vy) < Y_SPEED_MARGIN) {
      safe_pass_counter += 1;
    } else {
      safe_pass_counter = 0;
      ready_pass_through = 0;
    }

    if (safe_pass_counter > 10) {
      safe_pass_counter = 0;
      ready_pass_through = 1;
    }

    // Reinitialization after gate is cleared and turn is made(called from velocity guidance module)
    if (init_pos_filter == 1) {
      init_pos_filter = 0;
      //assumed initial position at other end of the gate
      predicted_x_gate = INITIAL_X;//0;
      predicted_y_gate = INITIAL_Y;//1.5;
    }

    //State filter

    float dt = time_gate_detected - last_processed;

    fps_filter = 1.0 / dt;

    // predict the new location:
    float dx_gate = dt * body_vx; //(cos(current_angle_gate) * gate_turn_rate * current_distance);
    float dy_gate = dt * body_vy; //(velocity_gate - sin(current_angle_gate) * gate_turn_rate * current_distance);
    predicted_x_gate = previous_x_gate + dx_gate;
    predicted_y_gate = previous_y_gate + dy_gate;
    predicted_z_gate = previous_z_gate;

    float sonar_alt = stateGetPositionNed_f()->z;

    if (gate_detected == 1) {
      // Mix the measurement with the prediction:
      float weight_measurement;
      if (uncertainty_gate > 150) {
        weight_measurement = 1.0f;
        uncertainty_gate = 151;//max
      } else {
        weight_measurement = 0.7;  //(GOOD_FIT-best_quality)/GOOD_FIT;//check constant weight
      }

      filtered_x_gate = weight_measurement * x_dist + (1.0f - weight_measurement) * predicted_x_gate;
      filtered_y_gate = weight_measurement * y_dist + (1.0f - weight_measurement) * predicted_y_gate;
      filtered_z_gate = weight_measurement * (z_dist + sonar_alt) + (1.0f - weight_measurement) * predicted_z_gate;

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
    delta_z_gate = filtered_z_gate - sonar_alt;

    last_processed = time_gate_detected;
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
  int filter = 1;
  int gen_alg = 1;
  uint16_t i;
  int x, y;
  int y_low = 0, y_high = 0;
  int x_low1 = 0, x_high1 = 0, x_low2 = 0, x_high2 = 0;
  int sz = 0, szx1 = 0, szx2 = 0;

  float quality;
  best_quality = 0;
  n_gates = 0;

  //color picker
  //check_color_center(img,&y_center_picker,&cb_center,&cr_center);

  for (i = 0; i < n_samples; i++) {
    // get a random coordinate:
    x = rand() % img->w;
    y = rand() % img->h;

    //check_color(img, 1, 1);
    // check if it has the right color
    if (check_color(img, x, y)) {
      // snake up and down:
      snake_up_and_down(img, x, y, &y_low, &y_high);
      sz = y_high - y_low;

      y_low = y_low + (sz * gate_thickness);
      y_high = y_high - (sz * gate_thickness);

      y = (y_high + y_low) / 2;

      // if the stretch is long enough
      if (sz > min_pixel_size) {
        // snake left and right:
        snake_left_and_right(img, x, y_low, &x_low1, &x_high1);
        snake_left_and_right(img, x, y_high, &x_low2, &x_high2);

        x_low1 = x_low1 + (sz * gate_thickness);
        x_high1 = x_high1 - (sz * gate_thickness);
        x_low2 = x_low2 + (sz * gate_thickness);
        x_high2 = x_high2 - (sz * gate_thickness);

        // sizes of the left-right stretches: in y pixel coordinates
        szx1 = (x_high1 - x_low1);
        szx2 = (x_high2 - x_low2);

        // if the size is big enough:
        if (szx1 > min_pixel_size) {
          // draw four lines on the image:
          x = (x_high1 + x_low1) / 2;//was+
          // set the size to the largest line found:
          sz = (sz > szx1) ? sz : szx1;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality);
          gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
            //draw_gate(img, gates[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        } else if (szx2 > min_pixel_size) {
          x = (x_high2 + x_low2) / 2;//was +
          // set the size to the largest line found:
          sz = (sz > szx2) ? sz : szx2;
          // create the gate:
          gates[n_gates].x = x;
          gates[n_gates].y = y;
          gates[n_gates].sz = sz / 2;
          // check the gate quality:
          check_gate(img, gates[n_gates], &quality);
          gates[n_gates].gate_q = quality;
          // only increment the number of gates if the quality is sufficient
          // else it will be overwritten by the next one
          if (quality > best_quality) { //min_gate_quality)
            //draw_gate(img, gates[n_gates]);
            best_quality = quality;
            n_gates++;
          }
        }
        if (n_gates >= MAX_GATES) {
          break;
        }
      }
    }
  }

  // do an additional fit to improve the gate detection:
  if (best_quality > min_gate_quality && n_gates > 0) {
    /*
    // temporary variables:
    float x_center, y_center, radius, fitness, angle_1, angle_2, psi;
    int clock_arms = 1;

    // prepare the Region of Interest (ROI), which is larger than the gate:
    float size_factor = 1.5;//1.25;
    int16_t ROI_size = (int16_t) (((float) gates[n_gates-1].sz) * size_factor);
    int16_t min_x = gates[n_gates-1].x - ROI_size;
    min_x = (min_x < 0) ? 0 : min_x;
    int16_t max_x = gates[n_gates-1].x + ROI_size;
    max_x = (max_x < img->w) ? max_x : img->w;
    int16_t min_y = gates[n_gates-1].y - ROI_size;
    min_y = (min_y < 0) ? 0 : min_y;
    int16_t max_y = gates[n_gates-1].y + ROI_size;
    max_y = (max_y < img->h) ? max_y : img->h;

    draw_gate(img, gates[n_gates-1]);

    // detect the gate:
    gate_detection(img, &x_center, &y_center, &radius, &fitness, &(gates[n_gates-1].x), &(gates[n_gates-1].y), &(gates[n_gates-1].sz),
                    (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi);

    // store the information in the gate:
    gates[n_gates-1].x = (int) x_center;
    gates[n_gates-1].y = (int) y_center;
    gates[n_gates-1].sz = (int) radius;
    */
    // temporary variables:
    float x_center, y_center, radius, fitness, angle_1, angle_2, psi;
    int clock_arms = 1;

    // prepare the Region of Interest (ROI), which is larger than the gate:
    float size_factor = 1.5;//2;//1.25;
    /*int16_t ROI_size = (int16_t) (((float) gates[n_gates-1].sz) * size_factor);
    int16_t min_x = gates[n_gates-1].x - ROI_size;
    min_x = (min_x < 0) ? 0 : min_x;
    int16_t max_x = gates[n_gates-1].x + ROI_size;
    max_x = (max_x < img->w) ? max_x : img->w;
    int16_t min_y = gates[n_gates-1].y - ROI_size;
    min_y = (min_y < 0) ? 0 : min_y;
    int16_t max_y = gates[n_gates-1].y + ROI_size;
    max_y = (max_y < img->h) ? max_y : img->h;*/

    if (gen_alg) {
      int max_candidate_gates = 5;
      best_fitness = 100;
      if (n_gates > 0 && n_gates < max_candidate_gates) {
        for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
          int16_t min_x = gates[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates[gate_nr].x + ROI_size;
          max_x = (max_x < img->w) ? max_x : img->w;
          int16_t min_y = gates[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates[gate_nr].y + ROI_size;
          max_y = (max_y < img->h) ? max_y : img->h;

          //draw_gate(img, gates[gate_nr]);

          int gates_x = gates[gate_nr].x;
          int gates_y = gates[gate_nr].y;
          int gates_sz = gates[gate_nr].sz;

          // detect the gate:
          gate_detection(img, &x_center, &y_center, &radius, &fitness, &gates_x, &gates_y, &gates_sz,
                         min_x, min_y, max_x, max_y, clock_arms, &angle_1, &angle_2, &psi);
          if (fitness < best_fitness) {
            best_fitness = fitness;
            // store the information in the gate:
            best_gate.x = (int) x_center;
            best_gate.y = (int) y_center;
            best_gate.sz = (int) radius;
          }

        }
        for (int gate_nr = 0; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates[gate_nr]);
        }
      } else if (n_gates >= max_candidate_gates) {
        for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          int16_t ROI_size = (int16_t)(((float) gates[gate_nr].sz) * size_factor);
          int16_t min_x = gates[gate_nr].x - ROI_size;
          min_x = (min_x < 0) ? 0 : min_x;
          int16_t max_x = gates[gate_nr].x + ROI_size;
          max_x = (max_x < img->w) ? max_x : img->w;
          int16_t min_y = gates[gate_nr].y - ROI_size;
          min_y = (min_y < 0) ? 0 : min_y;
          int16_t max_y = gates[gate_nr].y + ROI_size;
          max_y = (max_y < img->h) ? max_y : img->h;
          //draw_gate(img, gates[gate_nr]);
          // detect the gate:
          gate_detection(img, &x_center, &y_center, &radius, &fitness, &(gates[gate_nr].x), &(gates[gate_nr].y),
                         &(gates[gate_nr].sz),
                         (uint16_t) min_x, (uint16_t) min_y, (uint16_t) max_x, (uint16_t) max_y, clock_arms, &angle_1, &angle_2, &psi);
          if (fitness < best_fitness) {
            best_fitness = fitness;
            // store the information in the gate:
            best_gate.x = (int) x_center;
            best_gate.y = (int) y_center;
            best_gate.sz = (int) radius;
          }
        }

        for (int gate_nr = n_gates - max_candidate_gates; gate_nr < n_gates; gate_nr += 1) {
          draw_gate(img, gates[gate_nr]);
        }
      }
      //draw_gate(img, best_gate);
    }
  }

  //color filtered version of image for overlay and debugging
  if (filter) {
    //int color_count =
    image_yuv422_colorfilt(img, img,
                      color_lum_min, color_lum_max,
                      color_cb_min, color_cb_max,
                      color_cr_min, color_cr_max);
  }

  if (best_quality > min_gate_quality && n_gates > 0) {
    current_quality = best_quality;
    //draw_gate(img, best_gate);
    gate_quality = gates[n_gates - 1].gate_q;
    //image_yuv422_set_color(img,img,gates[n_gates-1].x,gates[n_gates-1].y);

    //calculate_gate_position(gates[n_gates-1].x,gates[n_gates-1].y,gates[n_gates-1].sz,img,gates[n_gates-1]);
    calculate_gate_position(best_gate.x, best_gate.y, best_gate.sz, img, best_gate);
    time_gate_detected = sec_of_sys_time_ticks(img->pprz_ts);
  } else {
    gate_detected = 0;
    current_quality = 0;
  }
  return img; // snake_gate_detection did not make a new image
}


void draw_gate(struct image_t *im, struct gate_img gate)
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

extern void check_gate(struct image_t *im, struct gate_img gate, float *quality)
{
  int n_points, n_colored_points;
  n_points = 0;
  n_colored_points = 0;
  int np, nc;

  // check the four lines of which the gate consists:
  struct point_t from, to;
  from.x = gate.x - gate.sz;
  from.y = gate.y - gate.sz;
  to.x = gate.x - gate.sz;
  to.y = gate.y + gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x - gate.sz;
  from.y = gate.y + gate.sz;
  to.x = gate.x + gate.sz;
  to.y = gate.y + gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y + gate.sz;
  to.x = gate.x + gate.sz;
  to.y = gate.y - gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np;
  n_colored_points += nc;

  from.x = gate.x + gate.sz;
  from.y = gate.y - gate.sz;
  to.x = gate.x - gate.sz;
  to.y = gate.y - gate.sz;
  check_line(im, from, to, &np, &nc);
  n_points += np;
  n_colored_points += nc;

  // the quality is the ratio of colored points / number of points:
  if (n_points == 0) {
    (*quality) = 0;
  } else {
    (*quality) = ((float) n_colored_points) / ((float) n_points);
  }
}

void check_line(struct image_t *im, struct point_t Q1, struct point_t Q2, int *n_points, int *n_colored_points)
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

void snake_up_and_down(struct image_t *im, int x, int y, int *y_low, int *y_high)
{
  int done = 0;
  int x_initial = x;
  (*y_low) = y;

  // snake towards negative y (down?)
  while ((*y_low) > 0 && !done) {
    if (check_color(im, x, (*y_low) - 1)) {
      (*y_low)--;
    } else if (check_color(im, x + 1, (*y_low) - 1)) {
      x++;
      (*y_low)--;
    } else if (check_color(im, x - 1, (*y_low) - 1)) {
      x--;
      (*y_low)--;
    } else {
      done = 1;
    }
  }

  x = x_initial;
  (*y_high) = y;
  done = 0;
  // snake towards positive y (up?)
  while ((*y_high) < im->h - 1 && !done) {

    if (check_color(im, x, (*y_high) + 1)) {
      (*y_high)++;
    } else if (x < im->w - 1 && check_color(im, x + 1, (*y_high) + 1)) {
      x++;
      (*y_high)++;
    } else if (x > 0 && check_color(im, x - 1, (*y_high) + 1)) {
      x--;
      (*y_high)++;
    } else {
      done = 1;
    }
  }
}

void snake_left_and_right(struct image_t *im, int x, int y, int *x_low, int *x_high)
{
  int done = 0;
  int y_initial = y;
  (*x_low) = x;

  // snake towards negative x (left)
  while ((*x_low) > 0 && !done) {
    if (check_color(im, (*x_low) - 1, y)) {
      (*x_low)--;
    } else if (y < im->h - 1 && check_color(im, (*x_low) - 1, y + 1)) {
      y++;
      (*x_low)--;
    } else if (y > 0 && check_color(im, (*x_low) - 1, y - 1)) {
      y--;
      (*x_low)--;
    } else {
      done = 1;
    }
  }

  y = y_initial;
  (*x_high) = x;
  done = 0;
  // snake towards positive x (right)
  while ((*x_high) < im->w - 1 && !done) {

    if (check_color(im, (*x_high) + 1, y)) {
      (*x_high)++;
    } else if (y < im->h - 1 && check_color(im, (*x_high) + 1, y++)) {
      y++;
      (*x_high)++;
    } else if (y > 0 && check_color(im, (*x_high) + 1, y - 1)) {
      y--;
      (*x_high)++;
    } else {
      done = 1;
    }
  }
}

void snake_gate_detection_init(void)
{
  listener = cv_add_to_device(&SGD_CAMERA, snake_gate_detection_func);
  time_gate_detected = last_processed = get_sys_time_float();
}

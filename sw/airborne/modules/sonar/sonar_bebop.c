/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/sonar/sonar_bebop.c
 *  @brief Parrot Bebop Sonar driver
 */

#include "sonar_bebop.h"
#include "generated/airframe.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/spi.h"
#include "subsystems/abi.h"
#include <pthread.h>
#include "subsystems/datalink/downlink.h"

#ifdef SITL
#include "state.h"
#endif

static bool obstacle_mode = false;


struct SonarBebop sonar_bebop;
static uint8_t sonar_bebop_spi_d[16] = {0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                        0x00, 0x00, 0x00};
static struct spi_transaction sonar_bebop_spi_t;
static pthread_t sonar_bebop_thread;
static void *sonar_bebop_read(void *data);

void sonar_bebop_init(void) {
  sonar_bebop.meas = 0;
  sonar_bebop.offset = 0;
  sonar_bebop_spi_t.status = SPITransDone;
  sonar_bebop_spi_t.select = SPISelectUnselect;
  sonar_bebop_spi_t.dss = SPIDss8bit;
  sonar_bebop_spi_t.output_buf = sonar_bebop_spi_d;
  sonar_bebop_spi_t.output_length = 16;
  sonar_bebop_spi_t.input_buf = NULL;
  sonar_bebop_spi_t.input_length = 0;
  sonar_bebop.current_alg = 0;

  int rc = pthread_create(&sonar_bebop_thread, NULL, sonar_bebop_read, NULL);
  if (rc < 0) {
    return;
  }
}

static void increment_circular_index(uint8_t *index, uint8_t limit){
  *index = (*index + 1) % limit;
}

static uint8_t get_index_from_offset(uint8_t index, int8_t offset, uint8_t limit){
  return (index - offset + limit) % limit;
}

void sonar_obstacle_detect_on(void)
{
  obstacle_mode = true;
}
void sonar_obstacle_detect_off(void)
{
  obstacle_mode = false;
}


#define SLOPE_THRESH_OBS 6.        // gradient threshold for outlier detections when trying to hold altitude [m/s]
#define SLOPE_THRESH 6.        // gradient threshold for outlier detections when changing altitude [m/s]
#define OBSTACLE_ACCEPTANCE 4   // number of outliers before it is accepted as an obstacle
#define DT 0.01                 // sample period of sensor (100 Hz)


/**
 * Read ADC value to update sonar measurement
 */
static void *sonar_bebop_read(void *data __attribute__((unused))) {
  while (true) {

#ifndef SITL

    uint16_t i;
    uint16_t adc_buffer[8192];
    static float current_obstacle_height = 0.;
    static float prev_sent_distance = 0.;
    static float prev_meas_distances[OBSTACLE_ACCEPTANCE] = {0.};
    static uint8_t curr_meas_index = 0;
    static bool outlier_detected = false;
    static int k_outliers = 0;
    static int sonar_diff_pos = 0;
    static int sonar_diff_neg = 0;

    /* Start ADC and send sonar output */
    adc_enable(&adc0, 1);
    sonar_bebop_spi_t.status = SPITransDone;
    spi_submit(&spi0, &sonar_bebop_spi_t);
    while (sonar_bebop_spi_t.status != SPITransSuccess);
    adc_read(&adc0, adc_buffer, 8192);
    adc_enable(&adc0, 0);

    /* Find the peeks */
    uint16_t start_send = 0;
    uint16_t stop_send = 0;
    uint16_t first_peek = 0;
    uint16_t lowest_value = 4095;

    for (i = 0; i < 8192; i++) {
      uint16_t adc_val = adc_buffer[i] >> 4;
      if (start_send == 0 && adc_val == 4095)
        start_send = i;
      else if (start_send != 0 && stop_send == 0 && adc_val != 4095) {
        stop_send = i - 1;
        i += 300;
        continue;
      }

      if (start_send != 0 && stop_send != 0 && first_peek == 0 && adc_val < lowest_value)
        lowest_value = adc_val;
      else if (start_send != 0 && stop_send != 0 && adc_val > lowest_value + 100) {
        first_peek = i;
        lowest_value = adc_val - 100;
      }
      else if (start_send != 0 && stop_send != 0 && first_peek != 0 && adc_val + 100 < (adc_buffer[first_peek] >> 4)) {
        break;
      }
    }

    /* Calculate the distance from the peeks */
    uint16_t diff = stop_send - start_send;
    int16_t peek_distance = first_peek - (stop_send - diff / 2);

    if (first_peek <= stop_send || diff > 250) {
      peek_distance = 0;
    }

    sonar_bebop.distance = peek_distance / 1000.0;

    if (obstacle_mode == true) {
      float sonar_diff = (sonar_bebop.distance - prev_sent_distance);
      if (fabs(sonar_diff) / ((k_outliers + 1) * DT) > SLOPE_THRESH_OBS) {
        // sharp outlier detected, track to determine if noise or obstacle
        outlier_detected = true;
        k_outliers++;

        //Find the sign convention of difference
        if (sonar_diff > 0) {
          sonar_diff_pos++;
        }
        else {
          sonar_diff_neg++;
        }

      }
      else {
        // no noise or new obstacle
        outlier_detected = false;
        k_outliers = 0;
        sonar_diff_pos = 0;
        sonar_diff_neg = 0;
      }

      // Analyse new data point for outlier and obstacle identification
      if ((k_outliers >= OBSTACLE_ACCEPTANCE) &&
          ((sonar_diff_pos >= OBSTACLE_ACCEPTANCE - 1) || (sonar_diff_neg >= OBSTACLE_ACCEPTANCE - 1))) {
        outlier_detected = false;
        k_outliers = 0;
        sonar_diff_pos = 0;
        sonar_diff_neg = 0;
        current_obstacle_height += prev_sent_distance - sonar_bebop.distance;    // use average from prev_distance
        // assume small obstacles are just the ground, this will help reset any errors in the obstacle height estimation
        if (fabs(current_obstacle_height) < 0.1) {
          current_obstacle_height = 0.;
        }
      }
    }
    else // We use this mode if we are trying to change the altitude
    {
      current_obstacle_height = 0;
      float sonar_diff = (sonar_bebop.distance - prev_sent_distance);
      if (fabs(sonar_diff) / ((k_outliers + 1) * DT) > SLOPE_THRESH) {
        // sharp outlier detected, noise
        outlier_detected = true;
        k_outliers++;
      }
      else {
        // no noise
        outlier_detected = false;
        k_outliers = 0;
      }
    }


#else // SITL
    sonar_bebop.distance = stateGetPositionEnu_f()->z;
    Bound(sonar_bebop.distance, 0.1f, 7.0f);
    uint16_t peek_distance = 1;
#endif // SITL

    if (peek_distance > 0 && !outlier_detected) {
      prev_sent_distance = sonar_bebop.distance;
      sonar_bebop.current_alg = sonar_bebop.distance + current_obstacle_height;
      // Send ABI message
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, sonar_bebop.current_alg);

#ifdef SENSOR_SYNC_SEND_SONAR
      // Send Telemetry report
      DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, &sonar_bebop.distance, &current_obstacle_height, &sonar_bebop.current_alg, &k_outliers, &obstacle_mode);
#endif
    }
  }
  usleep(10000); //100Hz
  return NULL;
}



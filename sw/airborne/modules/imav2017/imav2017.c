/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/imav2017/imav2017.c"
 * @author Kirk Scheper
 * 
 */

#include "modules/imav2017/imav2017.h"

#include "filters/median_filter.h"
#include "subsystems/datalink/downlink.h"

// output
float gate_distance, gate_y_offset;

// median filter
struct MedianFilterFloat psi_filter, theta_filter, depth_filter, w_filter;
float psi_f, theta_f, depth_f, w_f;

void imav2017_init(void)
{
  init_median_filter_f(&psi_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&theta_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&depth_filter, MEDIAN_DEFAULT_SIZE);
  init_median_filter_f(&w_filter, MEDIAN_DEFAULT_SIZE);
}

static const float gate_size_m = 1.1f;
void imav2017_set_gate(uint8_t quality, float w, float h,
    float psi, float theta, float depth)
{
  // filter incoming angles and depth
  psi_f = update_median_filter_f(&psi_filter, psi);
  theta_f = update_median_filter_f(&theta_filter, theta);
  depth_f = update_median_filter_f(&depth_filter, depth);
  w_f = update_median_filter_f(&depth_filter, w);

  gate_distance = gate_size_m / w_f;
  gate_y_offset = gate_distance * sinf(psi_f);

  float q = (float)quality;
  //DOWNLINK_SEND_TEMP_ADC(DOWNLINK_TRANSPORT, DOWNLINK_DEVICE, &psi_f, &gate_y_offset, &gate_distance);
}

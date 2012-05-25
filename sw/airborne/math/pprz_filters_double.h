/*
* This file is part of Paparazzi.
 *
 * Copyright (C) 2012 Pranay Sinha, transition Robotics, Inc. <psinha@transition-robotics.com>
 * Copyright (C) 2010-2011 Greg Horn <ghorn@stanford.edu>
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * pprz_filter_double.h
 */

#include <math.h>
#include <stdio.h>


#ifndef __PPRZ_FILTERS_DOUBLE_H__
#define __PPRZ_FILTERS_DOUBLE_H__



/* 
* Some exponential IIR-based filters
* You provide the time period between each sample (dt)
* as well as the time period corresponding to the cutoff frequency (tau)
*/

static inline void
expo_low_pass(double dt, double tau, double *output, double input)
{
  double emdt = exp(-dt/tau);
  *output = (*output)*emdt + input*(1-emdt);
}

static inline void
expo_high_pass(double dt, double tau, double *output, double input)
{
  double emdt = exp(-dt/tau);
  *output = input - ((*output)*emdt + input*(1-emdt));
}

static inline void
expo_band_pass(double dt, double tau_lo, double tau_hi, double *output, double input)
{
  double emdt_lo = exp(-dt/tau_lo);
  double emdt_hi = exp(-dt/tau_hi);
  *output = ((*output)*emdt_hi + input*(1-emdt_hi)) - ((*output)*emdt_lo + input*(1-emdt_lo));
}

static inline void
expo_notch(double dt, double tau_lo, double tau_hi, double *output, double input)
{
  double emdt_lo = exp(-dt/tau_lo);
  double emdt_hi = exp(-dt/tau_hi);
  *output = input - (((*output)*emdt_hi + input*(1-emdt_hi)) - ((*output)*emdt_lo + input*(1-emdt_lo)));
}



/* 
* Some computationally simpler versions of the IIR-type
* You provide the cutoff fraction of normalised transition frequency (ft)
* Where cutoff_frac = Cut-off frequency / Sampling frequency
*/

static inline void
simple_low_pass(double cutoff_frac, double *output, double input)
{
  *output += cutoff_frac*(input - *output);
}

static inline void
simple_high_pass(double cutoff_frac, double *output, double input)
{
  *output = input - (*output + cutoff_frac*(input - *output));
}

static inline void
simple_band_pass(double cutoff_frac_lo, double cutoff_frac_hi, double *output, double input)
{
  *output = (cutoff_frac_hi-cutoff_frac_lo)(input - *output);
}

static inline void
simple_notch(double cutoff_frac_lo, double cutoff_frac_hi, double *output, double input)
{
  *output = input - (cutoff_frac_hi-cutoff_frac_lo)(input - *output);
}


/*
* Now lets try some moving average filters
* 
*/

#ifndef MAX_MOVING_AVERAGE_WINDOW_SIZE
#define MAX_MOVING_AVERAGE_WINDOW_SIZE 100
#endif

struct moving_average {
  double sum;                      			/* Sum of samples in buffer (avg = sum / window_size)   */
  double values[MAX_MOVING_AVERAGE_WINDOW_SIZE];	/* Buffer for sample values                             */
  uint8_t  head;                     			/* Position index of write head in buffer               */
  uint32_t  window_size;             			/* Number of samples to use in buffer (used for avg)    */
  double current_average;             			/* Current average				        */
};
	

static inline void 
simple_moving_average(struct moving_average * output, double input)
{
  uint8_t new_head = output->head + 1;
  if (new_head >= output->window_size) {
    new_head=0;
    }
  output->sum -= output->values[new_head];
  output->values[new_head] = input;
  output->sum += input;
  output->head = new_head;
  output->current_average = output->sum / output->window_size;
} 

static inline void 
cumulative_moving_average(int32_t *sample_counter, double *output, double input)
{
  double advance_ratio = *sample_counter/(*sample_counter+1);
  *output = *output * advance_ratio + input*(1-advance_ratio);
  *sample_counter++;
} 

static inline void
exponential_moving_average(double dt, double tau, double *time, double *output, double input)
{
  double emdt = exp(-*time/tau);
  *output = (*output)*emdt + input*(1-emdt);
  *time += dt;
}


/*
* Here weight is the fraction by which each incoming value is weighted for the purposes of averageing.
*/

static inline void
weighted_moving_average(double weight, double *output, double input)
{
  *output = (*output)*(1-weight) + input*weight;
}












#endif //__PPRZ_FILTERS_DOUBLE_H__

/*
 * $Id: OPTICFLOW_module.h 3079 2009-03-11 16:55:42Z gautier $
 *
 * Copyright (C) 2009  Gautier Hattenberger
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

/** \file OPTICFLOW_module.h
 *
 * OPTICFLOW module with blinking LEDs
 */

#ifndef OPTICFLOW_MODULE_H
#define OPTICFLOW_MODULE_H

#include "std.h"
#include <inttypes.h>

// ADC Result
extern uint16_t adc_opticflow;

// Optic Flow Settings
extern uint16_t adc_opticflow_catastrophic;
extern uint16_t adc_opticflow_low;

extern float opticflow_climb_rate;
extern float opticflow_descend_rate;

extern uint8_t opticflow_status;
extern float max_opticflow_phi;
extern float max_opticflow_theta;
extern float opticflow_ref_alt;
extern uint8_t wp_af;
extern uint8_t wp_border;


// Module Functions
void init_opticflow(void);
void periodic_opticflow(void);
void start_opticflow(void);
void stop_opticflow(void);
bool_t opticflow_setup(uint8_t, uint8_t);
#endif


/*
 * Copyright (C) Bas Pijnacker Hordijk
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
 * @file "modules/event_opticflow/event_opticflow.h"
 * @author Bas Pijnacker Hordijk
 * Event based opticflow using DVS camera
 */

#ifndef EVENT_OPTICFLOW_H
#define EVENT_OPTICFLOW_H

#include <inttypes.h>
#include "flow_field_estimation.h"

// Guidance definitions
//#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Module state (extern here, since the high speed logger module uses the info in this state
struct module_state {
  struct flowField field;
  struct flowStats stats;
  struct FloatRates ratesMA;
  float z_NED;
  float lastTime;
  float moduleFrequency;
  enum updateStatus status;
};

extern struct module_state eofState;

// Setting variables
extern uint8_t useNormalFlow;
extern uint8_t enableDerotation;
extern float statsFilterTimeConstant;
extern float minPosVariance;
extern float minSpeedVariance;
extern float minEventRate;

// Module main functions
extern void event_optic_flow_init(void);
extern void event_optic_flow_start(void);
extern void event_optic_flow_periodic(void);
extern void event_optic_flow_stop(void);

#endif


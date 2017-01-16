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
 * Event based optic flow detection and control using the Dynamic Vision Sensor (DVS).
 * Implementation is based on the following:
 * - The MAV used in this application is a customized MavTec drone on which the
 *    DVS is mounted, facing downwards.
 * - The DVS is connected through USB to an Odroid XU4 board which reads
 *    the raw event input.
 * - The Odroid processes and filters the input into 'optic flow events'.
 * - These new events are sent through UART to the Paparazzi autopilot.
 * - Real-time data is logged in Paparazzi to an SD card by the 'high speed logger' module.
 */

#ifndef EVENT_OPTICFLOW_H
#define EVENT_OPTICFLOW_H

#include <inttypes.h>
#include "flow_field_estimation.h"

// Guidance definitions (DISABLED: USE OPTICAL_FLOW_LANDING NOW)
//#define GUIDANCE_H_MODE_MODULE_SETTING GUIDANCE_H_MODE_HOVER
//#define GUIDANCE_V_MODE_MODULE_SETTING GUIDANCE_V_MODE_MODULE

// Module state (extern here, since the high speed logger module uses the info in this state
struct module_state {
  struct  flowField field;
  struct  flowStats stats;
  struct  FloatRates ratesMA;
  float   z_NED;
  float   wxTruth, wyTruth, DTruth;
  float   lastTime;
  float   moduleFrequency;
  enum    updateStatus status;
  bool    caerInputReceived;
  int32_t NNew;

  bool    controlReset;
  bool    landing;
  bool    divergenceUpdated;
  float   divergenceControlLast;
  int32_t controlThrottleLast;
  int32_t nominalThrottleEnter;
};

extern struct module_state eofState;

// Setting variables
extern uint8_t enableDerotation;
extern float filterTimeConstant;
extern float inlierMaxDiff;
extern float derotationMovingAverageFactor;
extern float minPosVariance;
extern float minEventRate;
extern float minR2;
extern bool irLedSwitch;
extern float divergenceControlGainP;
extern float divergenceControlSetpoint;
extern float divergenceControlHeightLimit;
extern uint8_t divergenceControlUseVision;

// Module main functions
extern void event_optic_flow_init(void);
extern void event_optic_flow_start(void);
extern void event_optic_flow_periodic(void);
extern void event_optic_flow_stop(void);

// Vertical control loop functions (DISABLED: USE OPTICAL_FLOW_LANDING NOW)
//extern void guidance_v_module_init(void);
//extern void guidance_v_module_enter(void);
//extern void guidance_v_module_run(bool in_flight);

#endif


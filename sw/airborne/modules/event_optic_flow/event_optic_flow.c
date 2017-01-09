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
 * @file "modules/event_optic_flow/event_optic_flow.c"
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

#include "event_optic_flow.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_float.h"
#include "state.h"

#include "paparazzi.h"
#include "firmwares/rotorcraft/stabilization.h"
//#include "firmwares/rotorcraft/guidance/guidance_v.h"


#ifndef DVS_PORT
#error Please define UART port connected to the DVS128 event-based camera. e.g <define name="DVS_PORT" value="uart0"/>
#endif

// Module settings
#ifndef EOF_ENABLE_DEROTATION
#define EOF_ENABLE_DEROTATION 1
#endif
PRINT_CONFIG_VAR(EOF_ENABLE_DEROTATION)

#ifndef EOF_FILTER_TIME_CONSTANT
#define EOF_FILTER_TIME_CONSTANT 0.02f
#endif
PRINT_CONFIG_VAR(EOF_FILTER_TIME_CONSTANT)

#ifndef EOF_INLIER_MAX_DIFF
#define EOF_INLIER_MAX_DIFF 0.3f
#endif
PRINT_CONFIG_VAR(EOF_INLIER_MAX_DIFF)

#ifndef EOF_DEROTATION_MOVING_AVERAGE_FACTOR
#define EOF_DEROTATION_MOVING_AVERAGE_FACTOR 1.0f
#endif
PRINT_CONFIG_VAR(EOF_DEROTATION_MOVING_AVERAGE_FACTOR)

#ifndef EOF_MIN_EVENT_RATE
#define EOF_MIN_EVENT_RATE 1500.0f
#endif
PRINT_CONFIG_VAR(EOF_MIN_EVENT_RATE)

#ifndef EOF_MIN_POSITION_VARIANCE
#define EOF_MIN_POSITION_VARIANCE 600.0f
#endif
PRINT_CONFIG_VAR(EOF_MIN_POSITION_VARIANCE)

#ifndef EOF_MIN_R2
#define EOF_MIN_R2 1.0f
#endif
PRINT_CONFIG_VAR(EOF_MIN_R2)

#ifndef EOF_DIVERGENCE_CONTROL_PGAIN
#define EOF_DIVERGENCE_CONTROL_PGAIN 0.15f
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_PGAIN)

#ifndef EOF_DIVERGENCE_CONTROL_DIV_SETPOINT
#define EOF_DIVERGENCE_CONTROL_DIV_SETPOINT 0.3f
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_DIV_SETPOINT)

#ifndef EOF_DIVERGENCE_CONTROL_HEIGHT_LIMIT
#define EOF_DIVERGENCE_CONTROL_HEIGHT_LIMIT 0.5f
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_HEIGHT_LIMIT)

#ifndef EOF_DIVERGENCE_CONTROL_USE_VISION
#define EOF_DIVERGENCE_CONTROL_USE_VISION 1
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_USE_VISION)

#ifndef EOF_CONTROL_HOVER
#define EOF_CONTROL_HOVER 0
#endif

#ifndef EOF_CONTROL_LANDING
#define EOF_CONTROL_LANDING 0
#endif

#define IR_LEDS_SWITCH 0

#ifndef OPTICFLOW_SENDER_ID
#define OPTICFLOW_SENDER_ID 1
#endif

/**************
 * DEFINITIONS *
 ***************/

// State definition
struct module_state eofState;

// Sensing parameters
uint8_t enableDerotation = EOF_ENABLE_DEROTATION;
float filterTimeConstant = EOF_FILTER_TIME_CONSTANT;
float inlierMaxDiff = EOF_INLIER_MAX_DIFF;
float derotationMovingAverageFactor = EOF_DEROTATION_MOVING_AVERAGE_FACTOR;

// Confidence thresholds
float minPosVariance = EOF_MIN_POSITION_VARIANCE;
float minEventRate = EOF_MIN_EVENT_RATE;
float minR2 = EOF_MIN_R2;

// Control parameters
float divergenceControlGainP = EOF_DIVERGENCE_CONTROL_PGAIN;
float divergenceControlSetpoint = EOF_DIVERGENCE_CONTROL_DIV_SETPOINT;
float divergenceControlHeightLimit = EOF_DIVERGENCE_CONTROL_HEIGHT_LIMIT;
uint8_t divergenceControlUseVision = EOF_DIVERGENCE_CONTROL_USE_VISION;

// Logging controls
bool irLedSwitch = IR_LEDS_SWITCH;

// Constants
const int32_t MAX_NUMBER_OF_UART_EVENTS = 100;
const float MOVING_AVERAGE_MIN_WINDOW = 5.0f;
const uint8_t EVENT_SEPARATOR = 255;
const float UART_INT16_TO_FLOAT = 10.0f;
const float LENS_DISTANCE_TO_CENTER = 0.13f; // approximate distance of lens focal length to center of OptiTrack markers
const uint32_t EVENT_BYTE_SIZE = 13; // +1 for separator
const float inactivityDecayFactor = 0.8f;
const float power = 1;
const float LANDING_THRUST_FRACTION = 0.95f; //TODO find MAVTEC value
const float CONTROL_CONFIDENCE_LIMIT = 0.2f;
const float CONTROL_CONFIDENCE_MAX_DT = 0.2f;

// SWITCH THIS ON TO ENABLE CONTROL THROTTLE
const bool ASSIGN_CTRL = false; //TODO Strange, had to rename this to make code compatible with optical_flow_landing

// Camera intrinsic parameters
const struct cameraIntrinsicParameters dvs128Intrinsics = {
    .principalPointX = 76.70f,
    .principalPointY = 56.93f,
    .focalLengthX = 115.0f,
    .focalLengthY = 115.0f
};

// Internal function declarations (definitions below)
enum updateStatus processUARTInput(struct flowStats* s, int32_t* N);
static void sendFlowFieldState(struct transport_tx *trans, struct link_device *dev);
int16_t uartGetInt16(struct uart_periph *p);
int32_t uartGetInt32(struct uart_periph *p);
void divergenceControlReset(void);


/*************************
 * MAIN SENSING FUNCTIONS *
 *************************/
void event_optic_flow_init(void) {
  register_periodic_telemetry(DefaultPeriodic,
      PPRZ_MSG_ID_EVENT_OPTIC_FLOW_EST, sendFlowFieldState);
}

void event_optic_flow_start(void) {
  // Timing
  eofState.lastTime = get_sys_time_float();
  // Reset low pass filter for rates
  eofState.ratesMA.p = 0;
  eofState.ratesMA.q = 0;
  eofState.ratesMA.r = 0;
  // (Re-)initialization
  eofState.moduleFrequency = 100.0f;
  eofState.z_NED = 0.0f;
  eofState.wxTruth = 0.0f;
  eofState.wyTruth = 0.0f;
  eofState.DTruth = 0.0f;
  struct flowField field = {0., 0., 0., 0., 0., 0.,0};
  eofState.field = field;
  flowStatsInit(&eofState.stats);
  eofState.caerInputReceived = false;
}

void event_optic_flow_periodic(void) {
  struct FloatRates *rates = stateGetBodyRates_f();
  // Moving average filtering of body rates
  eofState.ratesMA.p += (rates->p - eofState.ratesMA.p) * derotationMovingAverageFactor;
  eofState.ratesMA.q += (rates->q - eofState.ratesMA.q) * derotationMovingAverageFactor;
  eofState.ratesMA.r += (rates->r - eofState.ratesMA.r) * derotationMovingAverageFactor;

  // Obtain UART data if available
  int32_t NNew;
  enum updateStatus status = processUARTInput(&eofState.stats, &NNew);

  // Timing bookkeeping, do this after the most uncertain computations,
  // but before operations where timing info is necessary
  float currentTime = get_sys_time_float();
  float dt = currentTime - eofState.lastTime;
  eofState.moduleFrequency = 1.0f/dt;
  eofState.lastTime = currentTime;
  eofState.stats.eventRate = (float) NNew / dt;
  float filterFactor = dt/filterTimeConstant;
  if (filterFactor < 0.01f) {
    filterFactor = 0.01f; // always perform a minimal update
  }
  if (filterFactor > 1.0f) {
    filterFactor = 1.0f;
  }

  if (status == UPDATE_STATS) {
    // If new events are received, recompute flow field
    // In case the flow field is ill-posed, do not update
    status = recomputeFlowField(&eofState.field, &eofState.stats,filterFactor,
        inlierMaxDiff, minEventRate, minPosVariance, minR2, power, dvs128Intrinsics);
  }

  // If no update has been performed, decay flow field parameters towards zero
  if (status != UPDATE_SUCCESS) {
    eofState.field.confidence = 0;
  }
  else {
    // Assign timestamp to last update
    eofState.field.t = currentTime;
    // Allow controller to update
//    eofState.divergenceUpdated = true;
    uint32_t now_ts = get_sys_time_usec();
    AbiSendMsgOPTICAL_FLOW(OPTICFLOW_SENDER_ID, now_ts,
        0,//FIXME only divergence is sent now
        0,
        0,
        0,
        0,
        eofState.field.D,
        0.0);
  }
  // Set  status globally
  eofState.status = status;

  // Reset sums for next iteration
  int32_t i;
  float retainFactor = 1.0f - filterFactor;
  for (i = 0; i < N_FIELD_DIRECTIONS; i++) {
    eofState.stats.sumS [i] *= retainFactor;
    eofState.stats.sumSS[i] *= retainFactor;
    eofState.stats.sumV [i] *= retainFactor;
    eofState.stats.sumVV[i] *= retainFactor;
    eofState.stats.sumSV[i] *= retainFactor;
    eofState.stats.N[i] *= retainFactor;
  }

  //TODO get rid of wx/wyDerotated correctly; they are no longer necessary
  eofState.field.wxDerotated = eofState.field.wx;
  eofState.field.wyDerotated = eofState.field.wy;

  // Update height/ground truth speeds from Optitrack
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();

  //  struct FloatRMat *rot = stateGetNedToBodyRMat_f();
  struct FloatEulers *ang = stateGetNedToBodyEulers_f();
  eofState.z_NED = pos->z; // for downlink

  //TODO implement transformation below for orientation corrected ground truth
  /*struct NedCoor_f velB;
  // Transformation of speeds to body frame
  velB.x = rot->m[0][0] * vel->x + rot->m[0][1] * vel->y + rot->m[0][2] * vel->z;
  velB.y = rot->m[1][0] * vel->x + rot->m[1][1] * vel->y + rot->m[1][2] * vel->z;
  velB.z = rot->m[2][0] * vel->x + rot->m[2][1] * vel->y + rot->m[2][2] * vel->z;
  float R = -pos->z/(cosf(ang->theta)*cosf(ang->phi));*/

  eofState.wxTruth = (vel->y*cosf(ang->psi) -vel->x*sinf(ang->psi)) / (pos->z - 0.01);
  eofState.wyTruth = (vel->x*cosf(ang->psi) +vel->y*sinf(ang->psi)) / (pos->z - 0.01);
  eofState.DTruth = -vel->z / (pos->z - 0.01);

  // Set hover control signals (not used for now)
  if (EOF_CONTROL_HOVER) {

    // Assuming a perfectly aligned downward facing camera,
    // the camera X-axis is opposite to the body Y-axis
    // and the Y-axis is aligned to its X-axis
    // Further assumption: body Euler angles are small
    float vxB = eofState.z_NED * -eofState.field.wyDerotated;
    float vyB = eofState.z_NED * -eofState.field.wxDerotated;
    float vzB = eofState.z_NED * eofState.field.D;
    uint32_t timestamp = get_sys_time_usec();

    // Update control state
    AbiSendMsgVELOCITY_ESTIMATE(1, timestamp, vxB, vyB, vzB, 0);
  }

  // TEST CONTROLLER MAINLOOP
  //guidance_v_module_run(true);
}

void event_optic_flow_stop(void) {
  //TODO is now present as dummy, may be removed if not required
}

/*******************************
 * VERTICAL GUIDANCE FUNCTIONS *
 *******************************/
/*
void UNUSED_guidance_v_module_init() {
  //TODO is this part necessary?
  divergenceControlReset();
  eofState.nominalThrottleEnter = guidance_v_nominal_throttle * MAX_PPRZ;
  eofState.controlThrottleLast = eofState.nominalThrottleEnter; // set to nominal
}

void UNUSED_guidance_v_module_enter() {
  divergenceControlReset();
}

void UNUSED_guidance_v_module_run(bool in_flight) {
  if (!in_flight) {
    // When not flying and in mode module:
    // Reset state
    divergenceControlReset();
  }
  else {

     // UPDATE

    if (divergenceControlUseVision) {
      // Use latest divergence estimate
      if (eofState.divergenceUpdated) {
        eofState.divergenceControlLast = eofState.field.D;
        eofState.divergenceUpdated = false;
      }
      else {
        // after re-entering the module, the divergence should be equal to the set point:
        if (eofState.controlReset) {
          eofState.divergenceControlLast = divergenceControlSetpoint;
          int32_t nominal_throttle = eofState.nominalThrottleEnter;
          if (ASSIGN_CONTROL) {
            stabilization_cmd[COMMAND_THRUST] = nominal_throttle;
          }
          eofState.controlReset = false;
          eofState.controlThrottleLast = nominal_throttle;
        }
        // else: do nothing
        return;
      }
    }
    else {
      // Use ground truth divergence
      // Update height/ground truth speeds from Optitrack
      struct NedCoor_f *pos = stateGetPositionNed_f();
      struct NedCoor_f *vel = stateGetSpeedNed_f();
      float DTruth = -vel->z / (pos->z - 0.01);
      float deltaD = DTruth-eofState.divergenceControlLast;
      // Cap update rate to prevent outliers
      Bound(deltaD, -inlierMaxDiff, inlierMaxDiff);
      eofState.divergenceControlLast += deltaD;
      eofState.DTruth = eofState.divergenceControlLast;
    }

    // Cap divergence in case of unreliable values
    float divergenceLimit = 1.5;
    Bound(eofState.divergenceControlLast, -divergenceLimit, divergenceLimit);

    // CONTROL

    int32_t nominalThrottle = eofState.nominalThrottleEnter;

    // landing indicates whether the drone is already performing a final landing procedure (flare):
    //    if (!eofState.landing) {
    // use the divergence for control:
    float err = divergenceControlSetpoint - eofState.divergenceControlLast;
    // Negative P-gain - positive control yields negative increase in div
    int32_t thrust = nominalThrottle - divergenceControlGainP * err * MAX_PPRZ;

    //        if (eofState.z_NED > -divergenceControlHeightLimit) {
    //        // land by setting 90% nominal thrust:
    //          eofState.landing = true;
    //          thrust = LANDING_THRUST_FRACTION * nominalThrottle;
    //        }
    // bound thrust:
    Bound(thrust, 0.2 * MAX_PPRZ, MAX_PPRZ);
    if (ASSIGN_CONTROL){
      stabilization_cmd[COMMAND_THRUST] = thrust;
    }
    eofState.controlThrottleLast = thrust;

//    } else {
//      // land with constant fraction of nominal thrust:
//      int32_t thrust = LANDING_THRUST_FRACTION * nominalThrottle;
//      Bound(thrust, 0.6 * nominalThrottle, 0.8 * MAX_PPRZ);
//      if (ASSIGN_CONTROL) {
//        stabilization_cmd[COMMAND_THRUST] = thrust;
//      }
//      eofState.controlThrottleLast = thrust;
//    }
  }
}*/

/***********************
 * SUPPORTING FUNCTIONS
 ***********************/
int16_t uartGetInt16(struct uart_periph *p) {
  int16_t out = 0;
  out |= uart_getch(p);
  out |= uart_getch(p) << 8;
  return out;
}

int32_t uartGetInt32(struct uart_periph *p) {
  int32_t out = 0;
  out |= uart_getch(p);
  out |= uart_getch(p) << 8;
  out |= uart_getch(p) << 16;
  out |= uart_getch(p) << 24;
  return out;
}

enum updateStatus processUARTInput(struct flowStats* s, int32_t *N) {
  enum updateStatus returnStatus = UPDATE_NONE;

  *N = 0;
  // Now scan across received data and extract events
  // Scan until read pointer is one byte behind ith the write pointer
  static bool synchronized = false;
  while(uart_char_available(&DVS_PORT) > (int32_t) EVENT_BYTE_SIZE
      && *N < 500) {
    // Timestamp syncing at first event reception, by generating artificial event rate
    if (!eofState.caerInputReceived) {
      if (uart_getch(&DVS_PORT) == EVENT_SEPARATOR) {
        eofState.caerInputReceived = TRUE;
        *N = 10;
        returnStatus = UPDATE_WARNING_RATE;
      }
    }
    else {
      if (synchronized) {
        // Next set of bytes contains a new event
        struct flowEvent e;
        uint8_t separator;
        int16_t x,y,u,v;
        x = uartGetInt16(&DVS_PORT);
        y = uartGetInt16(&DVS_PORT);
        e.t = uartGetInt32(&DVS_PORT);
        u = uartGetInt16(&DVS_PORT);
        v = uartGetInt16(&DVS_PORT);
        separator = uart_getch(&DVS_PORT);
        if (separator == EVENT_SEPARATOR) {
          // Full event received - we can process this further
          //TODO add timestamp checking - reject events that are outdated

          // Extract floating point position and velocity
          e.x = (float) x / UART_INT16_TO_FLOAT;
          e.y = (float) y / UART_INT16_TO_FLOAT;
          e.u = (float) u / UART_INT16_TO_FLOAT;
          e.v = (float) v / UART_INT16_TO_FLOAT;

          flowStatsUpdate(s, e, eofState.ratesMA, enableDerotation, dvs128Intrinsics);
          returnStatus = UPDATE_STATS;
          //        if (!eofState.caerInputReceived) {
          //          eofState.caerInputReceived = TRUE;
          //        }
          (*N)++;
        }
        else {
          // we are apparently out of sync - do not process event
          synchronized = false;
        }
      }
      else {
        // (Re)synchronize at next separator
        if (uart_getch(&DVS_PORT) == EVENT_SEPARATOR) {
          synchronized = true;
        }
      }
    }
  }
  return returnStatus;
}

static void sendFlowFieldState(struct transport_tx *trans, struct link_device *dev) {
  float fps = eofState.moduleFrequency;
  uint8_t status = (uint8_t) eofState.status;
  float confidence = eofState.field.confidence;
  float eventRate = eofState.stats.eventRate;
  float wx = eofState.field.wx;
  float wy = eofState.field.wy;
  float D  = eofState.field.D;
  float p = eofState.ratesMA.p;
  float q = eofState.ratesMA.q;
  //float q = eofState.divergenceControlLast;
  float wxTruth = eofState.wxTruth;
  float wyTruth = eofState.wyTruth;
  float DTruth = eofState.DTruth;
  int32_t controlThrottle = eofState.controlThrottleLast;
  uint8_t controlMode = eofState.landing;

  pprz_msg_send_EVENT_OPTIC_FLOW_EST(trans, dev, AC_ID,
      &fps, &status, &confidence, &eventRate, &wx, &wy, &D, &p, &q,
      &wxTruth,&wyTruth,&DTruth,&controlThrottle,&controlMode);
}

void divergenceControlReset(void) {
  eofState.controlReset = true;
  eofState.landing = false;
  eofState.divergenceUpdated = false;
  eofState.divergenceControlLast = 0.0f;
  eofState.nominalThrottleEnter = stabilization_cmd[COMMAND_THRUST];
  eofState.controlThrottleLast = eofState.nominalThrottleEnter; // set to nominal
}

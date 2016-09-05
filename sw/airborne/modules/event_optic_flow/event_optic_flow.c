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
 * Event based opticflow using DVS camera
 */

#include "event_optic_flow.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_float.h"
#include "state.h"


#ifndef DVS_PORT
#error Please define uart port connected to the dvs event based camera. e.g <define name="DVS_PORT" value="uart0"/>
#endif

// Module settings
#ifndef EOF_ENABLE_DEROTATION
#define EOF_ENABLE_DEROTATION 1
#endif
PRINT_CONFIG_VAR(EOF_ENABLE_DEROTATION)

#ifndef EOF_FILTER_TIME_CONSTANT
#define EOF_FILTER_TIME_CONSTANT 0.05f
#endif
PRINT_CONFIG_VAR(EOF_FILTER_TIME_CONSTANT)

#ifndef EOF_FLOW_MAX_SPEED_DIFF
#define EOF_FLOW_MAX_SPEED_DIFF 20.0f
#endif
PRINT_CONFIG_VAR(EOF_FLOW_MAX_SPEED_DIFF)

#ifndef EOF_MIN_EVENT_RATE
#define EOF_MIN_EVENT_RATE 100.0f
#endif
PRINT_CONFIG_VAR(EOF_MIN_EVENT_RATE)

#ifndef EOF_MIN_POSITION_VARIANCE
#define EOF_MIN_POSITION_VARIANCE 100.0f
#endif
PRINT_CONFIG_VAR(EOF_MIN_POSITION_VARIANCE)

#ifndef EOF_DIVERGENCE_CONTROL_PGAIN
#define EOF_DIVERGENCE_CONTROL_PGAIN 1.0f
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_PGAIN)

#ifndef EOF_DIVERGENCE_CONTROL_DIV_SETPOINT
#define EOF_DIVERGENCE_CONTROL_DIV_SETPOINT 0.3f
#endif
PRINT_CONFIG_VAR(EOF_DIVERGENCE_CONTROL_DIV_SETPOINT)

#ifndef EOF_CONTROL_HOVER
#define EOF_CONTROL_HOVER 0
#endif

#ifndef EOF_CONTROL_LANDING
#define EOF_CONTROL_LANDING 0
#endif

// State redeclaration
struct module_state eofState;

// Algorithm parameters
uint8_t enableDerotation = EOF_ENABLE_DEROTATION;
float statsFilterTimeConstant = EOF_FILTER_TIME_CONSTANT;
float flowMaxSpeedDiff = EOF_FLOW_MAX_SPEED_DIFF;
float divergenceControlGainP = EOF_DIVERGENCE_CONTROL_PGAIN;
float divergenceControlSetpoint = EOF_DIVERGENCE_CONTROL_DIV_SETPOINT;

// Confidence thresholds
float minPosVariance = EOF_MIN_POSITION_VARIANCE;
float minEventRate = EOF_MIN_EVENT_RATE;

// Constants
const int32_t MAX_NUMBER_OF_UART_EVENTS = 100;
const float MOVING_AVERAGE_MIN_WINDOW = 5.0f;
const uint8_t EVENT_SEPARATOR = 255;
const float FLOW_INT16_TO_FLOAT = 100.0f;
const uint32_t EVENT_BYTE_SIZE = sizeof(struct flowEvent) + 1; // +1 for separator
const float inactivityDecayFactor = 0.9f;
const float derotationMovingAverageFactor = 0.5f;

// Camera intrinsics definition
struct cameraIntrinsicParameters dvs128Intrinsics = {
    .principalPointX = 76.70f,
    .principalPointY = 56.93f,
    .focalLengthX = 115.0f,
    .focalLengthY = 115.0f
};

// Internal function declarations (definitions below)
enum updateStatus processUARTInput(struct flowStats* s, double filterTimeConstant);
static void sendFlowFieldState(struct transport_tx *trans, struct link_device *dev);
uint16_t checkBufferFreeSpace(void);
void incrementBufferPos(uint16_t* pos);
uint8_t ringBufferGetByte(void);
int16_t ringBufferGetInt16(void);
int32_t ringBufferGetInt32(void);
void divergenceLandingControllerInit(void);
void divergenceLandingControllerRun(void);

// ----- Implementations start here -----
void event_optic_flow_init(void) {
	struct flowField field = {0., 0., 0., 0., 0., {0.,0.,0.},0.};
	field.p[0] = 0;
	field.p[1] = 0;
	field.p[2] = 0;
	field.t = 0;
	struct flowStats stats = {0., 0., 0., 0., 0., 0., 0., 0., 0.,
	    0., 0., 0.,0., 0., 0.,0.};
	eofState.field = field;
	eofState.stats = stats;

	register_periodic_telemetry(DefaultPeriodic,
	    PPRZ_MSG_ID_EVENT_OPTIC_FLOW_EST, sendFlowFieldState);
}

void event_optic_flow_start(void) {
	// Timing
	eofState.lastTime = get_sys_time_float();
	// Reset low pass filter for rates
	eofState.ratesMA.p = 0;
	eofState.ratesMA.q = 0;
	eofState.moduleFrequency = 100.0f;
	eofState.z_NED = 0.0f;
	struct flowField field = {0., 0., 0., 0., 0., {0.,0.,0.},0.};
	field.p[0] = 0;
	field.p[1] = 0;
	field.p[2] = 0;
	field.t = 0;
	struct flowStats stats = {0., 0., 0., 0., 0., 0., 0., 0., 0.,
	    0., 0., 0.,0., 0., 0.,0.};
	eofState.field = field;
  eofState.stats = stats;
}

void event_optic_flow_periodic(void) {
	// Obtain UART data if available
	enum updateStatus status = processUARTInput(&eofState.stats, statsFilterTimeConstant);
	if (status == UPDATE_STATS) {
		// If new events are received, recompute flow field
		// In case the flow field is ill-posed, do not update
		status = recomputeFlowField(&eofState.field, &eofState.stats,
		    minEventRate, minPosVariance, dvs128Intrinsics);
	}
	// Timing bookkeeping, do this after the most uncertain computations,
	// but before operations where timing info is necessary
	float currentTime = get_sys_time_float();
	float dt = currentTime - eofState.lastTime;
	eofState.moduleFrequency = 1/dt;
	eofState.lastTime = currentTime;

	// If no update has been performed, decay flow field parameters towards zero
	if (status != UPDATE_SUCCESS) {
	  eofState.field.wx *= inactivityDecayFactor;
	  eofState.field.wy *= inactivityDecayFactor;
	  eofState.field.D *= inactivityDecayFactor;
	}
	else {
	  // Assign timestamp to last update
	  eofState.field.t = currentTime;
	}
  // Set confidence level globally
  eofState.status = status;

	// Derotate flow field if enabled
	if (enableDerotation) {
		struct FloatRates *rates = stateGetBodyRates_f();
		// Moving average filtering of body rates
		eofState.ratesMA.p += (rates->p - eofState.ratesMA.p) * derotationMovingAverageFactor;
		eofState.ratesMA.q += (rates->q - eofState.ratesMA.q) * derotationMovingAverageFactor;
		derotateFlowField(&eofState.field, &eofState.ratesMA);
	}
	else {
	  // Default: simply copy result
	  eofState.field.wxDerotated = eofState.field.wx;
	  eofState.field.wyDerotated = eofState.field.wy;
	}

	// Update height from Optitrack
  struct NedCoor_f *pos = stateGetPositionNed_f();
  eofState.z_NED = pos->z; // for downlink

	// Set control signals
	if (EOF_CONTROL_HOVER) {

	  // Assuming a perfectly aligned downward facing camera,
	  // the camera X-axis is opposite to the body Y-axis
	  // and the Y-axis is aligned to its X-axis
	  // Further assumption: body Euler angles are small
	  float vxNED = eofState.z_NED * eofState.field.wyDerotated;
	  float vyNED = eofState.z_NED * -eofState.field.wxDerotated;
	  float vzNED = eofState.z_NED * eofState.field.D/2;
	  uint32_t timestamp = get_sys_time_usec();

	  // Update control state
	  AbiSendMsgVELOCITY_ESTIMATE(1, timestamp,vxNED,vyNED,vzNED,0);
	}
	if (EOF_CONTROL_LANDING) {
	  divergenceLandingControllerRun();
	}
	//TODO implement SD logging (use modules/loggers/sdlog_chibios/sdLog.h?)
}

void event_optic_flow_stop(void) {
	//TODO is now present as dummy, may be removed if not required
}

// Simple ring buffer definition
static uint8_t uartRingBuffer[UART_RX_BUFFER_SIZE]; // local communication buffer
static uint16_t writePos = 0;
static uint16_t readPos = 0;

uint16_t checkBufferFreeSpace(void) {
  return (readPos + UART_RX_BUFFER_SIZE - writePos - 1) % UART_RX_BUFFER_SIZE;
}

void incrementBufferPos(uint16_t* pos) {
  *pos = (*pos + 1) % UART_RX_BUFFER_SIZE;
}

uint8_t ringBufferGetByte(void) {
  uint8_t byte = uartRingBuffer[readPos];
  incrementBufferPos(&readPos);
  return byte;
}

int16_t ringBufferGetInt16(void) {
  int16_t out = 0;
  out |= ringBufferGetByte();
  out |= ringBufferGetByte() << 8;
  return out;
}

int32_t ringBufferGetInt32(void) {
  int32_t out = 0;
  out |= ringBufferGetByte();
  out |= ringBufferGetByte() << 8;
  out |= ringBufferGetByte() << 16;
  out |= ringBufferGetByte() << 24;
  return out;
}

enum updateStatus processUARTInput(struct flowStats* s, double filterTimeConstant) {
  enum updateStatus returnStatus = UPDATE_NONE;
	// Copy UART data to buffer if not full
	while( checkBufferFreeSpace() > 0 && uart_char_available(&DVS_PORT)) {
		uartRingBuffer[writePos] = uart_getch(&DVS_PORT);          // copy over incoming data
		incrementBufferPos(&writePos);
	}

	// Now scan across received data and extract events
	// Scan until read pointer is one byte behind ith the write pointer
	static bool synchronized;
	while((writePos + UART_RX_BUFFER_SIZE - readPos) % UART_RX_BUFFER_SIZE > (int32_t) EVENT_BYTE_SIZE) {
	  if (synchronized) {
	    // Next data contains a new event
	    struct flowEvent e;
	    uint8_t separator;
	    int16_t u,v;
	    e.x = ringBufferGetByte();
	    e.y = ringBufferGetByte();
	    e.t = ringBufferGetInt32();
	    u = ringBufferGetInt16();
	    v = ringBufferGetInt16();
	    e.u = (float) u / FLOW_INT16_TO_FLOAT;
	    e.v = (float) v / FLOW_INT16_TO_FLOAT;
	    separator = ringBufferGetByte();
	    if (separator == EVENT_SEPARATOR) {
	      // Full event received - this can be processed further
	      updateFlowStats(s, e, eofState.field, filterTimeConstant, MOVING_AVERAGE_MIN_WINDOW, flowMaxSpeedDiff);
	      returnStatus = UPDATE_STATS;
	    }
	    else {
	      // we are apparently out of sync - do not process event
	      synchronized = false;
	    }
	  }
	  else {
	    // (Re)synchronize at next separator
	    if (ringBufferGetByte() == EVENT_SEPARATOR) {
	      synchronized = true;
	    }
	  }
	}
	return returnStatus;
}

static void sendFlowFieldState(struct transport_tx *trans, struct link_device *dev) {
  float fps = eofState.moduleFrequency;
  uint8_t confidence = (uint8_t) eofState.status;
  float eventRate = eofState.stats.eventRate;
  float wx = eofState.field.wx;
  float wy = eofState.field.wy;
  float D  = eofState.field.D;
  float wxDerotated = eofState.field.wxDerotated;
  float wyDerotated = eofState.field.wyDerotated;
  float vx = eofState.z_NED * wyDerotated;
  float vy = eofState.z_NED * -wxDerotated;

  pprz_msg_send_EVENT_OPTIC_FLOW_EST(trans, dev, AC_ID,
      &fps, &confidence, &eventRate, &wx, &wy, &D, &wxDerotated, &wyDerotated,&vx,&vy);
}

void divergenceLandingControllerInit(void) {
  //TODO implement
}

void divergenceLandingControllerRun(void) {
  //TODO implement
}


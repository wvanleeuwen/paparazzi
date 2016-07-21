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
#include "flow_field_estimation.h"

#include "mcu_periph/uart.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/abi.h"
#include "subsystems/datalink/telemetry.h"
#include "math/pprz_algebra_float.h"
#include "state.h"


#ifndef DVS_PORT
#error Please define uart port connected to the dvs event based camera. e.g <define name="DVS_PORT" value="uart0"/>
#endif

#ifndef EOF_ENABLE_NORMALFLOW
#define EOF_ENABLE_NORMALFLOW 0
#endif

#ifndef EOF_DEROTATION
#define EOF_DEROTATION 0
#endif

#ifndef EOF_CONTROL_HOVER
#define EOF_CONTROL_HOVER 0
#endif

#ifndef EOF_CONTROL_LANDING
#define EOF_CONTROL_LANDING 0
#endif


// Module state - made static so that it is available in all module functions
struct module_state {
  struct flowField field;
  struct flowStats stats;
  float z_NED;
  float lastTime;
  float moduleFrequency;
  enum updateStatus status;
};

static struct module_state moduleState;

// Algorithm parameters
const float inactivityDecayFactor = 0.9;
const float statsFilterTimeConstant = 0.05;

// Confidence thresholds
const float minPosVariance = 100;
const float minSpeedVariance = 100;
const float minEventRate = 100;

// Constants
const int32_t MAX_NUMBER_OF_UART_EVENTS = 100;
const float MOVING_AVERAGE_MIN_WINDOW = 10;
const uint8_t EVENT_SEPARATOR = 255;
const float FLOW_INT16_TO_FLOAT = 100;
const uint32_t eventByteSize = sizeof(struct flowEvent) + 1; // +1 for separator

// Camera intrinsics definition
struct cameraIntrinsicParameters dvs128Intrinsics = {
    .principalPointX = 76.70,
    .principalPointY = 56.93,
    .focalLengthX = 115,
    .focalLengthY = 115
};

// Internal function declarations (definitions below)
enum updateStatus processUARTInput(struct flowStats* s, double filterTimeConstant);
static void sendFlowFieldState(struct transport_tx *trans, struct link_device *dev);
uint16_t checkBufferFreeSpace(void);
void incrementBufferPos(uint16_t* pos);
uint8_t ringBufferGetByte(void);
int16_t ringBufferGetInt16(void);
int32_t ringBufferGetInt32(void);

// ----- Implementations start here -----
void event_optic_flow_init(void) {
	// Fast initialization, setting all struct members to zero
	// with short calls
	struct flowField field = {0};
	struct flowStats stats = {0};
	moduleState.field = field;
	moduleState.stats = stats;

	register_periodic_telemetry(DefaultPeriodic,
	    PPRZ_MSG_ID_EVENT_OPTIC_FLOW_EST, sendFlowFieldState);
}

void event_optic_flow_start(void) {
	// Timing
	moduleState.lastTime = get_sys_time_float();
}

void event_optic_flow_periodic(void) {
	// Obtain UART data if available
	enum updateStatus status = processUARTInput(&moduleState.stats, statsFilterTimeConstant);
	if (status == UPDATE_STATS) {
		// If new events are received, recompute flow field
		// In case the flow field is ill-posed, do not update
		status = recomputeFlowField(&moduleState.field, &moduleState.stats, EOF_ENABLE_NORMALFLOW,
		    minEventRate, minPosVariance, minSpeedVariance, dvs128Intrinsics);
	}
	// Timing bookkeeping, do this after the most uncertain computations,
	// but before operations where timing info is necessary
	float currentTime = get_sys_time_float();
	float dt = currentTime - moduleState.lastTime;
	moduleState.moduleFrequency = 1/dt;
	moduleState.lastTime = currentTime;

	// If no update has been performed, decay flow field parameters towards zero
	if (status != UPDATE_SUCCESS) {
	  moduleState.field.wx *= inactivityDecayFactor;
	  moduleState.field.wy *= inactivityDecayFactor;
	  moduleState.field.D *= inactivityDecayFactor;
	}
	else {
	  // Assign timestamp to last update
	  moduleState.field.t = currentTime;
	}
  // Set confidence level globally
  moduleState.status = status;

	// Derotate flow field if enabled
	if (EOF_DEROTATION) {
		struct FloatRates *rates = stateGetBodyRates_f();
		derotateFlowField(&moduleState.field, rates);
	}
	else {
	  // Default: simply copy result
	  moduleState.field.wxDerotated = moduleState.field.wx;
	  moduleState.field.wyDerotated = moduleState.field.wy;
	}

	// Set control signals
	if (EOF_CONTROL_HOVER) {
	  struct NedCoor_f *pos = stateGetPositionNed_f();  // for alt/height
	  moduleState.z_NED = pos->z;

	  // Assuming a perfectly aligned downward facing camera,
	  // the camera X-axis is opposite to the body Y-axis
	  // and the Y-axis is aligned to its X-axis
	  float vx = pos->z * moduleState.field.wyDerotated;
	  float vy = pos->z * -moduleState.field.wxDerotated;
	  float vz = pos->z * moduleState.field.D/2;
	  uint32_t timestamp = get_sys_time_usec();
	  // Update control state
	  AbiSendMsgVELOCITY_ESTIMATE(1, timestamp,vx,vy,vz,0);
	}

	//TODO implement constant divergence landing controller
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
	while((writePos + UART_RX_BUFFER_SIZE - readPos) % UART_RX_BUFFER_SIZE > (int32_t) eventByteSize) {
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
	      updateFlowStats(s, e, filterTimeConstant, MOVING_AVERAGE_MIN_WINDOW);
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
  float fps = moduleState.moduleFrequency;
  uint8_t confidence = (uint8_t) moduleState.status;
  float eventRate = moduleState.stats.eventRate;
  float wx = moduleState.field.wx;
  float wy = moduleState.field.wy;
  float D  = moduleState.field.D;
  float wxDerotated = moduleState.field.wxDerotated;
  float wyDerotated = moduleState.field.wyDerotated;
  float vx = moduleState.z_NED * wyDerotated;
  float vy = moduleState.z_NED * -wxDerotated;
  pprz_msg_send_EVENT_OPTIC_FLOW_EST(trans, dev, AC_ID,
      &fps, &confidence, &eventRate, &wx, &wy, &D, &wxDerotated, &wyDerotated, &vx, &vy);
}

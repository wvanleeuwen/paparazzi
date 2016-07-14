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
#include "subsystems/datalink/downlink.h"
#include "math/pprz_algebra_float.h"

#include "filters/low_pass_filter.h"

#ifndef DVS_PORT
#error Please define uart port connected to the dvs event based camera. e.g <define name="DVS_PORT" value="uart0"/>
#endif

#ifndef EOF_DEROTATION
#define EOF_DEROTATION 0
#endif

#ifndef EOF_CONTROL
#define EOF_CONTROL 0
#endif

// Structs
/**
 * Flow event struct, simplified version of the cAER implementation.
 * It contains all fields passed from the DVS through UART.
 * Note that polarity and validity information are not transferred:
 * all received events are assumed to be valid.
 */
struct flowEvent {
  uint8_t x,y;
  int32_t t;
  int16_t u,v;
};

/**
 * Flow field parameter struct.
 * Contains the parameters ventral flow and divergence
 */
struct flowField {
  float wx;
  float wy;
  float D;
  int32_t t;
};

/**
 * Flow statistics container.
 * These statistics are re-evaluated at every event and form the basis
 * for flow field recomputation. At all times they represent the mean of
 * N previous vector coordinates or the mean of cross-products of two
 * vector coordinates.
 *
 * E.g. 's<x><x>' refers to Sum_i^N {<x_i>*<x_i>} / N.
 *
 * These mean values can be used to compute, among others, variance:
 * Var{x} = (Sum_i^N {x_i^2} - (Sum_i^N {x_i})^2) / N
 *      = mxx - mx^2
 * And similarly, covariance:
 * Cov{x,y} = mxy - mx * my
 *
 * And ultimately they are used for computing the flow field as a
 * least-squares solution.
 */
struct flowStats {
  float mx, my, mu, mv;
  float mxx, myy;
  float mxu, myv;
  float muu, mvv, mww;
  float muv, mvw, muw;
  float mum, mvm, mwm;
  float eventRate;
};

enum updateStatus {
  UPDATE_SUCCES,
  UPDATE_NONE,
  UPDATE_WARNING_RATE,
  UPDATE_WARNING_SPREAD
};

// Module state - made static so that it is available in all module functions
struct module_state {
  struct flowField field;
  struct flowStats stats;
  float lastTime;
  float moduleFrequency;
};

static struct module_state moduleState;

// Algorithm parameters
const float inactivityDecayPerSecond = 2;
const float statsFilterTimeConstant = 0.01;

// Confidence thresholds
const float minPosVariance = 100;
const float minSpeedVariance = 100;
const float minEventRate = 100;

// Constants
const int32_t MAX_NUMBER_OF_UART_EVENTS = 100;
const int32_t MOVING_AVERAGE_MIN_WINDOW = 10;
const float FLT_MIN_RESOLUTION = 1e-3;
const float DVS128_PRINCIPAL_POINT_X = 76.70;
const float DVS128_PRINCIPAL_POINT_Y = 56.93;
const float DVS128_FOCAL_LENGTH = 115;
const uint8_t EVENT_SEPARATOR = 255;
const float FLOW_INT16_TO_FLOAT = 100;
const uint32_t eventByteSize = sizeof(struct flowEvent) + 1; // +1 for separator

// Function declarations (definitions below)
bool processUARTInput(struct flowStats* s, float filterTimeConstant);
void updateFlowStats(struct flowStats* s, struct flowEvent e, float filterTimeConstant);
void decayParameterToZero(float* par, float decay);
void decayFlowFieldParameters(struct flowField* field, float decay);
uint8_t recomputeFlowField(struct flowField* field, struct flowStats* s);
void derotateFlowField(struct flowField* field, struct FloatRates rates);
void downlinkFlowFieldState(struct flowField* field, uint8_t status);

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
}

void event_optic_flow_start(void) {
	// Timing
	moduleState.lastTime = get_sys_time_float();
}

void event_optic_flow_periodic(void) {
	// Obtain UART data if available
	enum updateStatus status = UPDATE_NONE;
	if (processUARTInput(&moduleState.stats, statsFilterTimeConstant)) {
		// If new events are received, recompute flow field
		// In case the flow field is ill-posed, do not update
		status = recomputeFlowField(&moduleState.field, &moduleState.stats);
	}
	// Timing bookkeeping, do this after the most uncertain computations,
	// but before operations where timing info is necessary
	float currentTime = get_sys_time_float();
	float dt = currentTime - moduleState.lastTime;
	moduleState.moduleFrequency = 1/dt;
	moduleState.lastTime = currentTime;

	// If no update has been performed, decay flow field parameters towards zero
	if (status == UPDATE_NONE) {
		decayFlowFieldParameters(&moduleState.field,
				inactivityDecayPerSecond/moduleState.moduleFrequency);
	}

	// Derotate flow field
	if (EOF_DEROTATION) {
		struct FloatRates rates;
		//TODO implement such that the current body rates are acquired
		derotateFlowField(&moduleState.field, rates);
	}

	// Set control signals
	if (EOF_CONTROL) {
		//TODO implement controller
	}

	// Send flow field info to ground station
	downlinkFlowFieldState(&moduleState.field, status);
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

bool processUARTInput(struct flowStats* s, float filterTimeConstant) {
	// Copy UART data to buffer
	// check buffer full
	while( checkBufferFreeSpace() > 0 && uart_char_available(&DVS_PORT)) {
		uartRingBuffer[writePos] = uart_getch(&DVS_PORT);          // copy over incoming data
		incrementBufferPos(&writePos);
	}

	// Now scan across received data and extract events
	bool eventsFound = false;

	// Scan until read pointer is one byte behind ith the write pointer
	static bool synchronized;
	while((writePos + UART_RX_BUFFER_SIZE - readPos) % UART_RX_BUFFER_SIZE > (int32_t) eventByteSize) {
	  if (synchronized) {
	    // Next data contains a new event
	    struct flowEvent e;
	    uint8_t separator;
	    e.x = ringBufferGetByte();
	    e.y = ringBufferGetByte();
	    e.t = ringBufferGetInt32();
	    e.u = ringBufferGetInt16();
	    e.v = ringBufferGetInt16();
	    separator = ringBufferGetByte();
	    if (separator == EVENT_SEPARATOR) {
	      // Full event received - this can be processed further
	      updateFlowStats(s, e, filterTimeConstant);
	      eventsFound = true;
	    }
	    else {
	      // we are apparently out of sync - do not process event
	      synchronized = false;
	    }
	  }
	  else {
	    // Resynchronize at next separator
	    if (ringBufferGetByte() == EVENT_SEPARATOR) {
	      synchronized = true;
	    }
	  }
	}

	return eventsFound;
}

void updateFlowStats(struct flowStats* s, struct flowEvent e, float filterTimeConstant) {
  static int32_t tPrevious = 0;

  float x = (float) e.x;
  float y = (float) e.y;
  float u = (float) e.u / FLOW_INT16_TO_FLOAT;
  float v = (float) e.v / FLOW_INT16_TO_FLOAT;

  // Compute dot product of position/flow
  float w = x * u + y * v;
  // Compute squared magnitude
  float m = u * u + v * v;

  // Update stats through hybrid moving averaging/low-pass filtering
  double dt = ((double)(e.t - tPrevious))/1e6;
  if (dt <= 0) {
    dt = 1e-6;
  }
  double tFactor =  dt / filterTimeConstant;
  if (tFactor > 1/MOVING_AVERAGE_MIN_WINDOW)
    tFactor = 1/MOVING_AVERAGE_MIN_WINDOW;

  s->mx  += (x - s->mx ) * tFactor;
  s->my  += (y - s->my ) * tFactor;
  s->mu  += (u - s->mu ) * tFactor;
  s->mv  += (v - s->mv ) * tFactor;

  s->mxx += (x * x - s->mxx) * tFactor;
  s->myy += (y * y - s->myy) * tFactor;
  s->mxu += (x * u - s->mxu) * tFactor;
  s->myv += (y * v - s->myv) * tFactor;

  s->muu += (u * u - s->muu) * tFactor;
  s->mvv += (v * v - s->mvv) * tFactor;
  s->mww += (w * w - s->mww) * tFactor;
  s->muv += (u * v - s->muv) * tFactor;
  s->mvw += (v * w - s->mvw) * tFactor;
  s->muw += (u * w - s->muw) * tFactor;
  s->mum += (u * m - s->mum) * tFactor;
  s->mvm += (v * m - s->mvm) * tFactor;
  s->mwm += (w * m - s->mwm) * tFactor;

  s->eventRate += (1/dt - s->eventRate) * tFactor;

  tPrevious = e.t;
}

/**
 * Linear decay of a parameter
 */
void decayParameterToZero(float* par, float decay) {
  float val = *par;
  if (val > 0) {
    val -= decay;
    if (val < 0) val = 0; // if sign switched, set to zero
  }
  else {
    if (val < 0) {
      val += decay;
      if (val > 0) val = 0; // if sign switched, set to zero
    }
  }
}

void decayFlowFieldParameters(struct flowField* field, float decay) {
  // Note that decay must be positive definite!
  if (decay <= 0) {
    //TODO toggle error/warning
    return;
  }
  decayParameterToZero(&field->wx, decay);
  decayParameterToZero(&field->wy, decay);
  decayParameterToZero(&field->D, decay);
}

/**
 * Use latest flow statistics to update flow field parameters.
 */
enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s) {
  float p[3];

  // Check speed variances to choose method
  float varU = s->muu - pow(s->mu,2);
  float varV = s->mvv - pow(s->mv,2);
  float covUV = s->muv - s->mu * s->mv;
  if (varU > minSpeedVariance && varV > minSpeedVariance
      && fabs(covUV) > minSpeedVariance) {
    /* Normal flow method. This is more accurate but cannot handle
     * low speed variance.
     * The linear system to be solved here is
     *
     * [suu, suv, suw    [p[0]     [sum
     *  suv, svv, svw * p[1]  = svm
     *  suw, svw, sww]    p[2]]   swm]
     *
     *  with w = x*u + y*v
     *  and m = u^2 + v^2
     */

    // Compute determinant
    float D = - s->mww * pow(s->muv,2) + 2*s->muv * s->muw * s->mvw
        - s->mvv * pow(s->muw,2) - s->muu * pow(s->mvw,2) + s->muu * s->mvv * s->mww;
    if (fabs(D) < FLT_MIN_RESOLUTION) {
      return UPDATE_NONE;
    }
    // Solve system of equations
    p[0]= 1/D* (s->muw*s->mvm*s->mvw - s->mum*s->mvw*s->mvw - s->muv*s->mvm*s->mww
        + s->muv*s->mvw*s->mwm - s->muw*s->mvv*s->mwm + s->mum*s->mvv*s->mww);
    p[1]= 1/D* (s->mum*s->muw*s->mvw - s->muw*s->muw*s->mvm + s->muv*s->muw*s->mwm
        - s->mum*s->muv*s->mww + s->muu*s->mvm*s->mww - s->muu*s->mvw*s->mwm);
    p[2]= 1/D* (s->muv*s->muw*s->mvm - s->muv*s->muv*s->mwm + s->mum*s->muv*s->mvw
        - s->mum*s->muw*s->mvv - s->muu*s->mvm*s->mvw + s->muu*s->mvv*s->mwm);
  }
  else {
    /* A normal flow solution is inaccurate in this case,
     * so the assumption of regular optic flow is made.
     *
     * The linear system to be solved is then:
     *
     * [1,  sx,     0    [p[0]     [su
     *  sx, sxx+syy,  sy  * p[1]  = sxu+syv
     *  0,  sy,     1]    p[2]]   sv]
     */

    // Compute determinant
    float D = - pow(s->mx,2) - pow(s->my,2) + s->mxx + s->myy;
        if (fabs(D) < FLT_MIN_RESOLUTION) {
          return UPDATE_NONE;
        }
        p[0] = 1/D*(-s->mu*pow(s->my,2) + s->mv*s->mx*s->my + s->mu*s->mxx
            + s->mu*s->myy - s->mx*s->mxu - s->mx*s->myv);
        p[1] = 1/D*(-s->mv*pow(s->mx,2) + s->mu*s->my*s->mx + s->mv*s->mxx
            + s->mv*s->myy - s->mxu*s->my - s->my*s->myv);
        p[2] = 1/D*(s->mxu + s->myv - s->mu*s->mx - s->mv*s->my);
  }

  // Convert solution to ventral flow and divergence
  field->wx = (p[0] + DVS128_PRINCIPAL_POINT_X*p[2])/DVS128_FOCAL_LENGTH;
  field->wy = (p[1] + DVS128_PRINCIPAL_POINT_Y*p[2])/DVS128_FOCAL_LENGTH;
  field->D  = 2*p[2];

  // Quality checking, return 'low confidence' indicator if
  if (s->eventRate < minEventRate) {
    return UPDATE_WARNING_RATE;
  }
  float varX = s->mxx - pow(s->mx,2);
  float varY = s->myy - pow(s->my,2);
  if (varX < minPosVariance || varY < minPosVariance) {
    return UPDATE_WARNING_RATE;
  }

  // If no problem was found, update is successful
  return UPDATE_SUCCES;
}

void derotateFlowField(struct flowField* field, struct FloatRates rates) {
	//TODO implement
}
void downlinkFlowFieldState(struct flowField* field, enum updateStatus status) {
	//TODO implement
}

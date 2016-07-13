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
#define DVS_PORT "" // dummy value to disable error in Eclipse
#endif

#ifndef EOF_DEROTATION
#define EOF_DEROTATION 0
#endif

#ifndef EOF_CONTROL
#define EOF_CONTROL 0
#endif

// Function declarations (definitions below)
bool updateFlowStatsFromUART(FlowStats* s);
void decayParameterToZero(float* par, float decay);
void decayFlowFieldParameters(FlowField* field, float decay);
UpdateStatus recomputeFlowField(FlowField* field, FlowStats* s);
void derotateFlowField(FlowField* field, struct FloatRates rates);
void downlinkFlowFieldState(FlowField* field);

// Module state - made static so that it is available in all module functions
struct module_state {
	FlowField field;
	FlowStats stats;
	float lastTime;
	float moduleFrequency;
};

static struct module_state moduleState;

// Algorithm parameters
const float inactivityDecayPerSecond = 2;

// Confidence thresholds
const float minPosVariance = 100;
const float minSpeedVariance = 100;
const float minEventRate = 100;

// Constants
const int32_t MAX_NUMBER_OF_UART_EVENTS = 100;
const float FLT_MIN_RESOLUTION = 1e-3;
const float DVS128_PRINCIPAL_POINT_X = 76.70;
const float DVS128_PRINCIPAL_POINT_Y = 56.93;
const float DVS128_FOCAL_LENGTH = 115;

// ----- Implementations start here -----
void event_optic_flow_init(void) {
	// Fast initialization, setting all struct members to zero
	// with short calls
	FlowField field = {0};
	FlowStats stats = {0};
	moduleState.field = field;
	moduleState.stats = stats;
}

void event_optic_flow_start(void) {
	// Fast re-initialization, setting all struct members to zero
	// with short calls
	FlowField field = {0};
	FlowStats stats = {0};
	moduleState.field = field;
	moduleState.stats = stats;

	// Timing
	sys_time_init();
	moduleState.lastTime = 0;
}

void event_optic_flow_periodic(void) {
	// Obtain UART data if available
	UpdateStatus updateStatus = UPDATE_NONE;
	if (updateFlowStatsFromUART(&moduleState.stats)) {
		// If updated, recompute flow field
		// In case the flow field is ill-posed, do not update
		updateStatus = recomputeFlowField(&moduleState.field, &moduleState.stats);
	}
	// Timing bookkeeping, do this after the most uncertain computations,
	// but before operations where timing info is necessary
	float currentTime = get_sys_time_float();
	float dt = currentTime - moduleState.lastTime;
	moduleState.moduleFrequency = 1/dt;
	moduleState.lastTime = currentTime;

	// If no update has been performed, decay flow field parameters towards zero
	if (updateStatus == UPDATE_NONE) {
		decayFlowFieldParameters(&moduleState.field,
				inactivityDecayPerSecond/moduleState.moduleFrequency);
	}

	// Derotate flow field
	if (EOF_DEROTATION) {
		struct FloatRates rates; //TODO implement such that this struct will get the current body rates
		derotateFlowField(&moduleState.field, rates);
	}

	// Set control signals
	if (EOF_CONTROL) {
		//TODO implement controller
	}

	// Send flow field info to ground station
	downlinkFlowFieldState(&moduleState.field);
}

void event_optic_flow_stop(void) {
	//TODO is now present as dummy, may be removed if not required
}

bool updateFlowStatsFromUART(FlowStats* s) {
	//TODO expand implementation
	static uint8_t buffer[UART_RX_BUFFER_SIZE]; // local communication buffer
	static uint16_t buf_loc = 0;                // circular buffer index location

	while(uart_char_available(&DVS_PORT))
	{
		buffer[buf_loc] = uart_getch(&DVS_PORT);          // copy over incoming data
		buf_loc = (buf_loc + 1) % UART_RX_BUFFER_SIZE;    // implement circular buffer
	}
	// send to ground station for debugging
	DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 32, &(buffer[(buf_loc-32)%UART_RX_BUFFER_SIZE]));


	return false;
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

void decayFlowFieldParameters(FlowField* field, float decay) {
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
UpdateStatus recomputeFlowField(FlowField* field, FlowStats* s) {
	float p[3];

	// Check angular spread to choose method
	float varU = s->suu - pow(s->su,2);
	float varV = s->svv - pow(s->sv,2);
	float covUV = s->suv - s->su * s->sv;
	if (varU > minSpeedVariance && varV > minSpeedVariance
			&& fabs(covUV) > minSpeedVariance) {
		/* The linear system to be solved here is
		 *
		 * [suu, suv, suw	   [p[0]	   [sum
		 * 	suv, svv, svw	*	p[1]	=	svm
		 * 	suw, svw, sww]		p[2]]		swm]
		 *
		 * 	with w = x*u + y*v
		 * 	and m = u^2 + v^2
		 */
		// Compute determinant
		float D = - s->sww * pow(s->suv,2) + 2*s->suv * s->suw * s->svw
				- s->svv * pow(s->suw,2) - s->suu * pow(s->svw,2) + s->suu * s->svv * s->sww;
		if (fabs(D) < FLT_MIN_RESOLUTION) {
			return UPDATE_NONE;
		}
		// Solve system of equations
		p[0]= 1/D* (s->suw*s->svm*s->svw - s->sum*s->svw*s->svw - s->suv*s->svm*s->sww
				+ s->suv*s->svw*s->swm - s->suw*s->svv*s->swm + s->sum*s->svv*s->sww);
		p[1]= 1/D* (s->sum*s->suw*s->svw - s->suw*s->suw*s->svm + s->suv*s->suw*s->swm
				- s->sum*s->suv*s->sww + s->suu*s->svm*s->sww - s->suu*s->svw*s->swm);
		p[2]= 1/D* (s->suv*s->suw*s->svm - s->suv*s->suv*s->swm + s->sum*s->suv*s->svw
				- s->sum*s->suw*s->svv - s->suu*s->svm*s->svw + s->suu*s->svv*s->swm);
	}
	else {
		/* A normal flow solution is inaccurate in this case
		 *
		 * The linear system to be solved is then:
		 *
		 * [1, 	sx, 		0	   [p[0]	   [su
		 * 	sx, sxx+syy, 	sy	*	p[1]	=	sxu+syv
		 * 	0, 	sy, 		1]		p[2]]		sv]
		 */

		// Compute determinant
		float D = - pow(s->sx,2) - pow(s->sy,2) + s->sxx + s->syy;
        if (fabs(D) < FLT_MIN_RESOLUTION) {
        	return UPDATE_NONE;
        }
        p[0] = 1/D*(-s->su*pow(s->sy,2) + s->sv*s->sx*s->sy + s->su*s->sxx
        		+ s->su*s->syy - s->sx*s->sxu - s->sx*s->syv);
        p[1] = 1/D*(-s->sv*pow(s->sx,2) + s->su*s->sy*s->sx + s->sv*s->sxx
        		+ s->sv*s->syy - s->sxu*s->sy - s->sy*s->syv);
        p[2] = 1/D*(s->sxu + s->syv - s->su*s->sx - s->sv*s->sy);
	}

	// Convert solution to ventral flow and divergence
	field->wx = (p[0] + DVS128_PRINCIPAL_POINT_X*p[2])/DVS128_FOCAL_LENGTH;
	field->wy = (p[1] + DVS128_PRINCIPAL_POINT_Y*p[2])/DVS128_FOCAL_LENGTH;
	field->D  = 2*p[2];

	// Quality checking, return 'low confidence' indicator if
	if (s->eventRate < minEventRate) {
		return UPDATE_LOW_CONFIDENCE;
	}
	float varX = s->sxx - pow(s->sx,2);
	float varY = s->syy - pow(s->sy,2);
	if (varX < minPosVariance || varY < minPosVariance) {
		return UPDATE_LOW_CONFIDENCE;
	}

	// If no problem was found, update is successful
	return UPDATE_SUCCES;
}

void derotateFlowField(FlowField* field, struct FloatRates rates) {
	//TODO implement
}
void downlinkFlowFieldState(FlowField* field) {
	//TODO implement
}

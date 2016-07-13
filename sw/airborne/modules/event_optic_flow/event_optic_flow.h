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

extern void event_optic_flow_init(void);
extern void event_optic_flow_start(void);
extern void event_optic_flow_periodic(void);
extern void event_optic_flow_stop(void);

/**
 * Flow event struct, simplified version of the cAER implementation.
 * It contains all fields passed from the DVS through UART.
 * Note that polarity and validity information are not transferred:
 * all received events are assumed to be valid.
 */
typedef struct flow_event {
	int8_t x,y;
	int32_t t;
	float u,v;
} FlowEvent;

/**
 * Flow field parameter struct.
 * Contains the parameters ventral flow and divergence
 */
typedef struct flow_field {
	float wx;
	float wy;
	float D;
	int32_t t;
} FlowField;

/**
 * Flow statistics container.
 * These statistics are re-evaluated at every event and form the basis
 * for flow field recomputation. At all times they represent the sum of
 * N previous vector coordinates or the sum of cross-products of two
 * vector coordinates.
 *
 * E.g. 's<x><x>' refers to Sum_i^N {<x_i>*<x_i>}.
 *
 * These sums are then used to obtain estimates of mean and variance.
 * E.g. Mean(x) = Sum_i^N {x_i} / N
 * and Var(x) = (Sum_i^N {x_i^2} - Sum_i^N {x_i}) / N
 *
 * And ultimately they are used for computing the flow field as a
 * least-squares solution.
 */
typedef struct flow_stats {
	float sx, sy, su, sv;
	float sxx, syy;
	float sxu, syv;
	float suu, svv, sww;
	float suv, svw, suw;
	float sum, svm, swm;
	float eventRate;
	float timeConstant;
} FlowStats;


typedef enum {
	UPDATE_SUCCES,
	UPDATE_NONE,
	UPDATE_LOW_CONFIDENCE
} UpdateStatus;


#endif


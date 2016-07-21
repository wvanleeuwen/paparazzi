/*
 * flow_field_computation.h
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#ifndef SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_
#define SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_

#include <stdbool.h>
#include <inttypes.h>
#include "math/pprz_algebra_float.h"

/**
 * Flow event struct, simplified version of the cAER implementation.
 * It contains all fields passed from the DVS through UART.
 * Note that polarity and validity information are not transferred:
 * all received events are assumed to be valid.
 */
struct flowEvent {
  uint8_t x,y;
  int32_t t;
  float u,v;
};

/**
 * Flow field parameter struct.
 * Contains the parameters ventral flow and divergence
 */
struct flowField {
  float wx;
  float wy;
  float wxDerotated;
  float wyDerotated;
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

/**
 * Camera intrinsic parameters struct.
 */
struct cameraIntrinsicParameters {
  float principalPointX;
  float principalPointY;
  float focalLengthX;
  float focalLengthY;
};

/**
 * Indicator for flow field computation result in a module periodic function call
 */
enum updateStatus {
  UPDATE_SUCCESS,           // Successful flow field update
  UPDATE_NONE,              // No incoming data, no update
  UPDATE_STATS,             // Stats were updated - the flow field still needs to be recomputed
  UPDATE_WARNING_SINGULAR,  // No update, flow field system is singular
  UPDATE_WARNING_RATE,      // No update, event rate is too low
  UPDATE_WARNING_SPREAD     // No update, there is too little spread in flow vector position
};

/**
 * Performs an update of all flow field statistics with a new event.
 */
void updateFlowStats(struct flowStats* s, struct flowEvent e, float filterTimeConstant,
    int32_t movingAverageWindow);

/**
 * Recomputation of the flow field using the latest statistics.
 */
enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    bool enableNormalFlow, float minEventRate, float minSpeedVariance, float minPosVariance,
    struct cameraIntrinsicParameters intrinsics);

/**
 * Derotation of the flow field parameters using rotational rate measurements.
 */
void derotateFlowField(struct flowField* field, struct FloatRates* rates);

#endif /* SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_FLOW_FIELD_ESTIMATION_H_ */

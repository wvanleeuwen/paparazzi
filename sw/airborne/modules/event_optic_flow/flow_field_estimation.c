/*
 * flow_field_computation.c
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#include "flow_field_estimation.h"

float FLT_MIN_RESOLUTION = 1e-3;

/**
 * Recomputation of optic flow statistics with a newly arrived flow event.
 *
 * @param s The flow statistics to be updated.
 * @param e The new flow event.
 * @param filterTimeConstant The time constant for the low pass filter.
 * @param movingAverageWindow The fallback moving average window, for when the time difference is too high.
 */
void updateFlowStats(struct flowStats* s, struct flowEvent e, float filterTimeConstant,
    float movingAverageWindow) {
  static int32_t tPrevious = 0;

  float x = (float) e.x;
  float y = (float) e.y;
  float u = e.u;
  float v = e.v;

  // Dot product of position/flow
  float w = x * u + y * v;
  // Squared flow magnitude
  float m = u * u + v * v;

  // Update stats through hybrid moving averaging/low-pass filtering
  float dt = ((float)(e.t - tPrevious))/1e6;
  if (dt <= 0) {
    dt = 1e-6;
  }
  float tFactor =  dt / filterTimeConstant;
  // To prevent very large updates after a large dt, the update reduces to a moving average
  if (tFactor > 1/movingAverageWindow)
    tFactor = 1/movingAverageWindow;

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
 * Use latest flow statistics to update flow field parameters.
 */
enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    bool enableNormalFlow, float minEventRate, float minSpeedVariance, float minPosVariance,
    struct cameraIntrinsicParameters intrinsics) {
  float p[3];

  // Quality checking for event rate
  if (s->eventRate < minEventRate) {
    return UPDATE_WARNING_RATE;
  }
  // Similar for position variance
  float varX = s->mxx - pow(s->mx,2);
  float varY = s->myy - pow(s->my,2);
  if (varX < minPosVariance || varY < minPosVariance) {
    return UPDATE_WARNING_SPREAD;
  }

  // Check speed variances to choose method
  float varU = s->muu - pow(s->mu,2);
  float varV = s->mvv - pow(s->mv,2);
  float covUV = s->muv - s->mu * s->mv;

  if (enableNormalFlow && varU > minSpeedVariance && varV > minSpeedVariance
      && fabs(covUV) > minSpeedVariance) {
    /* Normal flow method. This is theoretically more accurate, but noisy in practice.
     * The linear system to be solved here is:
     *
     * [suu, suv, suw    [p[0]     [sum
     *  suv, svv, svw *   p[1]  =   svm
     *  suw, svw, sww]    p[2]]     swm]
     *
     *  with w = x*u + y*v
     *  and  m = u^2 + v^2
     */

    // Compute determinant
    float D = - s->mww * pow(s->muv,2) + 2*s->muv * s->muw * s->mvw
        - s->mvv * pow(s->muw,2) - s->muu * pow(s->mvw,2) + s->muu * s->mvv * s->mww;
    if (fabs(D) < FLT_MIN_RESOLUTION) {
      return UPDATE_WARNING_SINGULAR;
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
     * [1,  sx,      0     [p[0]    [su
     *  sx, sxx+syy, sy  *  p[1]  =  sxu+syv
     *  0,  sy,      1]     p[2]]    sv]
     */

    // Compute determinant
    float D = - pow(s->mx,2) - pow(s->my,2) + s->mxx + s->myy;
        if (fabs(D) < FLT_MIN_RESOLUTION) {
          return UPDATE_WARNING_SINGULAR;
        }
        p[0] = 1/D*(-s->mu*pow(s->my,2) + s->mv*s->mx*s->my + s->mu*s->mxx
            + s->mu*s->myy - s->mx*s->mxu - s->mx*s->myv);
        p[1] = 1/D*(-s->mv*pow(s->mx,2) + s->mu*s->my*s->mx + s->mv*s->mxx
            + s->mv*s->myy - s->mxu*s->my - s->my*s->myv);
        p[2] = 1/D*(s->mxu + s->myv - s->mu*s->mx - s->mv*s->my);
  }

  // Convert solution to ventral flow and divergence using camera intrinsics
  field->wx = (p[0] + intrinsics.principalPointX*p[2])/intrinsics.focalLengthX;
  field->wy = (p[1] + intrinsics.principalPointY*p[2])/intrinsics.focalLengthY;
  field->D  = 2*p[2];

  // If no problem was found, update is successful
  return UPDATE_SUCCESS;
}

/**
 * Simple derotation of the optic flow field parameters.
 *
 * @param field The flow field to be derotated.
 * @param rates The input body rates.
 */
void derotateFlowField(struct flowField* field, struct FloatRates* rates) {
  float p = rates->p;
  float q = rates->q;
  field->wxDerotated = field->wx - p;
  field->wyDerotated = field->wy - q;
}

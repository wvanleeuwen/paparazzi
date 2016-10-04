/*
 * flow_field_computation.c
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#include "flow_field_estimation.h"

float FLT_MIN_RESOLUTION = 1e-3;


void updateFlowStats(struct flowStats* s, struct flowEvent e, struct flowField lastField,
    float filterTimeConstant, float movingAverageWindow, float maxSpeedDifference) {
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

  // Limit outlier influence based on predicted flow magnitude
  float mPred = lastField.p[0]*u + lastField.p[1]*v + lastField.p[2]*w;
  float dm = m - mPred;
  float dmMax = powf(maxSpeedDifference,2);
  if (fabsf(dm) > dmMax) {
    tFactor *= fabsf(dmMax/dm);
  }

  // Compute weights
  float Wx = fabsf(u/sqrtf(m));
  float Wy = fabsf(v/sqrtf(m));

  // Update statistics, weighted by orientation
  s->mx  += (x * Wx - s->mx ) * tFactor;
  s->mu  += (u * Wx - s->mu ) * tFactor;
  s->mxx += (x * x * Wx - s->mxx) * tFactor;
  s->mxu += (x * u * Wx - s->mxu) * tFactor;

  s->my  += (y * Wy - s->my ) * tFactor;
  s->mv  += (v * Wy - s->mv ) * tFactor;
  s->myy += (y * y * Wy - s->myy) * tFactor;
  s->myv += (y * v * Wy - s->myv) * tFactor;

  s->mwx += (Wx - s->mwx) * tFactor;
  s->mwy += (Wy - s->mwy) * tFactor;

  s->sx  += (x - s->sx ) * tFactor;
  s->sy  += (y - s->sy ) * tFactor;
  s->sxx += (x * x - s->sxx) * tFactor;
  s->syy += (y * y - s->syy) * tFactor;
  s->sxy += (x * y - s->sxy) * tFactor;

  s->eventRate += (1/dt - s->eventRate) * tFactor;

  tPrevious = e.t;
}

/**
 * Use latest flow statistics to update flow field parameters.
 */
enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    float minEventRate, float minPosVariance,
    struct cameraIntrinsicParameters intrinsics) {

  // Quality checking for event rate
  if (s->eventRate < minEventRate) {
    return UPDATE_WARNING_RATE;
  }
  // Compute position variances
  float varX  = s->sxx - pow(s->sx,2);
  float varY  = s->syy - pow(s->sy,2);
  float covXY = s->sxy - s->sx*s->sy;

  // Compute variance eigenvalues analytically
  float d = powf(varX - varY,2) + 4*powf(covXY,2);
  if (d < 0) {
    return UPDATE_WARNING_SPREAD;
  }
  float eig1 = (varX + varY + sqrtf(d))/2;
  float eig2 = (varX + varY - sqrtf(d))/2;

  if (eig1 < minPosVariance || eig2 < minPosVariance) {
    return UPDATE_WARNING_SPREAD;
  }

  /*
   * The linear system to be solved is:
   *
   * [1,  sx,      0     [p[0]    [su
   *  sx, sxx+syy, sy  *  p[1]  =  sxu+syv
   *  0,  sy,      1]     p[2]]    sv]
   *
   *  where the entries are weighted by orientation.
   */

  // Compute determinant
  float D = pow(s->mx,2) + pow(s->my,2) - s->mxx - s->myy;
  if (fabs(D) < FLT_MIN_RESOLUTION) {
    return UPDATE_WARNING_SINGULAR;
  }
  field->p[0] = 1/D*(s->mu*s->my*s->my - s->mv*s->mx*s->my - s->mu*s->mxx
      - s->mu*s->myy + s->mxu*s->mx + s->mx*s->myv);
  field->p[1] = 1/D*(s->mv*s->mx*s->mx - s->mu*s->my*s->mx - s->mv*s->mxx
      - s->mv*s->myy + s->mxu*s->my + s->myv*s->my);
  field->p[2] = 1/D*(s->mu*s->mx + s->mv*s->my - s->mxu - s->myv);


  // Convert solution to ventral flow and divergence using camera intrinsics
  field->wx = (field->p[0] + intrinsics.principalPointX*field->p[2])/intrinsics.focalLengthX;
  field->wy = (field->p[1] + intrinsics.principalPointY*field->p[2])/intrinsics.focalLengthY;
  field->D  = 2*field->p[2];

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
  field->wyDerotated = field->wy + q;
}

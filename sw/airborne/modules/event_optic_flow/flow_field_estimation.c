/*
 * flow_field_computation.c
 *
 *  Created on: Jul 18, 2016
 *      Author: bas
 */

#include "flow_field_estimation.h"
#include "mcu_periph/sys_time.h"

float DET_MIN_RESOLUTION = 1;

void flowStatsInit(struct flowStats *s) {
  s->eventRate = 0;
  int32_t i;
  for (i = 0; i < N_FIELD_DIRECTIONS; i++) {
    s->ms[i] = 0;
    s->mss[i] = 0;
    s->mV[i] = 0;
    s->mVV[i] = 0;
    s->msV[i] = 0;
    s->N[i] = 0;
    s->tLast[i] = 0;
    s->angles[i] = ((float) i)/N_FIELD_DIRECTIONS*M_PI;
    s->cos_angles[i] = cosf(s->angles[i]);
    s->sin_angles[i] = sinf(s->angles[i]);
  }
}

void flowStatsUpdate(struct flowStats* s, struct flowEvent e, struct flowField lastField,
    float filterTimeConstant, float movingAverageWindow, float maxSpeedDifference,
    struct cameraIntrinsicParameters intrinsics) {
  // X,Y are defined around the camera's principal point
    float x = e.x - intrinsics.principalPointX;
    float y = e.y - intrinsics.principalPointY;
    float u = e.u;
    float v = e.v;

    // Dot product of position/flow
//    float w = x * u + y * v;
    // Squared flow magnitude
//    float m = u * u + v * v;

    // Find direction/index of flow
    float alpha = atan2f(v,u);
    alpha += M_PI / (2 * N_FIELD_DIRECTIONS);
    if (alpha < 0) {
      alpha += M_PI;
    }
    if (alpha >= M_PI) {
      alpha -= M_PI;
    }
    int32_t a = (int32_t) (N_FIELD_DIRECTIONS*alpha/M_PI);

    // Update stats through hybrid moving averaging/low-pass filtering
//    float dt = ((float)(e.t - s->tLast[a]))/1e6;
//    if (dt <= 0) {
//      dt = 1e-8;
//    }
//    float tFactor =  dt / filterTimeConstant;
//    // To prevent very large updates after a large dt, the update reduces to a moving average
//    if (tFactor > 1/movingAverageWindow)
//      tFactor = 1/movingAverageWindow;

    // Limit outlier influence based on predicted flow magnitude
//    float p[3] = {
//        lastField.wx * intrinsics.focalLengthX,
//        lastField.wy * intrinsics.focalLengthY,
//        lastField.D
//    };
//    float mPred = p[0]*u + p[1]*v + p[2]*w;
//    float dm = m - mPred;
//    float dmMax = powf(maxSpeedDifference,2);
//    if (fabsf(dm) > dmMax) {
//      tFactor *= fabsf(dmMax/dm);
//    }

    // Transform flow to direction reference frame
    float S = x * s->cos_angles[a] + y * s->sin_angles[a];
    float V = u * s->cos_angles[a] + v * s->sin_angles[a];

    // Update statistics only in the direction of the flow
    s->ms [a] += S;//(S   - s->ms [a]) * tFactor;
    s->mss[a] += S*S;//(S*S - s->mss[a]) * tFactor;
    s->mV [a] += V;//(V   - s->mV [a]) * tFactor;
    s->mVV[a] += V*V;//(V*V - s->mVV[a]) * tFactor;
    s->msV[a] += S*V;//(S*V - s->msV[a]) * tFactor;
    s->N[a] += 1;
    s->tLast[a] = e.t;
}

enum updateStatus recomputeFlowField(struct flowField* field, struct flowStats* s,
    float minEventRate, float minPosVariance, float minR2, float power,
    struct cameraIntrinsicParameters intrinsics) {

  // Compute rate confidence value
    float c_rate = 1;
    if (s->eventRate < minEventRate) {
      c_rate *= powf(s->eventRate / minEventRate, power);
    }

    uint32_t i;
    float varS[N_FIELD_DIRECTIONS];
    float c_var[N_FIELD_DIRECTIONS];
    float A[3][3] = {{0,0,0},{0,0,0},{0,0,0}};
    float Y[3] = {0,0,0};
    float sumV = 0;
    float sumVV = 0;
    float sumW = 0;

    // Loop over all directions and collect total flow field information
    for (i = 0; i < N_FIELD_DIRECTIONS; i++) {
      // Compute position variances and confidence values from mean statistics
      varS[i] = s->mss[i] - s->ms[i] * s->ms[i];
      c_var[i] = 1;
      if (varS[i] < minPosVariance) {
        c_var[i] *= powf(varS[i] / minPosVariance, power);
      }
      if (s->N[i] == 0) continue;
      s->N[i] *= c_var[i];
      // Fill in matrix entries (A is used as upper triangular matrix)
      A[0][0] += s->N[i] * s->cos_angles[i] * s->cos_angles[i];
      A[1][1] += s->N[i] * s->sin_angles[i] * s->sin_angles[i];
      A[2][2] += s->N[i] * s->mss[i];
      A[0][1] += s->N[i] * s->cos_angles[i] * s->sin_angles[i];
      A[0][2] += s->N[i] * s->cos_angles[i] * s->ms[i];
      A[1][2] += s->N[i] * s->sin_angles[i] * s->ms[i];
      Y[0] += s->N[i] * s->cos_angles[i] * s->mV[i];
      Y[1] += s->N[i] * s->sin_angles[i] * s->mV[i];
      Y[2] += s->N[i] * s->msV[i];
      sumV += s->N[i] * s->mV[i];
      sumVV += s->N[i] * s->mVV[i];
      sumW += s->N[i];
    }

    // Compute determinant
    float det = A[0][0] * A[1][1] * A[2][2]
        + 2*A[0][1] * A[0][2] * A[1][2]
        - A[0][0] * A[1][2] * A[1][2]
        - A[1][1] * A[0][2] * A[0][2]
          - A[2][2] * A[0][1] * A[0][1];
    if (fabs(det) < DET_MIN_RESOLUTION) {
      return UPDATE_WARNING_SINGULAR;
    }

    // Compute matrix inverse solution
    float p[3];
    p[0] = 1/det*(A[0][2]*Y[1]*A[1][2] - Y[0]*A[1][2]*A[1][2] - A[0][1]*Y[1]*A[2][2] + A[0][1]*A[1][2]*Y[2] - A[0][2]*A[1][1]*Y[2] + Y[0]*A[1][1]*A[2][2]);
    p[1] = 1/det*(Y[0]*A[0][2]*A[1][2] - Y[1]*A[0][2]*A[0][2] + A[0][1]*A[0][2]*Y[2] - Y[0]*A[0][1]*A[2][2] + A[0][0]*Y[1]*A[2][2] - A[0][0]*A[1][2]*Y[2]);
    p[2] = 1/det*(A[0][1]*A[0][2]*Y[1] - Y[2]*A[0][1]*A[0][1] + Y[0]*A[0][1]*A[1][2] - Y[0]*A[0][2]*A[1][1] - A[0][0]*Y[1]*A[1][2] + A[0][0]*A[1][1]*Y[2]);

    // To check coherence in the flow field, we compute the R2 fit value
    float residualSumSquares = sumVV - (p[0]*Y[0] + p[1]*Y[1] + p[2]*Y[2]);
    float totalSumSquares = sumVV - sumV*sumV/sumW;
    float R2 = 1 - residualSumSquares / totalSumSquares;
    float c_R2 = 1;
    if (R2 < minR2) {
      c_R2 *= powf(R2 / minR2, power);
      if (c_R2 < 0) {
        c_R2 = 0;
      }
    }

    // Convert solution to ventral flow and divergence using camera intrinsics
    float wx = p[0]/intrinsics.focalLengthX;
    float wy = p[1]/intrinsics.focalLengthY;
    float D  = p[2];

    // Now update the field parameters based on the confidence values
    float c_var_max = 0;
    for (i = 0; i < N_FIELD_DIRECTIONS; i++) {
      if (c_var[i] > c_var_max) {
        c_var_max = c_var[i];
      }
    }

    float c_total = c_rate * c_var_max * c_R2;
    field->wx += (wx - field->wx) * c_total;
    field->wy += (wy - field->wy) * c_total;
    field->D  += (D  - field->D ) * c_total;
    field->confidence = c_total;

    // If no problem was found, update is successful
    return UPDATE_SUCCESS;
}


void derotateFlowField(struct flowField* field, struct FloatRates* rates) {
  float p = rates->p;
  float q = rates->q;
  field->wxDerotated = field->wx - p;
  field->wyDerotated = field->wy + q;
}

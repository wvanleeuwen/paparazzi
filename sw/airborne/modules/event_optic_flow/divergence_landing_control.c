/*
 * divergence_landing_control.c
 *
 *  Created on: Sep 21, 2016
 *      Author: bas
 */

#include "divergence_landing_control.h"

#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/datalink/telemetry.h"
#include "paparazzi.h"
#include "firmwares/rotorcraft/stabilization.h"

#include <time.h>

void divergence_control_init(struct divergenceControlState *DCState) {
  divergence_control_reset(DCState);
  DCState->landing_thrust_fraction = 0.9f;
  DCState->lp_factor = 0.95;
  DCState->nominal_thrust = 0.710f; //TODO find value for MavTec
  DCState->pgain = 1.0f;
  DCState->divergence_setpoint = 0.0f;
}

void divergence_control_reset(struct divergenceControlState *DCState) {
  DCState->agl = 0.0f;
  DCState->agl_landing_limit = 0.5f;
  DCState->agl_lp = 0.0f;
  DCState->divergence = 0.0f;
  DCState->divergence_measured = 0.0f;
  DCState->divergence_new = false;
  DCState->vel = 0.0f;
  DCState->reset = true;
}

extern void divergence_control_update_setpoint(struct divergenceControlState *DCState, float DNew) {
  DCState->divergence_measured = DNew;
  DCState->divergence_new = true;
}

void divergence_control_run(bool in_flight, struct divergenceControlState *DCState){
  float lp_height; // low-pass height

  // ensure dt >= 0
  if (dt < 0) { dt = 0.0f; }

  // get delta time, dt, to scale the divergence measurements correctly when using "simulated" vision:
  struct timespec spec;
  clock_gettime(CLOCK_REALTIME, &spec);
  long new_time = spec.tv_nsec / 1.0E6;
  long delta_t = new_time - previous_time;
  dt += ((float)delta_t) / 1000.0f;
  if (dt > 10.0f) {
    dt = 0.0f;
    return;
  }
  previous_time = new_time;

  if (!in_flight) {
    // When not flying and in mode module:
    // Reset state
    divergence_control_reset(state);
  }
  else {

    /***************
     * MEASUREMENT
     ***************/

//    if (DCState->VISION_METHOD == 0) {
//
//      // SIMULATED DIVERGENCE USING OPTITRACK HEIGHT
//      DCState->agl = (float) gps.lla_pos.alt / 1000.0f;
//      // else we get an immediate jump in divergence when switching on.
//      if (DCState->agl_lp < 1E-5) {
//        DCState->agl_lp = DCState->agl;
//      }
//      if (fabs(DCState->agl - DCState->agl_lp) > 1.0f) {
//        // ignore outliers:
//        DCState->agl = DCState->agl_lp;
//      }
//      // calculate the new low-pass height and the velocity
//      lp_height = DCState->agl_lp * DCState->lp_factor + DCState->agl * (1.0f - DCState->lp_factor);
//
//      // only calculate velocity and divergence if dt is large enough:
//      if (dt > 0.0001f) {
//        DCState->vel = (lp_height - DCState->agl_lp) / dt;
//        DCState->agl_lp = lp_height;
//
//        // calculate the fake divergence:
//        if (DCState->agl_lp > 0.0001f) {
//          DCState->divergence = DCState->vel / DCState->agl_lp;
//        } else {
//          DCState->divergence = 1000.0f;
//          // perform no control with this value (keeping thrust the same)
//          return;
//        }
//        // reset dt:
//        dt = 0.0f;
//      }
//    }
//    else {
      // USE REAL VISION OUTPUTS:
      if (DCState->divergence_new && dt > 1E-5) {
        DCState->divergence = DCState->divergence_measured;
        DCState->divergence_new = false;
        dt = 0.0f;
      }
      else {
        // after re-entering the module, the divergence should be equal to the set point:
        if (DCState->reset) {
          DCState->divergence = DCState->divergence_setpoint;
          dt = 0.0f;
          int32_t nominal_throttle = DCState->nominal_thrust * MAX_PPRZ;
          stabilization_cmd[COMMAND_THRUST] = nominal_throttle;
          DCState->reset = false;
        }
        // else: do nothing, let dt increment
        return;
      }
//    }

    /***********
     * CONTROL
     ***********/

    int32_t nominal_throttle = DCState->nominal_thrust * MAX_PPRZ;

    // landing indicates whether the drone is already performing a final landing procedure (flare):
    if (!landing) {
        // use the divergence for control:
        float err = DCState->divergence_setpoint - DCState->divergence;
        int32_t thrust = nominal_throttle + DCState->pgain * err * MAX_PPRZ;

        // bound thrust:
        Bound(thrust, 0.8 * nominal_throttle, 0.75 * MAX_PPRZ);
        normalized_thrust = (float)(thrust / (MAX_PPRZ / 100));

        if (DCState->agl_lp < 1.0) {
        // land by setting 90% nominal thrust:
          landing = 1;
          thrust = DCState->landing_thrust_fraction * nominal_throttle;
        }
        stabilization_cmd[COMMAND_THRUST] = thrust;
    } else {
      // land with constant fraction of nominal thrust:
      int32_t thrust = DCState->landing_thrust_fraction * nominal_throttle;
      Bound(thrust, 0.6 * nominal_throttle, 0.9 * MAX_PPRZ);
      stabilization_cmd[COMMAND_THRUST] = thrust;
    }
  }
}


/*
 * divergence_landing_control.h
 *
 *  Created on: Sep 21, 2016
 *      Author: bas
 */

#ifndef SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_DIVERGENCE_LANDING_CONTROL_H_
#define SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_DIVERGENCE_LANDING_CONTROL_H_

#include <stdbool.h>

struct divergenceControlState {
  float agl;                    ///< agl = height from Optitrack
  float agl_lp;                 ///< low-pass version of agl
  float agl_landing_limit;      ///< minimum height before disabling constant divergence approach
  float lp_factor;              ///< low-pass factor in [0,1], with 0 purely using the current measurement
  float vel;                    ///< vertical velocity as determined with sonar (only used when using "fake" divergence)
  float divergence_setpoint;    ///< setpoint for constant divergence approach
  float divergence;             ///< divergence value used for control
  float divergence_measured;    ///< latest divergence measurement
  bool  divergence_new;         ///< new measurement available (should be set true whenever divergence_measured is changed)
  float pgain;                  ///< P-gain for constant divergence control (from divergence error to thrust)
  float nominal_thrust;         ///< nominal thrust around which the PID-control operates
  float landing_thrust_fraction;///< fraction of nominal thrust applied when landing
  bool  reset;                  ///< indicates if a reset was made before this iteration
};

extern void divergence_control_init(struct divergenceControlState *DCState);
extern void divergence_control_update_setpoint(struct divergenceControlState *DCState, float DNew);
extern void divergence_control_run(bool in_flight, struct divergenceControlState *DCState);

#endif /* SW_AIRBORNE_MODULES_EVENT_OPTIC_FLOW_DIVERGENCE_LANDING_CONTROL_H_ */

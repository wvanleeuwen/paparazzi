#include "modules/ATMOS/newTransition.h"
#include "modules/ATMOS/hoverPropsOff.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"
#include "math/pprz_algebra_int.h"
#include "firmwares/rotorcraft/actuators/supervision.h"
#include "modules/ATMOS/hoverPropsOff.h"
#include "firmwares/rotorcraft/force_allocation_laws.h"
#include "generated/airframe.h"

int32_t thrustActivationRatio = 0;
int32_t transveh_orig_thrust_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_THRUST_COEF;
//float transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR] = TRANSVEH_PREP_THRUST_COEF_SCALING;
int8_t transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR] = { -22, -22, 11, 11 };

int8_t transveh_propIsForwardThruster[SUPERVISION_NB_MOTOR] = { 0 , 0 , 1 , 1 };

//int8_t transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR] = { -22, -22, 44, 44 };
enum TransitionState transition_state;
enum TransitionState required_transition_state;
uint8_t transitionProgress;
int transition_setting;

void transveh_transition_init(void) {
  transition_state = HOVER;
  transitionProgress = 100;
  transition_percentage = 100;
  required_transition_state = HOVER;

  //transveh_orig_thrust_coef[SUPERVISION_NB_MOTOR] = SUPERVISION_THRUST_COEF;
  //transveh_prep_thrust_coef_scaling = TRANSVEH_PREP_THRUST_COEF_SCALING; //like {-0.2 -0.2 +0.4 +0.4}
  //transveh_prep_thrust_coef_scaling[SUPERVISION_NB_MOTOR]  = {-0.2, -0.2, +0.4, +0.4};
}

void transveh_transition_periodic(void) {
  if (required_transition_state == HOVER_NOW) {
    HoverPropsOn();
    PrepForTransitionToHover();
    transitionProgress = 100;
    transition_percentage = 100;
  }
  else if (required_transition_state == FORWARD && transitionProgress > 0) {

    if (pctIsBetween(transitionProgress,40,80))   //rotated 0.9*(100-80)=18 degrees, start moving thrust to forward props
      PrepForTransitionToForwardSmoothly((79-transitionProgress)*(100/39));
    else if (transitionProgress == 10)   //rotated 0.9*90=81 degrees, kill hover props
      HoverPropsOff();
    transitionProgress--;
    transition_percentage = transitionProgress;
  }
  else if (required_transition_state == SEMIFORWARD && transitionProgress > 60) {
   if (pctIsBetween(transitionProgress,60,90))
     PrepForTransitionToForwardSmoothly((89-transitionProgress)*(100/29));
   transitionProgress--;
    transition_percentage = transitionProgress;
  }
  else if (required_transition_state == HOVER && transitionProgress < 100) {
    if (transitionProgress == 10)   //rotated 0.9*10=9 degrees AOA, unkill hover props
      HoverPropsOn();
    if (transitionProgress == 50)   //rotated 9 till 0.9*50=45 degrees, more thrust on hover props
      PrepForTransitionToHover();

    transitionProgress+=2;
    transition_percentage = transitionProgress;
  }
}

bool_t pctIsBetween(uint8_t v,uint8_t l,uint8_t h) {
  return (v >= l && v < h);
}


void newTransition_doTransition(uint8_t val) {
  if (val == TRANSITION_HOVER)
    required_transition_state = HOVER;
  else if (val == TRANSITION_HOVER_EMERGENCY)
    required_transition_state = HOVER_NOW;
  else if (val == TRANSITION_FORWARD_HALF)
      required_transition_state = SEMIFORWARD;
  else
    required_transition_state = FORWARD;
}

/*
void transveh_transition_smooth_transition(int32_t desired, int32_t *actual) {
  if (   *actual > (desired - transitionBfpPerTick)
      && *actual < (desired + transitionBfpPerTick))     {
    *actual = desired;
  }
  if (desired != *actual) {
    if (*actual < desired) {
            *actual += transitionBfpPerTick;
    }
    if (*actual > desired) {
            *actual -= transitionBfpPerTick;
    }
  }
}*/


// amount between 0 and 100
void PrepForTransitionToForwardSmoothly(uint8_t amount) {
  for (uint8_t i = 0; i < SUPERVISION_NB_MOTOR; i++) {
    int32_t new_thrust_coef =   transveh_orig_thrust_coef[i]
                + (float)transveh_orig_thrust_coef[i]*(transveh_prep_thrust_coef_scaling[i]*(amount/10000));

    //int32_t new_thrust_coef = 256;
    if (new_thrust_coef < -SUPERVISION_SCALE) {
      thrust_coef[i] = -SUPERVISION_SCALE;
    }
    else if (new_thrust_coef > SUPERVISION_SCALE) {
          thrust_coef[i] = SUPERVISION_SCALE;
    }
    else {
      thrust_coef[i] = new_thrust_coef;
    }
  }
}


void PrepForTransitionToHover(void) {
  for (uint8_t i = 0; i < SUPERVISION_NB_MOTOR; i++) {
    thrust_coef[i] = transveh_orig_thrust_coef[i];
  }
}


//amount in PERCENTAGE frac
void newTransition_ThrustActivationRatioSet(uint32_t amount) {
  thrustActivationRatio = amount;
  /* ONLY CORRECT FOR POSITIVE THRUST COEFFICIENTS !!! */
  for (uint8_t i = 0; i < SUPERVISION_NB_MOTOR; i++) {
    int32_t new_thrust_coef;
    if (transveh_propIsForwardThruster[i]) {
      new_thrust_coef = transveh_orig_thrust_coef[i]
                        + (SUPERVISION_SCALE-transveh_orig_thrust_coef[i])*(amount/(1<<INT32_PERCENTAGE_FRAC));
    }
    else {
      new_thrust_coef = transveh_orig_thrust_coef[i]
                        - ((amount/(1<<INT32_PERCENTAGE_FRAC)) * transveh_orig_thrust_coef[i]);
    }

    //bounding stuff
    if (new_thrust_coef > SUPERVISION_SCALE) {
      thrust_coef[i] = SUPERVISION_SCALE;
      HoverPropsOn();
    }
    else if (new_thrust_coef <= 0) {
      thrust_coef[i] = 0;
      HoverPropsOff();
    }
    else {
      thrust_coef[i] = new_thrust_coef;
      HoverPropsOn();
    }
  }
}

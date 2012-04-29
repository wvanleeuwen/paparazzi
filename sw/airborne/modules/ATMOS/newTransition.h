#ifndef TRANSVEH_TRANSITION_H
#define TRANSVEH_TRANSITION_H
#include "firmwares/rotorcraft/force_allocation_laws.h"
#include "std.h"

#if !TRANSITION_PCT_PER_TICK
#define TRANSITION_PCT_PER_TICK 10
#endif

enum TransitionState { HOVER,
                       PREP_FOR_TRANSITION_ROTATE_TO_FORWARD,
                       PREP_FOR_TRANSITION_ROTATE_TO_HOVER,
                       PREP_FOR_TRANSITION_CHANGE_THRUST_RATIO_TO_FORWARD,
                       PREP_FOR_TRANSITION_CHANGE_THRUST_RATIO_TO_HOVER,
                       PREP_FOR_TRANSITION_HOVER_PROPS_OFF,
                       PREP_FOR_TRANSITION_HOVER_PROPS_ON,
                       FORWARD,
                       HOVER_NOW};

extern enum TransitionState transition_state;
extern enum TransitionState required_transition_state;

#define TRANSITION_HOVER 0
#define TRANSITION_FORWARD 1
#define TRANSITION_HOVER_EMERGENCY 2

extern int transition_setting;

extern void transveh_transition_init(void);
extern void transveh_transition_periodic(void);

extern bool_t pctIsBetween(uint8_t v,uint8_t l,uint8_t h);

extern void newTransition_doTransition(uint8_t val);

void PrepForTransitionToForwardSmoothly(uint8_t amount);
void PrepForTransitionToHover(void);


#endif

/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 *
 * @author Roland
 * follows based on stereo
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H
#include <inttypes.h>

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"

extern float ref_alt;

typedef enum{EXPLORE,HORIZONTAL_HOVER} demo_type;
extern demo_type demonstration_type;
extern float ref_disparity_to_keep;
extern uint8_t GO_FORWARD;
extern uint8_t STABILISE;
extern uint8_t TURN;
extern uint8_t INIT_FORWARD;
extern uint8_t current_state;

extern int stateGoForward(void);
extern int stateStabilise(void);
extern int stateTurn(void);
extern void allowedToChangeHeading(void);
extern void stereocam_forward_velocity_init(void);
extern void stereocam_forward_velocity_periodic(void);

#endif


/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_checker/green_checker.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to accept green pole (floor) in the cyberzoo
 */

#ifndef GREEN_CHECKER_H
#define GREEN_CHECKER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t safeToGoForwardsGreen;
extern float incrementForAcceptObjectsGreen;
extern uint16_t trajectoryConfidenceGreen;
extern void green_checker_init(void);
extern void green_checker_periodic(void);

extern uint8_t moveWaypointForwards_green(uint8_t, float);
extern uint8_t moveWaypointGreen(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_heading_green(int32_t *, int32_t);
extern uint8_t chooseRandomIncrementAcceptance_green(void);
extern uint8_t calculateForwardsGreen(struct EnuCoor_i *, float);

#endif


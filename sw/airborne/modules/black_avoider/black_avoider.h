/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/black_avoider/black_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef BLACK_AVOIDER_H
#define BLACK_AVOIDER_H
#include <inttypes.h>
#include "state.h"

extern uint8_t safeToGoForwardsBlack;
extern float incrementForAvoidanceBlack;
extern uint16_t trajectoryConfidenceBlack;
extern void black_avoider_init(void);
extern void black_avoider_periodic(void);
extern uint8_t moveWaypointForwardBlack(uint8_t, float);
extern uint8_t moveWaypointBlack(uint8_t, struct EnuCoor_i *);
extern uint8_t increase_nav_headingBlack(int32_t *, float);
extern uint8_t chooseRandomIncrementAvoidanceBlack(void);
extern uint8_t calculateForwardsBlack(struct EnuCoor_i *, float);

#endif


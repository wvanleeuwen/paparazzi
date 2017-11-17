/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef GREEN_CHECKER_H
#define GREEN_CHECKER_H
#include <inttypes.h>

extern uint8_t safeToGoForward;
extern int32_t incrementForAvoidanceObjects;
extern void green_checker_init(void);
extern void green_checker_periodic(void);
extern uint8_t moveWaypointForwards_green(uint8_t waypoint, float distanceMeters);
extern uint8_t increase_nav_heading_green(int32_t *heading, int32_t increment);
extern uint8_t chooseRandomIncrementAvoidance_green(void);

#endif


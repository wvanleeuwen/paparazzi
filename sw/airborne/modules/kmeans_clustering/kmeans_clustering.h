/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/kmeans_clustering/kmeans_clustering.h"
 */

#ifndef KMEANS_CLUSTERING_H
#define KMEANS_CLUSTERING_H
#include <inttypes.h>

//extern uint8_t safeToGoForward;
//extern int32_t incrementForAvoidanceObjects;
extern void kmeans_clustering_init(void);
extern void kmeans_clustering_periodic(void);
//extern uint8_t moveWaypointForwards_green(uint8_t waypoint, float distanceMeters);
//extern uint8_t increase_nav_heading_green(int32_t *heading, int32_t increment);
//extern uint8_t chooseRandomIncrementAvoidance_green(void);

#endif


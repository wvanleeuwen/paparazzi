/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/kmeans_clustering/kmeans_clustering.c"
 */

#include "modules/kmeans_clustering/kmeans_clustering.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

//uint8_t safeToGoForwards = false;
//int tresholdColorCounter = 200;
//int32_t incrementForAvoidanceObjects;

void kmeans_clustering_init()
{
  // Initialise random values
  //srand(time(NULL));
  //chooseRandomIncrementAvoidance();
	//printf("Initialized kmeans_clustering: on this moment this module is doing nothing yet.\n");
}
void kmeans_clustering_periodic()
{
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  //safeToGoaForwards = color_count < tresholdColorCounter;
  //printf("Checking if this function is called %d threshold: %d now: %d \n", color_count, tresholdColorCounter,
  //       safeToGoForwards);
	//printf("Periodically called kmeans_clustering: on this moment this module is doing nothing yet.\n");
}

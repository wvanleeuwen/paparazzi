/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/green_checker/green_checker.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/green_checker/green_checker.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "state.h"
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t safeToGoForwards = false;
int tresholdColorCounter = 200;
int32_t incrementForAvoidanceObjects;

void green_checker_init()
{
  // Initialise the variables of the colorfilter to accept green
  color_lum_min = 20;
  color_lum_max = 255;
  color_cb_min = 0;
  color_cb_max = 120;
  color_cr_min = 0;
  color_cr_max = 120;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}
void green_checker_periodic()
{
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  safeToGoForwards = color_count < tresholdColorCounter;
  printf("Checking if this function is called %d threshold: %d now: %d \n", color_count, tresholdColorCounter,
         safeToGoForwards);
}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading_green(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(*heading); // HEADING HAS INT32_ANGLE_FRAC....
  return false;
}
uint8_t moveWaypointForwards_green(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  struct EnuCoor_i *pos = stateGetPositionEnu_i(); // Get your current position

  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));

  // Now determine where to place the waypoint you want to go to
  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  new_coor.z = pos->z; // Keep the height the same

  // Set the waypoint to the calculated position
  waypoint_set_xy_i(waypoint, new_coor.x, new_coor.y);

  return false;
}

uint8_t chooseRandomIncrementAvoidance_green()
{

  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidanceObjects = 350;
  } else {
	incrementForAvoidanceObjects = -350;
  }
  return false;
}


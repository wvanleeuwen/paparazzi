/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/black_avoider/black_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "generated/flight_plan.h"
#include "modules/computer_vision/colorfilter.h"
#include "modules/black_avoider/black_avoider.h"

#define BLACK_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[black_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if BLACK_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t safeToGoForwardBlack        = false;
int tresholdColorCountBlack         = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidanceBlack;
uint16_t trajectoryConfidenceBlack  = 1;
float maxDistanceBlack               = 2.25;

/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void black_avoider_init()
{
	// Initialise the variables of the colorfilter to accept black
	  color_lum_min = 0;
	  color_lum_max = 10;
	  color_cb_min  = 105;
	  color_cb_max  = 135;
	  color_cr_min  = 115;
	  color_cr_max  = 146;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void black_avoider_periodic()
{
  // Check the amount of orange. If this is above a threshold
  // you want to turn a certain amount of degrees
  safeToGoForwardBlack = color_count < tresholdColorCountBlack;
  VERBOSE_PRINT("Color_count: %d  threshold: %d safe: %d \n", color_count, tresholdColorCountBlack, safeToGoForwardBlack);
  float moveDistance = fmin(maxDistanceBlack, 0.05 * trajectoryConfidenceBlack);
  if(safeToGoForwardBlack){
      moveWaypointForwardBlack(WP_GOAL, moveDistance);
      moveWaypointForwardBlack(WP_TRAJECTORY, 1.25 * moveDistance);
      nav_set_heading_towards_waypoint(WP_GOAL);
      chooseRandomIncrementAvoidanceBlack();
      trajectoryConfidenceBlack += 1;
  }
  else{
      waypoint_set_here_2d(WP_GOAL);
      waypoint_set_here_2d(WP_TRAJECTORY);
      increase_nav_headingBlack(&nav_heading, incrementForAvoidanceBlack);
      if(trajectoryConfidenceBlack > 5){
          trajectoryConfidenceBlack -= 4;
      }
      else{
          trajectoryConfidenceBlack = 1;
      }
  }
  return;
}

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_headingBlack(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwardsBlack(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypointBlack(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForwardBlack(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwardsBlack(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidanceBlack()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidanceBlack = 10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidanceBlack);
  } else {
    incrementForAvoidanceBlack = -10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidanceBlack);
  }
  return false;
}


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

#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "firmwares/rotorcraft/navigation.h"

#include "generated/flight_plan.h"
#include "modules/computer_vision/colorfilter.h"
#include "modules/green_checker/green_checker.h"

#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[green_checker->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t safeToGoForwardsGreen        = false;
int tresholdColorCounterGreen          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAcceptObjectsGreen;
uint16_t trajectoryConfidenceGreen   = 1;
float maxDistanceGreen              = 2.25;

void green_checker_init()
{
  // Initialise the variables of the colorfilter to accept green
  color_lum_min = 0;
  color_lum_max = 150;
  color_cb_min = 0;
  color_cb_max = 120;
  color_cr_min = 0;
  color_cr_max = 120;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAcceptance_green();
  //printf("green_checker_init; init green color only.\n");
}
void green_checker_periodic()
{
	//printf("### green_checker_periodic ####\n");
  // Check the amount of green. If this is under the threshold
  // you want to turn a certain amount of degrees
	safeToGoForwardsGreen = color_count > tresholdColorCounterGreen;

  //VERBOSE_PRINT("Color_count: %d  threshold: %d safe: %d \n", color_count, tresholdColorCounterGreen, safeToGoForwardsGreen);
    float moveDistance = fmin(maxDistanceGreen, 0.05 * trajectoryConfidenceGreen);
    if(safeToGoForwardsGreen){
    	moveWaypointForwards_green(WP_GOAL, moveDistance);
    	moveWaypointForwards_green(WP_TRAJECTORY, 1.25 * moveDistance);
        nav_set_heading_towards_waypoint(WP_GOAL);
        chooseRandomIncrementAcceptance_green();
        trajectoryConfidenceGreen += 1;
    }
    else{
        waypoint_set_here_2d(WP_GOAL);
        waypoint_set_here_2d(WP_TRAJECTORY);
        increase_nav_headingBlack(&nav_heading, incrementForAcceptObjectsGreen);
        if(trajectoryConfidenceGreen < 5){
            trajectoryConfidenceGreen -= 4;
        }
        else{
            trajectoryConfidenceGreen = 1;
        }
    }
    return;

}


/**
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading_green(int32_t *heading, int32_t incrementDegrees)
{
	  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL( incrementDegrees / 180.0 * M_PI);
	  // Check if your turn made it go out of bounds...
	  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
	  *heading = newHeading;
	  //VERBOSE_PRINT("Increasing heading to %f\n", ANGLE_FLOAT_OF_BFP(*heading) * 180 / M_PI);
	  return false;
}

/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
uint8_t calculateForwardsGreen(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  //VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y), ANGLE_FLOAT_OF_BFP(eulerAngles->psi)*180/M_PI);
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypointGreen(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  //VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}

/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForwards_green(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwardsGreen(&new_coor, distanceMeters);
  moveWaypointGreen(waypoint, &new_coor);
  return false;
}

uint8_t chooseRandomIncrementAcceptance_green()
{
	// not avoiding green, but accepting it;
	// when less green => probably a distortion -> can be obstacle
  int r = rand() % 2;
  if (r == 0) {
	  incrementForAcceptObjectsGreen = -350;
  } else {
	  incrementForAcceptObjectsGreen = 350;
  }
  return false;
}


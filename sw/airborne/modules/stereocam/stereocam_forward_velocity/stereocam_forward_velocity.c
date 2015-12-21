/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.c"
 * @author Roland
 * follows a person on the stereo histogram image.
 * It searches for the highest peak and adjusts its roll and pitch to hover at a nice distance.
 */

#include "modules/stereocam/stereocam_forward_velocity/stereocam_forward_velocity.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"
#include "subsystems/abi.h"

#include "subsystems/datalink/telemetry.h"

// Know waypoint numbers and blocks
#include "generated/flight_plan.h"
 float ref_pitch=0.0;
 float ref_roll=0.0;
void stereocam_forward_velocity_init()
{

}
void array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
  }
}
int disparity_velocity_step = 0;
int disparity_velocity_max_time = 500;
int distancesRecorded = 0;
int timeStepsRecorded = 0;
int velocity_disparity_outliers = 0;
float distancesHistory[500];
float timeStepHistory[500];
#define LENGTH_VELOCITY_HISTORY 6
float velocityHistory[LENGTH_VELOCITY_HISTORY];
int indexVelocityHistory=0;
float sumVelocities=0.0;
uint8_t GO_FORWARD=0;
uint8_t TURN=1;
uint8_t current_state=0;
int totalTurningSeenNothing=0;
uint8_t detectedWall=0;
float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity)
{
	    disparity_velocity_step += 1;
	    float new_dist = 0.0;
	    if (distancesRecorded > 0) {
	      new_dist = alpha * distancesHistory[distancesRecorded - 1] + (1 - alpha) * distance;
	    }
	    // Deal with outliers:
	    // Single outliers are discarded, while persisting outliers will lead to an array reset:
	    if (distancesRecorded > 0 && fabs(new_dist - distancesHistory[distancesRecorded - 1]) > 0.5) {
	      velocity_disparity_outliers += 1;
	      if (velocity_disparity_outliers >= MAX_SUBSEQUENT_OUTLIERS) {
	        // The drone has probably turned in a new direction
	        distancesHistory[0] = new_dist;
	        distancesRecorded = 1;

	        timeStepHistory[0] = disparity_velocity_step;
	        timeStepsRecorded = 1;
	        velocity_disparity_outliers = 0;
	      }
	    } else {
	        //append
	      velocity_disparity_outliers = 0;
	      timeStepHistory[timeStepsRecorded] = disparity_velocity_step;
	      distancesHistory[distancesRecorded] = new_dist;
	      distancesRecorded++;
	      timeStepsRecorded++;
	    }

	    //determine velocity (very simple method):
	    float velocityFound = 0.0;
	    if (distancesRecorded > n_steps_velocity) {
	      velocityFound = distancesHistory[distancesRecorded - n_steps_velocity] - distancesHistory[distancesRecorded - 1];
	    }
	    // keep maximum array size:
	    if (distancesRecorded > disparity_velocity_max_time) {
	    	array_pop(distancesHistory, disparity_velocity_max_time);
	    }
	    if (timeStepsRecorded > disparity_velocity_max_time) {
	    	array_pop(timeStepHistory, disparity_velocity_max_time);
	    }
	    return velocityFound;
}
void increase_nav_heading(int32_t *heading, int32_t increment);
void increase_nav_heading(int32_t *heading, int32_t increment)
{
  *heading = *heading + increment;
}
void stereocam_forward_velocity_periodic()
{
  if (stereocam_data.fresh) {
    stereocam_data.fresh = 0;
	uint8_t closest = stereocam_data.data[4];
    int horizontalVelocity = stereocam_data.data[8]-127;
    int upDownVelocity = stereocam_data.data[9] -127;
    float  BASELINE_STEREO_MM = 60.0;
    float BRANDSPUNTSAFSTAND_STEREO = 118.0 * 6.0 * 2.0;
	float dist = 5.0;
	if (closest > 0) {
	  dist = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)closest)) / 1000;
	}
	float velocityFound = calculateForwardVelocity(dist,0.65, 5,5);


    float guidoVelocityHor = horizontalVelocity/100.0;
    velocityHistory[indexVelocityHistory++]=guidoVelocityHor;
    if(indexVelocityHistory>=LENGTH_VELOCITY_HISTORY){
    	array_pop(velocityHistory,LENGTH_VELOCITY_HISTORY);
    	indexVelocityHistory--;
    }
    float sumVelocities=0.0;
    int indexH=0;
    for(indexH=0;indexH<indexVelocityHistory;indexH++){
    	sumVelocities+=velocityHistory[indexH];
    }
    guidoVelocityHor=sumVelocities/indexVelocityHistory;
    if (autopilot_mode != AP_MODE_NAV) {
    	sumVelocities=0.0;
     }
    sumVelocities+=guidoVelocityHor;
    int timeStamp = 0;
    //float guidoVelocityZ = upDownVelocity/100.0;
    float guidoVelocityZ=0.0;
    float noiseUs = 0.3f;
    DOWNLINK_SEND_STEREO_VELOCITY(DefaultChannel, DefaultDevice, &closest, &dist, &velocityFound,&guidoVelocityHor,&guidoVelocityZ,&current_state);

    AbiSendMsgVELOCITY_ESTIMATE(STEREO_VELOCITY_ID, timeStamp, velocityFound, guidoVelocityHor,
                                guidoVelocityZ,
                                noiseUs);
	ref_pitch=0.05;
    ref_roll=0.0;

    if(current_state==GO_FORWARD){
		if(closest>60){
			ref_pitch=0.2;
			detectedWall=1;
		}
		else if(closest>50){
			ref_pitch=0.1;
		}

		float p_gain = 0.4;
		float i_gain = 0.03;

		float max_roll=0.2;
		float rollToTake = p_gain * guidoVelocityHor+sumVelocities*i_gain;

		if(rollToTake>max_roll){
			ref_roll=max_roll;
		}
		else if(rollToTake<(-1.0*max_roll)){
			ref_roll=-(1.0*max_roll);
		}
		else{
			ref_roll=rollToTake;
		}

		if(velocityFound<0.5 && velocityFound>-0.5){
			if(closest < 60 && detectedWall){
				totalTurningSeenNothing=0;
				current_state=TURN;
				detectedWall=0;
			}
		}

    }
    else if(current_state==TURN){
    	if(autopilot_mode == AP_MODE_NAV){
    		increase_nav_heading(&nav_heading,860);
    	 }
    	if(closest<50){
    		totalTurningSeenNothing++;
    		if(totalTurningSeenNothing>0){
    			current_state=GO_FORWARD;
    			detectedWall=0;
    		}
    	}
    	else{
    		totalTurningSeenNothing=0;
    	}

    }
    else{
    	current_state=GO_FORWARD;
    }
    DOWNLINK_SEND_REFROLLPITCH(DefaultChannel, DefaultDevice, &ref_roll,&ref_pitch);

  }
}

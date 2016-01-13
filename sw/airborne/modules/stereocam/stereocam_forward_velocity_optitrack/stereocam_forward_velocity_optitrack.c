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

#include "modules/stereocam/stereocam_forward_velocity_optitrack/stereocam_forward_velocity_optitrack.h"
#include "modules/stereocam/stereocam.h"
#include "state.h"
#include "navigation.h"
#include "subsystems/abi.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"

#define AVERAGE_VELOCITY 0
// Know waypoint numbers and blocks
#include "generated/flight_plan.h"


 struct Gains{
	 float pGain;
	 float dGain;
	 float iGain;
 };
typedef struct Gains gains;

gains stabilisationLateralGains;
gains forwardLateralGains;

int turnFactors[]={300,300,300,200,100,100,100,100};
int countFactorsTurning=6;
int indexTurnFactors=0;


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

float sumHorizontalVelocities=0.0;
uint8_t GO_FORWARD=0;
uint8_t STABILISE=1;
uint8_t TURN=2;
uint8_t INIT_FORWARD=3;
uint8_t current_state=2;
int totalStabiliseStateCount = 0;
int totalTurningSeenNothing=0;
float previousLateralSpeed = 0.0;
uint8_t detectedWall=0;
float velocityAverageAlpha = 0.65;
float previousHorizontalVelocity = 0.0;
//#define DANGEROUS_CLOSE_DISPARITY 24
int DANGEROUS_CLOSE_DISPARITY=24;
//#define CLOSE_DISPARITY 18
#define INIT_CLOSE_DISP 18
int CLOSE_DISPARITY=INIT_CLOSE_DISP;
//#define LOW_AMOUNT_PIXELS_IN_DROPLET 20
int LOW_AMOUNT_PIXELS_IN_DROPLET=20;
float ref_disparity_to_keep=INIT_CLOSE_DISP;
float pitch_compensation = 0.0;
float roll_compensation=0.0;
int initFastForwardCount = 0;
int goForwardXStages=3;
int counterStab=0;
float previousStabRoll=0.0;
float ref_alt=1.0;
typedef enum{USE_DROPLET,USE_CLOSEST_DISPARITY} something;
demo_type demonstration_type = EXPLORE;
something sf = USE_DROPLET;
float headingStereocamStab=0.0;
float someGainWhenDoingNothing=0.0;

float somePitchGainWhenDoingNothing=0.0;
float previousStabPitch=0.0;
uint8_t initialisedTurn=0;
uint8_t turnMultiplier=1;


 int stateGoForward(){
return current_state==GO_FORWARD;
}
 int stateStabilise(){

	 return current_state==STABILISE;
}
 int stateTurn(){

	 return current_state==TURN;
}
void stereocam_forward_velocity_init()
{
	stabilisationLateralGains.pGain=0.6;
	stabilisationLateralGains.dGain=0.05;
	stabilisationLateralGains.iGain=0.01;
	forwardLateralGains.pGain=0.6;
}

void array_pop(float *array, int lengthArray);
void array_pop(float *array, int lengthArray)
{
  int index;
  for (index = 1; index < lengthArray; index++) {
    array[index - 1] = array[index];
  }
}

float calculateForwardVelocity(float distance,float alpha,int MAX_SUBSEQUENT_OUTLIERS,int n_steps_velocity);
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
void increase_nav_heading(int32_t *headingToChange, int32_t increment);
void increase_nav_heading(int32_t *headingToChange, int32_t increment)
{
  *headingToChange = *headingToChange + increment;
}

uint8_t dangerousClose(uint8_t closeValue){
	return closeValue>DANGEROUS_CLOSE_DISPARITY || closeValue==0;
}
uint8_t simplyClose(uint8_t closeValue){
	return closeValue>CLOSE_DISPARITY || closeValue==0;
}
void stereocam_forward_velocity_periodic()
{

  if (stereocam_data.fresh && stereocam_data.len>20) {
    stereocam_data.fresh = 0;
	uint8_t closest = stereocam_data.data[4];

	uint8_t disparitiesInDroplet = stereocam_data.data[5];
    int horizontalVelocity = stereocam_data.data[8]-127;
    int upDownVelocity = stereocam_data.data[9] -127;
	uint8_t disparityLeft = stereocam_data.data[10] ;
	uint8_t disparityRight = stereocam_data.data[11];

    float  BASELINE_STEREO_MM = 60.0;
    float BRANDSPUNTSAFSTAND_STEREO = 118.0 * 6.0 * 2.0;
	float dist = 5.0;
	if (closest > 0) {
	  dist = ((BASELINE_STEREO_MM * BRANDSPUNTSAFSTAND_STEREO / (float)closest)) / 1000;
	}
	float velocityFound = calculateForwardVelocity(dist,0.65, 5,5);

    float guidoVelocityHorStereoboard = horizontalVelocity/100.0;
    float guidoVelocityHor = 0.0;

    // Set the velocity to either the average of the last few velocities, or take the current velocity with alpha times the previous one
    guidoVelocityHor = guidoVelocityHorStereoboard*velocityAverageAlpha + (1-velocityAverageAlpha)*previousHorizontalVelocity;
    sumHorizontalVelocities+=guidoVelocityHor;
    previousHorizontalVelocity= guidoVelocityHorStereoboard;

    int timeStamp = 0;
    //float guidoVelocityZ = upDownVelocity/100.0;
    float guidoVelocityZ=0.0;
    float noiseUs = 0.3f;

	if(autopilot_mode != AP_MODE_NAV){
    	 ref_alt= -state.ned_pos_f.z;
    	 current_state=TURN;
    	 headingStereocamStab=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	 roll_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);
    	 pitch_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);
    }
    if(demonstration_type==HORIZONTAL_HOVER){
    	current_state=STABILISE;
    }

    float differenceD = guidoVelocityHor -previousLateralSpeed;
    previousLateralSpeed=guidoVelocityHor;
    counterStab++;
    if(current_state==GO_FORWARD){
    	  struct EnuCoor_i new_coor;
		  struct EnuCoor_i *pos = stateGetPositionEnu_i();
		  float sin_heading = sinf(ANGLE_FLOAT_OF_BFP(nav_heading));
		  float cos_heading = cosf(ANGLE_FLOAT_OF_BFP(nav_heading));
		  float NAV_LINE_AVOID_SEGMENT_LENGTH = 0.55;
		  new_coor.x = pos->x + POS_BFP_OF_REAL(sin_heading * (NAV_LINE_AVOID_SEGMENT_LENGTH));
		  new_coor.y = pos->y + POS_BFP_OF_REAL(cos_heading * (NAV_LINE_AVOID_SEGMENT_LENGTH));
		  new_coor.z = pos->z;
		  waypoint_set_xy_i(WP_STDBY, new_coor.x, new_coor.y);
		  NavSetWaypointHere(WP_last_wp);
    	if(sf==USE_CLOSEST_DISPARITY){
			if(dangerousClose(closest)){
				detectedWall=1;
			}
			else if(simplyClose(closest)){
				detectedWall=1;
			}
    	}
    	else{
    		if(disparitiesInDroplet>30){
    			detectedWall=1;
    		}
    	}

		float max_roll=0.25;
		float rollToTake = forwardLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;
		if(counterStab%4==0){

		}


		if(detectedWall){
			totalTurningSeenNothing=0;
			current_state=STABILISE;
			totalStabiliseStateCount=0;
			detectedWall=0;
		}

    }
    else if(current_state==STABILISE){

    	float stab_pitch_pgain=0.08;
    	float pitchDiff = closest- ref_disparity_to_keep;
    	float pitchToTake = stab_pitch_pgain*pitchDiff;
    	if(dangerousClose(closest)){
    		pitchToTake=0.2;
    	}
    	float max_roll=0.25;
		float rollToTake =stabilisationLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;

		if(counterStab%4==0){

			if(guidoVelocityHor<0.25 && guidoVelocityHor>-0.25){

				if(closest < (DANGEROUS_CLOSE_DISPARITY) && velocityFound <0){
					current_state=TURN;
					indexTurnFactors=0;
					initialisedTurn=0;
				}
			}
		}
		else{

		}

       }
    else if(current_state==TURN){
    	if(!initialisedTurn){
        	 initialisedTurn=1;
        	 if(disparityLeft>disparityRight-30){
        		 turnMultiplier=1;
        	 }
        	 else{
        		 turnMultiplier=-1;
        	 }
    	}


    	headingStereocamStab += turnMultiplier*5.0;
		  if (headingStereocamStab > 360.0){
			  headingStereocamStab -= 360.0;
		  }
		  if(headingStereocamStab < 0){
			  headingStereocamStab += 360;
		  }

    	indexTurnFactors+=1;
    	if(indexTurnFactors>countFactorsTurning){
    		indexTurnFactors = countFactorsTurning;
    	}
    	if(indexTurnFactors > 3){
    		if(sf==USE_CLOSEST_DISPARITY){
    			if(closest<CLOSE_DISPARITY){
					totalTurningSeenNothing++;
					if(totalTurningSeenNothing>2){
						current_state=GO_FORWARD;
						detectedWall=0;
					}
    			}
    		}
    		else{
    			if(disparitiesInDroplet<LOW_AMOUNT_PIXELS_IN_DROPLET){
    				totalTurningSeenNothing++;
					if(totalTurningSeenNothing>2){
						current_state=GO_FORWARD;
						detectedWall=0;
					}
    			}
    		}
    	}
    	else{
    		totalTurningSeenNothing=0;
    	}

    }
    else if(current_state==INIT_FORWARD){
    	initFastForwardCount++;
    	if(initFastForwardCount >= goForwardXStages){
    		initFastForwardCount = 0;
    		current_state=GO_FORWARD;
    	}
     }
    else{
    	current_state=GO_FORWARD;
    }
    nav_set_heading_deg(headingStereocamStab);

    DOWNLINK_SEND_STEREO_VELOCITY(DefaultChannel, DefaultDevice, &closest, &disparitiesInDroplet,&dist, &velocityFound,&guidoVelocityHor,&ref_disparity_to_keep,&current_state,&guidance_h_trim_att_integrator.x,&disparityLeft,&disparityRight);
//    DOWNLINK_SEND_REFROLLPITCH(DefaultChannel, DefaultDevice, &somePitchGainWhenDoingNothing,&ref_pitch);

  }
}

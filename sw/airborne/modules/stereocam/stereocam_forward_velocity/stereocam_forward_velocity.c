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
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "subsystems/datalink/telemetry.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_quat_int.h"
#include "subsystems/radio_control.h"


#define AVERAGE_VELOCITY 0
// Know waypoint numbers and blocks
#include "generated/flight_plan.h"
float ref_pitch=0.0;
float ref_roll=0.0;


 struct Gains{
	 float pGain;
	 float dGain;
	 float iGain;
 };
typedef struct Gains gains;

gains stabilisationLateralGains;
gains forwardLateralGains;


int indexTurnFactors=0;


int disparity_velocity_step = 0;
int disparity_velocity_max_time = 500;
int distancesRecorded = 0;
int timeStepsRecorded = 0;

int indexVelocityHistory=0;
float sumVelocities=0.0;

float sumHorizontalVelocities=0.0;
uint8_t GO_FORWARD=0;
uint8_t STABILISE=1;
uint8_t TURN=2;
uint8_t current_state=2;
int totalStabiliseStateCount = 0;
int totalTurningSeenNothing=0;
float previousLateralSpeed = 0.0;
uint8_t detectedWall=0;
float velocityAverageAlpha = 0.65;
float previousHorizontalVelocity = 0.0;
//#define DANGEROUS_CLOSE_DISPARITY 24
int DANGEROUS_CLOSE_DISPARITY=30;
//#define CLOSE_DISPARITY 18
#define INIT_CLOSE_DISP 22
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
something wayToDetermineState = USE_CLOSEST_DISPARITY;
float headingStereocamStab=0.0;
float previousStabPitch=0.0;
uint8_t initialisedTurn=0;
uint8_t turnMultiplier=1;
int timeInStableMode=0;
void stereocam_forward_velocity_init()
{
	stabilisationLateralGains.pGain=0.6;
	stabilisationLateralGains.dGain=0.05;
	stabilisationLateralGains.iGain=0.01;
	forwardLateralGains.pGain=0.6;
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

	ref_pitch=0.0;
    ref_roll=0.0;
    if(autopilot_mode != AP_MODE_NAV){
    	 ref_alt= -state.ned_pos_f.z;
    	 current_state=STABILISE;
    	 headingStereocamStab=ANGLE_FLOAT_OF_BFP(INT32_DEG_OF_RAD(stab_att_sp_euler.psi));
    	 roll_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.phi);
    	 pitch_compensation=ANGLE_FLOAT_OF_BFP(stab_att_sp_euler.theta);
    	 totalStabiliseStateCount=0;
    }
    if(demonstration_type==HORIZONTAL_HOVER){
    	current_state=STABILISE;
    }

    float differenceD = guidoVelocityHor -previousLateralSpeed;
    previousLateralSpeed=guidoVelocityHor;
    counterStab++;
    if(current_state==GO_FORWARD){
    	ref_pitch=-0.1;

    	if(wayToDetermineState==USE_CLOSEST_DISPARITY){
			if(dangerousClose(closest)){
				ref_pitch=0.2;
				detectedWall=1;
			}
			else if(simplyClose(closest)){
				ref_pitch=0.1;
				detectedWall=1;
			}
    	}
    	else{
    		if(disparitiesInDroplet>30){
    			ref_pitch=0.0;
    			detectedWall=1;
    		}
    	}

		float max_roll=0.25;
		float rollToTake = forwardLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;
		if(counterStab%4==0){
			if(rollToTake>max_roll){
				ref_roll=max_roll;
			}
			else if(rollToTake<(-1.0*max_roll)){
				ref_roll=-(1.0*max_roll);
			}
			else{
				ref_roll=rollToTake;
			}
		}


		if(detectedWall){
			totalTurningSeenNothing=0;
			current_state=STABILISE;
			totalStabiliseStateCount=0;
			detectedWall=0;
		}

    }
    else if(current_state==STABILISE){
    	totalStabiliseStateCount++;
    	float stab_pitch_pgain=0.04;
    	float pitchDiff = closest- ref_disparity_to_keep;
    	float pitchToTake = stab_pitch_pgain*pitchDiff;
    	if(dangerousClose(closest)){
    		pitchToTake=0.2;
    	}
    	ref_pitch=0.0;
		float max_roll=0.25;
		float rollToTake =stabilisationLateralGains.pGain * guidoVelocityHor;
		rollToTake*=-1;

		if(counterStab%4==0){
			if(rollToTake>max_roll){
				ref_roll=max_roll;
			}
			else if(rollToTake<(-1.0*max_roll)){
				ref_roll=-(1.0*max_roll);
			}
			else{
				ref_roll=rollToTake;
			}

			if(pitchToTake>0.15){
				ref_pitch=0.15;
			}
			else if (pitchToTake<-0.15){
				ref_pitch=-0.15;
			}
			else{
				ref_pitch=pitchToTake;
			}

			previousStabRoll=ref_roll;
			previousStabPitch=ref_pitch;


		    if(demonstration_type!=HORIZONTAL_HOVER){
				if(guidoVelocityHor<0.25 && guidoVelocityHor>-0.25){
					if((!dangerousClose(closest)) || totalStabiliseStateCount > 50 ){
						current_state=TURN;
						indexTurnFactors=0;
						initialisedTurn=0;
					}
				}
		    }
		}
		else{
			ref_pitch=previousStabPitch;
			ref_roll=0.5*previousStabRoll;
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

    	ref_pitch=0.0;
    	ref_roll=0.0;
    	headingStereocamStab += turnMultiplier*5.0;
		  if (headingStereocamStab > 360.0){
			  headingStereocamStab -= 360.0;
		  }
		  if(headingStereocamStab < 0){
			  headingStereocamStab += 360;
		  }

    	//increase_nav_heading(&nav_heading,turnFactors[indexTurnFactors]);
    	indexTurnFactors+=1;

    	if(indexTurnFactors > 3){
    		if(wayToDetermineState==USE_CLOSEST_DISPARITY){
    			if(!dangerousClose(closest) && !simplyClose(closest)){
					totalTurningSeenNothing++;
					current_state=GO_FORWARD;
					detectedWall=0;
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
    else{
    	current_state=GO_FORWARD;
    }
    nav_set_heading_deg(headingStereocamStab);

    ref_pitch += pitch_compensation;
    ref_roll += roll_compensation;

    if(ref_pitch>0.15){
		ref_pitch=0.15;
	}
	else if (ref_pitch<-0.15){
		ref_pitch=-0.15;
	}


    if(ref_roll>0.15){
    	ref_roll=0.15;
	}
	else if (ref_roll<-0.15){
		ref_roll=-0.15;
	}
    float addedRollJoystick = ((float)radio_control.values[RADIO_ROLL])/((float)MAX_PPRZ);//RADIO_CONTROL_NB_CHANNEL
    if(addedRollJoystick>0.25){
    	addedRollJoystick=0.25;
	}
	else if (addedRollJoystick<-0.25){
		addedRollJoystick=-0.25;
	}
    ref_roll+= addedRollJoystick;


    float addedPitchJoystick = ((float)radio_control.values[RADIO_PITCH])/((float)MAX_PPRZ);//RADIO_CONTROL_NB_CHANNEL
       if(addedPitchJoystick>0.25){
    	   addedPitchJoystick=0.25;
   	}
   	else if (addedPitchJoystick<-0.25){
   		addedPitchJoystick=-0.25;
   	}
    ref_pitch+= addedPitchJoystick;
    DOWNLINK_SEND_STEREO_VELOCITY(DefaultChannel, DefaultDevice, &closest, &disparitiesInDroplet,&dist, &guidoVelocityHor,&guidoVelocityHor,&ref_disparity_to_keep,&current_state,&totalStabiliseStateCount,&disparityLeft,&disparityRight,&stabilisationLateralGains.pGain);
    DOWNLINK_SEND_REFROLLPITCH(DefaultChannel, DefaultDevice, &ref_roll,&ref_pitch);
//*/
  }
}

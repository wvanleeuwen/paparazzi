/*
 * Copyright (C) Wilco Vlenterie
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/computer_vision/autoswarm/autoswarm.h"
 * @author Wilco Vlenterie
 * Autonomous bebop swarming module based on vision
 */

//#define BOARD_CONFIG "boards/bebop.h"

#include "autoswarm_opencv.h"

#include <math.h>					// Used for sin/cos/tan/M_PI
#include <ctime> 					// Used to write time to results.txt
#include <algorithm> 				// Used for min() and max()
#include <string>					// Used for strlen
//#include <random>

extern "C" {
#include <firmwares/rotorcraft/navigation.h>
#include <state.h>
//#include <generated/flight_plan.h>
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include <computer_vision/active_random_filter.h>

// Defining function that require opencv to be loaded
static void 			identifyNeighbours	(vector<trackResults> trackRes);
static void 			updateWaypoints 	(struct NedCoor_f *pos, vector<double> cPos, vector<double> totV);
static vector<double> 	calcLocalVelocity	(struct NedCoor_f *pos);
static vector<double> 	calcDiffVelocity	(void);
static vector<double> 	calcCamPosition		(struct NedCoor_f *pos, vector<double> totV, vector<double> gi);
static vector<double> 	calcGlobalVelocity 	(struct NedCoor_f *pos);
static vector<double> 	limitYaw 			(struct NedCoor_f *pos, vector<double> cPos);
static vector<double> 	limitVelocityYaw	(vector<double> totV);
static vector<double> 	limitNorm			(vector<double> totV, double maxNorm);
// Debug options
#define AUTOSWARM_SHOW_WAYPOINT 		 0 		// Show the updated positions of the waypoints
#define AUTOSWARM_SHOW_MEM			 0 		// Show the neighbours identified and their location

// Runtime options
#define AUTOSWARM_WRITE_RESULTS 		 0		// Write measurements to text file
#define AUTOSWARM_CALIBRATE_CAM 		 0 		// Calibrate the camera
#define AUTOSWARM_BENCHMARK 			 0 		// Print benchmark table
#define AUTOSWARM_MEASURE_FPS 			 1 		// Measure the FPS using built-in clock and framecount
#define AUTOSWARM_BODYFRAME 			 0 		// Fake euler angles and pos to be 0
#define AUTOSWARM_SAVE_FRAME 			 0 		// Save a frame for post-processing

//Optional function declarations
#if AUTOSWARM_BENCHMARK
static void 			initBenchmark		(void);
static void 			addBenchmark		(const char * title);
static void 			showBenchmark		(void);
#endif

// Set up swarm parameters
int 	AUTOSWARM_MODE 				= 2; // 0: follow (deprecated), 1: look in direction of flight, 2: look in direction of global component
double 	AUTOSWARM_SEPERATION 		= 1.75;
double 	AUTOSWARM_E 					= 0.005; // Was 0.01x - 0.0005 at 12m/s OK (but close)
double 	AUTOSWARM_EPS 				= 0.05;
double 	AUTOSWARM_GLOBAL 			= 0.9;
int    	AUTOSWARM_FPS 				= 15;
double 	AUTOSWARM_AMAX 				= 12; 	// m/s			// 4m/s op 90% global = CRASH!
double 	AUTOSWARM_VMAX 				= 12; 	// m/s
double 	AUTOSWARM_YAWRATEMAX 		= 70; 	// deg/s
int    	AUTOSWARM_MEMORY 			= 1.5; 	// seconds
double 	AUTOSWARM_HOME 				= 0.3;

// Set up global attractor parameters
int 	AUTOSWARM_ATTRACTOR			= 1;
double 	AUTOSWARM_CIRCLE_R 			= 2.25;
double 	AUTOSWARM_DEADZONE 			= 0.1;

// Initialize parameters to be assigned during runtime
static struct FloatEulers * eulerAngles;
static struct NedCoor_f * 	groundSpeed;
static struct NedCoor_f * 	pos;
vector<memoryBlock> 	neighbourMem;

static int 	runCount 				= 0;
int 	maxId 						= 0;
static const char * flight_blocks[] = FP_BLOCKS;
extern uint8_t nav_block;
extern vector<trackResults> trackRes;

#if AUTOSWARM_BENCHMARK
vector<double> benchmark_time;
vector<double> benchmark_avg_time;
vector<const char*> benchmark_title;
clock_t bench_start, bench_end;
#endif

#if AUTOSWARM_MEASURE_FPS
static time_t 	startTime;
static time_t 	currentTime;
static int 		curT;
#endif

#if AUTOSWARM_WRITE_RESULTS
#if !AUTOSWARM_MEASURE_FPS
time_t 	startTime;
time_t 	currentTime;
int 	curT;
#endif
FILE * 	pFile;
char	resultFile [50];
#endif

void autoswarm_opencv_init(int globalMode)
{
#if AUTOSWARM_MEASURE_FPS
	startTime 			= time(0);
#endif
#if AUTOSWARM_WRITE_RESULTS
#if !AUTOSWARM_MEASURE_FPS
	startTime 			= time(0);
#endif
	//tm * startTM 		= localtime(&startTime);
	//sprintf(resultFile, "/data/ftp/internal_000/%d-%02d-%02d_%02d-%02d-%02d.txt", startTM->tm_year, startTM->tm_mon, startTM->tm_mday, startTM->tm_hour, startTM->tm_min, startTM->tm_sec);
	//printf("Writing tracking results to: %s\n", resultFile);
	pFile = fopen("/data/ftp/internal_000/results.txt","w");
	fprintf(pFile,"ID\tmem\ti\tt\tposX\tposY\tposZ\tobjX\tobjY\tobjZ\tcPosX\tcposY\tvX\tvY\tgX\tgY\tlX\tlY\tdX\tdY\n");
	fclose(pFile);
#endif
	AUTOSWARM_ATTRACTOR = globalMode;
	printf("[AS] initialized\n");
	return;
}

void autoswarm_opencv_run()
{
	// Computer vision compatibility function used to call trackObjects and (optionally) parse modified data back to rtp stream as YUV
	//if(nav_block < 3){ return; }	// Engines have not started yet, lets save some battery life and skip image processing for now
#if AUTOSWARM_MEASURE_FPS
	if(runCount > 0)
	{
		currentTime 	= time(0); 												// Get the current time
		curT 			= difftime(currentTime,startTime); 						// Calculate time-difference between startTime and currentTime
		printf("Measured FPS: %0.2f\n", runCount / ((double) curT));
	}
#endif
#if AUTOSWARM_BENCHMARK
	if(runCount > 0)
	{
		addBenchmark("Lost time");
		showBenchmark();
	}
	initBenchmark();
#endif
	eulerAngles 			= stateGetNedToBodyEulers_f(); 	// Get Euler angles
	pos 					= stateGetPositionNed_f(); 		// Get your current position
	runCount++; 									// Update global run-counter
	groundSpeed = stateGetSpeedNed_f(); 			// Get groundspeed
#if AUTOSWARM_BENCHMARK
	addBenchmark("Declared variables");
#endif
	identifyNeighbours(trackRes); 	// ID neighbours according to previous location
#if AUTOSWARM_BENCHMARK
	addBenchmark("Identified neighbours");
#endif
	vector<double> totV(3), cPos, li, gi, di; 							// Initialize total contribution for output
	li 		= calcLocalVelocity(pos);									// Get the contribution due to the neighbours we see and have memorized
	gi 		= calcGlobalVelocity(pos); 									// Get the contribution due to the "attraction" towards global origin
	di 		= calcDiffVelocity(); 										// TODO: Include this?
	totV[0] = li[0] + gi[0] + di[0]; 									// Average the X local contribution (#neighbours independent) and add the X global contribution
	totV[1] = li[1] + gi[1] + di[1]; 									// Do the same for Y
	totV[2] = li[2] + gi[2] + di[2]; 									// Do the same for Z
	totV 	= limitNorm(totV, AUTOSWARM_AMAX); 							// Check if ideal velocity exceeds AUTOSWARM_AMAX
	totV[0] = groundSpeed->x + totV[0]; 								// Inputting totV as an acceleration
	totV[1] = groundSpeed->y + totV[1]; 								// Inputting totV as an acceleration
	totV 	= limitNorm(totV, AUTOSWARM_VMAX); 							// Limit it again (otherwise speed could escalate)
	if(AUTOSWARM_MODE!=2) totV = limitVelocityYaw(totV); 				// Limit the velocity when relative angle gets larger
	cPos 	= calcCamPosition(pos, totV, gi); 							// Calculate CAM/heading
	cPos 	= limitYaw(pos, cPos); 										// Limit yaw
#if AUTOSWARM_BENCHMARK
	addBenchmark("Calculated swarming dynamics");
#endif
	updateWaypoints(pos,cPos,totV); 									// Translate velocity contribution to updated waypoint
	if(!strcmp("Swarm",flight_blocks[nav_block]) || !strcmp("Swarm Home",flight_blocks[nav_block]))
	{
		nav_set_heading_towards_waypoint(WP__CAM); 					// Currently in block "Swarm" or "Swarm Home" so update heading
	}
#if AUTOSWARM_BENCHMARK
	addBenchmark("Updated waypoints and heading");
#endif
#if AUTOSWARM_WRITE_RESULTS 												// See if we want to write results
#if !AUTOSWARM_MEASURE_FPS
	currentTime = time(0); 												// Get the current time
	curT 		= difftime(currentTime,startTime); 						// Calculate time-difference between startTime and currentTime
#endif
	pFile 		= fopen("/data/ftp/internal_000/results.txt","a");		// Open file for appending TODO: Check for errors opening file
#endif
#if AUTOSWARM_WRITE_RESULTS || AUTOSWARM_SHOW_MEM
		for(unsigned int r=0; r < neighbourMem.size(); r++) 			// Print to file & terminal
		{
#if AUTOSWARM_SHOW_MEM
			char memChar = 'm';
			if(neighbourMem[r].lastSeen == runCount){ memChar = 'w';}
#endif
#if AUTOSWARM_WRITE_RESULTS
			int memInt = 0;
			if(neighbourMem[r].lastSeen == runCount){ memInt = 1; }
#endif
#if AUTOSWARM_SHOW_MEM
			printf("%i - Object (%c %i) at (%0.2f m, %0.2f m, %0.2f m) pos(%0.2f, %0.2f, %0.2f)\n", runCount, memChar, neighbourMem[r].id, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, pos->x, pos->y, pos->z); 														// Print to terminal
#endif
#if AUTOSWARM_WRITE_RESULTS
			fprintf(pFile, "%i\t%i\t%i\t%i\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\n", neighbourMem[r].id, memInt, runCount, curT, pos->x, pos->y, pos->z, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, cPos[0], cPos[1], totV[0], totV[1], gi[0], gi[1], li[0], li[1], di[0], di[1]); // if file writing is enabled, write to file
#endif
		}
#endif
#if AUTOSWARM_WRITE_RESULTS
	fclose(pFile);	// Close file
#if AUTOSWARM_BENCHMARK
	addBenchmark("Wrote to terminal/file");
#endif
#endif
	if((AUTOSWARM_SHOW_MEM==1 && neighbourMem.size() > 0) || AUTOSWARM_SHOW_WAYPOINT || AUTOSWARM_BENCHMARK) printf("\n"); // Separate terminal output by newline
#if AUTOSWARM_BENCHMARK
	addBenchmark("Complete");
#endif
	return;
}

void identifyNeighbours(vector<trackResults> trackRes)
{
	// First lets clear the old elements which we will no longer be using
	for(unsigned int i=0; i < neighbourMem.size();)
	{
		if(runCount - neighbourMem[i].lastSeen > AUTOSWARM_MEMORY * AUTOSWARM_FPS)
		{
			neighbourMem.erase(neighbourMem.begin() + i);
		}else 	i++;
	}
	// We now only have memory samples from the past ~2 seconds so lets try to identify the neighbours we saw
	bool identified;
	for(unsigned int r=0; r < trackRes.size(); r++)
	{
		identified = false;
		for(unsigned int i=0; i < neighbourMem.size(); i++)
		{
			double radius	= (runCount - neighbourMem[i].lastSeen) * AUTOSWARM_VMAX / ((double) AUTOSWARM_FPS);
			double dx 		= trackRes[r].x_w - neighbourMem[i].x_w;
			if(dx <= radius)
			{
				double dy 	= trackRes[r].y_w - neighbourMem[i].y_w;
				if(dy <= radius)
				{
					if(sqrt(pow(dx, 2.0) + pow(dy, 2.0)) <= radius)
					{
						neighbourMem[i].lastSeen 	= runCount;
						neighbourMem[i].x_w 		= trackRes[r].x_w;
						neighbourMem[i].y_w 		= trackRes[r].y_w;
						neighbourMem[i].z_w 		= trackRes[r].z_w;
						neighbourMem[i].x_p 		= trackRes[r].x_p;
						neighbourMem[i].y_p 		= trackRes[r].y_p;
						identified = true;
						break;
					}
				}
			}
		}
		if(identified == false)
		{
			memoryBlock curN;
			curN.lastSeen 	= runCount;
			curN.id 		= maxId;
			curN.x_w 		= trackRes[r].x_w;
			curN.y_w 		= trackRes[r].y_w;
			curN.z_w 		= trackRes[r].z_w;
			curN.x_p 		= trackRes[r].x_p;
			curN.y_p 		= trackRes[r].y_p;
			neighbourMem.push_back(curN);
			maxId++;
		}
	}
	return;
}

vector<double> calcLocalVelocity(struct NedCoor_f *pos)
{
	double li_fac=0, range; 		// Initialize variables
	vector<double> li(3); li[0] = 0; li[1] = 0; li[2] = 0; 	// Eclipse was complaining about vector<double> li {0 0 0}; declaration
	unsigned int r; // Initialize r
	for(r=0; r < neighbourMem.size(); r++) // For all neighbours found
	{
		range = sqrt(pow((pos->x - neighbourMem[r].x_w), 2.0) +  pow((pos->y - neighbourMem[r].y_w), 2.0));
		li_fac = 12 * AUTOSWARM_E / range * (pow(AUTOSWARM_SEPERATION / range, 12.0) - pow(AUTOSWARM_SEPERATION / range, 6.0)); // Local contribution scaling factor
		li[0] += li_fac * (pos->x - neighbourMem[r].x_w) / range; // Local contribution in x
		li[1] += li_fac * (pos->y - neighbourMem[r].y_w) / range; // Local contribution in y
		//li[2] += li_fac * (pos->z - neighbourMem[r].z_w) / trackRes[r].r_c; // Local contribution in z
	}
	if (r > 0) 	// Only if we found anyone
	{
		li[0] = li[0] / r; 				// Average the X local contribution (#neighbours independent) and add the X global contribution
		li[1] = li[1] / r; 				// Do the same for Y
		li[2] = li[2] / r; 				// Do the same for Z
	}else{
		// TODO: What if I don't see anyone?
		//setGlobalOrigin(0, 0, 1.25); // Set origin to middle of field (see if we can spot neighbours) but maintain a safe altitude (TODO: change from fixed 1.25m)
	}
	return li; // Return the total contribution
}

vector<double> calcGlobalVelocity(struct NedCoor_f *pos)
{
	vector<double> gi(3);
	switch(AUTOSWARM_ATTRACTOR)
	{
	case AUTOSWARM_POINT :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > AUTOSWARM_DEADZONE)
		{
			gi[0] 			= AUTOSWARM_GLOBAL * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			gi[1] 			= AUTOSWARM_GLOBAL * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
			gi[2] 			= 0;
		}else{
			gi[0] = 0.0;
			gi[1] = 0.0;
			gi[2] = 0.0;
		}
		break;
	}
	case AUTOSWARM_BUCKET :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > AUTOSWARM_DEADZONE)
		{
			double gScalar 	= AUTOSWARM_GLOBAL * (1 - 1 / (1 + exp(4 / AUTOSWARM_SEPERATION * (cr - AUTOSWARM_SEPERATION))));
			gi[0] 			= gScalar * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			gi[1] 			= gScalar * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
			gi[2] 			= 0;
		}else{
			gi[0] = 0.0;
			gi[1] = 0.0;
			gi[2] = 0.0;
		}
		break;
	}
	case AUTOSWARM_CIRCLE_CW :
	case AUTOSWARM_CIRCLE_CC :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > AUTOSWARM_DEADZONE)
		{
			double angle;
			if(cr >= AUTOSWARM_CIRCLE_R)
			{
				angle = 90 * AUTOSWARM_CIRCLE_R / cr;
			}else{
				angle = 90 + 90 * (AUTOSWARM_CIRCLE_R - cr) / AUTOSWARM_CIRCLE_R;  // From 90 to 180 over the length of AUTOSWARM_CIRCLE_R
			}
			if (AUTOSWARM_ATTRACTOR != AUTOSWARM_CIRCLE_CC) 	angle = -angle; 	// Switch between cc (+) and cw (-)
			double xContrib = AUTOSWARM_GLOBAL * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			double yContrib = AUTOSWARM_GLOBAL * AUTOSWARM_AMAX / ((double) AUTOSWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
			gi[0] 			= cos(angle / 180 * M_PI) * xContrib - sin(angle / 180 * M_PI) * yContrib;
			gi[1] 			= sin(angle / 180 * M_PI) * xContrib + cos(angle / 180 * M_PI) * yContrib;
			gi[2] 			= 0;
		}else{
			gi[0] = 0.0;
			gi[1] = 0.0;
			gi[2] = 0.0;
		}
		break;
	}
	default : gi[0] = 0.0; gi[1] = 0.0; gi[2] = 0.0; break;
	}
	return gi;
}

vector<double> calcDiffVelocity(void)
{
	vector<double> di(3);
	di[0] = - AUTOSWARM_EPS * groundSpeed->x / ((double) AUTOSWARM_FPS);
	di[1] = - AUTOSWARM_EPS * groundSpeed->y / ((double) AUTOSWARM_FPS);
	di[2] = 0; //- AUTOSWARM_EPS * groundSpeed->z;
	return di;
}

vector<double> calcCamPosition(struct NedCoor_f *pos, vector<double> totV, vector<double> gi)
{
	vector<double> cPos(3); cPos[0] = 0; cPos[1] = 0; cPos[2] = 0;

	if(AUTOSWARM_MODE==0) // Follow mode, find c.g. of neighbours, point camera there and set global position there
	{
		double x_total=0, y_total=0, z_total=0; 		// Initialize variables
		unsigned int r; 					// Initialize r
		for(r=0; r < neighbourMem.size(); r++) // For all neighbours found
		{
			x_total += neighbourMem[r].x_w; // Calculate to find global origin
			y_total += neighbourMem[r].y_w; // Calculate to find global origin
			z_total += neighbourMem[r].z_w; // Calculate to find global origin
		}
		if(r>0)
		{
			setGlobalOrigin(x_total / r, y_total / r, 1);
		}
		cPos[0] = x_total / r;					// Find average X position of neighbours
		cPos[1] = y_total / r;					// Same for Y
		cPos[2] = z_total / r; 					// Same for Z
	}
	if(AUTOSWARM_MODE == 1) // Watch where you're going mode. Point WP_CAM on unity circle towards totV
	{
		double velR = sqrt(pow(totV[0], 2.0) + pow(totV[1], 2.0));
		if(velR > 0)
		{
			cPos[0] = pos->x + totV[0] / velR;
			cPos[1] = pos->y + totV[1] / velR;
			cPos[2] = pos->z;
		}else{
			cPos[0] = pos->x;
			cPos[1] = pos->y;
			cPos[2] = pos->z;
		}
	}
	if(AUTOSWARM_MODE == 2) // Watch the global objective mode. Point WP_CAM on unity circle towards global component
	{
		double velR = sqrt(pow(gi[0], 2.0) + pow(gi[1], 2.0));
		if(velR > 0)
		{
			cPos[0] 	= pos->x + gi[0] / velR;
			cPos[1] 	= pos->y + gi[1] / velR;
			cPos[2] 	= pos->z;
		}else
		{
			cPos[0] 	= pos->x + cos(eulerAngles->psi) * 1;
			cPos[1] 	= pos->y + sin(eulerAngles->psi) * 1;
			cPos[2] 	= pos->z;
		}
	}
	return cPos;
}

vector<double> limitYaw(struct NedCoor_f *pos, vector<double> cPos)
{
	double cameraHeading 	= atan2(cPos[1] - pos->y, cPos[0] - pos->x);
	double relHeading 		= cameraHeading - eulerAngles->psi; // Z axis is defined downwards for psi so * -1 for the atan2
	if(relHeading > M_PI) 	relHeading -= 2 * M_PI;
	if(relHeading < -M_PI) 	relHeading += 2 * M_PI;
	if(abs(relHeading) > AUTOSWARM_YAWRATEMAX  / ((double) AUTOSWARM_FPS) / 180 * M_PI)
	{
		double newHeading = relHeading/abs(relHeading) * (AUTOSWARM_YAWRATEMAX / ((double) AUTOSWARM_FPS) / 180 * M_PI) + eulerAngles->psi;
		cPos[0] 	= pos->x + cos(newHeading) * 1;
		cPos[1] 	= pos->y + sin(newHeading) * 1;
		cPos[2] 	= pos->z;
		//printf("DES YAW: %0.2f, ALL YAW: %0.2f, YAW: %0.2f, CAMYAW: %0.2f, NEWYAW: %0.2f OR %0.2f\n", relHeading * 180 / M_PI, (newHeading - eulerAngles->psi) * 180 / M_PI, eulerAngles->psi * 180 / M_PI, cameraHeading * 180 / M_PI, newHeading * 180 / M_PI, atan2(cPos[1] - pos->y, cPos[0] - pos->x) * 180 / M_PI);
	}
	return cPos;
}

vector<double> limitVelocityYaw(vector<double> totV)
{
	double totVHeading 	= atan2(totV[1], totV[0]);
	double relHeading 	= totVHeading - eulerAngles->psi; // Z axis is defined downwards for psi so * -1 for the atan2
	double vTurn;
	//printf("Speed mag: %0.4f m/s\tvHead: %0.2f deg\tPsi: %0.2f deg\trelHeading: %0.2f deg\t",sqrt(pow(totV[0], 2.0) + pow(totV[1], 2.0)), totVHeading * 180 / M_PI, eulerAngles->psi * 180 / M_PI, relHeading * 180 / M_PI);
	if(abs(relHeading) > acos(0.01))	vTurn = 0.01 * sqrt(pow(totV[0], 2.0) + pow(totV[1], 2.0));
	else								vTurn = cos(relHeading) * sqrt(pow(totV[0], 2.0) + pow(totV[1], 2.0));
	//printf("Resized: %0.6f m/s\n", vTurn);
	totV[0] = cos(totVHeading) * vTurn;
	totV[1] = sin(totVHeading) * vTurn;
	return totV;
}

vector<double> limitNorm(vector<double> totV, double maxNorm)
{
	double vR = sqrt(totV[0] * totV[0] + totV[1] * totV[1]);// + totV[2] * totV[2]); // Find the distance to our new desired position
	if (vR > maxNorm) // Is the new desired position unachievable due to velocity constraint?
	{
		totV[0] = maxNorm * totV[0] / vR; // Scale the totV X component so that ||totV|| = maxNorm
		totV[1] = maxNorm * totV[1] / vR; // Do the same for Y
		totV[2] = 0; //AUTOSWARM_VMAX / ((double) AUTOSWARM_FPS) * totV[2] / vR; // Do the same for Z
	}
	return totV;
}

void updateWaypoints(struct NedCoor_f *pos, vector<double> cPos, vector<double> totV)
{
	// WATCH OUT! waypoint_set_xy_i is in ENU so N(x) and E(y) axis are changed compared to x_w and y_w
	waypoint_set_xy_i(WP__GOAL, POS_BFP_OF_REAL(pos->y + totV[1]), POS_BFP_OF_REAL(pos->x + totV[0]));						// Update WP_GOAL waypoint to add totV relative to our position
	waypoint_set_xy_i(WP__CAM, POS_BFP_OF_REAL(cPos[1]), POS_BFP_OF_REAL(cPos[0]));						// Update WP_GOAL waypoint to add totV relative to our position
	//waypoint_set_xy_i(WP_GLOBAL, POS_BFP_OF_REAL(globalOrigin.cy), POS_BFP_OF_REAL(globalOrigin.cx)); 						// Update WP_GLOBAL waypoint to globalOrigin
	setGlobalOrigin(waypoint_get_y(WP_GLOBAL), waypoint_get_x(WP_GLOBAL), 1);
	if(AUTOSWARM_SHOW_WAYPOINT) printf("WP_CAM (%0.2f m, %0.2f m) \tWP_GOAL (%0.2f m, %0.2f m) \tWP_GLOBAL (%0.2f m, %0.2f m)\n", cPos[0], cPos[1], pos->x + totV[0], pos->y + totV[1], globalOrigin.cx, globalOrigin.cy);	//, pos->z + totV[2]);	// Print to terminal
}

bool amIhome(void){
	struct EnuCoor_f *pos = stateGetPositionEnu_f();
	if(sqrt(pow(pos->x - waypoint_get_x(WP__TD),2.0) + pow(pos->y - waypoint_get_y(WP__TD),2.0)) < AUTOSWARM_HOME)
	{
		return true;
	}else{
		return false;
	}
}

#if AUTOSWARM_BENCHMARK
void initBenchmark()
{
	bench_start = clock();
	benchmark_time.clear();
	benchmark_title.clear();
}

void addBenchmark(const char * title)
{
	bench_end 	= clock();
	double time = ((double) (bench_end - bench_start)) / 1000000;
	bench_start = bench_end;
	benchmark_time.push_back(time);
	benchmark_title.push_back(title);
}

void showBenchmark()
{
	double totalTime = 0;
	unsigned int maxlength = 0;
	for(unsigned int i=0; i < benchmark_time.size(); i++)
	{
		totalTime += benchmark_time[i];
		if(runCount==1)
		{
			benchmark_avg_time.push_back(0);
		}
		if(strlen(benchmark_title[i]) > maxlength)
		{
			maxlength = strlen(benchmark_title[i]);
		}
	}
	for(unsigned int i=0; i < benchmark_time.size(); i++)
	{
		benchmark_avg_time[i] = (benchmark_avg_time[i] * (runCount - 1) + benchmark_time[i]/totalTime*100) / runCount;
		if(strlen(benchmark_title[i]) < maxlength - 18)
		{
			printf("%s\t\t\t\t%2.2f percent\n", benchmark_title[i], benchmark_avg_time[i]);
		}else{
			if(strlen(benchmark_title[i]) < maxlength - 12)
			{
				printf("%s\t\t\t%2.2f percent\n", benchmark_title[i], benchmark_avg_time[i]);
			}else{
				if(strlen(benchmark_title[i]) < maxlength - 5)
				{
					printf("%s\t\t%2.2f percent\n", benchmark_title[i], benchmark_avg_time[i]);
				}else{
					printf("%s\t%2.2f percent\n", benchmark_title[i], benchmark_avg_time[i]);
				}
			}
		}

	}
}
#endif

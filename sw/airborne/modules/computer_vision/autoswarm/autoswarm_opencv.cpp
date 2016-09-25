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
#include <subsystems/navigation/waypoints.h>
#include <state.h>
//#include <generated/flight_plan.h>
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

// Defining function that require opencv to be loaded
static void 			trackGreyObjects	(Mat& sourceFrame, Mat& greyFrame, vector<trackResults>* trackRes);
static void 			addContour			(vector<Point> contour, vector<trackResults>* trackRes);
static void 			identifyNeighbours	(vector<trackResults> trackRes);
static void 			updateWaypoints 	(struct NedCoor_f *pos, vector<double> cPos, vector<double> totV);
static void 			cam2body 			(trackResults* trackRes);
static void 			body2world 			(trackResults* trackRes, struct NedCoor_f *pos, struct 	FloatEulers * eulerAngles);
static bool 			rndRedGrayscale		(Mat& sourceFrame, Mat& destFrame, int sampleSize);
static bool 			pixFindContour		(Mat& sourceFrame, Mat& destFrame, int row, int col, int prevDir, bool cascade);
static double 			correctRadius		(double r, double f, double k);
static Rect 			setISPvars 			(int width, int height, bool crop = false);
static vector<double> 	calcLocalVelocity	(struct NedCoor_f *pos);
static vector<double> 	calcDiffVelocity	(void);
static vector<double> 	calcCamPosition		(struct NedCoor_f *pos, vector<double> totV, vector<double> gi);
static vector<double> 	estimatePosition	(int xp, int yp, double area, double k = 0, int calArea = 0, int orbDiag = 0);
static vector<double> 	calcGlobalVelocity 	(struct NedCoor_f *pos);
static vector<double> 	limitYaw 			(struct NedCoor_f *pos, vector<double> cPos);
static vector<double> 	limitVelocityYaw	(vector<double> totV);
static vector<double> 	limitNorm			(vector<double> totV, double maxNorm);
static void 			addObject			(void);
static Rect 			enlargeRectangle	(Mat& sourceFrame, Rect rectangle, double scale);
static bool 			inRectangle			(Point pt, Rect rectangle);

// Debug options
#define WV_DEBUG_SHOW_REJECT 		 0 		// Show why shapes are rejected
#define WV_DEBUG_SHOW_WAYPOINT 		 0 		// Show the updated positions of the waypoints
#define WV_DEBUG_SHOW_MEM			 0 		// Show the neighbours identified and their location

// Runtime options
#define WV_OPT_WRITE_RESULTS 		 0		// Write measurements to text file
#define WV_OPT_MOD_VIDEO 			 0 		// Modify the frame to show relevant info
#define WV_OPT_CALIBRATE_CAM 		 0 		// Calibrate the camera
#define WV_OPT_BENCHMARK 			 0 		// Print benchmark table
#define WV_OPT_CV_CONTOURS 			 0 		// Use opencv to detect contours
#define WV_OPT_ISP_CROP 			 0 		// Use the ISP to crop the frame according to FOV-Y
#define WV_OPT_MEASURE_FPS 			 0 		// Measure the FPS using built-in clock and framecount
#define WV_OPT_BODYFRAME 			 0 		// Fake euler angles and pos to be 0
#define WV_OPT_SAVE_FRAME 			 0 		// Save a frame for post-processing

//Optional function declarations
#if WV_OPT_CALIBRATE_CAM
static void 			calibrateEstimation (vector<trackResults> trackRes);
#endif
#if WV_OPT_MOD_VIDEO
static void 			mat2Buffer			(Mat& source, char* buf, bool greyscale = false);
#endif
#if WV_OPT_BENCHMARK
static void 			initBenchmark		(void);
static void 			addBenchmark		(const char * title);
static void 			showBenchmark		(void);
#endif
#if WV_OPT_SAVE_FRAME
static void 			saveBuffer			(char * img, Mat sourceFrame, const char *filename);
#endif

// Set up tracking parameters
int 	WV_TRACK_MIN_CROP_AREA 		= 300;
int 	WV_TRACK_RND_PIX_SAMPLE 	= 5000;
int 	AUTOSWARM_MAX_LAYERS  		= 100000;
double 	WV_TRACK_MIN_CIRCLE_SIZE 	= 260;
double 	WV_TRACK_MAX_CIRCLE_DEF 	= 0.4;

int 	FILTER_SAMPLE_STYLE 		= FILTER_STYLE_RANDOM;
int 	FILTER_FLOOD_STYLE 			= FILTER_FLOOD_CW;
int 	WV_FILTER_Y_MIN 			= 0;  // 0 					[0,65 84,135 170,255]zoo 45
int 	WV_FILTER_Y_MAX 			= 255; // 255
int 	WV_FILTER_CB_MIN 			= 80; // 84
int 	WV_FILTER_CB_MAX 			= 135; // 113
int 	WV_FILTER_CR_MIN 			= 170; // 218 -> 150?
int 	WV_FILTER_CR_MAX 			= 255; // 240 -> 255?
int 	WV_TRACK_IMAGE_CROP_FOVY 	= 45; 		// Degrees
int 	WV_FILTER_SAVE_RESULTS 		= 0;
double 	WV_FILTER_CROP_X 			= 1.2;

// Set up platform parameters
int 	WV_BEBOP_CAMERA_ANGLE 		= -13; 		// Degrees
double 	WV_BEBOP_CAMERA_OFFSET_X 	= 0.10; 	// Meters

// Set up swarm parameters
int 	WV_SWARM_MODE 				= 2; // 0: follow (deprecated), 1: look in direction of flight, 2: look in direction of global component
double 	WV_SWARM_SEPERATION 		= 1.75;
double 	WV_SWARM_E 					= 0.005; // Was 0.01x - 0.0005 at 12m/s OK (but close)
double 	WV_SWARM_EPS 				= 0.05;
double 	WV_SWARM_GLOBAL 			= 0.9;
int    	WV_SWARM_FPS 				= 15;
double 	WV_SWARM_AMAX 				= 8; 	// m/s			// 4m/s op 90% global = CRASH!
double 	WV_SWARM_VMAX 				= 8; 	// m/s
double 	WV_SWARM_YAWRATEMAX 		= 70; 	// deg/s
int    	WV_SWARM_MEMORY 			= 1.5; 	// seconds
double 	WV_SWARM_HOME 				= 0.3;

// Set up global attractor parameters
int 	WV_GLOBAL_ATTRACTOR			= 1;
double 	WV_GLOBAL_CIRCLE_R 			= 2.25;
double 	WV_GLOBAL_DEADZONE 			= 0.1;

// Initialize parameters to be assigned during runtime
struct 	FloatEulers * 	eulerAngles;
struct 	NedCoor_f * 	groundSpeed;
double 	ispScalar;
int 	ispWidth;
int 	ispHeight;
int 	cropCol;
int 	runCount = 0;
int 	pixCount = 0;
int 	pixSucCount = 0;
int 	shotSucces 	= 0;
int 	shotFail 	= 0;
int 	maxId = 0;
int 	layerDepth = 0;
int 	sample = 0;
vector<memoryBlock> neighbourMem;
extern uint8_t nav_block;
static const char * flight_blocks[] = FP_BLOCKS;
Rect 	objCrop;
vector<Rect> cropAreas;
vector<Point> objCont;
vector<vector<Point> > allContours;

#if WV_OPT_BENCHMARK
vector<double> benchmark_time;
vector<double> benchmark_avg_time;
vector<const char*> benchmark_title;
clock_t bench_start, bench_end;
#endif

#if WV_OPT_MEASURE_FPS
time_t 	startTime;
time_t 	currentTime;
int 	curT;
#endif

#if WV_OPT_WRITE_RESULTS
#if !WV_OPT_MEASURE_FPS
time_t 	startTime;
time_t 	currentTime;
int 	curT;
#endif
FILE * 	pFile;
char	resultFile [50];
#endif

void autoswarm_opencv_init(int globalMode)
{
#if WV_OPT_MEASURE_FPS
	startTime 			= time(0);
#endif
#if WV_OPT_WRITE_RESULTS
#if !WV_OPT_MEASURE_FPS
	startTime 			= time(0);
#endif
	//tm * startTM 		= localtime(&startTime);
	//sprintf(resultFile, "/data/ftp/internal_000/%d-%02d-%02d_%02d-%02d-%02d.txt", startTM->tm_year, startTM->tm_mon, startTM->tm_mday, startTM->tm_hour, startTM->tm_min, startTM->tm_sec);
	//printf("Writing tracking results to: %s\n", resultFile);
	pFile = fopen("/data/ftp/internal_000/results.txt","w");
	fprintf(pFile,"ID\tmem\ti\tt\tposX\tposY\tposZ\tobjX\tobjY\tobjZ\tcPosX\tcposY\tvX\tvY\tgX\tgY\tlX\tlY\tdX\tdY\n");
	fclose(pFile);
#endif
	WV_GLOBAL_ATTRACTOR = globalMode;
	if(!WV_OPT_CV_CONTOURS && FILTER_FLOOD_STYLE != FILTER_FLOOD_CW)
	{
		printf("[AS-CFG-ERR] No openCV contours is only possible with clockwise flooding.\n");
	}
	printf("[AS] initialized\n");
	return;
}

void autoswarm_opencv_run(char* img, int width, int height)
{
	// Computer vision compatibility function used to call trackObjects and (optionally) parse modified data back to rtp stream as YUV
	if(nav_block < 3){ return; }	// Engines have not started yet, lets save some battery life and skip image processing for now
#if WV_OPT_MEASURE_FPS
	if(runCount > 0)
	{
		currentTime 	= time(0); 												// Get the current time
		curT 			= difftime(currentTime,startTime); 						// Calculate time-difference between startTime and currentTime
		printf("Measured FPS: %0.2f\n", runCount / ((double) curT));
	}
#endif
#if WV_OPT_BENCHMARK
	if(runCount > 0)
	{
		addBenchmark("Lost time");
		showBenchmark();
	}
	initBenchmark();
#endif
	eulerAngles 			= stateGetNedToBodyEulers_f(); 	// Get Euler angles
	struct NedCoor_f *pos 	= stateGetPositionNed_f(); 		// Get your current position
	Mat sourceFrame(height, width, CV_8UC2, img); 	// Initialize current frame in openCV (UYVY) 2 channel
#if WV_OPT_BENCHMARK
	addBenchmark("Read image");
#endif
	runCount++; 									// Update global run-counter
	groundSpeed = stateGetSpeedNed_f(); 			// Get groundspeed
	Rect crop 	= setISPvars(width, height, true); 	// Calculate ISP related parameters
#if !WV_OPT_ISP_CROP
	sourceFrame = sourceFrame(crop); 				// Crop the frame
#endif
	Mat frameGrey; 									// Initialize frameGrey (to hold the thresholded image)
	if(FILTER_FLOOD_STYLE != FILTER_FLOOD_CW || WV_OPT_CV_CONTOURS)
	{
		Mat frameGrey(sourceFrame.rows, sourceFrame.cols, CV_8UC1, cvScalar(0.0)); 	// Only when using opencv contours the frame is filled with zeros
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Declared variables");
#endif
	vector<trackResults> trackRes; 							// Create empty struct _trackResults to store our neighbours in
	trackGreyObjects(sourceFrame, frameGrey, &trackRes); 	// Track objects in sourceFrame
#if WV_OPT_BENCHMARK
	addBenchmark("Tracked objects");
#endif
#if WV_OPT_MOD_VIDEO
	if(FILTER_FLOOD_STYLE != FILTER_FLOOD_CW || WV_OPT_CV_CONTOURS)
	{
		sourceFrame.setTo(Scalar(0.0, 127.0)); 	// Make sourceFrame black
		frameGrey.copyTo(sourceFrame); 			// Copy threshold result to black frame
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Initialized frames");
#endif
#endif
	for(unsigned int r=0; r < trackRes.size(); r++)			// Convert angles & Write/Print output
	{
		cam2body(&trackRes[r]);								// Convert from camera angles to body angles (correct for roll)
#if WV_OPT_BODYFRAME
		struct NedCoor_f fakePos;
		fakePos.x = 0.0;
		fakePos.y = 0.0;
		fakePos.z = 0.0;
		struct 	FloatEulers fakeEulerAngles;
		fakeEulerAngles.phi 	= 0.0;
		fakeEulerAngles.psi 	= 0.0;
		fakeEulerAngles.theta 	= 0.0;
		body2world(&trackRes[r], &fakePos, &fakeEulerAngles); 		// Convert from body angles to world coordinates (correct yaw and pitch)
#else
		body2world(&trackRes[r], pos, eulerAngles); 		// Convert from body angles to world coordinates (correct yaw and pitch)
#endif
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Body and World transformations done");
#endif

#if WV_OPT_MOD_VIDEO
	for(unsigned int r=0; r < trackRes.size(); r++)			// Convert angles & Write/Print output
	{
		circle(sourceFrame,cvPoint(trackRes[r].x_p - cropCol, trackRes[r].y_p), sqrt(trackRes[r].area_p / M_PI), cvScalar(0,255,255), 5);
		char text[15];
#if WV_OPT_MEASURE_FPS
		sprintf(text,"FPS: %0.2f", runCount / ((double) curT));
#else
		sprintf(text,"frame %i", runCount);
#endif
		putText(sourceFrame, text, Point(10,sourceFrame.rows-100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,255), 5);
	}
	line(sourceFrame, Point(0,0), Point(0, sourceFrame.rows-1), Scalar(0,255,255), 5);
	line(sourceFrame, Point(sourceFrame.cols - 1,0), Point(sourceFrame.cols - 1, sourceFrame.rows-1), Scalar(0,255,255), 5);
	if(FILTER_FLOOD_STYLE != FILTER_FLOOD_CW || WV_OPT_CV_CONTOURS)
	{
		for(unsigned int r=0; r < cropAreas.size(); r++)
		{
			if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
			{
				rectangle(sourceFrame, cropAreas[r], Scalar(0,255,255), 5);
			}
		}
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Drew circles, lines and rectangles");
#endif
#endif
	sourceFrame.release();			// Release Mat
	frameGrey.release(); 			// Release Mat
#if WV_OPT_BENCHMARK
	addBenchmark("Released Frames");
#endif
	identifyNeighbours(trackRes); 	// ID neighbours according to previous location
#if WV_OPT_BENCHMARK
	addBenchmark("Identified neighbours");
#endif
	vector<double> totV(3), cPos, li, gi, di; 							// Initialize total contribution for output
	li 		= calcLocalVelocity(pos);									// Get the contribution due to the neighbours we see and have memorized
	gi 		= calcGlobalVelocity(pos); 									// Get the contribution due to the "attraction" towards global origin
	di 		= calcDiffVelocity(); 										// TODO: Include this?
	totV[0] = li[0] + gi[0] + di[0]; 									// Average the X local contribution (#neighbours independent) and add the X global contribution
	totV[1] = li[1] + gi[1] + di[1]; 									// Do the same for Y
	totV[2] = li[2] + gi[2] + di[2]; 									// Do the same for Z
	totV 	= limitNorm(totV, WV_SWARM_AMAX / ((double) WV_SWARM_FPS)); // Check if ideal velocity exceeds WV_SWARM_AMAX
	totV[0] = groundSpeed->x / ((double) WV_SWARM_FPS) + totV[0]; 		// Inputting totV as an acceleration
	totV[1] = groundSpeed->y / ((double) WV_SWARM_FPS) + totV[1]; 		// Inputting totV as an acceleration
	totV 	= limitNorm(totV, WV_SWARM_VMAX / ((double) WV_SWARM_FPS)); // Limit it again (otherwise speed could escalate)
	if(WV_SWARM_MODE!=2) totV = limitVelocityYaw(totV); 				// Limit the velocity when relative angle gets larger
	cPos 	= calcCamPosition(pos, totV, gi); 							// Calculate CAM/heading
	cPos 	= limitYaw(pos, cPos); 										// Limit yaw
#if WV_OPT_BENCHMARK
	addBenchmark("Calculated swarming dynamics");
#endif
	updateWaypoints(pos,cPos,totV); 									// Translate velocity contribution to updated waypoint
	if(!strcmp("Swarm",flight_blocks[nav_block]) || !strcmp("Swarm Home",flight_blocks[nav_block]))
	{
		nav_set_heading_towards_waypoint(WP__CAM); 					// Currently in block "Swarm" or "Swarm Home" so update heading
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Updated waypoints and heading");
#endif

#if WV_OPT_WRITE_RESULTS 												// See if we want to write results
#if !WV_OPT_MEASURE_FPS
	currentTime = time(0); 												// Get the current time
	curT 		= difftime(currentTime,startTime); 						// Calculate time-difference between startTime and currentTime
#endif
	pFile 		= fopen("/data/ftp/internal_000/results.txt","a");		// Open file for appending TODO: Check for errors opening file
#endif

#if WV_OPT_WRITE_RESULTS || WV_DEBUG_SHOW_MEM
		for(unsigned int r=0; r < neighbourMem.size(); r++) 			// Print to file & terminal
		{
#if WV_DEBUG_SHOW_MEM
			char memChar = 'm';
			if(neighbourMem[r].lastSeen == runCount){ memChar = 'w';}
#endif
#if WV_OPT_WRITE_RESULTS
			int memInt = 0;
			if(neighbourMem[r].lastSeen == runCount){ memInt = 1; }
#endif
#if WV_DEBUG_SHOW_MEM
			printf("%i - Object (%c %i) at (%0.2f m, %0.2f m, %0.2f m) pos(%0.2f, %0.2f, %0.2f)\n", runCount, memChar, neighbourMem[r].id, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, pos->x, pos->y, pos->z); 														// Print to terminal
#endif
#if WV_OPT_WRITE_RESULTS
			fprintf(pFile, "%i\t%i\t%i\t%i\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\n", neighbourMem[r].id, memInt, runCount, curT, pos->x, pos->y, pos->z, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, cPos[0], cPos[1], totV[0], totV[1], gi[0], gi[1], li[0], li[1], di[0], di[1]); // if file writing is enabled, write to file
#endif
		}
#endif

#if WV_OPT_WRITE_RESULTS
	fclose(pFile);	// Close file
#if WV_OPT_BENCHMARK
	addBenchmark("Wrote to terminal/file");
#endif
#endif
	if((WV_DEBUG_SHOW_MEM==1 && neighbourMem.size() > 0) || WV_DEBUG_SHOW_WAYPOINT==1 || WV_OPT_BENCHMARK==1) printf("\n"); // Separate terminal output by newline
#if WV_OPT_BENCHMARK
	addBenchmark("Complete");
#endif
#if WV_OPT_CALIBRATE_CAM
	if(runCount==10) // First and second frame are often not yet detected properly so let's calibrate the tenth frame
	{
		calibrateEstimation(trackRes);
	}
#endif
	return;
}

void identifyNeighbours(vector<trackResults> trackRes)
{
	// First lets clear the old elements which we will no longer be using
	for(unsigned int i=0; i < neighbourMem.size();)
	{
		if(runCount - neighbourMem[i].lastSeen > WV_SWARM_MEMORY * WV_SWARM_FPS)
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
			double radius	= (runCount - neighbourMem[i].lastSeen) * WV_SWARM_VMAX / ((double) WV_SWARM_FPS);
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
		li_fac = 12 * WV_SWARM_E / range * (pow(WV_SWARM_SEPERATION / range, 12.0) - pow(WV_SWARM_SEPERATION / range, 6.0)); // Local contribution scaling factor
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
	switch(WV_GLOBAL_ATTRACTOR)
	{
	case WV_GLOBAL_POINT :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > WV_GLOBAL_DEADZONE)
		{
			gi[0] 			= WV_SWARM_GLOBAL * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			gi[1] 			= WV_SWARM_GLOBAL * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
			gi[2] 			= 0;
		}else{
			gi[0] = 0.0;
			gi[1] = 0.0;
			gi[2] = 0.0;
		}
		break;
	}
	case WV_GLOBAL_BUCKET :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > WV_GLOBAL_DEADZONE)
		{
			double gScalar 	= WV_SWARM_GLOBAL * (1 - 1 / (1 + exp(4 / WV_SWARM_SEPERATION * (cr - WV_SWARM_SEPERATION))));
			gi[0] 			= gScalar * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			gi[1] 			= gScalar * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
			gi[2] 			= 0;
		}else{
			gi[0] = 0.0;
			gi[1] = 0.0;
			gi[2] = 0.0;
		}
		break;
	}
	case WV_GLOBAL_CIRCLE_CW :
	case WV_GLOBAL_CIRCLE_CC :
	{
		//double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0) + pow(globalOrigin.cz - pos->z, 2.0));
		double cr 	= sqrt(pow(globalOrigin.cx - pos->x, 2.0) + pow(globalOrigin.cy - pos->y, 2.0));
		if (cr > WV_GLOBAL_DEADZONE)
		{
			double angle;
			if(cr >= WV_GLOBAL_CIRCLE_R)
			{
				angle = 90 * WV_GLOBAL_CIRCLE_R / cr;
			}else{
				angle = 90 + 90 * (WV_GLOBAL_CIRCLE_R - cr) / WV_GLOBAL_CIRCLE_R;  // From 90 to 180 over the length of WV_GLOBAL_CIRCLE_R
			}
			if (WV_GLOBAL_ATTRACTOR != WV_GLOBAL_CIRCLE_CC) 	angle = -angle; 	// Switch between cc (+) and cw (-)
			double xContrib = WV_SWARM_GLOBAL * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cx - pos->x) / cr;
			double yContrib = WV_SWARM_GLOBAL * WV_SWARM_AMAX / ((double) WV_SWARM_FPS) * (globalOrigin.cy - pos->y) / cr;
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
	di[0] = - WV_SWARM_EPS * groundSpeed->x / ((double) WV_SWARM_FPS);
	di[1] = - WV_SWARM_EPS * groundSpeed->y / ((double) WV_SWARM_FPS);
	di[2] = 0; //- WV_SWARM_EPS * groundSpeed->z;
	return di;
}

vector<double> calcCamPosition(struct NedCoor_f *pos, vector<double> totV, vector<double> gi)
{
	vector<double> cPos(3); cPos[0] = 0; cPos[1] = 0; cPos[2] = 0;

	if(WV_SWARM_MODE==0) // Follow mode, find c.g. of neighbours, point camera there and set global position there
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
	if(WV_SWARM_MODE == 1) // Watch where you're going mode. Point WP_CAM on unity circle towards totV
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
	if(WV_SWARM_MODE == 2) // Watch the global objective mode. Point WP_CAM on unity circle towards global component
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
	if(abs(relHeading) > WV_SWARM_YAWRATEMAX  / ((double) WV_SWARM_FPS) / 180 * M_PI)
	{
		double newHeading = relHeading/abs(relHeading) * (WV_SWARM_YAWRATEMAX / ((double) WV_SWARM_FPS) / 180 * M_PI) + eulerAngles->psi;
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
		totV[2] = 0; //WV_SWARM_VMAX / ((double) WV_SWARM_FPS) * totV[2] / vR; // Do the same for Z
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
	if(WV_DEBUG_SHOW_WAYPOINT) printf("WP_CAM (%0.2f m, %0.2f m) \tWP_GOAL (%0.2f m, %0.2f m) \tWP_GLOBAL (%0.2f m, %0.2f m)\n", cPos[0], cPos[1], pos->x + totV[0], pos->y + totV[1], globalOrigin.cx, globalOrigin.cy);	//, pos->z + totV[2]);	// Print to terminal
}

Rect setISPvars(int width, int height, bool crop)
{
	int heightRange = 3288; // Property of CMOS sensor
	ispScalar 		= (double) 16 / round(16 / ((double) width / heightRange)); // (re)Calculate the scalar set originally in bebop_front_camera.c
#if WV_OPT_ISP_CROP
	ispHeight 		= MT9F002_OUTPUT_WIDTH;
	ispWidth 		= MT9F002_OUTPUT_HEIGHT;
#else
	ispHeight 		= width;
	ispWidth 		= height;
#endif
	if (crop==true)
	{
		double fovY 			= WV_TRACK_IMAGE_CROP_FOVY * M_PI / 180;
		double cmosPixelSize 	= 0.0000014; 	// 1.4um (see manual of mt9f002 CMOS sensor)
		double focalLength		= (2400 * cmosPixelSize * 1000 / ispScalar) / (4 * sin(M_PI / 4));
		double cY 				= round(sin((-eulerAngles->theta - WV_BEBOP_CAMERA_ANGLE * M_PI / 180)) * 2 * focalLength * ispScalar / (1000 * cmosPixelSize));
		double desHeight 		= round(sin(fovY / 4) * 4 * focalLength * ispScalar / (1000 * cmosPixelSize) + ispWidth * tan(eulerAngles->phi));
		double desOffset 		= round((ispHeight - desHeight) / 2) + cY;

		if(desOffset < 0) 			desOffset = 0;
		if(desOffset > ispHeight) 	desOffset = 0;

		if(desHeight < 0) 			desHeight = ispHeight;
		if(desHeight > ispHeight) 	desHeight = ispHeight;

		if(desHeight + desOffset > ispHeight)
		{
			desOffset = ispHeight - desHeight;
		}
#if WV_OPT_ISP_CROP
		cropCol 	= 0;
		Rect crop 	= cvRect(0,0,desHeight,ispWidth);
		mt9f002.offset_x = MT9F002_INITIAL_OFFSET_X + desOffset;
		mt9f002.output_width = desHeight / ispScalar;
		mt9f002_set_resolution(&mt9f002);
#else
		cropCol 	= desOffset;
		Rect crop 	= cvRect(desOffset,0,desHeight,ispWidth);
#endif
		return crop;
	}else{
		return cvRect(0, 0, width, height);
	}
}

void trackGreyObjects(Mat& sourceFrame, Mat& frameGrey, vector<trackResults>* trackRes)
{
	// Main function for tracking object on a frame
	pixCount 	= 0;
	pixSucCount = 0;
	rndRedGrayscale(sourceFrame, frameGrey, WV_TRACK_RND_PIX_SAMPLE);
#if WV_OPT_BENCHMARK
	addBenchmark("image Thresholded");
#endif
	if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
	{
		//printf("Pixels processed: %i, pixel passed: %i\n",pixCount, pixSucCount);
		for(unsigned int tc=0; tc < allContours.size(); tc++)
		{
			addContour(allContours[tc], trackRes);
		}
	}else{
		vector<vector<Point> > contours;
		for(unsigned int r=0; r < cropAreas.size(); r++)
		{
			if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
			{
				contours.clear();
#if WV_OPT_MOD_VIDEO
				findContours(frameGrey(cropAreas[r]).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#else
				findContours(frameGrey(cropAreas[r]), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#endif
				for(unsigned int tc=0; tc < contours.size(); tc++)
				{
					addContour(contours[tc], trackRes);
				}
			}
		}
	}
#if WV_OPT_BENCHMARK
	addBenchmark("Contours found");
#endif
	return;
}

void addContour(vector<Point> contour, vector<trackResults>* trackRes)
{
	double contArea = contourArea(contour);
	if (contArea > (WV_TRACK_MIN_CROP_AREA * ispScalar * ispScalar))
	{
		Point2f objCentre;
		float 	objRadius;
		minEnclosingCircle(contour,objCentre,objRadius);
		float objArea = M_PI*objRadius*objRadius;
		if(objArea > (WV_TRACK_MIN_CIRCLE_SIZE * ispScalar * ispScalar) && contArea > objArea*WV_TRACK_MAX_CIRCLE_DEF)
		{
			trackResults curRes;
			vector<double> position(3);
			curRes.x_p 		= objCentre.x + cropCol;
			curRes.y_p 		= objCentre.y;
			//printf("Object at pixel location: %0.2f px %0.2f px\n",objCentre.x, objCentre.y);
			curRes.area_p 	= objArea;
			position 		= estimatePosition(curRes.x_p, curRes.y_p, curRes.area_p); // Estimate position in camera reference frame based on pixel location and area
			curRes.x_c 		= position[0];
			curRes.y_c 		= position[1];
			curRes.r_c 		= position[2];
			trackRes->push_back(curRes); 	// Save results and push into trackRes
		}else if(WV_DEBUG_SHOW_REJECT) 	printf("Rejected. object %f, area %f, fill %f < min fill %f.\n",objArea, contArea, contArea / objArea, WV_TRACK_MAX_CIRCLE_DEF);
	}else if(WV_DEBUG_SHOW_REJECT) 	printf("Rejected. Area %0.1f\n",contArea);
	return;
}

bool rndRedGrayscale(Mat& sourceFrame, Mat& destFrame, int sampleSize)
{
	bool obj_detected = false;
	if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
	{
		allContours.clear();
	}else{
		cropAreas.clear();
	}
	if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
	{
		if(FILTER_SAMPLE_STYLE > 0)
		{
			for(unsigned int r=0; r < neighbourMem.size(); r++)
			{
				if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
				{
					objCont.clear();
				}else{
					objCrop.x 		= neighbourMem[r].x_p - cropCol;
					objCrop.y 		= neighbourMem[r].y_p;
					objCrop.width 	= 0;
					objCrop.height 	= 0;
				}
				if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p, neighbourMem[r].x_p - cropCol, 0, true)){
					if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p + 100, neighbourMem[r].x_p - cropCol, 0, true)){
						if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p + 100, neighbourMem[r].x_p - cropCol + 100, 0, true)){
							if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p, neighbourMem[r].x_p - cropCol + 100, 0, true)){
								if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p - 100, neighbourMem[r].x_p - cropCol + 100, 0, true)){
									if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p - 100, neighbourMem[r].x_p - cropCol, 0, true)){
										if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p - 100, neighbourMem[r].x_p - cropCol - 100, 0, true)){
											if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p, neighbourMem[r].x_p - cropCol - 100, 0, true)){
												if(!pixFindContour(sourceFrame, destFrame, neighbourMem[r].y_p + 100, neighbourMem[r].x_p - cropCol - 100, 0, true)){
													break;
												}
											}
										}
									}
								}
							}
						}
					}
				}
				if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
				{
					allContours.push_back(objCont);
				}else{
					objCrop = enlargeRectangle(sourceFrame, objCrop, WV_FILTER_CROP_X);
					addObject();
				}
			}
		}
		switch(FILTER_SAMPLE_STYLE)
		{
		case FILTER_STYLE_FULL : // FULL
		{
			for(int r = 0; r < sourceFrame.rows; r++)
			{
				for(int c= 0; c < sourceFrame.cols; c++)
				{
					layerDepth = 0;
					if(pixFindContour(sourceFrame, destFrame, r, c, 0, false))
					{
						obj_detected = true;
					}
				}
			}
			break;
		}
		case FILTER_STYLE_GRID :
		{
			int spacing = (int) sqrt((sourceFrame.rows * sourceFrame.cols) / sampleSize);
			for(int r = spacing; r < sourceFrame.rows; r+=spacing)
			{
				for(int c=spacing; c < sourceFrame.cols; c+=spacing)
				{
					layerDepth = 0;
					sample++;
					if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
					{
						objCont.clear();
					}else{
						objCrop.x 		= c;
						objCrop.y 		= r;
						objCrop.width 	= 0;
						objCrop.height 	= 0;
					}
					if(pixFindContour(sourceFrame, destFrame, r, c, 0, true))
					{
						if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
						{
							allContours.push_back(objCont);
						}else{
							objCrop = enlargeRectangle(sourceFrame, objCrop, WV_FILTER_CROP_X);
							addObject();
						}
						obj_detected = true;
					}
				}
			}
			break;
		}
		case FILTER_STYLE_RANDOM :
		{
			int rndRow, rndCol;
			for(int i = 0; i<sampleSize; i++)
			{
				layerDepth = 0;
				sample++;
				//pixFindContour(sourceFrame, destFrame, rowDis(gen), colDis(gen));
				rndRow = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
				rndCol = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));
				if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
				{
					objCont.clear();
				}else{
					objCrop.x 		= rndCol;
					objCrop.y 		= rndRow;
					objCrop.width 	= 0;
					objCrop.height 	= 0;
				}
				if(pixFindContour(sourceFrame, destFrame, rndRow, rndCol, 0, true))
				{
					if(FILTER_FLOOD_STYLE == FILTER_FLOOD_CW && !WV_OPT_CV_CONTOURS)
					{
						allContours.push_back(objCont);
					}else{
						objCrop = enlargeRectangle(sourceFrame, objCrop, WV_FILTER_CROP_X);
						addObject();
					}
					obj_detected = true;
				}
			}
			break;
		}
		}
	}
	return obj_detected;
}

bool pixFindContour(Mat& sourceFrame, Mat& destFrame, int row, int col, int prevDir, bool cascade)
{
	layerDepth++;
	pixCount++;
	if(layerDepth > AUTOSWARM_MAX_LAYERS || row < 0 || col < 0 || row > sourceFrame.rows-1 || col > sourceFrame.cols-1 || sourceFrame.at<Vec2b>(row,col)[1] == 1)
	{
		return false;
	}
	int Cb, Y, Cr;
	if (((col & 1) == 0 && (cropCol & 1) == 0) || ((col & 1) == 1 && (cropCol & 1) == 1))
	{
		// Even col number
		Cb = sourceFrame.at<Vec2b>(row,col  )[0]; // U1
		Y  = sourceFrame.at<Vec2b>(row,col  )[1]; // Y1
		Cr = sourceFrame.at<Vec2b>(row,col+1)[0]; // V2
	}else{
		// Uneven col number
		Cb = sourceFrame.at<Vec2b>(row,col-1)[0]; // U1
		Cr = sourceFrame.at<Vec2b>(row,col  )[0]; // V2
		Y  = sourceFrame.at<Vec2b>(row,col  )[1]; // Y2
	}
	if(Y >= WV_FILTER_Y_MIN && Y <= WV_FILTER_Y_MAX && Cb >= WV_FILTER_CB_MIN && Cb <= WV_FILTER_CB_MAX && Cr >= WV_FILTER_CR_MIN && Cr <= WV_FILTER_CR_MAX)
	{
		if(cascade)
		{
			if(prevDir != 0 && (FILTER_FLOOD_STYLE != FILTER_FLOOD_CW || WV_OPT_CV_CONTOURS))
			{
				pixSucCount++;
				destFrame.at<uint8_t>(row,col) = 255;
			}
			sourceFrame.at<Vec2b>(row,col  )[1] = 1;
			switch(FILTER_FLOOD_STYLE)
			{
			case FILTER_FLOOD_OMNI :
			{
				if(prevDir==3 || row<=0 || !pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true))
				{
					if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
					if(prevDir==4 || col>=(sourceFrame.cols - 1) || !pixFindContour(sourceFrame, destFrame, row, col+1, 2, true))
					{
						if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
						if(prevDir==1 || row>=(sourceFrame.rows - 1) || !pixFindContour(sourceFrame, destFrame, row+1, col, 3, true))
						{
							if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
							if(prevDir==2 || col<=0 || !pixFindContour(sourceFrame, destFrame, row, col-1, 4, true))
							{
								return false;
							}
						}
					}
				}
				break;
			}
			case FILTER_FLOOD_CW :
			{
				switch(prevDir)
				{
				case 0 : // No previous direction
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 0, true)) break;																// UP
					if(!WV_OPT_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; 										// RIGHT
					if(row < (sourceFrame.rows -1) && col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row + 1, col + 1, 2, true)) break; 		// DOWN-RIGHT
					break;
				case 1 : // Came from pixel below
					if(col > 0) pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true); 								// LEFT
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true)) break;						// UP
					if(!WV_OPT_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; // RIGHT
					break;
				case 2 : // Came from pixel left
					if(row > 0) pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true);								// UP
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; // RIGHT
					if(!WV_OPT_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(row < (sourceFrame.rows -1) && pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true)) break;	// DOWN
					break;
				case 3 : // Came from pixel above
					if(col < (sourceFrame.cols -1)) pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true); 			// RIGHT
					if(row < (sourceFrame.rows -1) && pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true)) break;	// DOWN
					if(!WV_OPT_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col > 0 && pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true)) break; 					// LEFT
					break;
				case 4 : // Came from pixel right
					if(row < (sourceFrame.rows -1)) pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true);			// DOWN
					if(col > 0 && pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true)) break; 					// LEFT
					if(!WV_OPT_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true)) break;						// UP
					break;
				}
				break;
			}
			default : 	break;
			}
			if(FILTER_FLOOD_STYLE != FILTER_FLOOD_CW || WV_OPT_CV_CONTOURS)
			{
				objCrop.width 		= max(objCrop.x + objCrop.width, col) - min(objCrop.x, col);
				objCrop.height 		= max(objCrop.y + objCrop.height, row) - min(objCrop.y, row);
				objCrop.x 			= min(objCrop.x, col);
				objCrop.y 			= min(objCrop.y, row);
			}
			return true;
		}else{
			return true;
		}
	}else{
		sourceFrame.at<Vec2b>(row,col  )[1] = 1;
		return false;
	}
}

vector<double> estimatePosition(int xp, int yp, double area, double k, int calArea, int orbDiag)
{
	// This function estimates the 3D position (in camera  coordinate system) according to pixel position
	// (Default) calibration parameters
	if(k==0) 		k 		= 1.0; // Fisheye correction factor (1.085)
	if(calArea==0) 	calArea = 3390; // Calibrate at full resolution (5330)
	if(orbDiag==0) 	orbDiag = 2500; // Measured circular image diagonal using resolution of 2024x2048 org: 2400 (2200)
	// Calculate corrected calibration parameters
	calArea 				= (int) round(calArea * (ispWidth / 2048.0) * (ispWidth / 2048.0));
	orbDiag 				= (int) round(orbDiag * (ispWidth / 2048.0));
	double cmosPixelSize 	= 0.0000014; 	// 1.4um (see manual of CMOS sensor)
	double fovDiag 			= M_PI; 		// [degrees] Diagonal field of view (see bebop manual)
	// Calculate relevant parameters
	int cX 					= round(ispHeight / 2);
	int cY 					= round(ispWidth / 2);
	double frameSizeDiag 	= orbDiag * cmosPixelSize * 1000 / ispScalar; // [mm] Find used diagonal size of CMOS sensor
	double f 				= frameSizeDiag / (4 * sin(fovDiag / 4)); // [mm]
	double useableX 		= (ispWidth - sqrt(calArea / M_PI));
	double useableY 		= (ispHeight - sqrt(calArea / M_PI));
	// Calculate FoV in x and y direction
	double fovX 			= 4*asin((useableX * 1 / ispScalar * cmosPixelSize * 1000)/(4 * f));
	double fovY				= 4*asin((useableY * 1 / ispScalar * cmosPixelSize * 1000)/(4 * f));
	//printf("fovX: %0.2f - fovY: %0.2f\n",fovX * 180 / M_PI, fovY * 180 / M_PI);
	// rotate frame cc 90 degrees (ISP is rotated 90 degrees cc)
	int x 					= yp - cY;
	int y 					= xp - cX;
	// Convert to polar coordinates
	double r 				= sqrt((double) x * x + y * y) * 1 / ispScalar * cmosPixelSize * 1000; // [mm] radial distance from middle of CMOS
	double theta 			= atan2(y,x);
	// Correct radius for radial distortion using correction parameter k
	double corR 			= correctRadius(r, f, k);
	// Convert back to Cartesian
	double corX 			= corR * cos(theta);
	double corY 			= corR * sin(theta);
	double corArea 			= area * pow(corR / r, 2.0);	// radius = sqrt(area) / sqrt(M_PI) 	---> 	newRadius = radius * corR / r 	---> 	corArea = M_PI * newRadius * newRadius
	// Calculate distance
	double dist 			= sqrt(calArea) / sqrt(corArea);
	// Calculate max width and height of undistorted frame
	double maxX 			= correctRadius(useableX / 2 * 1 / ispScalar * cmosPixelSize * 1000, f, k);
	double maxY				= correctRadius(useableY / 2 * 1 / ispScalar * cmosPixelSize * 1000, f, k);
	// Calculate angles wrt camera
	double xAngle 			= fovX / 2 * corX / maxX; // Assume all non-linearities have been removed from corX and corY
	double yAngle 			= fovY / 2 * corY / maxY; // Then scale them according to the max values and axis FoV
	// Store in dest and parse it back
	vector<double> dest(3);
	dest[0] = xAngle;
	dest[1] = yAngle;
	dest[2] = dist;
	return dest;
}

double correctRadius(double r, double f, double k)
{
	// This function calculates the corrected radius for radial distortion
	// According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
	return f / k * tan(asin(sin(atan(r / f)) * k));
}

void cam2body(trackResults* trackRes)
{
	// Neighbour position returned in 2 angles and a radius.
	// x_c is the angle wrt vertical camera axis. 	Defined clockwise/right positive
	// y_c is angle wrt camera horizon axis. 		Defined upwards positive
	// r_c is radial distance in m.
	trackRes->x_b = trackRes->r_c * cos(-trackRes->y_c - M_PI / 180 * WV_BEBOP_CAMERA_ANGLE) * cos(trackRes->x_c) + WV_BEBOP_CAMERA_OFFSET_X;
	trackRes->y_b = trackRes->r_c * cos(-trackRes->y_c - M_PI / 180 * WV_BEBOP_CAMERA_ANGLE) * sin(trackRes->x_c);
	trackRes->z_b = trackRes->r_c * sin(-trackRes->y_c - M_PI / 180 * WV_BEBOP_CAMERA_ANGLE);
	return;
}

void body2world(trackResults* trackRes, struct NedCoor_f *pos, struct 	FloatEulers * eulerAngles)
{
	double phi 		= eulerAngles->phi;
	double theta 	= eulerAngles->theta;
	double psi 		= eulerAngles->psi;
	Matx33f rotX(	 1,  		 0, 	 	 0,
					 0,			 cos(phi), 	-sin(phi),
					 0,			 sin(phi),   cos(phi));
	Matx33f rotY(	 cos(theta), 0, 		 sin(theta),
					 0,  		 1, 		 0,
					-sin(theta), 0, 		 cos(theta));
	Matx33f rotZ(	 cos(psi),  -sin(psi), 	 0,
					 sin(psi),   cos(psi), 	 0,
					 0, 		 0, 		 1);
	Matx31f bodyPos(trackRes->x_b, trackRes->y_b, trackRes->z_b);
	Matx31f worldPos = rotZ * rotY * rotX * bodyPos;
	trackRes->x_w = worldPos(0,0) + pos->x;
	trackRes->y_w = worldPos(1,0) + pos->y;
	trackRes->z_w = worldPos(2,0);// + pos->z; TODO: include!!
	return;
}

bool amIhome(void){
	struct EnuCoor_f *pos = stateGetPositionEnu_f();
	if(sqrt(pow(pos->x - waypoint_get_x(WP__TD),2.0) + pow(pos->y - waypoint_get_y(WP__TD),2.0)) < WV_SWARM_HOME)
	{
		return true;
	}else{
		return false;
	}
}

#if WV_OPT_CALIBRATE_CAM
void calibrateEstimation(vector<trackResults> trackRes)
{
	printf("Starting calibration!\n");
	struct NedCoor_f fakePos;
	fakePos.x = 0.0;
	fakePos.y = 0.0;
	fakePos.z = 0.0;
	struct 	FloatEulers	fakeEulerAngles;
	fakeEulerAngles.phi 	= 0.0;
	fakeEulerAngles.psi 	= 0.0;
	fakeEulerAngles.theta 	= 0.0;

	vector< vector<double> > calPositions(6, vector<double>(3));
	calPositions[0][0] 	=  1.00;
	calPositions[0][1] 	=  0.00;
	calPositions[0][2] 	=  0.25;

	calPositions[1][0] 	=  1.00;
	calPositions[1][1] 	=  -1.00;
	calPositions[1][2] 	=  0.25;

	calPositions[2][0] 	=  1.00;
	calPositions[2][1] 	= 1.00;
	calPositions[2][2] 	=  0.25;

	calPositions[3][0] 	=  2.00;
	calPositions[3][1] 	= -0.50;
	calPositions[3][2] 	=  0.25;

	calPositions[4][0] 	=  2.00;
	calPositions[4][1] 	=  0.50;
	calPositions[4][2] 	=  0.25;

	double k_opt;
	double k_min 		= 1.0;
	double k_max 		= 1.15;
	double k_step 		= 0.0025;

	int calArea_opt;
	int calArea_min 	= 1100;
	int calArea_max 	= 5100;
	int calArea_step 	=   10;

	int orbDiag_opt;
	int orbDiag_min 	= 1900;
	int orbDiag_max 	= 2500;
	int orbDiag_step 	=   10;

	double err, opt_err = 1000;
	int i=0, totI = (int) ceil((k_max - k_min) / k_step) * ceil((calArea_max - calArea_min) / calArea_step) * ceil((orbDiag_max - orbDiag_min) / orbDiag_step);
	for(double k = k_min; k <= k_max; k += k_step)
	{
		for(int calArea = calArea_min; calArea <= calArea_max; calArea += calArea_step)
		{
			for(double orbDiag = orbDiag_min; orbDiag <= orbDiag_max; orbDiag += orbDiag_step)
			{
				err = 0;
				for(unsigned int r=0; r < trackRes.size(); r++)		// Convert angles & Write/Print output
				{
					vector<double> position(3);
					position 		= estimatePosition(trackRes[r].x_p, trackRes[r].y_p, trackRes[r].area_p, k, calArea, orbDiag);
					trackRes[r].x_c = position[0];
					trackRes[r].y_c = position[1];
					trackRes[r].r_c = position[2];
					cam2body(&trackRes[r]);							// Convert from camera angles to body angles (correct for roll)
					body2world(&trackRes[r], &fakePos , &fakeEulerAngles); 	// Convert from body angles to world coordinates (correct yaw and pitch)
					double ball_err = 1000;
					for(unsigned int i=0; i < calPositions.size(); i++)
					{
						double cur_ball_err = pow(trackRes[r].x_w - calPositions[i][0], 2.0) + pow(trackRes[r].y_w - calPositions[i][1], 2.0);// + pow(trackRes[r].z_w - calPositions[i][2], 2.0);
						if(cur_ball_err < ball_err)
						{
							ball_err = cur_ball_err;
						}
					}
					err += ball_err;
				}
				err = err / trackRes.size();
				if(err < opt_err)
				{
					opt_err 	= err;
					k_opt 		= k;
					calArea_opt = calArea;
					orbDiag_opt = orbDiag;
				}
				i++;
				printf("\r%0.2f percent", 100 * i / ((double) totI));
			}
		}
	}
	printf("\nCalibration finished. Avg error: %0.3f.\t k=%0.3f\t calArea=%i\t orbDiag=%i\n", sqrt(opt_err), k_opt, calArea_opt, orbDiag_opt);
}
#endif

#if WV_OPT_SAVE_FRAME
void saveBuffer(char * img, Mat sourceFrame, const char *filename)
{
	char path[100];
	sprintf(path,"/data/ftp/internal_000/%s", filename);
	FILE * iFile = fopen(path,"w");
	printf("Writing imagebuffer(%i x %i) to file %s  ... ", sourceFrame.rows, sourceFrame.cols, path);
	for(int row = 0; row < sourceFrame.rows; row++)
	{
		for(int col = 0; col < sourceFrame.cols; col++)
		{
			fprintf(iFile, "%i,%i", img[(row*sourceFrame.cols+col)*2+0], img[(row*sourceFrame.cols+col)*2+1]);
			if(col != sourceFrame.cols-1)
			{
				fprintf(iFile, ",");
			}
		}
		fprintf(iFile,"\n");
	}
	fclose(iFile);
	printf(" done.\n");
}
#endif

void addObject(void)
{
	for(unsigned int r=0; r < cropAreas.size(); r++)
	{
		bool overlap = false;
		if(!overlap && (inRectangle(Point(objCrop.x, objCrop.y), cropAreas[r]) || inRectangle(Point(objCrop.x + objCrop.width, objCrop.y), cropAreas[r]) || inRectangle(Point(objCrop.x + objCrop.width, objCrop.y + objCrop.height), cropAreas[r]) || inRectangle(Point(objCrop.x, objCrop.y + objCrop.height), cropAreas[r])))
		{
			overlap = true; // One of the corner points is inside the cropAreas rectangle
		}
		if(!overlap && objCrop.x >= cropAreas[r].x && (objCrop.x + objCrop.width) <= (cropAreas[r].x + cropAreas[r].width) && objCrop.y <= cropAreas[r].y && (objCrop.y + objCrop.height) >= (cropAreas[r].y + cropAreas[r].height))
		{
			overlap = true; // less wide, yet fully overlapping in height
		}
		if(!overlap && objCrop.y >= cropAreas[r].y && (objCrop.y + objCrop.height) <= (cropAreas[r].y + cropAreas[r].height) && objCrop.x <= cropAreas[r].x && (objCrop.x + objCrop.width) >= (cropAreas[r].x + cropAreas[r].width))
		{
			overlap = true; // less high, yet fully overlapping in width
		}
		if(!overlap && (inRectangle(Point(cropAreas[r].x, cropAreas[r].y), objCrop) || inRectangle(Point(cropAreas[r].x + cropAreas[r].width, cropAreas[r].y), objCrop) || inRectangle(Point(cropAreas[r].x + cropAreas[r].width, cropAreas[r].y + cropAreas[r].height), objCrop) || inRectangle(Point(cropAreas[r].x, cropAreas[r].y + cropAreas[r].height), objCrop)))
		{
			overlap = true; // One of the corner points is inside the objCrop rectangle
		}
		if(overlap == true)
		{
			objCrop.width 		= max(objCrop.x + objCrop.width, cropAreas[r].x + cropAreas[r].width) - min(objCrop.x, cropAreas[r].x);
			objCrop.height 		= max(objCrop.y + objCrop.height, cropAreas[r].y + cropAreas[r].height) - min(objCrop.y, cropAreas[r].y);
			objCrop.x 			= min(objCrop.x, cropAreas[r].x);
			objCrop.y 			= min(objCrop.y, cropAreas[r].y);
			cropAreas[r].x 		= 0;
			cropAreas[r].y 		= 0;
			cropAreas[r].width 	= 0;
			cropAreas[r].height = 0;
		}
	}
	if(objCrop.width * objCrop.height >= WV_TRACK_MIN_CROP_AREA)
	{
		cropAreas.push_back(objCrop);
	}
	return;
}

bool inRectangle(Point pt, Rect rectangle)
{
	if(pt.x >= rectangle.x && pt.x <= (rectangle.x + rectangle.width) && pt.y >= rectangle.y && pt.y <= (rectangle.y + rectangle.height))
	{
		return true;
	}else{
		return false;
	}
}

Rect enlargeRectangle(Mat& sourceFrame, Rect rectangle, double scale){
	int Hincrease = round(scale / 2 * rectangle.width);
	int Vincrease = round(scale / 2 * rectangle.height);
	rectangle.width = min(sourceFrame.cols - 1, rectangle.x + rectangle.width + Hincrease) - max(0, rectangle.x - Hincrease);
	rectangle.height = min(sourceFrame.rows - 1, rectangle.y + rectangle.height + Vincrease) - max(0, rectangle.y - Vincrease);
	rectangle.x = max(0, rectangle.x - Hincrease);
	rectangle.y = max(0, rectangle.y - Vincrease);
	return rectangle;
}

#if WV_OPT_MOD_VIDEO
void mat2Buffer(Mat& source,char* buf, bool greyscale)
{
	// Put it in place of the original image for video stream and compatibility with cv_run()
	for (int row=0; row < source.rows; row++){
		for (int col=0; col < source.cols; col++){
			// Put back YUV image (Y1U/Y2V)
			if(greyscale)
			{
				buf[(row*source.cols+col)*2+0] 	= 127;
				buf[(row*source.cols+col)*2+1] 	= source.at<uint8_t>(row,col);
			}else{
				buf[(row*source.cols+col)*2+0] 	= source.at<Vec2b>(row,col)[0];
				buf[(row*source.cols+col)*2+1] 	= source.at<Vec2b>(row,col)[1];
			}
		}
	}
	return;
}
#endif

#if WV_OPT_BENCHMARK
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

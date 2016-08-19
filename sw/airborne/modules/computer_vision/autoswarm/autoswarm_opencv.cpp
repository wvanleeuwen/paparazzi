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

#define BOARD_CONFIG "boards/bebop.h"

#include "autoswarm_opencv.h"

#include <math.h>					// Used for sin/cos/tan/M_PI
#include <ctime> 					// Used to write time to results.txt
#include <algorithm> 				// Used for min() and max()
//#include <random>

extern "C" {
#include <firmwares/rotorcraft/navigation.h>
#include <subsystems/navigation/waypoints.h>
#include <state.h>
#include <generated/flight_plan.h>
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;
#include "modules/computer_vision/opencv_image_functions.h"

// Defining function that require opencv to be loaded
static void 			trackGreyObjects	(Mat& sourceFrame, Mat& greyFrame, vector<trackResults>* trackRes, Rect ROI = cvRect(0, 0, 0, 0));
static void 			identifyNeighbours	(vector<trackResults> trackRes);
static void 			updateWaypoints 	(struct NedCoor_f *pos, vector<double> cPos, vector<double> totV);
static void 			cam2body 			(trackResults* trackRes);
static void 			body2world 			(trackResults* trackRes, struct NedCoor_f *pos, struct 	FloatEulers * eulerAngles);
static bool 			rndRedGrayscale		(Mat& sourceFrame, Mat& destFrame, int sampleSize);
static bool 			pixFindContour		(Mat& sourceFrame, Mat& destFrame, int row, int col, int prevDir);
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

// Debug options
#define WV_DEBUG_SHOW_REJECT 		 0
#define WV_DEBUG_SHOW_WAYPOINT 		 1
#define WV_DEBUG_SHOW_MEM			 1

// Runtime options
#define WV_OPT_WRITE_RESULTS 		 1
#define WV_OPT_MOD_VIDEO 			 1
#define WV_OPT_CALIBRATE_CAM 		 0
#define AUTOSWARM_EVALUATE_FILTER 	 0

// Global options definitions
#define WV_GLOBAL_POINT 			0
#define WV_GLOBAL_BUCKET 			1
#define WV_GLOBAL_CIRCLE_CW 		2
#define WV_GLOBAL_CIRCLE_CC 		3

#if WV_OPT_CALIBRATE_CAM
static void 			calibrateEstimation (vector<trackResults> trackRes);
#endif
#if WV_OPT_MOD_VIDEO
static void 			mat2Buffer			(Mat& source, char* buf, bool greyscale = false);
#endif

// Set up tracking parameters
int 	WV_TRACK_MIN_CROP_AREA 		= 600;
int 	WV_TRACK_RND_PIX_SAMPLE 	= 10000;
int 	AUTOSWARM_MAX_LAYERS  		= 10000;
double 	WV_TRACK_MIN_CIRCLE_SIZE 	= 260;
double 	WV_TRACK_MAX_CIRCLE_DEF 	= 0.4;
double  WV_FILTER_CR 				= 1;
double  WV_FILTER_CG 				= 0.3;
double  WV_FILTER_CB 				= 0.3;
int 	WV_TRACK_GREY_THRESHOLD 	= 1; 		// [TEST 20160602 in optitrack] [HOME: 120] [OLD ZOO: 140] [OLD PPRZ 60]
int 	WV_TRACK_IMAGE_CROP_FOVY 	= 45; 		// Degrees

// Set up platform parameters
int 	WV_BEBOP_CAMERA_ANGLE 		= -20; 		// Degrees
double 	WV_BEBOP_CAMERA_OFFSET_X 	= 0.10; 	// Meters

// Set up swarm parameters
int 	WV_SWARM_MODE 				= 2; // 0: follow (deprecated), 1: look in direction of flight, 2: look in direction of global component
double 	WV_SWARM_SEPERATION 		= 1.8;
double 	WV_SWARM_E 					= 0.0009; // Was 0.01x - 0.0005 at 12m/s OK (but close)
double 	WV_SWARM_EPS 				= 0.05;
double 	WV_SWARM_GLOBAL 			= 0.9;
int    	WV_SWARM_FPS 				= 8;
double 	WV_SWARM_AMAX 				= 8; 	// m/s			// 4m/s op 90% global = CRASH!
double 	WV_SWARM_VMAX 				= 10; 	// m/s
double 	WV_SWARM_YAWRATEMAX 		= 70; 	// deg/s
int    	WV_SWARM_MEMORY 			= 3; 	// seconds
double 	WV_SWARM_HOME 				= 0.3;

// Set up global attractor parameters
int 	WV_GLOBAL_ATTRACTOR			= 1;
double 	WV_GLOBAL_CIRCLE_R 			= 1.5;
double 	WV_GLOBAL_DEADZONE 			= 0.1;

// Initialize parameters to be assigned during runtime
struct 	FloatEulers * 	eulerAngles;
struct 	NedCoor_f * 	groundSpeed;
double 	ispScalar;
int 	ispWidth;
int 	ispHeight;
int 	runCount = 0;
int 	pixCount = 0;
int 	maxId = 0;
vector<memoryBlock> neighbourMem;
extern uint8_t nav_block;
static const char * flight_blocks[] = FP_BLOCKS;

#if AUTOSWARM_COLOR_CALIBRATION
int YMin 	= 256;
int YMax 	= 0;
int CrMin 	= 256;
int CrMax 	= 0;
int CbMin 	= 256;
int CbMax 	= 0;
#endif

#if WV_OPT_WRITE_RESULTS
FILE * 	pFile;
time_t 	startTime;
char	resultFile [50];
time_t 	currentTime;
int 	curT;
#endif

int layerDepth = 0;
int sample = 0;

/* EXAMPLE
int opencv_example(char *img, int width, int height)
{
  // Create a new image, using the original bebop image.
  Mat M(height, width, CV_8UC2, img);
  Mat image;
  // If you want a color image, uncomment this line
   cvtColor(M, image, CV_YUV2BGR_Y422);
  // For a grayscale image, use this one
//  cvtColor(M, image, CV_YUV2GRAY_Y422);

  // Blur it, because we can
  blur(image, image, Size(5, 5));

  // Canny edges, only works with grayscale image
//  int edgeThresh = 35;
//  Canny(image, image, edgeThresh, edgeThresh * 3);

  // Convert back to YUV422, and put it in place of the original image
//  grayscale_opencv_to_yuv422(image, img, width, height);
  colorrgb_opencv_to_yuv422(image, img, width, height);

  return 0;
} */

void autoswarm_opencv_init(int globalMode)
{
#if WV_OPT_WRITE_RESULTS
	startTime 			= time(0);
	//tm * startTM 		= localtime(&startTime);
	//sprintf(resultFile, "/data/ftp/internal_000/%d-%02d-%02d_%02d-%02d-%02d.txt", startTM->tm_year, startTM->tm_mon, startTM->tm_mday, startTM->tm_hour, startTM->tm_min, startTM->tm_sec);
	//printf("Writing tracking results to: %s\n", resultFile);
	pFile = fopen("/data/ftp/internal_000/results.txt","w");
	fprintf(pFile,"ID\tmem\ti\tt\tposX\tposY\tposZ\tobjX\tobjY\tobjZ\tcPosX\tcposY\tvX\tvY\tgX\tgY\tlX\tlY\tdX\tdY\n");
	fclose(pFile);
#endif
	WV_GLOBAL_ATTRACTOR = globalMode;
	printf("AUTOSWARM initialized\n");
	return;
}

char* autoswarm_opencv_run(char* img, int width, int height)
{
	//if(nav_block < 3){ return img; }	// Engines have not started yet, lets save some battery life and skip image processing for now
	// Computer vision compatibility function used to call trackObjects and (optionally) parse modified data back to rtp stream as YUV
#if WV_OPT_WRITE_RESULTS 											// See if we want to write results
	currentTime = time(0); 											// Get the current time
	pFile 		= fopen("/data/ftp/internal_000/results.txt","a");	// Open file for appending TODO: Check for errors opening file
	curT 		= difftime(currentTime,startTime); 					// Calculate time-difference between startTime and currentTime
#endif
	vector<trackResults> trackRes; 							// Create empty struct _trackResults to store our neighbours in
#if MT9F002_RGB_OUTPUT
	Mat sourceFrame(height, width, CV_8UC4, img); 				// Initialize current frame in openCV (sRGB) 4 channel
#else
	Mat sourceFrame(height, width, CV_8UC2, img); 				// Initialize current frame in openCV (UYVY) 2 channel
#endif
	runCount++; 											// Update global run-counter
	eulerAngles = stateGetNedToBodyEulers_f(); 				// Get Euler angles
	groundSpeed = stateGetSpeedNed_f(); 					// Get groundspeed
	Rect crop 	= setISPvars(width, height, true); 			// Calculate ISP related parameters
	sourceFrame 	= sourceFrame(crop);
	Mat frameGrey(sourceFrame.rows, sourceFrame.cols, CV_8UC1, cvScalar(0.0));
	trackGreyObjects(sourceFrame, frameGrey, &trackRes, crop); 				// Track objects in sourceFrame
#if AUTOSWARM_COLOR_CALIBRATION
	printf("Y[%i, %i] Cb[%i, %i] Cr[%i, %i]\n",YMin, YMax, CbMin, CbMax, CrMin, CrMax);
#endif
#if WV_OPT_MOD_VIDEO
	vector<Mat> channels;
	Mat fullYUVFrame;
	Mat fullYFrame(height, width, CV_8UC1, cvScalar(0.0)); 		// Create empty matrix (Y channel)
	Mat fullUVFrame(height, width, CV_8UC1, cvScalar(127.0)); 	// Create empty matrix (UV channel)
	frameGrey.copyTo(fullYFrame(crop));
	channels.push_back(fullUVFrame);
	channels.push_back(fullYFrame);
	merge(channels, fullYUVFrame);
	fullYFrame.release();
	fullUVFrame.release();
#endif
	struct NedCoor_f *pos = stateGetPositionNed_f(); 		// Get your current position
	for(unsigned int r=0; r < trackRes.size(); r++)			// Convert angles & Write/Print output
	{
		cam2body(&trackRes[r]);								// Convert from camera angles to body angles (correct for roll)
		body2world(&trackRes[r], pos, eulerAngles); 		// Convert from body angles to world coordinates (correct yaw and pitch)
#if WV_OPT_MOD_VIDEO
		circle(fullYUVFrame,cvPoint(trackRes[r].x_p, trackRes[r].y_p), sqrt(trackRes[r].area_p / M_PI), cvScalar(0,0,255), 5);
#endif
	}
	identifyNeighbours(trackRes);
	vector<double> totV(3), cPos, li, gi, di; 				// Initialize total contribution for output
	li 		= calcLocalVelocity(pos);
	gi 		= calcGlobalVelocity(pos); 						// Get the contribution due to the "attraction" towards global origin
	di 		= calcDiffVelocity();
	totV[0] = li[0] + gi[0] + di[0]; 						// Average the X local contribution (#neighbours independent) and add the X global contribution
	totV[1] = li[1] + gi[1] + di[1]; 						// Do the same for Y
	totV[2] = li[2] + gi[2] + di[2]; 						// Do the same for Z
	totV 	= limitNorm(totV, WV_SWARM_AMAX / ((double) WV_SWARM_FPS)); 							// Check if ideal velocity exceeds WV_SWARM_AMAX
	// Inputting totV as an acceleration
	totV[0] = groundSpeed->x / ((double) WV_SWARM_FPS) + totV[0];
	totV[1] = groundSpeed->y / ((double) WV_SWARM_FPS) + totV[1];
	// Limit it again (otherwise speed could escalate)
	totV 		= limitNorm(totV, WV_SWARM_VMAX / ((double) WV_SWARM_FPS));
	if(WV_SWARM_MODE!=2) 	totV = limitVelocityYaw(totV); 	// Limit the velocity when relative angle gets larger
	cPos = calcCamPosition(pos, totV, gi);
	cPos = limitYaw(pos, cPos);
	for(unsigned int r=0; r < neighbourMem.size(); r++) 	// Print to file & terminal
	{
		char memChar = 'm'; int memInt = 0;
		if(neighbourMem[r].lastSeen == runCount){ memChar = 'w'; memInt = 1; }
		if(WV_DEBUG_SHOW_MEM) printf("Object (%c %i) at (%0.2f m, %0.2f m, %0.2f m) pos(%0.2f, %0.2f, %0.2f)\n", memChar, neighbourMem[r].id, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, pos->x, pos->y, pos->z); 														// Print to terminal
#if WV_OPT_WRITE_RESULTS
		fprintf(pFile, "%i\t%i\t%i\t%i\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\n", neighbourMem[r].id, memInt, runCount, curT, pos->x, pos->y, pos->z, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w, cPos[0], cPos[1], totV[0], totV[1], gi[0], gi[1], li[0], li[1], di[0], di[1]); // if file writing is enabled, write to file
#endif
	}
	updateWaypoints(pos,cPos,totV); 					// Translate velocity contribution to updated waypoint
	if(!strcmp("Swarm",flight_blocks[nav_block]) || !strcmp("Swarm Home",flight_blocks[nav_block])) // Check if currently in block "Swarm" or "Swarm Home"
	{
		nav_set_heading_towards_waypoint(WP__CAM);
	}
	if((WV_DEBUG_SHOW_MEM && neighbourMem.size() > 0) || WV_DEBUG_SHOW_WAYPOINT) printf("\n"); 										// Separate terminal output by newline
#if WV_OPT_WRITE_RESULTS
	fclose(pFile);	// Close file
#endif
#if WV_OPT_CALIBRATE_CAM
	if(runCount==10) // First and second frame are often not yet detected properly so let's calibrate the tenth frame
	{
		calibrateEstimation(trackRes);
	}
#endif
#if WV_OPT_MOD_VIDEO 	 											// See if we want to modify the video steam
	//coloryuv_opencv_to_yuv422(fullYUVFrame, img, width, height);
	mat2Buffer(fullYUVFrame, img, false);  							// Write openCV Mat to image_t buffer
	fullYUVFrame.release();
#endif
	sourceFrame.release();											// Release Mat
	frameGrey.release(); 										// Release Mat
	return img;													// Return image buffer as pointer
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
	ispHeight 		= width; 	// Save width and rotate 90degrees cc
	ispWidth 		= height; 	// Save height and rotate 90degrees cc
	if (crop==true)
	{
		double fovY 			= WV_TRACK_IMAGE_CROP_FOVY * M_PI / 180;
		double cmosPixelSize 	= 0.0000014; 	// 1.4um (see manual of CMOS sensor)
		double focalLength		= (2400 * cmosPixelSize * 1000 / ispScalar) / (4 * sin(M_PI / 4));
		double cY 				= round(sin((eulerAngles->theta - WV_BEBOP_CAMERA_ANGLE * M_PI / 180) / 4) * 2 * focalLength * ispScalar / (1000 * cmosPixelSize));
		double desHeight 		= round(sin(fovY / 4) * 4 * focalLength * ispScalar / (1000 * cmosPixelSize) + ispWidth * tan(eulerAngles->phi));
		double desOffset 		= cY + round((heightRange * ispScalar - desHeight) / 2);
		if(desOffset < 0) 			desOffset = 0;
		if(desOffset > ispHeight) 	desOffset = 0;

		if(desHeight < 0) 			desHeight = ispHeight;
		if(desHeight > ispHeight) 	desHeight = ispHeight;

		if(desHeight + desOffset > ispHeight)
		{
			desOffset = ispHeight - desHeight;
		}
		Rect crop = cvRect(desOffset,0,desHeight,ispWidth);
		return crop;
	}else{
		return cvRect(0, 0, width, height);
	}
}

void trackGreyObjects(Mat& sourceFrame, Mat& frameGrey, vector<trackResults>* trackRes, Rect ROI)
{
	// Main function for tracking object on a frame
	vector<vector<Point> > contours;
	//Mat frameGrey(sourceFrame.rows, sourceFrame.cols, CV_8UC1, cvScalar(0.0));
	//redGrayscale(sourceFrame, frameGrey);
	pixCount = 0;
	rndRedGrayscale(sourceFrame, frameGrey, WV_TRACK_RND_PIX_SAMPLE);
	printf("Pixels processed: %i\n",pixCount);
	findContours(frameGrey.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	for(unsigned int tc=0; tc < contours.size(); tc++)
	{
		double contArea = contourArea(contours[tc]);
		if (contArea > (WV_TRACK_MIN_CROP_AREA * ispScalar * ispScalar))
		{
			Point2f objCentre;
			float 	objRadius;
			minEnclosingCircle(contours[tc],objCentre,objRadius);
			float objArea = M_PI*objRadius*objRadius;
			if(objArea > (WV_TRACK_MIN_CIRCLE_SIZE * ispScalar * ispScalar) && contArea > objArea*WV_TRACK_MAX_CIRCLE_DEF)
			{
				trackResults curRes;
				vector<double> position(3);
				curRes.x_p 		= objCentre.x + ROI.x;
				curRes.y_p 		= objCentre.y;
				curRes.area_p 	= objArea;
				position 		= estimatePosition(curRes.x_p, curRes.y_p, curRes.area_p); // Estimate position in camera reference frame based on pixel location and area
				curRes.x_c 		= position[0];
				curRes.y_c 		= position[1];
				curRes.r_c 		= position[2];
				trackRes->push_back(curRes); 	// Save results and push into trackRes
			}else if(WV_DEBUG_SHOW_REJECT) 	printf("Rejected. object %f, area %f, fill %f < min fill %f.\n",objArea, contArea, contArea / objArea, WV_TRACK_MAX_CIRCLE_DEF);
		}else if(WV_DEBUG_SHOW_REJECT) 	printf("Rejected. Area %0.1f\n",contArea);
	}
	return;
}

bool rndRedGrayscale(Mat& sourceFrame, Mat& destFrame, int sampleSize)
{
	bool obj_detected = false;
	if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
	{
		//std::random_device rd;
		//std::mt19937 gen(rd());
		//std::uniform_int_distribution<> rowDis(0,sourceFrame.rows);
		//std::uniform_int_distribution<> colDis(0,sourceFrame.cols);
		int rndRow, rndCol;
		for(int i = 0; i<sampleSize; i++)
		{
			layerDepth = 0;
			sample++;
			//pixFindContour(sourceFrame, destFrame, rowDis(gen), colDis(gen));
			rndRow = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
			rndCol = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));

			if(pixFindContour(sourceFrame, destFrame, rndRow, rndCol, 0))
			{
				obj_detected = true;
			}
			//printf("Finished random pixel\n");
		}
	}
	return obj_detected;
}

bool pixFindContour(Mat& sourceFrame, Mat& destFrame, int row, int col, int prevDir)
{
	layerDepth++;
	pixCount++;
	int gr;
	if(layerDepth > AUTOSWARM_MAX_LAYERS || row < 0 || col < 0 || row > destFrame.rows-1 || col > destFrame.cols-1 || destFrame.at<uint8_t>(row,col) > 0)
	{
		return false;
	}
#if MT9F002_RGB_OUTPUT // FOR BGR
	int R, G, B;
	B 		= sourceFrame.at<Vec4b>(row,col)[0];
	G 		= sourceFrame.at<Vec4b>(row,col)[1];
	R 		= sourceFrame.at<Vec4b>(row,col)[2];
	gr 		= (int) (WV_FILTER_CR * R * R - WV_FILTER_CB * B * B - WV_FILTER_CG * G * G) / (255 * WV_FILTER_CR);
#else // FOR YUV422
	int Cb, Y, Cr, R, G, B;
	if ((col & 1) == 0)
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
	R 	= std::max((double) 0, std::min((double) 255, 1.164*(Y - 16) + 1.793*(Cr - 128)));
	G 	= std::max((double) 0, std::min((double) 255, 1.164*(Y - 16) - 0.534*(Cr - 128) - 0.213*(Cb - 128)));
	B 	= std::max((double) 0, std::min((double) 255, 1.164*(Y - 16) + 2.115*(Cr - 128)));
	gr 	= (int) std::max((double) 0, std::min((double) 255, (WV_FILTER_CR * pow(R,2) - WV_FILTER_CB * pow(B,2) - WV_FILTER_CG * pow(G,2)) / (25 * WV_FILTER_CR)));
#endif
#if AUTOSWARM_EVALUATE_FILTER
	if (gr > 1)
	{
		destFrame.at<uint8_t>(row,col) = gr;
	}
	if(gr >= WV_TRACK_GREY_THRESHOLD)
		{
#else
	if(gr >= WV_TRACK_GREY_THRESHOLD)
	{
		destFrame.at<uint8_t>(row,col) = 255;
#endif
		if(prevDir==3 || row<=0 || !pixFindContour(sourceFrame, destFrame, row - 1, col, 1))
		{
			if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
			if(prevDir==4 || col>=(sourceFrame.cols - 1) || !pixFindContour(sourceFrame, destFrame, row, col+1, 2))
			{
				if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
				if(prevDir==1 || row>=(sourceFrame.rows - 1) || !pixFindContour(sourceFrame, destFrame, row+1, col, 3))
				{
					if(layerDepth > AUTOSWARM_MAX_LAYERS) return false;
					if(prevDir==2 || col<=0 || !pixFindContour(sourceFrame, destFrame, row, col-1, 4))
					{
						return false;
					}
				}
			}
		}
		return true;
	}else{
		destFrame.at<uint8_t>(row,col) = 1;
		return false;
	}
}

vector<double> estimatePosition(int xp, int yp, double area, double k, int calArea, int orbDiag)
{
	// This function estimates the 3D position (in camera  coordinate system) according to pixel position
	// (Default) calibration parameters
	if(k==0) 		k 		= 1.085; // Fisheye correction factor
	if(calArea==0) 	calArea = 5330; // Calibrate at full resolution
	if(orbDiag==0) 	orbDiag = 2200; // Measured circular image diagonal using resolution of 2024x2048 org: 2400
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

	vector< vector<double> > calPositions(6, vector<double>(3));
	calPositions[0][0] 	=  1.00;
	calPositions[0][1] 	=  0.00;
	calPositions[0][2] 	=  0.25;

	calPositions[1][0] 	=  2.00;
	calPositions[1][1] 	=  0.00;
	calPositions[1][2] 	=  0.25;

	calPositions[2][0] 	=  2.00;
	calPositions[2][1] 	= -1.00;
	calPositions[2][2] 	=  0.25;

	calPositions[3][0] 	=  1.00;
	calPositions[3][1] 	= -1.00;
	calPositions[3][2] 	=  0.25;

	calPositions[4][0] 	=  1.00;
	calPositions[4][1] 	=  1.00;
	calPositions[4][2] 	=  0.25;

	calPositions[5][0] 	=  1.00;
	calPositions[5][1] 	=  2.00;
	calPositions[5][2]	=  0.25;

	double k_opt;
	double k_min 		= 1.0;
	double k_max 		= 1.15;
	double k_step 		= 0.0025;

	int calArea_opt;
	int calArea_min 	= 5100;
	int calArea_max 	= 6100;
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
					body2world(&trackRes[r], &fakePos , eulerAngles); 	// Convert from body angles to world coordinates (correct yaw and pitch)
					double ball_err = 1000;
					for(unsigned int i=0; i < calPositions.size(); i++)
					{
						double cur_ball_err = pow(trackRes[r].x_w - calPositions[i][0], 2.0) + pow(trackRes[r].y_w - calPositions[i][1], 2.0) + pow(trackRes[r].z_w - calPositions[i][2], 2.0);
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

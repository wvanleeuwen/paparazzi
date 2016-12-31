/*
 * Copyright (C) Wilco Vlenterie (wv-tud)
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
 * @file "modules/computer_vision//cv_active_random_filter.c"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */

#include "active_random_filter.h"
#include <vector>
#include <ctime>

extern "C" {
    #include "boards/bebop.h"                       // C header used for bebop specific settings
    #include <state.h>                              // C header used for state functions and data
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


#define PRINT(string,...) fprintf(stderr, "[AR-FILTER->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#define AR_FILTER_VERBOSE TRUE
#if AR_FILTER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

extern struct mt9f002_t mt9f002;

#define AR_FILTER_CV_CONTOURS   0 // Use opencv to detect contours
#define AR_FILTER_ISP_CROP      0 // Use the ISP to crop the frame according to FOV-Y
#define AR_FILTER_SHOW_REJECT   0 // Print why shapes are rejected
#define AR_FILTER_MOD_VIDEO     1 // Modify the frame to show relevant info
#define AR_FILTER_DRAW_CONTOURS 1 // Use drawContours function iso circle
#define AR_FILTER_DRAW_CIRCLES  0 // Draw circles
#define AR_FILTER_DRAW_BOXES 	0 // Draw boxes
#define AR_FILTER_SHOW_MEM      1 // Print object locations to terminal
#define AR_FILTER_SAVE_FRAME    0 // Save a frame for post-processing
#define AR_FILTER_MEASURE_FPS   1
#define AR_FILTER_CALIBRATE_CAM 0 // Calibrate camera
#define AR_FILTER_WORLDPOS 		0 // Use world coordinates
#define AR_FILTER_NOYAW 		1 // Output in body horizontal XY
#define AR_FILTER_TIMEOUT       50 // Frames from start

#define AR_FILTER_SAMPLE_STYLE  AR_FILTER_STYLE_RANDOM
#define AR_FILTER_FLOOD_STYLE   AR_FILTER_FLOOD_CW

static void             active_random_filter_header(Mat& sourceFrame);
static void             active_random_filter_footer(void);
static void 			trackGreyObjects	(Mat& sourceFrame, Mat& greyFrame);
static void             identifyObject     (trackResults* trackRes);
static bool 			addContour			(vector<Point> contour, uint16_t offsetX, uint16_t offsetY);
static void 			cam2body 			(trackResults* trackRes);
static void 			body2world 			(trackResults* trackRes, struct 	FloatEulers * eulerAngles);
static bool 			rndRedGrayscale		(Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize);
static int 				pixFindContour		(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade);
static double 			correctRadius		(double r, double f, double k);
static Rect 			setISPvars 			(struct FloatEulers* eulerAngles, uint16_t width, uint16_t height);
static vector<double> 	estimatePosition	(uint16_t xp, uint16_t yp, uint32_t area, double k = 0, uint16_t calArea = 0, uint16_t orbDiag = 0);
static void 			getNextDirection	(uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt);
static bool 			getNewPosition		(uint8_t nextDir, uint16_t* newRow, uint16_t* newCol, int* maxRow, int* maxCol);
static void             eraseMemory         (void);
static void             getYUVColours       (Mat& sourceFrame, uint16_t row, uint16_t col, uint8_t* Y, uint8_t* U, uint8_t* V);
// Optional function declarations
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
static bool             add_current_contour (void);
static void             add_point_to_contour(uint16_t* row, uint16_t* col);
static int              pixFindContour_cw   (Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade);
static Moments          contourMoments      ( void );
#else
static int              pixFindContour_omni (Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade);
static void             addObject           (void);
static Rect             enlargeRectangle    (Mat& sourceFrame, Rect rectangle, double scale);
static bool             inRectangle         (Point pt, Rect rectangle);
#endif
#if AR_FILTER_MOD_VIDEO
static void             mod_video           (Mat& sourceFrame, Mat& frameGrey);
#endif
#if AR_FILTER_CALIBRATE_CAM
static void 			calibrateEstimation (void);
#endif
#if AR_FILTER_SAVE_FRAME
static void 			saveBuffer			(Mat sourceFrame, const char *filename);
#endif

#if AR_FILTER_MEASURE_FPS
static time_t 	startTime;
static time_t 	currentTime;
static uint32_t curT;
#endif

// Set up tracking parameters
#define 	AR_FILTER_RND_PIX_SAMPLE 	2500
#define     AR_FILTER_MIN_LAYERS        10
#define 	AR_FILTER_MAX_LAYERS        3000
#define     AR_FILTER_MIN_POINTS        4

uint16_t    AR_FILTER_MIN_CROP_AREA     = 250;
double 	    AR_FILTER_MIN_CIRCLE_SIZE 	= 500;
double 	    AR_FILTER_MAX_CIRCLE_DEF 	= 0.05;

/* FAKE LIGHT
uint8_t     AR_FILTER_Y_MIN             = 0;                            // 0  [0,65 84,135 170,255]zoo 45
uint8_t     AR_FILTER_Y_MAX             = 255;                          // 255
uint8_t     AR_FILTER_U_MIN             = 118;                          // 84
uint8_t     AR_FILTER_U_MAX             = 170;                          // 113
uint8_t     AR_FILTER_V_MIN             = 155;                          // 218 -> 150?
uint8_t     AR_FILTER_V_MAX             = 255;                          // 240 -> 255?
*/

/* DAYLIGHT
uint8_t 	AR_FILTER_Y_MIN 			= 0;                           // 0  [0,65 84,135 170,255]zoo 45
uint8_t 	AR_FILTER_Y_MAX 			= 200;                          // 255
uint8_t 	AR_FILTER_U_MIN 			= 110;                          // 84
uint8_t 	AR_FILTER_U_MAX 			= 150;                          // 113
uint8_t 	AR_FILTER_V_MIN 			= 175;                          // 218 -> 150?
uint8_t 	AR_FILTER_V_MAX 			= 210;                          // 240 -> 255?
*/

/* DAYLIGHT 2 */
uint8_t     AR_FILTER_Y_MIN             = 123;                           // 0  [0,65 84,135 170,255]zoo 45
uint8_t     AR_FILTER_Y_MAX             = 222;                          // 255
uint8_t     AR_FILTER_U_MIN             = 109;                          // 84
uint8_t     AR_FILTER_U_MAX             = 130;                          // 113
uint8_t     AR_FILTER_V_MIN             = 150;                          // 218 -> 150?
uint8_t     AR_FILTER_V_MAX             = 209;                          // 240 -> 255?

double 	    AR_FILTER_IMAGE_CROP_FOVY 	= 45 * M_PI / 180.0; 		    // Radians
double 	    AR_FILTER_CROP_X 			= 1.2;
uint8_t     AR_FILTER_MEMORY 			= 15;
uint8_t     AR_FILTER_FPS               = 15;
double      AR_FILTER_VMAX              = 3.5;

// Set up platform parameters
double	    AR_FILTER_CAMERA_ANGLE 	    = -30 * M_PI / 180.0; 		    // Radians
double 	    AR_FILTER_CAMERA_OFFSET_X   = 0.10; 	                    // Meters

// Initialize parameters to be assigned during runtime
static uint16_t 	    pixCount        = 0;
static uint16_t 	    pixSucCount     = 0;
static uint16_t         pixDupCount     = 0;
static uint16_t         pixSrcCount     = 0;
static uint16_t         pixNofCount     = 0;
static uint16_t 	    layerDepth      = 0;
static uint16_t 	    sample          = 0;
static uint16_t 	    runCount        = 0;
static uint16_t 	    maxId 		    = 0;
static uint16_t         ispWidth;
static uint16_t         ispHeight;
static uint16_t         cropCol;
static double           ispScalar;
Rect 			        objCrop;
vector<Rect> 		    cropAreas;

#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
static Point            objCont_store[AR_FILTER_MAX_LAYERS+1];
static uint16_t         objCont_size    = 0;
static uint16_t         objCont_sCol    = 0;
static uint16_t         objCont_sRow    = 0;
#endif

vector<vector<Point> >  allContours;
vector<memoryBlock>     neighbourMem;
vector<trackResults>    trackRes;

void active_random_filter_init(void){
}

void active_random_filter(char* buff, uint16_t width, uint16_t height, struct FloatEulers* eulerAngles)
{
    if(runCount < AR_FILTER_TIMEOUT) {
        runCount++;
        PRINT("Timeout %d\n", AR_FILTER_TIMEOUT - runCount);
        return;
    }
    Mat sourceFrame (height, width, CV_8UC2, buff);                 // Initialize current frame in openCV (UYVY) 2 channel
    Mat frameGrey   (height, width, CV_8UC1, cvScalar(0.0));        // Initialize an empty 1 channel frame
    active_random_filter_header(sourceFrame);                       // Mostly printing and storing
    Rect crop 	= setISPvars(eulerAngles, width, height); 	        // Calculate ISP related parameters
	sourceFrame = sourceFrame(crop); 				                // Crop the frame
	trackGreyObjects(sourceFrame, frameGrey);                       // Track objects in sourceFrame
	eraseMemory();
	uint8_t r;
	for(r=0; r < trackRes.size(); r++){                             // Convert angles & Write/Print output
		cam2body(&trackRes[r]);						                // Convert from camera angles to body angles (correct for roll)
		body2world(&trackRes[r], eulerAngles); 		                // Convert from body angles to world coordinates (correct yaw and pitch)
		identifyObject(&trackRes[r]);                              // Identify the spotted neighbours
	}
#if AR_FILTER_MOD_VIDEO
	mod_video(sourceFrame, frameGrey);                              // Modify the sourceframe
#endif // AR_FILTER_MOD_VIDEO
	frameGrey.release(); 			                                // Release Mat
	sourceFrame.release();                                          // Release Mat
	active_random_filter_footer();
	return;
}

Rect setISPvars(struct FloatEulers* eulerAngles, uint16_t width, uint16_t height)
{
    // This function computes the cropping according to the desires FOV Y and the current euler angles
#if AR_FILTER_ISP_CROP
    ispHeight                   = MT9F002_OUTPUT_WIDTH;
    ispWidth                    = MT9F002_OUTPUT_HEIGHT;
#else
    ispHeight                   = width;
    ispWidth                    = height;
#endif
    ispScalar                   = mt9f002.output_scaler * 2.0/((double) mt9f002.x_odd_inc + 1.0);
    double horizonDeviation     = 0.5 * M_PI - eulerAngles->theta - AR_FILTER_CAMERA_ANGLE;
    uint16_t top                = (uint16_t) round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * ispScalar * (horizonDeviation + AR_FILTER_IMAGE_CROP_FOVY * 0.5) / M_PI);
    uint16_t desOffset          = (uint16_t) round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * ispScalar * (horizonDeviation - AR_FILTER_IMAGE_CROP_FOVY * 0.5) / M_PI);
    desOffset                  += 0.5 * ((uint16_t) ispWidth * tan(eulerAngles->phi));
    uint16_t desHeight          = top - desOffset - 0.5 * ((uint16_t) ispWidth * tan(eulerAngles->phi));
    if(desOffset > ispHeight){
        desOffset                   = 0;
    }
    if(desHeight > ispHeight){
        desHeight                   = ispHeight;
    }
    if((desHeight + desOffset) > ispHeight){
        desOffset                   = ispHeight - desHeight;
    }
    if((desOffset & 1) != 0){
        desOffset--;
    }
#if AR_FILTER_ISP_CROP
    cropCol                     = 0;
    mt9f002.offset_x            = MT9F002_INITIAL_OFFSET_X + desOffset / ispScalar;
    mt9f002.output_width        = desHeight / ispScalar;
    Rect crop                   = cvRect(0,0,width,height);
    mt9f002_set_resolution(&mt9f002);
#else
    cropCol                     = desOffset;
    Rect crop                   = cvRect(desOffset,0,desHeight,ispWidth);
#endif
    return crop;
}

void trackGreyObjects(Mat& sourceFrame, Mat& frameGrey)
{
    // Main function for tracking multiple objects on the frame
    pixCount    = 0;
    pixSucCount = 0;
    pixSrcCount = 0;
    pixNofCount = 0;
    pixDupCount = 0;
    rndRedGrayscale(sourceFrame, frameGrey, AR_FILTER_RND_PIX_SAMPLE);
    VERBOSE_PRINT("Total of %d contours found\n",allContours.size());
#if AR_FILTER_BENCHMARK
    addBenchmark("image Thresholded");
#endif // AR_FILTER_BENCHMARK
#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
    vector<vector<Point> > contours;
    for(unsigned int r=0; r < cropAreas.size(); r++)
    {
        if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
        {
            contours.clear();
#if AR_FILTER_MOD_VIDEO && AR_FILTER_DRAW_BOXES
            findContours(frameGrey(cropAreas[r]).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#else
            findContours(frameGrey(cropAreas[r]), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#endif //AR_FILTER_MOD_VIDEO && AR_FILTER_DRAW_BOXES
            for(unsigned int tc=0; tc < contours.size(); tc++)
            {
                addContour(contours[tc], (uint16_t) cropAreas[r].x, (uint16_t) cropAreas[r].y);
            }
        }
    }
#if AR_FILTER_BENCHMARK
    addBenchmark("Contours found");
#endif //AR_FILTER_BENCHMARK
#endif //AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
    return;
}

void eraseMemory(void){
    // This function let's the agent forget previous measurements
    for(unsigned int i=0; i < neighbourMem.size();)
    {
        if((runCount - neighbourMem[i].lastSeen) > AR_FILTER_MEMORY)
        {
            neighbourMem.erase(neighbourMem.begin() + i);
        }
        else{
            i++;
        }
    }
}

void identifyObject(trackResults* trackRes)
{
	// We now only have memory samples from the past AR_FILTER_MEMORY frames so lets try to identify the neighbours we saw
    for(unsigned int i=0; i < neighbourMem.size(); i++)
    {
        double radius	= (runCount - neighbourMem[i].lastSeen) * 1.0 / ((double) AR_FILTER_FPS) * AR_FILTER_VMAX;
        double dx 		= trackRes->x_w - neighbourMem[i].x_w;
        double dy       = trackRes->y_w - neighbourMem[i].y_w;
        if(dx <= radius && dy <= radius && sqrt(pow(dx, 2.0) + pow(dy, 2.0)) <= radius)
        {
            PRINT("Identified object %d at (%d, %d)p (%f, %f)w\n", neighbourMem[i].id, trackRes->x_p, trackRes->y_p, trackRes->x_w, trackRes->y_w);
            neighbourMem[i].lastSeen 	= runCount;
            neighbourMem[i].x_w 		= trackRes->x_w;
            neighbourMem[i].y_w 		= trackRes->y_w;
            neighbourMem[i].z_w 		= trackRes->z_w;
            neighbourMem[i].x_p 		= trackRes->x_p;
            neighbourMem[i].y_p 		= trackRes->y_p;
            neighbourMem[i].area_p      = trackRes->area_p;
            return;
        }
    }
    // We haven't identified
    PRINT("New object at (%d, %d)p (%f, %f)c (%f, %f)b (%f, %f)w\n", trackRes->x_p, trackRes->y_p, trackRes->x_c, trackRes->y_c, trackRes->x_b, trackRes->y_b, trackRes->x_w, trackRes->y_w);
    memoryBlock curN;
    curN.lastSeen 	= runCount;
    curN.id 		= maxId;
    curN.x_w 		= trackRes->x_w;
    curN.y_w 		= trackRes->y_w;
    curN.z_w 		= trackRes->z_w;
    curN.x_p 		= trackRes->x_p;
    curN.y_p 		= trackRes->y_p;
    curN.area_p     = trackRes->area_p;
    neighbourMem.push_back(curN);
    maxId++;
    return;
}

vector<double> estimatePosition(uint16_t xp, uint16_t yp, uint32_t area, double k, uint16_t calArea, uint16_t orbDiag)
{
    // This function estimates the 3D position (in camera  coordinate system) according to pixel position
    // (Default) calibration parameters
    if(!k)          k       = 1.085;                                                        // Fisheye correction factor (1.085)
    if(!calArea)    calArea = 9920;                                                         // Calibrate at full resolution (5330)
    if(!orbDiag)    orbDiag = sqrt(2) * (CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN);  // Measured circular image diagonal using full resolution
    // Calculate corrected calibration parameters
    calArea                 = (int) round(calArea * pow(ispScalar,2.0));
    double cmosPixelSize    = 0.0000014;                                                    // 1.4um (see manual of CMOS sensor)
    double ispScalingNorm   = 1 / ispScalar * cmosPixelSize * 1000;
    double fovDiag          = M_PI;                                                         // [radian] Diagonal field of view (see bebop manual)
    // Calculate relevant parameters
    uint16_t cX             = round(ispWidth * 0.5);
    uint16_t cY             = round(ispHeight * 0.5);
    double frameSizeDiag    = orbDiag * cmosPixelSize * 1000;                               // [mm] Find used diagonal size of CMOS sensor
    double f                = frameSizeDiag / (4 * sin(fovDiag / 4));                       // [mm]
    // Calculate FoV in x and y direction
    double fovX             = 4 * asin((((double) ispWidth) * ispScalingNorm)/(4 * f));
    double fovY             = 4 * asin((((double) ispHeight) *  ispScalingNorm)/(4 * f));
    int x                   = yp - cX;                                                      // rotate frame cc 90 degrees (ISP is rotated 90 degrees cc)
    int y                   = xp - cY;                                                      // rotate frame cc 90 degrees (ISP is rotated 90 degrees cc)
    // Convert to polar coordinates
    double r                = sqrt(pow((double) x, 2.0) + pow((double) y, 2.0)) * ispScalingNorm;   // [mm] radial distance from middle of CMOS
    double theta            = atan2(y,x);
    // Correct radius for radial distortion using correction parameter k
    double corR             = correctRadius(r, f, k);
    // Convert back to Cartesian
    double corX             = corR * cos(theta);
    double corY             = corR * sin(theta);
    double corArea          = area * pow(corR / r, 2.0);                                    // radius = sqrt(area) / sqrt(M_PI)     --->    newRadius = radius * corR / r   --->    corArea = M_PI * newRadius * newRadius
    // Calculate distance
    double dist             = sqrt((double) calArea) / sqrt(corArea);
    // Calculate max width and height of undistorted frame
    double maxX             = correctRadius(ispWidth * 0.5 * ispScalingNorm, f, k);
    double maxY             = correctRadius(ispHeight * 0.5 * ispScalingNorm, f, k);
    // Calculate angles wrt camera
    double xAngle           = (fovX * 0.5) * (corX / maxX);                                   // Assume all non-linearities have been removed from corX and corY
    double yAngle           = (fovY * 0.5) * (corY / maxY);                                   // Then scale them according to the max values and axis FoV
    // Store in dest and parse it back
    vector<double> dest(3);
    dest[0]                 = xAngle;
    dest[1]                 = yAngle;
    dest[2]                 = dist;
    VERBOSE_PRINT("pixel (%d, %d) angle (%.2f, %.2f) area(%d) dist(%.2f)\n",xp,yp,xAngle * 180/M_PI,yAngle * 180/M_PI, area, dist);
    return dest;
}

double correctRadius(double r, double f, double k)
{
    // This function calculates the corrected radius for radial distortion
    // According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
    return f / k * tan( asin( sin( atan( r / f ) ) * k ) );
}

void cam2body(trackResults* trackRes)
{
    // Neighbour position returned in 2 angles and a radius.
    // x_c is the angle wrt vertical camera axis.   Defined clockwise/right positive
    // y_c is angle wrt camera horizon axis.        Defined upwards positive
    // r_c is radial distance in m.
    trackRes->x_b = trackRes->r_c * cos( -trackRes->y_c - AR_FILTER_CAMERA_ANGLE ) * cos( trackRes->x_c ) + AR_FILTER_CAMERA_OFFSET_X;
    trackRes->y_b = trackRes->r_c * cos( -trackRes->y_c - AR_FILTER_CAMERA_ANGLE ) * sin( trackRes->x_c );
    trackRes->z_b = trackRes->r_c * sin( -trackRes->y_c - AR_FILTER_CAMERA_ANGLE );
    return;
}

void body2world(trackResults* trackRes, struct  FloatEulers * eulerAngles)
{
    struct NedCoor_f *pos;
#if AR_FILTER_WORLDPOS
    pos     = stateGetPositionNed_f();      // Get your current position
#else
    struct NedCoor_f fakePos;
    fakePos.x       = 0.0;
    fakePos.y       = 0.0;
    fakePos.z       = 0.0;
    pos             = &fakePos;
#endif
#if AR_FILTER_NOYAW
    double psi      = 0.0;
#else
    double psi      = eulerAngles->psi;
#endif
    Matx33f rotX(    1,                         0,                      0,
                     0,                         cos(eulerAngles->phi), -sin(eulerAngles->phi),
                     0,                         sin(eulerAngles->phi),  cos(eulerAngles->phi));

    Matx33f rotY(    cos(eulerAngles->theta),   0,                      sin(eulerAngles->theta),
                     0,          1,             0,
                    -sin(eulerAngles->theta),   0,                      cos(eulerAngles->theta));

    Matx33f rotZ(    cos(psi),                 -sin(psi),               0,
                     sin(psi),                  cos(psi),               0,
                     0,                         0,                      1);
    Matx31f bPos(trackRes->x_b, trackRes->y_b, trackRes->z_b);
    Matx31f wPos    = rotZ * rotY * rotX * bPos;
    trackRes->x_w   = wPos(0,0) + pos->x;
    trackRes->y_w   = wPos(1,0) + pos->y;
    trackRes->z_w   = wPos(2,0) + pos->z;
    return;
}

bool addContour(vector<Point> contour, uint16_t offsetX, uint16_t offsetY)
{
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
	VERBOSE_PRINT("Analyzing contour of length %d\n", objCont_size);
    Moments m = contourMoments();
#else
    VERBOSE_PRINT("Analyzing contour of length %d\n", contour.size());
    Moments m = moments(contour);
#endif // AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
    PRINT("m00: %f  x: %f  y: %f\n",m.m00, m.m10 / m.m00, m.m01 / m.m00);
    double e = (pow(m.mu20 - m.mu02,2.0) - 4 * pow(m.mu11,2.0)) / pow(m.mu20 + m.mu02,2.0);
    PRINT("eccentricity: %f\n", e);
    if (m.m00 > (AR_FILTER_MIN_CIRCLE_SIZE * ispScalar * ispScalar))
	{
		if(e < AR_FILTER_MAX_CIRCLE_DEF)
		{
			trackResults curRes;
			vector<double> position(3);
			curRes.x_p 		    = m.m10 / m.m00 + cropCol + offsetX;
			curRes.y_p 		    = m.m01 / m.m00 + offsetY;
			curRes.area_p 	    = (uint32_t) m.m00;
			position 		    = estimatePosition(curRes.x_p, curRes.y_p, curRes.area_p);  // Estimate position in camera reference frame based on pixel location and area
			curRes.x_c 		    = position[0];
			curRes.y_c 		    = position[1];
			curRes.r_c 		    = position[2];
			trackRes.push_back(curRes); 	                                                // Save results and push into trackRes
			return true;
		}else if(AR_FILTER_SHOW_REJECT) {
		    VERBOSE_PRINT("Rejected. object area %f, e %f.\n",m.m00, e);
		}

	}else if(AR_FILTER_SHOW_REJECT){
	    VERBOSE_PRINT("Rejected. Area %0.1f\n",m.m00);
	}
	return false;
}

#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
static Moments contourMoments( void ){
    Moments m;
    if( objCont_size == 0 )
        return m;

    double a00 = 0, a10 = 0, a01 = 0, a20 = 0, a11 = 0, a02 = 0, a30 = 0, a21 = 0, a12 = 0, a03 = 0;
    double xi, yi, xi2, yi2, xi_1, yi_1, xi_12, yi_12, dxy, xii_1, yii_1;

    xi_1 = objCont_store[objCont_size-1].x;
    yi_1 = objCont_store[objCont_size-1].y;

    xi_12 = xi_1 * xi_1;
    yi_12 = yi_1 * yi_1;

    for( int i = 0; i < objCont_size; i++ )
    {
        xi = objCont_store[i].x;
        yi = objCont_store[i].y;

        xi2 = xi * xi;
        yi2 = yi * yi;
        dxy = xi_1 * yi - xi * yi_1;
        xii_1 = xi_1 + xi;
        yii_1 = yi_1 + yi;

        a00 += dxy;
        a10 += dxy * xii_1;
        a01 += dxy * yii_1;
        a20 += dxy * (xi_1 * xii_1 + xi2);
        a11 += dxy * (xi_1 * (yii_1 + yi_1) + xi * (yii_1 + yi));
        a02 += dxy * (yi_1 * yii_1 + yi2);
        a30 += dxy * xii_1 * (xi_12 + xi2);
        a03 += dxy * yii_1 * (yi_12 + yi2);
        a21 += dxy * (xi_12 * (3 * yi_1 + yi) + 2 * xi * xi_1 * yii_1 +
                   xi2 * (yi_1 + 3 * yi));
        a12 += dxy * (yi_12 * (3 * xi_1 + xi) + 2 * yi * yi_1 * xii_1 +
                   yi2 * (xi_1 + 3 * xi));
        xi_1 = xi;
        yi_1 = yi;
        xi_12 = xi2;
        yi_12 = yi2;
    }

    if( fabs(a00) > 1.19209289550781250000e-7F)
    {
        double db1_2, db1_6, db1_12, db1_24, db1_20, db1_60;

        if( a00 > 0 )
        {
            db1_2 = 0.5;
            db1_6 = 0.16666666666666666666666666666667;
            db1_12 = 0.083333333333333333333333333333333;
            db1_24 = 0.041666666666666666666666666666667;
            db1_20 = 0.05;
            db1_60 = 0.016666666666666666666666666666667;
        }
        else
        {
            db1_2 = -0.5;
            db1_6 = -0.16666666666666666666666666666667;
            db1_12 = -0.083333333333333333333333333333333;
            db1_24 = -0.041666666666666666666666666666667;
            db1_20 = -0.05;
            db1_60 = -0.016666666666666666666666666666667;
        }

        // spatial moments
        m.m00 = a00 * db1_2;
        m.m10 = a10 * db1_6;
        m.m01 = a01 * db1_6;
        m.m20 = a20 * db1_12;
        m.m11 = a11 * db1_24;
        m.m02 = a02 * db1_12;
        m.m30 = a30 * db1_20;
        m.m21 = a21 * db1_60;
        m.m12 = a12 * db1_60;
        m.m03 = a03 * db1_20;

        double cx = 0, cy = 0;
        double mu20, mu11, mu02;
        double inv_m00 = 0.0;

        if( fabs(m.m00) > double(2.22044604925031308085e-16L))
        {
            inv_m00 = 1. / m.m00;
            cx = m.m10 * inv_m00;
            cy = m.m01 * inv_m00;
        }

        mu20 = m.m20 - m.m10 * cx;
        mu11 = m.m11 - m.m10 * cy;
        mu02 = m.m02 - m.m01 * cy;

        m.mu20 = mu20;
        m.mu11 = mu11;
        m.mu02 = mu02;

        m.mu30 = m.m30 - cx * (3 * mu20 + cx * m.m10);
        mu11 += mu11;
        m.mu21 = m.m21 - cx * (mu11 + cx * m.m01) - cy * mu20;
        m.mu12 = m.m12 - cy * (mu11 + cy * m.m10) - cx * mu02;
        m.mu03 = m.m03 - cy * (3 * mu02 + cy * m.m01);

        double inv_sqrt_m00 = std::sqrt(std::abs(inv_m00));
        double s2 = inv_m00*inv_m00, s3 = s2*inv_sqrt_m00;

        m.nu20 = m.mu20*s2; m.nu11 = m.mu11*s2; m.nu02 = m.mu02*s2;
        m.nu30 = m.mu30*s3; m.nu21 = m.mu21*s3; m.nu12 = m.mu12*s3; m.nu03 = m.mu03*s3;
    }
    return m;
}
#endif // AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW

bool rndRedGrayscale(Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize)
{
	bool obj_detected   = false;
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	objCont_size        = 0;
	allContours.clear();
#else
	cropAreas.clear();
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
	{
#if AR_FILTER_SAMPLE_STYLE > 0
	    for(unsigned int rnm=0; rnm < neighbourMem.size(); rnm++)
	    {
#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
	        objCrop.x 		        = neighbourMem[rnm].x_p - cropCol;
	        objCrop.y 		        = neighbourMem[rnm].y_p;
	        objCrop.width 	        = 0;
	        objCrop.height 	        = 0;
#endif //AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
	        // Let's search for previous blobs in a 3x3 + 8 grid centered around previous location
	        uint8_t searchPoints    = 17;
	        uint16_t sGridSize      =  0.5 * sqrt(neighbourMem[rnm].area_p / M_PI);
	        Point searchGrid[searchPoints];
	        searchGrid[0]   = Point(neighbourMem[rnm].y_p,                neighbourMem[rnm].x_p - cropCol);
	        searchGrid[1]   = Point(neighbourMem[rnm].y_p,                neighbourMem[rnm].x_p - cropCol + sGridSize);
	        searchGrid[2]   = Point(neighbourMem[rnm].y_p,                neighbourMem[rnm].x_p - cropCol - sGridSize);

	        searchGrid[3]   = Point(neighbourMem[rnm].y_p + sGridSize,    neighbourMem[rnm].x_p - cropCol);
	        searchGrid[4]   = Point(neighbourMem[rnm].y_p + sGridSize,    neighbourMem[rnm].x_p - cropCol + sGridSize);
	        searchGrid[5]   = Point(neighbourMem[rnm].y_p + sGridSize,    neighbourMem[rnm].x_p - cropCol - sGridSize);

	        searchGrid[6]   = Point(neighbourMem[rnm].y_p - sGridSize,    neighbourMem[rnm].x_p - cropCol);
	        searchGrid[7]   = Point(neighbourMem[rnm].y_p - sGridSize,    neighbourMem[rnm].x_p - cropCol + sGridSize);
	        searchGrid[8]   = Point(neighbourMem[rnm].y_p - sGridSize,    neighbourMem[rnm].x_p - cropCol - sGridSize);

	        searchGrid[9]   = Point(neighbourMem[rnm].y_p + 2*sGridSize,  neighbourMem[rnm].x_p - cropCol);
	        searchGrid[10]  = Point(neighbourMem[rnm].y_p - 2*sGridSize,  neighbourMem[rnm].x_p - cropCol);
	        searchGrid[11]  = Point(neighbourMem[rnm].y_p,                neighbourMem[rnm].x_p - cropCol + 2*sGridSize);
	        searchGrid[12]  = Point(neighbourMem[rnm].y_p,                neighbourMem[rnm].x_p - cropCol - 2*sGridSize);

	        searchGrid[13]  = Point(neighbourMem[rnm].y_p + 2*sGridSize,  neighbourMem[rnm].x_p - cropCol + 2*sGridSize);
	        searchGrid[14]  = Point(neighbourMem[rnm].y_p - 2*sGridSize,  neighbourMem[rnm].x_p - cropCol + 2*sGridSize);
	        searchGrid[15]  = Point(neighbourMem[rnm].y_p + 2*sGridSize,  neighbourMem[rnm].x_p - cropCol - 2*sGridSize);
	        searchGrid[16]  = Point(neighbourMem[rnm].y_p - 2*sGridSize,  neighbourMem[rnm].x_p - cropCol - 2*sGridSize);
	        uint8_t rsg;
	        bool    foundObj        = false; // We're pessimistic that we can find the same object
	        for(rsg=0; rsg < searchPoints; rsg++)
	        {
	            layerDepth              = 0;
	            if(pixFindContour(sourceFrame, destFrame, searchGrid[rsg].x, searchGrid[rsg].y, ARF_SEARCH, true) == ARF_FINISHED){
	                if(layerDepth > AR_FILTER_MIN_LAYERS){
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	                    if(add_current_contour()){
	                        int cId = trackRes.size() - 1;
	                        VERBOSE_PRINT("Found object %d from (%d, %d) at (%d, %d)\n",neighbourMem[rnm].id, neighbourMem[rnm].x_p, neighbourMem[rnm].y_p, trackRes[cId].x_p, trackRes[cId].y_p);
	                        foundObj                = true;
	                        obj_detected            = true;
	                        break;
	                    }
#else
	                    objCrop                 = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
	                    addObject();
	                    foundObj                = true;
	                    obj_detected            = true;
	                    break;
#endif // AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	                }
	            }
	        }
	        if(!foundObj){
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            objCont_size = 0;
#endif // AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            VERBOSE_PRINT("Could not find object %d\n",neighbourMem[rnm].id);
	        }
	    }
#endif //AR_FILTER_SAMPLE_STYLE > 0
#if AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_FULL
	    for(int r = 0; r < sourceFrame.rows; r++)
	    {
	        for(int c= 0; c < sourceFrame.cols; c++)
	        {
	            layerDepth              = 0;
	            if(pixFindContour(sourceFrame, destFrame, r, c, ARF_UP, false) == ARF_SUCCESS)
	            {
	                obj_detected            = true;
	            }
	        }
	    }
	    Rect fullCrop;
	    fullCrop.x 		= 1;
	    fullCrop.y 		= 0;
	    fullCrop.width 	= sourceFrame.cols-1;
	    fullCrop.height = sourceFrame.rows;
	    cropAreas.push_back(fullCrop);
#endif //AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_FULL
#if AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_GRID
	    int spacing     = (int) sqrt((sourceFrame.rows * sourceFrame.cols) / sampleSize);
	    for(int r = spacing; r < sourceFrame.rows; r+=spacing)
	    {
	        for(int c=spacing; c < sourceFrame.cols; c+=spacing)
	        {
	            sample++;
	            layerDepth      = 0;
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            //objCont.clear();
	            objCont_size    = 0;
#else
	            objCrop.x 		= c;
	            objCrop.y 		= r;
	            objCrop.width 	= 0;
	            objCrop.height 	= 0;
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            if(pixFindContour(sourceFrame, destFrame, r, c, ARF_SEARCH, true) == ARF_FINISHED)
	            {
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	                if(layerDepth > AR_FILTER_MIN_LAYERS)
	                {
	                    if(layerDepth > AR_FILTER_MIN_LAYERS)
	                    {
	                        add_current_contour();
	                        obj_detected = true;
	                    }
	                }
#else
	                objCrop         = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
	                obj_detected    = true;
	                addObject();
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            }
	        }
	    }
#endif //AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_GRID
#if AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_RANDOM
	    int rndRow, rndCol;
	    for(int i = 0; i<sampleSize; i++)
	    {
	        layerDepth  = 0;
	        sample++;
	        rndRow      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
	        rndCol      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	        objCont_size    = 0;
#else
	        objCrop.x 		= rndCol;
	        objCrop.y 		= rndRow;
	        objCrop.width 	= 0;
	        objCrop.height 	= 0;
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	        bool too_close  = false;
	        for(unsigned int tr = 0; tr < trackRes.size(); tr++){
	            if(sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)) <= 1.15 * sqrt(trackRes[tr].area_p / M_PI)){
	                //VERBOSE_PRINT("Not using starting point %d, %d due to distance to object %d at %d, %d (dist: %f < %f)\n", rndRow, rndCol, tr, trackRes[tr].y_p, trackRes[tr].x_p - cropCol, 1.15 * sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)), sqrt(trackRes[tr].area_p / M_PI));
	                too_close = true;
	                break;
	            }

	        }
	        if(too_close){
	            continue;
	        }
	        if(pixFindContour(sourceFrame, destFrame, rndRow, rndCol, ARF_SEARCH, true) == ARF_FINISHED)
	        {
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	            if(layerDepth > AR_FILTER_MIN_LAYERS)
	            {
	                if(layerDepth > AR_FILTER_MIN_LAYERS)
	                {
	                    add_current_contour();
	                    obj_detected = true;
	                }
	            }
#else
	            objCrop 		= enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
	            obj_detected 	= true;
	            addObject();
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	        }
	    }
	    /*
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
        VERBOSE_PRINT("Found %i contours, processing %i of %i pixels (%0.2f%%)\n", allContours.size(), pixCount, sourceFrame.rows * sourceFrame.cols, 100 * pixCount / ((float) sourceFrame.rows * sourceFrame.cols));
#else
        VERBOSE_PRINT("Found %i cropAreas, processing %i of %i pixels (%0.2f%%)\n", cropAreas.size(), pixCount, sourceFrame.rows * sourceFrame.cols, 100 * pixCount / ((float) sourceFrame.rows * sourceFrame.cols));
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS
	     */
#endif //AR_FILTER_SAMPLE_STYLE == AR_FILTER_STYLE_RANDOM
	}
	return obj_detected;
}

#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
bool add_current_contour(void)
{
    if(objCont_size > AR_FILTER_MIN_POINTS){
        vector<Point> objCont;
        if(addContour(objCont, (uint16_t) 0, (uint16_t) 0)){
#if AR_FILTER_DRAW_CONTOURS
            objCont.reserve(objCont_size);
            for(unsigned int r = 0; r < objCont_size; r++){
                objCont.push_back(objCont_store[r]);
            }
            allContours.push_back(objCont);
#endif
            return true;
        }
    }
    objCont_size = 0;
    return false;
}
#endif

#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
void add_point_to_contour(uint16_t* row, uint16_t* col)
{
    objCont_store[objCont_size] = Point(*col,*row);
    objCont_size++;
}
#endif

int pixFindContour(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade){
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
    return pixFindContour_cw(sourceFrame, destFrame, row, col, prevDir, cascade);
#else
    return pixFindContour_omni(sourceFrame, destFrame, row, col, prevDir, cascade);
#endif
}

#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
int pixFindContour_cw(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade){
    layerDepth++;
    pixCount++;
    if(prevDir == ARF_SEARCH && destFrame.at<uint8_t>(row, col) >= 75){
            pixDupCount++;
            return ARF_DUPLICATE;
    }
    else if(prevDir != ARF_SEARCH && destFrame.at<uint8_t>(row, col) == 75){
        if (col - objCont_sCol >= -1 && col - objCont_sCol <= 1 && row - objCont_sRow >= -1 && row - objCont_sRow <= 1 ){
            if(layerDepth > AR_FILTER_MIN_LAYERS){
                destFrame.at<uint8_t>(row, col) = 255;
                add_point_to_contour(&row,&col);
                VERBOSE_PRINT("ARF_FINISHED back at (%d, %d) near startpos (%d, %d) after %d pixels\n",row, col, objCont_sRow, objCont_sCol, layerDepth);
                pixSucCount++;
                return ARF_FINISHED;
            }
            else{
                VERBOSE_PRINT("ARF_NO_FOUND back at (%d, %d) near startpos (%d, %d) after only %d pixels\n",row, col, objCont_sRow, objCont_sCol, layerDepth);
                pixNofCount++;
                objCont_size = 0;
                return ARF_FINISHED;
            }
        }
        /*else{
            VERBOSE_PRINT("Found different starting position at (%d, %d) iso startpos (%d, %d) diff (%d, %d) after %d pixels\n",row, col, objCont_sRow, objCont_sCol, abs(col - objCont_sCol), abs(row - objCont_sRow), layerDepth);
        }*/
    }
    uint8_t U, Y, V;
    getYUVColours(sourceFrame, row, col, &Y, &U, &V);
    if(Y >= AR_FILTER_Y_MIN && Y <= AR_FILTER_Y_MAX && U >= AR_FILTER_U_MIN && U <= AR_FILTER_U_MAX && V >= AR_FILTER_V_MIN && V <= AR_FILTER_V_MAX){
        if(prevDir != ARF_SEARCH){
            destFrame.at<uint8_t>(row, col) = 255;
        }
        else{
            destFrame.at<uint8_t>(row, col) = 75;
        }
        if(cascade){
            uint8_t nextDirCnt, nextDir[6];
            uint16_t newRow, newCol;
            bool success = false;
            uint8_t d = 0, edge = 0;
            getNextDirection(prevDir, nextDir, &nextDirCnt);
            while(layerDepth < AR_FILTER_MAX_LAYERS && d < nextDirCnt && success == false){
                newRow              = row;
                newCol              = col;
                if(getNewPosition(nextDir[d], &newRow, &newCol, &sourceFrame.rows, &sourceFrame.cols)){
                    if(prevDir == ARF_SEARCH && d == 1){
                        objCont_sCol        = newCol;
                        objCont_sRow        = newRow;
                        objCont_size        = 0;
                    }
                    switch(pixFindContour(sourceFrame, destFrame, newRow, newCol, nextDir[d], true)){ // Catch the proper response for the tested pixel
                    case ARF_FINISHED : {
                        if(prevDir!=ARF_SEARCH){
                            if(prevDir != nextDir[d]){
                                add_point_to_contour(&row,&col);
                            }
                            pixSucCount++;
                        }
                        else{
                            pixSrcCount++;
                        }
                        return ARF_FINISHED;
                        break;
                    }
                    case ARF_SUCCESS : {
                        pixSucCount++;
                        return ARF_SUCCESS;
                        break;
                    }
                    case ARF_NO_FOUND : {
                        edge++;
                        break;
                    }
                    case ARF_DUPLICATE : {
                        pixDupCount++;
                        return ARF_DUPLICATE;
                        break;
                    }
                    case ARF_ERROR : {
                        if(layerDepth > AR_FILTER_MAX_LAYERS){
                            return ARF_FINISHED;
                        }
                        else{
                            edge++;
                        }
                        break;
                    }
                    default : {
                        return ARF_ERROR;
                    }
                    }
                }
                else{
                    edge++;
                }
                d++;
            }
            pixNofCount++;
            return ARF_NO_FOUND; // Dead end
        }
        else{
            pixSucCount++;
            return ARF_SUCCESS;
        }
    }
    else{
        pixNofCount++;
        return ARF_NO_FOUND;
    }
}

#else
int pixFindContour_omni(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade)
{
    layerDepth++;
    pixCount++;
    if(prevDir == ARF_SEARCH){
        prevDir = ARF_UP;
    }
    if(destFrame.at<uint8_t>(row, col) == 255){
        pixDupCount++;
        return ARF_DUPLICATE;
    }
    uint8_t U, Y, V;
    getYUVColours(sourceFrame, row, col, &Y, &U, &V);
    if(Y >= AR_FILTER_Y_MIN && Y <= AR_FILTER_Y_MAX && U >= AR_FILTER_U_MIN && U <= AR_FILTER_U_MAX && V >= AR_FILTER_V_MIN && V <= AR_FILTER_V_MAX)
    {
        if(prevDir != ARF_SEARCH)
        {
            destFrame.at<uint8_t>(row, col) = 255;
        }
        if(cascade)
        {
            pixSucCount++;
            uint8_t nextDir[3];
            uint8_t nextDirCnt  = 3;
            getNextDirection(prevDir, nextDir, &nextDirCnt);
            bool success        = false;
            uint16_t d          = 0;
            uint16_t newRow, newCol;
            int res[4]          = {ARF_NO_FOUND,ARF_NO_FOUND,ARF_NO_FOUND,ARF_NO_FOUND};
            while(d < nextDirCnt && success == false)
            {
                newRow              = row;
                newCol              = col;
                if(getNewPosition(nextDir[d], &newRow, &newCol, &sourceFrame.rows, &sourceFrame.cols)){
                    res[d]              = pixFindContour(sourceFrame, destFrame, newRow, newCol, nextDir[d], true);
                }
                else{
                    res[d]              = ARF_NO_FOUND;
                }
                d++;
            }
            if(res[0] <= ARF_NO_FOUND && res[1] <= ARF_NO_FOUND && res[2] <= ARF_NO_FOUND && res[3] <= ARF_NO_FOUND)
            {
                return ARF_FINISHED;
            }
            objCrop.width       = max<uint16_t>(objCrop.x + objCrop.width,  col) - min<uint16_t>(objCrop.x, col);
            objCrop.height      = max<uint16_t>(objCrop.y + objCrop.height, row) - min<uint16_t>(objCrop.y, row);
            objCrop.x           = min<uint16_t>(objCrop.x, col);
            objCrop.y           = min<uint16_t>(objCrop.y, row);
            return ARF_FINISHED;
        }else{
            pixSucCount++;
            return ARF_SUCCESS;
        }
    }else{
        pixNofCount++;
        return ARF_NO_FOUND;
    }
}
#endif // AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW

void getYUVColours(Mat& sourceFrame, uint16_t row, uint16_t col, uint8_t* Y, uint8_t* U, uint8_t* V){
    if (((col & 1) == 0 && (cropCol & 1) == 0) || ((col & 1) == 1 && (cropCol & 1) == 1))
    {
        // Even col number
        *U = sourceFrame.at<Vec2b>(row, col)[0]; // U1
        *Y = sourceFrame.at<Vec2b>(row, col)[1]; // Y1
        if(col + 1 < sourceFrame.cols)
        {
            *V = sourceFrame.at<Vec2b>(row, col + 1)[0]; // V2
        }else{
            *V = sourceFrame.at<Vec2b>(row, col - 1)[0]; // V2
        }
    }else{
        // Uneven col number
        *V = sourceFrame.at<Vec2b>(row, col)[0]; // V2
        *Y  = sourceFrame.at<Vec2b>(row, col)[1]; // Y2
        if(col > 0)
        {
            *U = sourceFrame.at<Vec2b>(row, col - 1)[0]; // U1
        }else{
            *U = sourceFrame.at<Vec2b>(row, col + 1)[0]; // U1
        }
    }
}

void getNextDirection(uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt)
{
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_OMNI
    switch(prevDir)
    {
	case ARF_UP :
	    nextDir[0] = ARF_LEFT;
	    nextDir[1] = ARF_UP;
	    nextDir[2] = ARF_RIGHT;
	    *nextDirCnt = 3;
	    break;
	case ARF_RIGHT :
	    nextDir[0] = ARF_UP;
	    nextDir[1] = ARF_RIGHT;
	    nextDir[2] = ARF_DOWN;
	    *nextDirCnt = 3;
	    break;
	case ARF_DOWN :
	    nextDir[0] = ARF_RIGHT;
	    nextDir[1] = ARF_DOWN;
	    nextDir[2] = ARF_LEFT;
	    *nextDirCnt = 3;
	    break;
	case ARF_LEFT :
	    nextDir[0] = ARF_DOWN;
	    nextDir[1] = ARF_LEFT;
	    nextDir[2] = ARF_UP;
	    *nextDirCnt = 3;
	    break;
	}
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_OMNI
#if AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
	*nextDirCnt = 5;
	switch(prevDir) // Find out which directions to try next
	{
	default:
	case ARF_SEARCH :
	    nextDir[0] = ARF_SEARCH;
	    nextDir[1] = ARF_UP_RIGHT;
	    nextDir[2] = ARF_RIGHT;
	    nextDir[3] = ARF_RIGHT_DOWN;
	    *nextDirCnt = 4;
	    break;
	case ARF_UP :
	    nextDir[0] = ARF_LEFT;
	    nextDir[1] = ARF_LEFT_UP;
	    nextDir[2] = ARF_UP;
	    nextDir[3] = ARF_UP_RIGHT;
	    nextDir[4] = ARF_RIGHT;
	    //nextDir[5] = ARF_RIGHT_DOWN;
	    break;
	case ARF_UP_RIGHT :
	    nextDir[0] = ARF_LEFT_UP;
	    nextDir[1] = ARF_UP;
	    nextDir[2] = ARF_UP_RIGHT;
	    nextDir[3] = ARF_RIGHT;
	    nextDir[4] = ARF_RIGHT_DOWN;
	    //nextDir[5] = ARF_DOWN;
	    break;
	case ARF_RIGHT :
	    nextDir[0] = ARF_UP;
	    nextDir[1] = ARF_UP_RIGHT;
	    nextDir[2] = ARF_RIGHT;
	    nextDir[3] = ARF_RIGHT_DOWN;
	    nextDir[4] = ARF_DOWN;
	    //nextDir[5] = ARF_DOWN_LEFT;
	    break;
	case ARF_RIGHT_DOWN :
	    nextDir[0] = ARF_UP_RIGHT;
	    nextDir[1] = ARF_RIGHT;
	    nextDir[2] = ARF_RIGHT_DOWN;
	    nextDir[3] = ARF_DOWN;
	    nextDir[4] = ARF_DOWN_LEFT;
	    //nextDir[5] = ARF_LEFT;
	    break;
	case ARF_DOWN :
	    nextDir[0] = ARF_RIGHT;
	    nextDir[1] = ARF_RIGHT_DOWN;
	    nextDir[2] = ARF_DOWN;
	    nextDir[3] = ARF_DOWN_LEFT;
	    nextDir[4] = ARF_LEFT;
	    //nextDir[5] = ARF_LEFT_UP;
	    break;
	case ARF_DOWN_LEFT :
	    nextDir[0] = ARF_RIGHT_DOWN;
	    nextDir[1] = ARF_DOWN;
	    nextDir[2] = ARF_DOWN_LEFT;
	    nextDir[3] = ARF_LEFT;
	    nextDir[4] = ARF_LEFT_UP;
	    //nextDir[5] = ARF_UP;
	    break;
	case ARF_LEFT :
	    nextDir[0] = ARF_DOWN;
	    nextDir[1] = ARF_DOWN_LEFT;
	    nextDir[2] = ARF_LEFT;
	    nextDir[3] = ARF_LEFT_UP;
	    nextDir[4] = ARF_UP;
	    //nextDir[5] = ARF_UP_RIGHT;
	    break;
	case ARF_LEFT_UP :
	    nextDir[0] = ARF_DOWN_LEFT;
	    nextDir[1] = ARF_LEFT;
	    nextDir[2] = ARF_LEFT_UP;
	    nextDir[3] = ARF_UP;
	    nextDir[4] = ARF_UP_RIGHT;
	    //nextDir[5] = ARF_RIGHT;
	    break;
	}
#endif //AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW
	return;
}

bool getNewPosition(uint8_t nextDir, uint16_t* newRow, uint16_t* newCol, int* maxRow, int* maxCol)
{
	switch(nextDir) // Set the location of the next pixel to test
	{
	case ARF_SEARCH :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_UP :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
            return false;
        }
	    break;
	case ARF_UP_RIGHT :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol < *maxCol - 1)
	    {
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_RIGHT :
	    if(*newCol < *maxCol - 1){
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_RIGHT_DOWN :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol < *maxCol - 1){
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_DOWN :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
		break;
	case ARF_DOWN_LEFT :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
		if(*newCol > 0){
		    *newCol += -1;
		}
		else{
		    return false;
		}
		break;
	case ARF_LEFT :
	    if(*newCol > 0){
	        *newCol += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_LEFT_UP :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol > 0){
	        *newCol += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	default:
	    VERBOSE_PRINT("[AR_FILTER-ERR] Invalid next-dir: %i\n", nextDir);
		return false;
		break;
	}
	return true;
}

#if AR_FILTER_MOD_VIDEO
void mod_video(Mat& sourceFrame, Mat& frameGrey)
{
	char text[200];
#if AR_FILTER_MEASURE_FPS
	sprintf(text,"%5.2f %5.d %0.0fs", (runCount - AR_FILTER_TIMEOUT) / ((double) curT),(runCount - AR_FILTER_TIMEOUT),((double) curT));
#else
	sprintf(text,"frame %i", runCount);
#endif // AR_FILTER_MEASURE_FPS
#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
#if AR_FILTER_DRAW_BOXES
		for(unsigned int r=0; r < cropAreas.size(); r++)
		{
			if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
			{
				vector<Mat> channels;
				Mat thr_frame(cropAreas[r].height, cropAreas[r].width, CV_8UC2, cvScalar(0.0,0.0));
				Mat emptyCH(cropAreas[r].height, cropAreas[r].width, CV_8UC1, cvScalar(127.0));
				channels.push_back(emptyCH);
				channels.push_back(frameGrey(cropAreas[r]));
				merge(channels, thr_frame);
				thr_frame.copyTo(sourceFrame(cropAreas[r])); 			               // Copy threshold result to black frame
				emptyCH.release();
				thr_frame.release();
				rectangle(sourceFrame, cropAreas[r], Scalar(0,255), 2);
			}
		}
#endif //AR_FILTER_DRAW_BOXES
#else
#if AR_FILTER_DRAW_CONTOURS
		drawContours(sourceFrame, allContours, -1, cvScalar(0,255), 2);
#endif //AR_FILTER_DRAW_CONTOURS
#endif //AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
#if AR_FILTER_DRAW_CIRCLES
	for(unsigned int r=0; r < trackRes.size(); r++)         // Convert angles & Write/Print output
	{
		circle(sourceFrame,cvPoint(trackRes[r].x_p - cropCol, trackRes[r].y_p), sqrt(trackRes[r].area_p / M_PI), cvScalar(0,255), 2);
	}
#endif //AR_FILTER_DRAW_CIRCLES
	putText(sourceFrame, text, Point(10,sourceFrame.rows-40), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,255), 2);
	for(unsigned int r=0; r < trackRes.size(); r++)         // Convert angles & Write/Print output
	{
	    sprintf(text,"x:%5.2f y:%5.2f", trackRes[r].x_w, trackRes[r].y_w);
	    putText(sourceFrame, text, Point(10,40+r*40), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,255), 2);
	}
	sprintf(text,"t:%4.1f%% o:%4.1f%%", pixCount/((float) ispHeight * ispWidth) * 100, pixSucCount/((float) pixCount) * 100);
	putText(sourceFrame, text, Point(10,sourceFrame.rows-120), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,255), 2);
	sprintf(text,"d:%4.1f%% n:%4.1f%% s:%4.1f%%", pixDupCount/((float) pixCount) * 100, pixNofCount/((float) pixCount) * 100, pixSrcCount/((float) pixCount) * 100);
	putText(sourceFrame, text, Point(10,sourceFrame.rows-80), FONT_HERSHEY_PLAIN, 2, Scalar(0,255,255), 2);

	line(sourceFrame, Point(0,0), Point(0, sourceFrame.rows-1), Scalar(0,255), 4);
	line(sourceFrame, Point(sourceFrame.cols - 1,0), Point(sourceFrame.cols - 1, sourceFrame.rows-1), Scalar(0,255), 4);
	return;
}
#endif // AR_FILTER_MOD_VIDEO

void active_random_filter_header(Mat& sourceFrame)
{
#if AR_FILTER_MEASURE_FPS
    if(runCount == AR_FILTER_TIMEOUT)
    {
        startTime       = time(0);
    }else{
        currentTime     = time(0);                                              // Get the current time
        curT            = difftime(currentTime,startTime);                      // Calculate time-difference between startTime and currentTime
        VERBOSE_PRINT("Measured FPS: %0.2f\n", (runCount - AR_FILTER_TIMEOUT) / ((double) curT));
    }
#endif
#if !AR_FILTER_CV_CONTOURS && AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW
    PRINT("[AR-FILTER-ERR] No openCV contours is only possible with clockwise flooding.\n");
#endif //!AR_FILTER_CV_CONTOURS && AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW
    trackRes.clear();
#if AR_FILTER_SAVE_FRAME
    if(runCount == 25) saveBuffer(sourceFrame, "testBuffer.txt");   // (optional) save a raw UYVY frame
#endif // AR_FILTER_SAVE_FRAME
}

void active_random_filter_footer(void)
{
#if AR_FILTER_SHOW_MEM
    for(unsigned int r=0; r < neighbourMem.size(); r++)        // Print to file & terminal
    {
        PRINT("%i - Object %d at (%0.2f m, %0.2f m, %0.2f m)\n", runCount, neighbourMem[r].id, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w);                                                        // Print to terminal
    }
    printf("\n");
#endif // AR_FILTER_SHOW_MEM
#if AR_FILTER_CALIBRATE_CAM
    if(runCount==50)    calibrateEstimation();
#endif // AR_FILTER_CALIBRATE_CAM
    VERBOSE_PRINT("pixCount: %d  (%.2f%%), pixSucCount: %d  (%.2f%%), pixDupCount: %d  (%.2f%%), pixNofCount: %d  (%.2f%%), pixSrcCount: %d  (%.2f%%)\n", pixCount, pixCount/((float) ispHeight * ispWidth) * 100, pixSucCount, pixSucCount/((float) pixCount) * 100, pixDupCount, pixDupCount/((float) pixCount) * 100, pixNofCount, pixNofCount/((float) pixCount) * 100, pixSrcCount, pixSrcCount/((float) pixCount) * 100);
    runCount++;                                                // Increase counter
}

#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
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
            objCrop.width       = max(objCrop.x + objCrop.width, cropAreas[r].x + cropAreas[r].width) - min(objCrop.x, cropAreas[r].x);
            objCrop.height      = max(objCrop.y + objCrop.height, cropAreas[r].y + cropAreas[r].height) - min(objCrop.y, cropAreas[r].y);
            objCrop.x           = min(objCrop.x, cropAreas[r].x);
            objCrop.y           = min(objCrop.y, cropAreas[r].y);
            cropAreas[r].x      = 0;
            cropAreas[r].y      = 0;
            cropAreas[r].width  = 0;
            cropAreas[r].height = 0;
        }
    }
    if(objCrop.width * objCrop.height >= AR_FILTER_MIN_CROP_AREA * ispScalar * ispScalar)
    {
        cropAreas.push_back(objCrop);
    }
    return;
}
#endif

#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
bool inRectangle(Point pt, Rect rectangle)
{
    if(pt.x >= rectangle.x && pt.x <= (rectangle.x + rectangle.width) && pt.y >= rectangle.y && pt.y <= (rectangle.y + rectangle.height))
    {
        return true;
    }else{
        return false;
    }
}
#endif

#if AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS
Rect enlargeRectangle(Mat& sourceFrame, Rect rectangle, double scale){
    int Hincrease       = round(scale / 2 * rectangle.width);
    int Vincrease       = round(scale / 2 * rectangle.height);
    rectangle.width     = min(sourceFrame.cols - 1, rectangle.x + rectangle.width + Hincrease) - max(0, rectangle.x - Hincrease);
    rectangle.height    = min(sourceFrame.rows - 1, rectangle.y + rectangle.height + Vincrease) - max(0, rectangle.y - Vincrease);
    rectangle.x         = max(0, rectangle.x - Hincrease);
    rectangle.y         = max(0, rectangle.y - Vincrease);
    return rectangle;
}
#endif

#if AR_FILTER_CALIBRATE_CAM
void calibrateEstimation(void)
{
	PRINT("Starting calibration!\n");
	struct 	FloatEulers	fakeEulerAngles;
	fakeEulerAngles.phi 	= 0.0;
	fakeEulerAngles.psi 	= 0.0;
	fakeEulerAngles.theta 	= 0.0;

	vector< vector<double> > calPositions(6, vector<double>(3));
	calPositions[0][0] 	=  1.00;
	calPositions[0][1] 	=  0.00;
	calPositions[0][2] 	=  0.50;
/*
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
*/
	double k_opt        = 0;
	double k_min 		= 1.0;
	double k_max 		= 1.05;
	double k_step 		= 0.0025;

	int calArea_opt     = 0;
	int calArea_min 	= 9000;
	int calArea_max 	= 12000;
	int calArea_step 	=   10;

	int orbDiag_opt     = 0;
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
					cam2body();							// Convert from camera angles to body angles (correct for roll)
					body2world(&fakeEulerAngles); 	// Convert from body angles to world coordinates (correct yaw and pitch)
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
				PRINT("\r%0.2f percent", 100 * i / ((double) totI));
			}
		}
	}
	PRINT("\nCalibration finished. Avg error: %0.3f.\t k=%0.3f\t calArea=%i\t orbDiag=%i\n", sqrt(opt_err), k_opt, calArea_opt, orbDiag_opt);
}
#endif

#if AR_FILTER_SAVE_FRAME
void saveBuffer(Mat sourceFrame, const char *filename)
{
	char path[100];
	sprintf(path,"/data/ftp/internal_000/%s", filename);
	FILE * iFile = fopen(path,"w");
	PRINT("Writing imagebuffer(%i x %i) to file %s  ... ", sourceFrame.rows, sourceFrame.cols, path);
	for(int row = 0; row < sourceFrame.rows; row++)
	{
		for(int col = 0; col < sourceFrame.cols; col++)
		{
			fprintf(iFile, "%i,%i", sourceFrame.at<Vec2b>(row,col)[0], sourceFrame.at<Vec2b>(row,col)[1]);
			if(col != sourceFrame.cols-1)
			{
				fprintf(iFile, ",");
			}
		}
		fprintf(iFile,"\n");
	}
	fclose(iFile);
	PRINT(" done.\n");
}
#endif

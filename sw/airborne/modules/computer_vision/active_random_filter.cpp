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

extern "C" {
#include <state.h>
//#include <random>
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#define AR_FILTER_CV_CONTOURS  0 // Use opencv to detect contours
#define AR_FILTER_ISP_CROP     0 // Use the ISP to crop the frame according to FOV-Y
#define AR_FILTER_SHOW_REJECT  0 // Print why shapes are rejected
#define AR_FILTER_MOD_VIDEO    0 // Modify the frame to show relevant info
#define AR_FILTER_SHOW_MEM     0 // Print object locations to terminal

static void 			trackGreyObjects	(Mat& sourceFrame, Mat& greyFrame, vector<trackResults>* trackRes);
static void 			addContour			(vector<Point> contour, vector<trackResults>* trackRes);
static void 			cam2body 			(trackResults* trackRes);
static void 			body2world 			(trackResults* trackRes, struct 	FloatEulers * eulerAngles);
static bool 			rndRedGrayscale		(Mat& sourceFrame, Mat& destFrame, int sampleSize);
static bool 			pixFindContour		(Mat& sourceFrame, Mat& destFrame, int row, int col, int prevDir, bool cascade);
static double 			correctRadius		(double r, double f, double k);
static Rect 			setISPvars 			(int width, int height, bool crop = false);
static vector<double> 	estimatePosition	(int xp, int yp, double area, double k = 0, int calArea = 0, int orbDiag = 0);
static void 			addObject			(void);
static Rect 			enlargeRectangle	(Mat& sourceFrame, Rect rectangle, double scale);
static bool 			inRectangle			(Point pt, Rect rectangle);
// Optional function declarations
#if AR_FILTER_CALIBRATE_CAM
static void 			calibrateEstimation (vector<trackResults> trackRes);
#endif
/* #if AR_FILTER_MOD_VIDEO
static void 			mat2Buffer			(Mat& source, char* buf, bool greyscale = false);
#endif */
#if AR_FILTER_SAVE_FRAME
static void 			saveBuffer			(char * img, Mat sourceFrame, const char *filename);
#endif

// Set up tracking parameters
int 	AR_FILTER_MIN_CROP_AREA 	= 170;
int 	AR_FILTER_RND_PIX_SAMPLE 	= 5000;
int 	AR_FILTER_MAX_LAYERS  		= 100000;
double 	AR_FILTER_MIN_CIRCLE_SIZE 	= 200;
double 	AR_FILTER_MAX_CIRCLE_DEF 	= 0.4;

int 	AR_FILTER_SAMPLE_STYLE 		= AR_FILTER_STYLE_RANDOM;
int 	AR_FILTER_FLOOD_STYLE 	    = AR_FILTER_FLOOD_CW;
int 	AR_FILTER_Y_MIN 			= 65;  // 0 					[0,65 84,135 170,255]zoo 45
int 	AR_FILTER_Y_MAX 			= 190; // 255
int 	AR_FILTER_CB_MIN 			= 75; // 84
int 	AR_FILTER_CB_MAX 			= 135; // 113
int 	AR_FILTER_CR_MIN 			= 170; // 218 -> 150?
int 	AR_FILTER_CR_MAX 			= 240; // 240 -> 255?
int 	AR_FILTER_IMAGE_CROP_FOVY 	= 45; 		// Degrees
int 	AR_FILTER_SAVE_RESULTS 		= 0;
double 	AR_FILTER_CROP_X 			= 1.2;

// Set up platform parameters
int 	AR_FILTER_BEBOP_CAMERA_ANGLE 		= -13; 		// Degrees
double 	AR_FILTER_BEBOP_CAMERA_OFFSET_X 	= 0.10; 	// Meters

// Initialize parameters to be assigned during runtime
struct 	FloatEulers * 	eulerAngles;
double 	ispScalar;
int 	ispWidth;
int 	ispHeight;
int 	cropCol;
int 	pixCount = 0;
int 	pixSucCount = 0;
int 	shotSucces 	= 0;
int 	shotFail 	= 0;
int 	layerDepth = 0;
int 	sample = 0;
int 	runCount = 0;
Rect 	objCrop;
vector<Rect> cropAreas;
vector<Point> objCont;
vector<vector<Point> > allContours;

vector<trackResults> trackRes; 							// Create empty struct _trackResults to store our neighbours in
vector<trackResults> newTrackRes;

void active_random_filter(char* buff, int width, int height)
{
	if(!AR_FILTER_CV_CONTOURS && AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW)
	{
		printf("[AR-FILTER-ERR] No openCV contours is only possible with clockwise flooding.\n");
	}
	newTrackRes.clear();
	eulerAngles 			= stateGetNedToBodyEulers_f(); 	// Get Euler angles
	Mat sourceFrame(height, width, CV_8UC2, buff); 	// Initialize current frame in openCV (UYVY) 2 channel
	Rect crop 	= setISPvars(width, height, true); 	// Calculate ISP related parameters
#if !AR_FILTER_ISP_CROP
	sourceFrame = sourceFrame(crop); 				// Crop the frame
#endif
	Mat frameGrey; 									// Initialize frameGrey (to hold the thresholded image)
	if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS)
	{
		Mat frameGrey(sourceFrame.rows, sourceFrame.cols, CV_8UC1, cvScalar(0.0)); 	// Only when using opencv contours the frame is filled with zeros
	}
	trackGreyObjects(sourceFrame, frameGrey, &newTrackRes); 	// Track objects in sourceFrame
#if AR_FILTER_MOD_VIDEO
	if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS)
	{
		sourceFrame.setTo(Scalar(0.0, 127.0)); 	// Make sourceFrame black
		frameGrey.copyTo(sourceFrame); 			// Copy threshold result to black frame
	}
#endif
	for(unsigned int r=0; r < newTrackRes.size(); r++)			// Convert angles & Write/Print output
	{
		cam2body(&newTrackRes[r]);						// Convert from camera angles to body angles (correct for roll)
		body2world(&newTrackRes[r], eulerAngles); 		// Convert from body angles to world coordinates (correct yaw and pitch)
	}
#if AR_FILTER_MOD_VIDEO
	char text[15];
#if AR_FILTER_MEASURE_FPS
	sprintf(text,"FPS: %0.2f", runCount / ((double) curT));
#else
	sprintf(text,"frame %i", runCount);
#endif
	putText(sourceFrame, text, Point(10,sourceFrame.rows-100), FONT_HERSHEY_SIMPLEX, 2, Scalar(0,255,255), 5);
	for(unsigned int r=0; r < newTrackRes.size(); r++)			// Convert angles & Write/Print output
	{
		circle(sourceFrame,cvPoint(newTrackRes[r].x_p - cropCol, newTrackRes[r].y_p), sqrt(newTrackRes[r].area_p / M_PI), cvScalar(0,255,255), 5);
	}
	line(sourceFrame, Point(0,0), Point(0, sourceFrame.rows-1), Scalar(0,255,255), 5);
	line(sourceFrame, Point(sourceFrame.cols - 1,0), Point(sourceFrame.cols - 1, sourceFrame.rows-1), Scalar(0,255,255), 5);
	if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS)
	{
		for(unsigned int r=0; r < cropAreas.size(); r++)
		{
			if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
			{
				rectangle(sourceFrame, cropAreas[r], Scalar(0,255,255), 5);
			}
		}
	}
#endif
	trackRes = newTrackRes;
#if AR_FILTER_SHOW_MEM
		for(unsigned int r=0; r < trackRes.size(); r++) 			// Print to file & terminal
		{
			printf("%i - Object at (%0.2f m, %0.2f m, %0.2f m)\n", runCount, trackRes[r].x_w, trackRes[r].y_w, trackRes[r].z_w); 														// Print to terminal
		}
#endif
	sourceFrame.release();			// Release Mat
	frameGrey.release(); 			// Release Mat
	runCount++;
#if AR_FILTER_CALIBRATE_CAM
	if(runCount==10) // First and second frame are often not yet detected properly so let's calibrate the tenth frame
	{
		calibrateEstimation(trackRes);
	}
#endif
	return;
}

Rect setISPvars(int width, int height, bool crop)
{
	int heightRange = 3288; // Property of CMOS sensor
	ispScalar 		= (double) 16 / round(16 / ((double) width / heightRange)); // (re)Calculate the scalar set originally in bebop_front_camera.c
#if AR_FILTER_ISP_CROP
	ispHeight 		= MT9F002_OUTPUT_WIDTH;
	ispWidth 		= MT9F002_OUTPUT_HEIGHT;
#else
	ispHeight 		= width;
	ispWidth 		= height;
#endif
	if (crop==true)
	{
		double fovY 			= AR_FILTER_IMAGE_CROP_FOVY * M_PI / 180;
		double cmosPixelSize 	= 0.0000014; 	// 1.4um (see manual of mt9f002 CMOS sensor)
		double focalLength		= (2400 * cmosPixelSize * 1000 / ispScalar) / (4 * sin(M_PI / 4));
		double cY 				= round(sin((-eulerAngles->theta - AR_FILTER_BEBOP_CAMERA_ANGLE * M_PI / 180)) * 2 * focalLength * ispScalar / (1000 * cmosPixelSize));
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
#if AR_FILTER_ISP_CROP
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
	rndRedGrayscale(sourceFrame, frameGrey, AR_FILTER_RND_PIX_SAMPLE);
#if AR_FILTER_BENCHMARK
	addBenchmark("image Thresholded");
#endif
	if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
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
#if AR_FILTER_MOD_VIDEO
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
#if AR_FILTER_BENCHMARK
	addBenchmark("Contours found");
#endif
	return;
}

void addContour(vector<Point> contour, vector<trackResults>* trackRes)
{
	double contArea = contourArea(contour);
	if (contArea > (AR_FILTER_MIN_CROP_AREA * ispScalar * ispScalar))
	{
		Point2f objCentre;
		float 	objRadius;
		minEnclosingCircle(contour,objCentre,objRadius);
		float objArea = M_PI*objRadius*objRadius;
		if(objArea > (AR_FILTER_MIN_CIRCLE_SIZE * ispScalar * ispScalar) && contArea > objArea*AR_FILTER_MAX_CIRCLE_DEF)
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
		}else if(AR_FILTER_SHOW_REJECT) 	printf("Rejected. object %f, area %f, fill %f < min fill %f.\n",objArea, contArea, contArea / objArea, AR_FILTER_MAX_CIRCLE_DEF);
	}else if(AR_FILTER_SHOW_REJECT) 	printf("Rejected. Area %0.1f\n",contArea);
	return;
}

bool rndRedGrayscale(Mat& sourceFrame, Mat& destFrame, int sampleSize)
{
	bool obj_detected = false;
	if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
	{
		allContours.clear();
	}else{
		cropAreas.clear();
	}
	if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
	{
		if(AR_FILTER_SAMPLE_STYLE > 0)
		{
			for(unsigned int r=0; r < trackRes.size(); r++)
			{
				if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
				{
					objCont.clear();
				}else{
					objCrop.x 		= trackRes[r].x_p - cropCol;
					objCrop.y 		= trackRes[r].y_p;
					objCrop.width 	= 0;
					objCrop.height 	= 0;
				}
				if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p, trackRes[r].x_p - cropCol, 0, true)){
					if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p + 100, trackRes[r].x_p - cropCol, 0, true)){
						if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p + 100, trackRes[r].x_p - cropCol + 100, 0, true)){
							if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p, trackRes[r].x_p - cropCol + 100, 0, true)){
								if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p - 100, trackRes[r].x_p - cropCol + 100, 0, true)){
									if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p - 100, trackRes[r].x_p - cropCol, 0, true)){
										if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p - 100, trackRes[r].x_p - cropCol - 100, 0, true)){
											if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p, trackRes[r].x_p - cropCol - 100, 0, true)){
												if(!pixFindContour(sourceFrame, destFrame, trackRes[r].y_p + 100, trackRes[r].x_p - cropCol - 100, 0, true)){
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
				if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
				{
					allContours.push_back(objCont);
				}else{
					objCrop = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
					addObject();
				}
			}
		}
		switch(AR_FILTER_SAMPLE_STYLE)
		{
		case AR_FILTER_STYLE_FULL : // FULL
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
		case AR_FILTER_STYLE_GRID :
		{
			int spacing = (int) sqrt((sourceFrame.rows * sourceFrame.cols) / sampleSize);
			for(int r = spacing; r < sourceFrame.rows; r+=spacing)
			{
				for(int c=spacing; c < sourceFrame.cols; c+=spacing)
				{
					layerDepth = 0;
					sample++;
					if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
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
						if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
						{
							allContours.push_back(objCont);
						}else{
							objCrop = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
							addObject();
						}
						obj_detected = true;
					}
				}
			}
			break;
		}
		case AR_FILTER_STYLE_RANDOM :
		{
			int rndRow, rndCol;
			for(int i = 0; i<sampleSize; i++)
			{
				layerDepth = 0;
				sample++;
				//pixFindContour(sourceFrame, destFrame, rowDis(gen), colDis(gen));
				rndRow = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
				rndCol = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));
				if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
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
					if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW && !AR_FILTER_CV_CONTOURS)
					{
						allContours.push_back(objCont);
					}else{
						objCrop = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
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
	if(layerDepth > AR_FILTER_MAX_LAYERS || row < 0 || col < 0 || row > sourceFrame.rows-1 || col > sourceFrame.cols-1 || sourceFrame.at<Vec2b>(row,col)[1] == 1)
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
	if(Y >= AR_FILTER_Y_MIN && Y <= AR_FILTER_Y_MAX && Cb >= AR_FILTER_CB_MIN && Cb <= AR_FILTER_CB_MAX && Cr >= AR_FILTER_CR_MIN && Cr <= AR_FILTER_CR_MAX)
	{
		if(cascade)
		{
			if(prevDir != 0 && (AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS))
			{
				pixSucCount++;
				destFrame.at<uint8_t>(row,col) = 255;
			}
			sourceFrame.at<Vec2b>(row,col  )[1] = 1;
			switch(AR_FILTER_FLOOD_STYLE)
			{
			case AR_FILTER_FLOOD_OMNI :
			{
				if(prevDir==3 || row<=0 || !pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true))
				{
					if(layerDepth > AR_FILTER_MAX_LAYERS) return false;
					if(prevDir==4 || col>=(sourceFrame.cols - 1) || !pixFindContour(sourceFrame, destFrame, row, col+1, 2, true))
					{
						if(layerDepth > AR_FILTER_MAX_LAYERS) return false;
						if(prevDir==1 || row>=(sourceFrame.rows - 1) || !pixFindContour(sourceFrame, destFrame, row+1, col, 3, true))
						{
							if(layerDepth > AR_FILTER_MAX_LAYERS) return false;
							if(prevDir==2 || col<=0 || !pixFindContour(sourceFrame, destFrame, row, col-1, 4, true))
							{
								return false;
							}
						}
					}
				}
				break;
			}
			case AR_FILTER_FLOOD_CW :
			{
				switch(prevDir)
				{
				case 0 : // No previous direction
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 0, true)) break;																// UP
					if(!AR_FILTER_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; 										// RIGHT
					if(row < (sourceFrame.rows -1) && col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row + 1, col + 1, 2, true)) break; 		// DOWN-RIGHT
					break;
				case 1 : // Came from pixel below
					if(col > 0) pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true); 								// LEFT
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true)) break;						// UP
					if(!AR_FILTER_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; // RIGHT
					break;
				case 2 : // Came from pixel left
					if(row > 0) pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true);								// UP
					if(col < (sourceFrame.cols -1) && pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true)) break; // RIGHT
					if(!AR_FILTER_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(row < (sourceFrame.rows -1) && pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true)) break;	// DOWN
					break;
				case 3 : // Came from pixel above
					if(col < (sourceFrame.cols -1)) pixFindContour(sourceFrame, destFrame, row, col + 1, 2, true); 			// RIGHT
					if(row < (sourceFrame.rows -1) && pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true)) break;	// DOWN
					if(!AR_FILTER_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(col > 0 && pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true)) break; 					// LEFT
					break;
				case 4 : // Came from pixel right
					if(row < (sourceFrame.rows -1)) pixFindContour(sourceFrame, destFrame, row + 1, col, 3, true);			// DOWN
					if(col > 0 && pixFindContour(sourceFrame, destFrame, row, col - 1, 4, true)) break; 					// LEFT
					if(!AR_FILTER_CV_CONTOURS) objCont.push_back(Point(col,row));
					if(row > 0 && pixFindContour(sourceFrame, destFrame, row - 1, col, 1, true)) break;						// UP
					break;
				}
				break;
			}
			default : 	break;
			}
			if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW || AR_FILTER_CV_CONTOURS)
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
	trackRes->x_b = trackRes->r_c * cos(-trackRes->y_c - M_PI / 180 * AR_FILTER_BEBOP_CAMERA_ANGLE) * cos(trackRes->x_c) + AR_FILTER_BEBOP_CAMERA_OFFSET_X;
	trackRes->y_b = trackRes->r_c * cos(-trackRes->y_c - M_PI / 180 * AR_FILTER_BEBOP_CAMERA_ANGLE) * sin(trackRes->x_c);
	trackRes->z_b = trackRes->r_c * sin(-trackRes->y_c - M_PI / 180 * AR_FILTER_BEBOP_CAMERA_ANGLE);
	return;
}

void body2world(trackResults* trackRes, struct 	FloatEulers * eulerAngles)
{
	struct NedCoor_f pos;
#if AR_FILTER_WORLDPOS
	pos 	= stateGetPositionNed_f(); 		// Get your current position
#else
	pos.x = 0.0;
	pos.y = 0.0;
	pos.z = 0.0;
#endif
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
	trackRes->x_w = worldPos(0,0) + pos.x;
	trackRes->y_w = worldPos(1,0) + pos.y;
	trackRes->z_w = worldPos(2,0) + pos.z;
	return;
}


#if AR_FILTER_CALIBRATE_CAM
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

#if AR_FILTER_SAVE_FRAME
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
	if(objCrop.width * objCrop.height >= AR_FILTER_MIN_CROP_AREA)
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

/* Deprecated
#if AR_FILTER_MOD_VIDEO
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
*/

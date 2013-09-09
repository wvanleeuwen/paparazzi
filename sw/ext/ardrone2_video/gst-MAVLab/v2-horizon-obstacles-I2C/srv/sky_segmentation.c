#include "srv.h"
#include "sky_segmentation.h" 
#include "print.h"
#include "gps.h"

#define GROUND 1
#define SKY -1
#define abs(x) x < 0 ? -x : x

// *****************
// INLINE FUNCTIONS:
// *****************

inline void groundPixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0x00;
	frame_buf[ip+1] = 0x00;
	frame_buf[ip+2] = 0x00;
	frame_buf[ip+3] = 0x00;
}

inline void redPixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0x00;
	frame_buf[ip+1] = 0x00;
	frame_buf[ip+2] = 0xff;
	frame_buf[ip+3] = 0x00;
}

inline void blackDot(unsigned char *frame_buf, int x, int y)
{
	unsigned int ip;
	int xx, yy;
	for(xx = x-1; xx <= x+1; xx++)
	{
		for(yy = y-1; yy <= y+1; yy++)
		{
			if(xx >= 0 && xx < imgWidth && yy >= 0 && yy < imgHeight)
			{
				ip = image_index(xx,yy);
				frame_buf[ip] = 0x7f;
				frame_buf[ip+1] = 0x00;
				frame_buf[ip+2] = 0x7f;
				frame_buf[ip+3] = 0x00;	
			}
		}
	}

	
} 

inline void setUncertainty(unsigned char *frame_buf, unsigned int ip, unsigned int uncertainty) 
{
	// if(uncertainty > 255) uncertainty = 255;
	frame_buf[ip] = 127;
	frame_buf[ip+1] = uncertainty;
	frame_buf[ip+2] = 127;
	frame_buf[ip+3] = uncertainty;
}

inline int isGroundPixel(unsigned char *frame_buf, unsigned int ip)
{
	if(frame_buf[ip] == 0x00 && frame_buf[ip+1] == 0x00 && frame_buf[ip+2] == 0x00 && 	frame_buf[ip+3] == 0x00) return 1;
	else return 0;
}

inline void linePixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0;
	frame_buf[ip+1] = 255;
	frame_buf[ip+2] = 255;
	frame_buf[ip+3] = 255;
}



// *****************
// HORIZON DETECTION
// *****************

extern int perceptronPitchRoll(unsigned char *frame_buf, int *pitch_pixel, int *roll_angle, int error_range)
{
	/*
	* perceptronPitchRoll estimates the formula for the line separating ground and sky pixels
	* In this first implementation, we use a segmented image. This is not necessary for this
	* approach; only the used pixels are necessary.
	* It returns an error value in [0, error_range] to indicate reliability.
	*/

	int n_train_samples, n_test_samples, sample;
	int weights[3], average_weights[3], gamma[3];
	int costs[2];
	int x,y,output,target, a, b;
	unsigned int ix;
	int err, update_rate, n_updates, i, div_factor, div_res;
	int verbose, verbose_average;
	int X_BORDER, Y_BORDER;
	int resolution = 10;
	
	X_BORDER = 10;
	Y_BORDER = 10;

	// prior equation: y = height / 2
	weights[0] = 0; // x
	weights[1] = 100*(-1); // y
	weights[2] = 100*(imgHeight / 2); // half image height

	// number of samples:
	n_train_samples = 1000;
	n_test_samples = 50;

	// weight averaging leads to more robust results
	average_weights[0] = 0;
	average_weights[1] = 0;
	average_weights[2] = 0;	
	update_rate = 10;
	n_updates = 0;
	div_res = 10;
	div_factor = (n_train_samples / update_rate) / (div_res);

	// relative learning rate
	gamma[0] = 1;
	gamma[1] = 1;
	gamma[2] = 10000;

	// weighted fitting:
	costs[0] = 1; // ground
	costs[1] = 1; // sky

	verbose = 0;
	verbose_average = 0;
	
	for(sample = 0; sample < n_train_samples; sample++)
	{
		if(sample > 10)
		{
			verbose = 0;
		}

		// get a random sample:
		//x = rand(imgWidth-1);
		//y = rand(imgHeight-1);
		x = X_BORDER + rand() % (imgWidth-1-2 * X_BORDER);
		y = Y_BORDER + rand() % (imgHeight-1-2 * Y_BORDER);
		ix = image_index(x,y);
		target = isGroundPixel(frame_buf, ix);
		target = (target > 0) ? 1 : -1;
		
		// compare the perceptron's output with the target
		output = weights[0] * x + weights[1] * y + weights[2];
		output = (output>0) ? 1 : -1; // sign function

		if(target != output)
		{
			if(verbose)
			{
				printf("Pre: w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
				printf("x,y,t,out = %d,%d,%d,%d\n\r", x, y, target, output);
			}

			if(target == 1) // ground
			{
				weights[0] += costs[0] * gamma[0] * (target - output) * x;
				weights[1] += costs[0] * gamma[1] * (target - output) * y;
				weights[2] += costs[0] * gamma[2] * (target - output);
				if(verbose)
				{
					printf("dw1,dw2,dw3 = %d,%d,%d\n\r", 
						costs[0] * gamma[0] * (target - output) * x,
						costs[0] * gamma[1] * (target - output) * y,
						costs[0] * gamma[2] * (target - output));
				}
			}
			else
			{
				weights[0] += costs[1] * gamma[0] * (target - output) * x;
				weights[1] += costs[1] * gamma[1] * (target - output) * y;
				weights[2] += costs[1] * gamma[2] * (target - output);				
				if(verbose)
				{
					printf("dw1,dw2,dw3 = %d,%d,%d\n\r", 
						costs[1] * gamma[0] * (target - output) * x,
						costs[1] * gamma[1] * (target - output) * y,
						costs[1] * gamma[2] * (target - output));
				}
			}
			if(verbose)
				printf("Post: w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
		}

		if(sample % update_rate == 0)
		{
			for(i = 0; i < 3; i++)
			{
				if(verbose_average) printf("aw[%d] = %d (bef), ", i, average_weights[i]);
				average_weights[i] += weights[i] / div_res;
				if(verbose_average) printf("aw[%d] = %d (aft), ", i, average_weights[i]);
			}
			n_updates++;
		}
	}

	if(verbose_average) printf("\n\r");
	
	for(i = 0; i < 3; i++)
	{
		if(verbose_average) printf("weights[%d] = %d, ", i, weights[i]);
		weights[i] = average_weights[i] / div_factor;
		if(verbose_average) printf("weights[%d] = %d, ", i, weights[i]);
	}

	if(verbose_average) printf("\n\r");	
	
	// determine the number of errors with this decision boundary
	err = 0;
	for(sample = 0; sample < n_test_samples; sample++)
	{
		// get a random sample:
		x = X_BORDER + rand() % (imgWidth-1-2 * X_BORDER);
		y = Y_BORDER + rand() % (imgHeight-1-2 * Y_BORDER);
		ix = image_index(x,y);
		target = isGroundPixel(frame_buf, ix);
		target = (target > 0) ? 1 : -1;
		
		// compare the perceptron's output with the target
		output = weights[0] * x + weights[1] * y + weights[2];
		output = (output>0) ? 1 : -1; // sign function
		if(target != output)
		{
			err++;
		}	
	}
	err = (err * error_range) / n_test_samples;

	// Determine pitch and roll
	//printf("w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
	// b is the value of y at x = 0, which is left in the screen.
	// we need the value of y in the middle:
	if (weights[1] == 0)
	{
		weights[1] = 1;
	}
	b = - weights[2] / weights[1];
	a = -resolution*weights[0] / weights[1];

	b = b + (((int)imgWidth / 2) * a) / resolution;
	(*pitch_pixel) = b - imgHeight / 2; // b linearly in [0, error_range]

	(*roll_angle) = atan2_fp_1deg(weights[1], -weights[0]) - 90;
	//printf("pitch = %d, roll = %d\n\r", (*pitch), (*roll));	
	// bring roll to [-LIMIT_ANGLE, LIMIT_ANGLE]
	
	drawRectangle(frame_buf, X_BORDER, imgWidth-1- X_BORDER, imgHeight-1- Y_BORDER, Y_BORDER);
	drawLine(frame_buf, -resolution*weights[0] / weights[1], -resolution*weights[2] / weights[1], resolution);
	return err;
}

extern int perceptronPitchRollEfficient(unsigned char *frame_buf, int *pitch_pixel, int *roll_angle, int n_train_samples, int n_test_samples, int error_range)
{
	/*
	* perceptronPitchRollEfficient estimates the formula for the line separating ground and sky pixels
	* only the used pixels are classified as sky or ground.
	* It returns an error value in [0, error_range] to indicate reliability.
	*/

	int sample, dw;
	int weights[3], average_weights[3], gamma[3];
	int costs[2];
	int x,y,output,target, a, b;
	unsigned int ix, maxY;
	int err, uncertainty;
	int verbose, update_rate, n_updates, i, div_factor;
	int X_BORDER, Y_BORDER;
	int resolution = 10;
	
	X_BORDER = 10;
	Y_BORDER = 10;

	// prior equation: y = height / 2
	weights[0] = 0; // x
	weights[1] = 100*(-1); // y
	weights[2] = 100*(imgHeight / 2); // half image height

	// relative learning rate
	gamma[0] = 1;
	gamma[1] = 1;
	gamma[2] = 10000;

	// weighted fitting:
	costs[0] = 1; // ground
	costs[1] = 1; // sky

	// number of samples:
	// n_train_samples = 1000;
	// n_test_samples = 50;

	// weight averaging leads to more robust results
	average_weights[0] = 0;
	average_weights[1] = 0;
	average_weights[2] = 0;	
	update_rate = 10;
	n_updates = 0;
	div_factor = (n_train_samples / update_rate);

	verbose = 0;
	
	// if using a tree that employs the maximum illumination:
	maxY = getMaximumY(frame_buf);

	for(sample = 0; sample < n_train_samples; sample++)
	{
		if(sample > 10)
		{
			verbose = 0;
		}

		// get a random sample:
		//x = rand(imgWidth-1);
		//y = rand(imgHeight-1);
		x = X_BORDER + rand() % (imgWidth-1-2 * X_BORDER);
		y = Y_BORDER + rand() % (imgHeight-1-2 * Y_BORDER);
		ix = image_index(x,y);

		// classify only pixels that are evaluated in the horizon process:
		target = segment_no_yco_PerPixel(frame_buf, x, y, maxY, &uncertainty);
		
		// compare the perceptron's output with the target
		output = weights[0] * x + weights[1] * y + weights[2];
		output = (output>0) ? 1 : -1; // sign function

		if(target != output)
		{
			if(verbose)
			{
				printf("Pre: w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
				printf("x,y,t,out = %d,%d,%d,%d\n\r", x, y, target, output);
			}

			if(target == 1) // sky
			{
				weights[0] += costs[0] * gamma[0] * (target - output) * x;
				weights[1] += costs[0] * gamma[1] * (target - output) * y;
				weights[2] += costs[0] * gamma[2] * (target - output);
				if(verbose)
				{
					printf("dw1,dw2,dw3 = %d,%d,%d\n\r", 
						costs[0] * gamma[0] * (target - output) * x,
						costs[0] * gamma[1] * (target - output) * y,
						costs[0] * gamma[2] * (target - output));
				}
			}
			else
			{
				weights[0] += costs[1] * gamma[0] * (target - output) * x;
				weights[1] += costs[1] * gamma[1] * (target - output) * y;
				weights[2] += costs[1] * gamma[2] * (target - output);				
				if(verbose)
				{
					printf("dw1,dw2,dw3 = %d,%d,%d\n\r", 
						costs[1] * gamma[0] * (target - output) * x,
						costs[1] * gamma[1] * (target - output) * y,
						costs[1] * gamma[2] * (target - output));
				}
			}
			if(verbose)
				printf("Post: w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
		}
		
		if(sample % update_rate == 0)
		{
			for(i = 0; i < 3; i++)
			{
				average_weights[i] += weights[i] / div_factor;
			}
			n_updates++;
		}
	}
	
	for(i = 0; i < 3; i++)
	{
		weights[i] = average_weights[i];
	}
	
	// determine the number of errors with this decision boundary
	err = 0;
	for(sample = 0; sample < n_test_samples; sample++)
	{
		// get a random sample:
		x = X_BORDER + rand() % (imgWidth-1-2 * X_BORDER);
		y = Y_BORDER + rand() % (imgHeight-1-2 * Y_BORDER);
		ix = image_index(x,y);

		target = segment_no_yco_PerPixel(frame_buf, x, y, maxY, &uncertainty);
		
		// compare the perceptron's output with the target
		output = weights[0] * x + weights[1] * y + weights[2];
		output = (output>0) ? 1 : -1; // sign function
		if(target != output)
		{
			err++;
		}	
	}
	err = (err * error_range) / n_test_samples;

	// Determine pitch and roll
	//printf("w1,w2,w3 = %d,%d,%d\n\r", weights[0], weights[1], weights[2]);
	// b is the value of y at x = 0, which is left in the screen.
	// we need the value of y in the middle:
	if (weights[1] == 0)
	{
		weights[1] = 1;
	}
	b = - weights[2] / weights[1];
	a = -resolution*weights[0] / weights[1];
	b = b + (((int)imgWidth / 2) * a) / resolution;
	(*pitch_pixel) = b - imgHeight / 2;
	(*roll_angle) = atan2_fp_1deg(weights[1], -weights[0]) - 90;
	
	return err;
}

extern void horizonToLineParameters(int pitch_pixel, int roll_angle, int* a, int* b)
{
	(*b) = 1000 * (pitch_pixel + imgHeight / 2);
	(*a) = -tan(roll_angle);
}

extern void drawLine(unsigned char *frame_buf, int a, int b, int resolution)
{
	int x, y, b_res;
	unsigned int ix;

	if(resolution == 0) resolution = 1;
	b_res = b / resolution;

	for(x = 0; x < imgWidth; x++)
	{
		y = (a * x) / resolution + b_res;
		if(y >= 0 && y < imgHeight)
		{
			ix = image_index(x,y);
			linePixel(frame_buf, ix);
		}
	}
}

extern void drawRectangle(unsigned char *frame_buf, int x_left, int x_right, int y_top, int y_bottom)
{
	int x, y, temp;
	unsigned int ix;

	if(x_right < x_left) 
	{
		temp = x_left;
		x_left = x_right;
		x_right = temp;
	}
	if(y_top < y_bottom)
	{
		temp = y_top;
		y_top = y_bottom;
		y_bottom = temp;
	}

	for(x = x_left; x < x_right; x++)
	{
		ix = image_index(x, y_top);
		linePixel(frame_buf, ix);
		ix = image_index(x, y_bottom);
		linePixel(frame_buf, ix);
	}
	for(y = y_bottom; y < y_top; y++)
	{
		ix = image_index(x_left, y);
		linePixel(frame_buf, ix);
		ix = image_index(x_right, y);
		linePixel(frame_buf, ix);
	}
}


// ******************
// FEATURE EXTRACTION
// ******************

extern int getPatchTexture(unsigned char *frame_buf, int x, int y, int patch_size)
{
	unsigned int value, ix;
	int half_patch_size, dx, dy, center_pixel, texture, indx, indy;
	half_patch_size = patch_size / 2;
	texture = 0;
	// correct coordinates of center pixel if necessary:
	x = (x < half_patch_size) ? half_patch_size : x;
	x = (x >= imgWidth - half_patch_size) ? imgWidth - half_patch_size - 1 : x;	
	y = (y < half_patch_size) ? half_patch_size : y;
	y = (y >= imgWidth - half_patch_size) ? imgWidth - half_patch_size - 1 : y;
	
	ix = image_index(x,y);
	center_pixel = (int)((((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1);
	for(dx = -half_patch_size; dx <= half_patch_size; dx++)
	{
		for(dy = -half_patch_size; dy <= half_patch_size; dy++)
		{
			if(!(dx == 0 && dy == 0))
			{
				indx = x + dx;
				indy = y + dy;
				ix = image_index(indx, indy);
				value = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
				texture += abs((int) value - (int) center_pixel);
			}
		}
	}
	texture /= (patch_size * patch_size - 1);
	return texture;
}

extern int get_FD_YCV(unsigned char *frame_buf, int x, int y)
{
	unsigned int Y, Cb, Cr, ix;
	int FD_YCV;

	ix = image_index(x, y);	
	Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
	Cb = (unsigned int)frame_buf[ix];
	Cr = (unsigned int)frame_buf[ix+2];
	
	// In the blackfin, values range from [0, 255] while the formula is based on 
	// values in the range [0,1]. Therefore we divide by 255 after the color channels.
	// This leaves the factor 100 with which all coefficients were multiplied.
	FD_YCV = (860 * (int)Y - 501 * (int) Cr + 2550 * (int) Cb) / 255 - 1545;  
	return FD_YCV;
}

extern int get_FD_CV(unsigned char *frame_buf, int x, int y)
{
	unsigned int Cb, Cr, ix;
	int FD_CV;

	ix = image_index(x, y);	
	Cb = (unsigned int)frame_buf[ix];
	Cr = (unsigned int)frame_buf[ix+2];
	
	// In the blackfin, values range from [0, 255] while the formula is based on 
	// values in the range [0,1]. Therefore we divide by 255 after the channels.
	// This leaves the factor 100 with which all coefficients were multiplied.
	FD_CV = (1975 * (int) Cb - 446 * (int) Cr) / 255 - 818;  
	return FD_CV;
}



extern int getPatchMean(unsigned char *frame_buf, int x, int y, int patch_size)
{
	unsigned int value, ix;
	int half_patch_size, dx, dy, mean, indx, indy;
	half_patch_size = patch_size / 2;
	mean = 0;
	// correct coordinates of center pixel if necessary:
	x = (x < half_patch_size) ? half_patch_size : x;
	x = (x >= imgWidth - half_patch_size) ? imgWidth - half_patch_size - 1 : x;	
	y = (y < half_patch_size) ? half_patch_size : y;
	y = (y >= imgWidth - half_patch_size) ? imgWidth - half_patch_size - 1 : y;
	
	for(dx = -half_patch_size; dx <= half_patch_size; dx++)
	{
		for(dy = -half_patch_size; dy <= half_patch_size; dy++)
		{
			indx = x + dx;
			indy = y + dy;
			ix = image_index(indx, indy);
			value = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
			mean += (int) value;
		}
	}
	mean /= (patch_size * patch_size);
	return mean;
}

extern int getHarrisPixel(unsigned char *frame_buf, int x, int y)
{
	int dx, dy, dx2, dxy, dy2, Harris, min_x, min_y, xx, yy, it;
	int smooth[9];
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 4;
	smooth[0] = 2;
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 1;// in MATLAB-language: [1,2,1;2,4,2;1,2,1]

	int smooth_factor = 1400; // this factor was chosen to keep Harris within bounds
	
	// determine the 3 x 3 patch around x,y:
	if(x <= 0) min_x = 0;
	else if(x >= imgWidth - 1) min_x = imgWidth - 2;
	else min_x = x - 1;
	if(y <= 0) min_y = 0;
	else if(y >= imgHeight - 1) min_y = imgHeight - 2;
	else min_y = y - 1;
	
	// use the patch to determine dx2, dxy, and dy2
	it = 0;
	dx2 = 0;
	dxy = 0;
	dy2 = 0;
	for(yy = min_y; yy < min_y + 3; yy++)
	{
		for(xx = min_x; xx < min_x + 3; xx++)
		{
			getGradientPixel(frame_buf, xx, yy, &dx, &dy);
			// approximation of smoothed second order derivatives:
			dx2 += smooth[it] * dx * dx;
			dxy += smooth[it] * dx * dy;
			dy2 += smooth[it] * dy * dy;
			// update iterator of the smoothing filter:
			it++;
		}
	}

	// correcting the approximations to keep Harris numbers within bounds:
	dx2 /= smooth_factor;
	dxy /= smooth_factor;
	dy2 /= smooth_factor;

	// Harris = (dx2 * dy2 - dxy*dxy) - k * (dx2 + dy2) * (dx2 + dy2);
	// where k = 0.04 in floating point
	Harris = (dx2 * dy2 - dxy*dxy) - ((dx2 + dy2) * (dx2 + dy2)) / 25;
	return Harris;
}

extern int getNoblePixel(unsigned char *frame_buf, int x, int y)
{
	int dx, dy, dx2, dxy, dy2, Noble, min_x, min_y, xx, yy, it;
	int smooth[9];
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 4;
	smooth[0] = 2;
	smooth[0] = 1;
	smooth[0] = 2;
	smooth[0] = 1;// in MATLAB-language: [1,2,1;2,4,2;1,2,1]
	int smooth_factor = 1400; // this factor was chosen to keep Harris within bounds
	
	// determine the 3 x 3 patch around x,y:
	if(x <= 0) min_x = 0;
	else if(x >= imgWidth - 1) min_x = imgWidth - 2;
	else min_x = x - 1;
	if(y <= 0) min_y = 0;
	else if(y >= imgHeight - 1) min_y = imgHeight - 2;
	else min_y = y - 1;
	
	// use the patch to determine dx2, dxy, and dy2
	it = 0;
	dx2 = 0;
	dxy = 0;
	dy2 = 0;
	for(yy = min_y; yy < min_y + 3; yy++)
	{
		for(xx = min_x; xx < min_x + 3; xx++)
		{
			getGradientPixel(frame_buf, xx, yy, &dx, &dy);
			// approximation of smoothed second order derivatives:
			dx2 += smooth[it] * dx * dx;
			dxy += smooth[it] * dx * dy;
			dy2 += smooth[it] * dy * dy;
			// update iterator of the smoothing filter:
			it++;
		}
	}

	// correcting the approximations to keep Harris numbers within bounds:
	dx2 /= smooth_factor;
	dxy /= smooth_factor;
	dy2 /= smooth_factor;

	// Noble = (dx2*dy2 - dxy*dxy) / (dx2 + dy2 + epsilon), where epsilon is a small number
	// Noble values are quite small, so we might have to multiply with a fixed number somewhere below
	if(dx2 + dy2 > 0)
	{
		Noble = (dx2 * dy2 - dxy * dxy) / (dx2 + dy2);
	}
	else
	{
		Noble = dx2 * dy2 - dxy * dxy;
		if(Noble > 0)
		{
			if(Noble <= 65) Noble *= 1000;
			else Noble = 65335;
		}
		else
		{
			Noble = 0;
		}
	}
	
	return Noble;
}




// This function gives the maximum of a subsampled version of the image
unsigned int getMaximumY(unsigned char *frame_buf) 
{
	unsigned int ix, y, max_y;
	unsigned int color_channels = 4;
	unsigned int step = 5 * color_channels;
	max_y = 0;
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 
		// we can speed things up by just looking at the first channel:
        y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		if(y > max_y) max_y = y;
    }
    return max_y;
}

extern unsigned int getMinimumY(unsigned char *frame_buf)
{
	unsigned int ix, y, min_y;
	unsigned int color_channels = 4;
	unsigned int step = 5 * color_channels;
	min_y = 255;
	for (ix=0; ix<(imgWidth*imgHeight*2); ix+= step)
	{ 
		// we can speed things up by just looking at the first channel:
        y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		if(y < min_y) min_y = y;
    }
    return min_y;

}


// For computational efficiency, we currently do not use any mask for determining the gradient.
extern void getGradientImage(unsigned char *frame_buf, unsigned char *frame_buf2, unsigned char *frame_buf3)
{
    unsigned int x,y,ix;
    int dx,dy;
    for(x = 0; x < imgWidth; x++)
    {
		for(y = 0; y < imgHeight; y++)
		{
			ix = image_index(x,y);
			getGradientPixel(frame_buf, x, y, &dx, &dy);
			
			// gradient has to be stored in unsigned format. We prefer to cut the values off at -127 / +127 than to reduce the resolution 
			dx = dx + 127;
			dx = (dx < 0) ? 0 : dx;
			dx = (dx > 255) ? 255 : dx;
			dy = dy + 127;
			dy = (dy < 0) ? 0 : dy;
			dy = (dy > 255) ? 255 : dy;
			frame_buf2[ix+1] = (unsigned int) dx;
			frame_buf2[ix+3] = (unsigned int) dx;
			frame_buf3[ix+1] = (unsigned int) dy;
			frame_buf3[ix+3] = (unsigned int) dy;
		}
    }
}

extern void getGradientPixel(unsigned char *frame_buf, int x, int y, int* dx, int* dy)
{
	unsigned int ix, Y1, Y2;
	unsigned int xx, yy;
	// currently we use [0 0 0; -1 0 1; 0 0 0] as mask for dx
	if(x >= 0 && x < imgWidth && y >= 0 && y < imgHeight)
	{
		if(x > 0)
		{
			xx = x - 1;
			yy = y;
			ix = image_index(xx,yy);
			Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		else
		{
			xx = 0; yy = y;
			ix = image_index(xx,yy);
			Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
	
		if(x < imgWidth - 1)
		{
			xx = x+1; yy = y;
			ix = image_index(xx,yy);
			Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		else
		{
			xx = imgWidth - 1; yy = y;
			ix = image_index(xx,yy);
			Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		
		(*dx) = ((int)Y2) - ((int)Y1);
		
		if(y > 0)
		{
			xx = x; yy = y - 1;
			ix = image_index(xx,yy);
			Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		else
		{
			xx = x; yy = 0;
			ix = image_index(xx,yy);
			Y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
	
		if(y < imgHeight - 1)
		{
			xx = x; yy = y + 1;
			ix = image_index(xx,yy);
			Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		else
		{
			xx = x; yy = imgHeight-1;
			ix = image_index(xx,yy);
			Y2 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
		}
		
		(*dy) = ((int)Y2) - ((int)Y1);
	}
}

extern int getGradient(unsigned char *frame_buf, int x, int y)
{
	int dx, dy, gra;

	if(x >= 0 && x < imgWidth && y >= 0 && y < imgHeight)
	{
		getGradientPixel(frame_buf, x, y, &dx, &dy);
		dx = abs(dx);
		dy = abs(dy);
		gra = dx + dy;
		return gra;
	}
	else
	{
		// coordinate not within image
		return 0;
	}
	
	
}




// ******************
// OBSTACLE DETECTION
// ******************

void getObstacles2Way(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL, int pitch_pixels, int roll_angle)
{
	// procedure:
	// 1) determine the horizon line on the basis of pitch and roll
	// 2) determine the central point projected on the horizon line
	// 3) determine the step_x in order to retain the same bin_size along the horizon line
	// 4) run over the image from left to right in lines parallel to the horizon line
	unsigned int ix;
	int a, b, halfWidth, halfHeight;
	int i, x, y, bin;
	int x1, y1, x2, y2, RESOLUTION;
	int a2, b2, x12, y12, bin_size, step_x;
	int xx, yy, x_start;
	int bin_surface, total_pixels;
	int start_bin, stop_bin;
	int CENTER_BINS = 1;
	halfWidth = imgWidth / 2;
	halfHeight = imgHeight / 2;
	bin_size = imgWidth / n_bins;

	// initialize bins:
    for(bin = 0; bin < n_bins; bin++)
    {
		obstacles[bin] = 0;
    }

	// 1) determine the horizon line on the basis of pitch and roll
	RESOLUTION = 1000;
	horizonToLineParameters(pitch_pixels, roll_angle, &a, &b);

	// 2) determine the central point projected on the horizon line:
	x1 = -halfWidth;
	y1 = (b + a * x1) / RESOLUTION;
	x2 = halfWidth;
	y2 = (b + a * x2) / RESOLUTION;
		

	if(a == 0) a = 1;
	a2 = (RESOLUTION / a) * RESOLUTION; // slope orthogonal to a
	b2 = halfHeight * RESOLUTION; // goes through the center
	x12 = (100*(b2 - b)) / (a + a2); // RESOLUTION factor disappears
	x12 /= 100;
	y12 = (a * x12 + b) / RESOLUTION;
	
	// only further process the image if the horizon line is entirely visible
	if(y1 >= 0 && y1 < imgHeight && y2 >= 0 && y2 < imgHeight)
	{
		// 3) determine the step_x in order to retain the same bin_size along the horizon line
		step_x = (int)isqrt(
			(unsigned int) (bin_size * bin_size) / (((a*a) / (RESOLUTION*RESOLUTION)) + 1)
		);

		// 4) run over the image from left to right in lines parallel to the horizon line
		x_start = x12 - (n_bins / 2) * step_x;
		
		for(i = 0; i > -halfHeight; i--)
		{
			for(x = x_start; x < halfWidth; x++)
			{
				// determine the appropriate bin:
				bin = (x - x_start) / step_x;
				if(bin >= n_bins) bin = n_bins - 1;
				y = (a * x + b) / RESOLUTION + i;
				// transform to image coordinates:
				xx = x + halfWidth;
				yy = y;
				
				if(xx >= 0 && xx < imgWidth && yy >= 0 && yy < imgHeight)
				{
					ix = image_index(xx,yy);
					if(isGroundPixel(frame_buf, ix))
					{
						// make pixel red
						redPixel(frame_buf, ix);
						// add pixel to obstacle bin:
						obstacles[bin]++;						
					}
				}
			}
		}

		// get the variables of interest and transform them to output form
		bin_surface = bin_size * halfHeight;
		total_pixels = imgWidth * halfHeight;
	    // obstacles[bin] should have a maximum corresponding to MAX_SIGNAL
		(*max_bin) = 0;
		(*obstacle_total) = 0;
		if(!CENTER_BINS)
		{
			start_bin = 0; stop_bin = n_bins;
		}
		else
		{
			start_bin = 2; stop_bin = n_bins - 2;
		}
	    for(bin = 0; bin < n_bins; bin++)
	    {
			obstacles[bin] *= MAX_SIGNAL;
			obstacles[bin] /= bin_surface;
			if(obstacles[bin] > (*max_bin))
			{
				(*max_bin) = obstacles[bin];
			}
			(*obstacle_total) += obstacles[bin];
	    }
		(*obstacle_total) /= n_bins;	

	}
	else
	{
		(*max_bin) = 0;
		(*obstacle_total) = 0;
	}
	// 1000 is the resolution of the tan-function
	drawLine((unsigned char *)FRAME_BUF, a, y1*RESOLUTION, RESOLUTION);
	blackDot((unsigned char *)FRAME_BUF, x12+halfWidth, y12);
}


void getObstacles(unsigned int* obstacles, unsigned int n_bins, unsigned char *frame_buf, unsigned int* max_bin, unsigned int* obstacle_total, int MAX_SIGNAL)
{
    unsigned int x,y,ix,GRND, bin_size, bin, HALF_HEIGHT, total_pixels, bin_surface;
	int start_bin, stop_bin;
	int CENTER_BINS = 1;
    // reset obstacles values
    for(bin = 0; bin < n_bins; bin++)
    {
		obstacles[bin] = 0;
    }
    bin_size = imgWidth / n_bins;
    HALF_HEIGHT = imgHeight / 2;
	bin_surface = bin_size * HALF_HEIGHT;
	total_pixels = imgWidth * HALF_HEIGHT;
    GRND = 0x00;
    // frame_buf contains the segmented image. All non-zero elements are sky.
    // This function assumes no pitch and roll. 
    for(x = 0; x < imgWidth; x++)
    {
		for(y = 0; y < HALF_HEIGHT; y++)
		{
		    ix = image_index(x,y);
		    if(frame_buf[ix] == GRND) // Of course, the original image could also use black pixels - of which probably few in the sky
		    {
				bin = x / bin_size;
				if(bin >= n_bins) 
				{	
					bin = n_bins-1;
				}
				obstacles[bin]++;
		    }
		}
    }
    // obstacles[bin] should have a maximum corresponding to MAX_SIGNAL
	(*max_bin) = 0;
	(*obstacle_total) = 0;
	if(!CENTER_BINS)
	{
		start_bin = 0; stop_bin = n_bins;
	}
	else
	{
		start_bin = 2; stop_bin = n_bins - 2;
	}
    for(bin = 0; bin < n_bins; bin++)
    {
		obstacles[bin] *= MAX_SIGNAL;
		obstacles[bin] /= bin_surface;
		if(obstacles[bin] > (*max_bin))
		{
			(*max_bin) = obstacles[bin];
		}
		(*obstacle_total) += obstacles[bin];
    }
	(*obstacle_total) /= n_bins;
}

void getUncertainty(unsigned int* uncertainty, unsigned int n_bins, unsigned char *frame_buf)
{
    unsigned int x,y, ix, bin_size, bin, HALF_HEIGHT, uncertainty_line, last_bin;
    // reset uncertainty values
    for(bin = 0; bin < n_bins; bin++)
    {
	uncertainty[bin] = 0;
    }
    bin_size = imgWidth / n_bins;
    HALF_HEIGHT = imgHeight / 2;
    // frame_buf contains the uncertainties in the range [0,50] with 50 maximally uncertain.
    // (if higher, the other class should be chosen).
    // This function assumes no pitch and roll. 
    last_bin = 0;

    for(x = 0; x < imgWidth; x++)
    {
	bin = x / bin_size;
	if(bin >= n_bins) bin = n_bins - 1;
	if(bin > last_bin) 
	{
	    // the final value in uncertainty[bin] is an average per pixel in the bin
	    uncertainty[bin-1] /= bin_size;
	    last_bin = bin;
	}
	// sum the uncertainty value over one vertical line:
	uncertainty_line = 0;
	for(y = 0; y < HALF_HEIGHT; y++)
	{
	    ix = image_index(x,y);
	    uncertainty_line += frame_buf[ix+1]; // Y-channel contains the uncertainty
	}
	uncertainty[bin] += uncertainty_line / HALF_HEIGHT; // per vertical line, the average uncertainty value per pixel is added to uncertainty[bin]
    }
    // the final value in uncertainty[bin] is an average per pixel in the bin
    uncertainty[n_bins-1] /= bin_size;
}


/***************************** 

	TREES for segmentation: 

******************************/

void segmentBWboard(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, threshold, value;
	unsigned int ix, Y, U, V, maxY, minY;
	
	// the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
	maxY = getMaximumY(frame_buf);
	minY = getMinimumY(frame_buf);

	threshold = (maxY + minY) / 2;

	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++) // we could divide imgHeight by 2 to speed things up
		{
			ix = image_index(x,y);
			Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
			if(Y >  threshold)
			{
				//	white
				if(Y > maxY) Y = maxY;
				setUncertainty(frame_buf2, ix, maxY - Y);
			}
			else
			{
				// black
				if(Y < minY) Y = minY;
				groundPixel(frame_buf, ix);
				setUncertainty(frame_buf2, ix, Y-minY);
			}
		}
	}
}

void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, threshold, value;
	unsigned int ix, Y, U, V, maxY;
	
	// the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
	maxY = getMaximumY(frame_buf);
	
	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++) // we could divide imgHeight by 2 to speed things up
		{
			ix = image_index(x,y);
			
			threshold = (imgHeight * 41) / 100;
			if(y <= threshold) // high in the image
			{
				value = getGradient(frame_buf, x, y);
				if(value <= 4) // little gradient
				{
					U = (unsigned int)frame_buf[ix];
					
					if(U <= 137)
					{
						Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
						if(Y <= (maxY * 30) / 100) // not very bright: was 59, now 30
						{
							V = (unsigned int)frame_buf[ix + 2];
							
							if(V <= 143) // check this one: are the colors right?
							{
								threshold = (imgHeight * 29) / 100;
								if(y <= threshold)
								{
									if(value <= 3) // was 1
									{
										// sky
										// 70.0%
										setUncertainty(frame_buf2, ix, 30);
									}
									else
									{
										// ground
										// 77.6%
										groundPixel(frame_buf, ix);
										setUncertainty(frame_buf2, ix, 22);
									}
								}
								else
								{
									// ground:
									// 87.76%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 12);
								}
							}
							else
							{
								// sky:
								// 76,53%
								setUncertainty(frame_buf2, ix, 23);
							}
						}
						else
						{
							// sky
							// 87.37% certainty
							setUncertainty(frame_buf2, ix, 12);
						}
					}
					else
					{
						// sky
						// 93,45% certainty
						setUncertainty(frame_buf2, ix, 6);
					}
				}
				else
				{
					// more gradient
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 30) / 100) // not very bright: was 59, now 30
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 141)
						{
							// ground:
							// 92.0%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 8);
						}
						else
						{
							if(U <= 152)
							{
								threshold = (imgHeight * 21) / 100;
								if(y <= threshold)
								{
									// sky:
									// 67.7%
									setUncertainty(frame_buf2, ix, 32);
								}
								else
								{
									// ground:
									// 80.0%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 20);
								}
							}
							else
							{
								// sky
								// 74.5%
								setUncertainty(frame_buf2, ix, 25);
							}
						}
					}
					else
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 135)
						{
							if(value <= 28) // medium gradient:
							{
								if(Y <= (maxY * 75) / 100)
								{
									// ground:
									// 71.5%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 28);
								}
								else
								{
									//sky:
									// 74.1% certainty
									setUncertainty(frame_buf2, ix, 26);
								}
							}
							else
							{
								// high gradient:
								// ground
								// 78.6%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 21);
							}
						}
						else
						{
							// sky
							// 79.5%
							setUncertainty(frame_buf2, ix, 20);
						}
					}
					
				}
			}
			else
			{
				threshold = (imgHeight * 57) / 100;
				if(y <= threshold)
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 58) / 100)
					{
						// ground
						// 94.5%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 5);
					}
					else
					{
						value = getGradient(frame_buf, x, y);
						if(value <= 16)
						{
							V = (unsigned int)frame_buf[ix + 2];
							if(V <= 118)
							{
								// sky
								// 85.1%
								setUncertainty(frame_buf2, ix, 15);
							}
							else
							{
								threshold = (imgHeight * 47) / 100;
								if(y <= threshold)
								{
									// sky:
									// 70.3%
									setUncertainty(frame_buf2, ix, 29);
								}
								else
								{
									// ground:
									// 71.9%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 28);
								}
							}
						}
						else
						{
							// ground
							// 84.2%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 16);
						}
					}
				}
				else
				{
					// ground: 98,74%
					groundPixel(frame_buf, ix);
					setUncertainty(frame_buf2, ix, 1);
				}
			}
		}
	}
}

extern void segment_no_yco(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, maxY, patch_size, patch_texture;
	int FD_YCV, FD_CV;
	unsigned int ix, Y, Cb, Cr;

	// global variables, so that they are calculated / initialized only once:
	maxY = getMaximumY(frame_buf);
	patch_size = 10;

	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++)
		{
			ix =image_index(x,y);

			FD_YCV = get_FD_YCV(frame_buf, x, y);
			if(FD_YCV <= 10)
			{
				Cr = (unsigned int)frame_buf[ix+2];
				if(Cr <= 153)
				{
					FD_YCV = get_FD_YCV(frame_buf, x, y);
					if(FD_YCV <= -125)
					{
						// ground: 98%
						setUncertainty(frame_buf2, ix, 2);
						groundPixel(frame_buf, ix);
					}
					else
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 4)
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 129)
							{
								// sky: 74%
								setUncertainty(frame_buf2, ix, 26);
							}
							else
							{
								// ground: 75%
								setUncertainty(frame_buf2, ix, 25);
								groundPixel(frame_buf, ix);
							}
						}
						else
						{
							// ground: 90%
							setUncertainty(frame_buf2, ix, 10);
							groundPixel(frame_buf, ix);
						}
					}
				}
				else
				{
					patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
					if(patch_texture <= 7)
					{
						// sky: 69%
						setUncertainty(frame_buf2, ix, 31);
					}
					else
					{
						// ground: 88%
						setUncertainty(frame_buf2, ix, 12);
						groundPixel(frame_buf, ix);
					}
				}
			}
			else
			{
				patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
				if(patch_texture <= 7)
				{
					FD_CV = get_FD_CV(frame_buf, x, y);
					if(FD_CV <= -83)
					{
						// ground: 67%
						setUncertainty(frame_buf2, ix, 33);
						groundPixel(frame_buf, ix);
					}
					else
					{
						// sky: 90%
						setUncertainty(frame_buf2, ix, 10);
					}
				}
				else
				{
					FD_YCV = get_FD_YCV(frame_buf, x, y);
					if(FD_YCV <= 164)
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 13)
						{
							FD_CV = get_FD_CV(frame_buf, x, y);
							if(FD_CV <= -52)
							{
								// ground: 79%
								setUncertainty(frame_buf2, ix, 21);
								groundPixel(frame_buf, ix);
							}
							else
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= (maxY * 60) / 100)
								{
									// ground: 76%
									setUncertainty(frame_buf2, ix, 24);
									groundPixel(frame_buf, ix);
								}
								else
								{
									// sky: 71%
									setUncertainty(frame_buf2, ix, 29);
								}
							}
						}
						else
						{
							// ground: 84%
							setUncertainty(frame_buf2, ix, 16);
							groundPixel(frame_buf, ix);
						}
					}
					else
					{
						// sky: 76%
						setUncertainty(frame_buf2, ix, 24);
					}
				}
			}
		}
	}
}


extern int segment_no_yco_PerPixel(unsigned char *frame_buf, int x, int y, unsigned int maxY, int* uncertainty)
{
	// use a pre-defined tree to segment a pixel in the image:
	int verbose, patch_size, patch_texture;
	int FD_YCV, FD_CV;
	unsigned int ix, Y, Cb, Cr;

	patch_size = 10;

	ix = image_index(x,y);

	FD_YCV = get_FD_YCV(frame_buf, x, y);
	if(FD_YCV <= 10)
	{
		Cr = (unsigned int)frame_buf[ix+2];
		if(Cr <= 153)
		{
			FD_YCV = get_FD_YCV(frame_buf, x, y);
			if(FD_YCV <= -125)
			{
				// ground: 98%
				(*uncertainty) = 2;
				return GROUND;
			}
			else
			{
				patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
				if(patch_texture <= 4)
				{
					Cr = (unsigned int)frame_buf[ix+2];
					if(Cr <= 129)
					{
						// sky: 74%
						(*uncertainty) = 26;
						return SKY;
					}
					else
					{
						// ground: 75%
						(*uncertainty) = 25;
						return GROUND;
					}
				}
				else
				{
					// ground: 90%
					(*uncertainty) = 10;
					return GROUND;
				}
			}
		}
		else
		{
			patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
			if(patch_texture <= 7)
			{
				// sky: 69%
				(*uncertainty) = 31;
				return SKY;
			}
			else
			{
				// ground: 88%
				(*uncertainty) = 12;
				return GROUND;
			}
		}
	}
	else
	{
		patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
		if(patch_texture <= 7)
		{
			FD_CV = get_FD_CV(frame_buf, x, y);
			if(FD_CV <= -83)
			{
				// ground: 67%
				(*uncertainty) = 33;
				return GROUND;
			}
			else
			{
				// sky: 90%
				(*uncertainty) = 10;
				return SKY;
			}
		}
		else
		{
			FD_YCV = get_FD_YCV(frame_buf, x, y);
			if(FD_YCV <= 164)
			{
				patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
				if(patch_texture <= 13)
				{
					FD_CV = get_FD_CV(frame_buf, x, y);
					if(FD_CV <= -52)
					{
						// ground: 79%
						(*uncertainty) = 21;
						return GROUND;
					}
					else
					{
						Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
						if(Y <= (maxY * 60) / 100)
						{
							// ground: 76%
							(*uncertainty) = 24;
							return GROUND;
						}
						else
						{
							// sky: 71%
							(*uncertainty) = 29;
							return SKY;
						}
					}
				}
				else
				{
					// ground: 84%
					(*uncertainty) = 16;
					return GROUND;
				}
			}
			else
			{
				// sky: 76%
				(*uncertainty) = 24;
				return SKY;
			}
		}
	}
}


extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, maxY, adjust_rel_Y, patch_size, patch_texture, adjust_patch_texture, adjust_Y, adjust_Cb, adjust_Cr, FD_YCV, adjust_FD_YCV, FD_CV, adjust_FD_CV;
	unsigned int ix, Y, Cb, Cr;

	// global variables, so that they are calculated / initialized only once:
	maxY = getMaximumY(frame_buf);
	patch_size = 10;

	// variables for adjusting thresholds:
	adjust_Y = adjust_factor * -11;
	adjust_Cb = adjust_factor * -4;
	adjust_Cr = adjust_factor * 3;
	adjust_rel_Y = adjust_factor * -5;
	adjust_patch_texture = adjust_factor * 2;
	adjust_FD_YCV = adjust_factor * -48;
	adjust_FD_CV = adjust_factor * -48;


	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++)
		{
			ix = image_index(x,y);

			FD_YCV = get_FD_YCV(frame_buf, x, y);
			if(FD_YCV <= 58 + adjust_FD_YCV)
			{
				Cr = (unsigned int)frame_buf[ix+2];
				if(Cr <= 150 + adjust_Cr)
				{
					FD_YCV = get_FD_YCV(frame_buf, x, y);
					if(FD_YCV <= -77 + adjust_FD_YCV)
					{
						// ground: 98%
						setUncertainty(frame_buf2, ix, 2);
						groundPixel(frame_buf, ix);
					}
					else
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 2 + adjust_patch_texture)
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 126 + adjust_Cr)
							{
								// sky: 74%
								setUncertainty(frame_buf2, ix, 26);
							}
							else
							{
								// ground: 75%
								setUncertainty(frame_buf2, ix, 25);
								groundPixel(frame_buf, ix);
							}
						}
						else
						{
							// ground: 90%
							setUncertainty(frame_buf2, ix, 10);
							groundPixel(frame_buf, ix);
						}
					}
				}
				else
				{
					patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
					if(patch_texture <= 4 + adjust_patch_texture)
					{
						// sky: 69%
						setUncertainty(frame_buf2, ix, 31);
					}
					else
					{
						// ground: 88%
						setUncertainty(frame_buf2, ix, 12);
						groundPixel(frame_buf, ix);
					}
				}
			}
			else
			{
				patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
				if(patch_texture <= 5 + adjust_patch_texture)
				{
					FD_CV = get_FD_CV(frame_buf, x, y);
					if(FD_CV <= -51 + adjust_FD_CV)
					{
						// ground: 67%
						setUncertainty(frame_buf2, ix, 33);
						groundPixel(frame_buf, ix);
					}
					else
					{
						// sky: 90%
						setUncertainty(frame_buf2, ix, 10);
					}
				}
				else
				{
					FD_YCV = get_FD_YCV(frame_buf, x, y);
					if(FD_YCV <= 212 + adjust_FD_YCV)
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 11 + adjust_patch_texture)
						{
							FD_CV = get_FD_CV(frame_buf, x, y);
							if(FD_CV <= -19 + adjust_FD_CV)
							{
								// ground: 79%
								setUncertainty(frame_buf2, ix, 21);
								groundPixel(frame_buf, ix);
							}
							else
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= (maxY * 66) / 100 + adjust_rel_Y)
								{
									// ground: 76%
									setUncertainty(frame_buf2, ix, 24);
									groundPixel(frame_buf, ix);
								}
								else
								{
									// sky: 71%
									setUncertainty(frame_buf2, ix, 29);
								}
							}
						}
						else
						{
							// ground: 84%
							setUncertainty(frame_buf2, ix, 16);
							groundPixel(frame_buf, ix);
						}
					}
					else
					{
						// sky: 76%
						setUncertainty(frame_buf2, ix, 24);
					}
				}
			}
		}
	}
}

/*

int segmentSkyPixelNoYCO(unsigned char *frame_buf, int x, int y, unsigned int maxY, int* uncertainty)
{
	// use a pre-defined tree to segment the image:
	int gradient;
	unsigned int ix, Y, Cb, Cr;
	
	
			ix = image_index(x,y);

			Cr = (unsigned int)frame_buf[ix+2];
			if(Cr <= 115)
			{
				gradient = getGradient(frame_buf, x, y);
				if(gradient <= 3)
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 82) / 100)
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 149)
						{
							Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
							if(Y <= (maxY * 53) / 100)
							{
								// ground: 86%
								(*uncertainty) = 14;
								return GROUND;
							}
							else
							{
								// sky: 73%
								(*uncertainty) = 27;
								return SKY;
							}
						}
						else
						{
							// sky: 89%
							(*uncertainty) = 11;
							return SKY;
						}
					}
					else
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 121)
						{
							// ground: 91%
							(*uncertainty) = 9;
							return GROUND;
						}
						else
						{
							// sky: 96%
							(*uncertainty) = 4;
							return SKY;
						}
					}
				}
				else
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 81) / 100)
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 153)
						{
							// ground: 92%
							(*uncertainty) = 8;
							return GROUND;
						}
						else
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 102)
							{
								// sky: 73%
								(*uncertainty) = 27;
								return SKY;
							}
							else
							{
								// ground: 74%
								(*uncertainty) = 26;
								return GROUND;
							}
						}
					}
					else
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 133)
						{
							// ground: 79%
							(*uncertainty) = 21;
							return GROUND;
						}
						else
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 12)
							{
								// sky: 81%
								(*uncertainty) = 19;
								return SKY;
							}
							else
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= 153)
								{
									// ground: 74%
									(*uncertainty) = 26;
									return GROUND;
								}
								else
								{
									// sky: 71%
									(*uncertainty) = 29;
									return SKY;
								}
							}
						}
					}
				}
			}
			else
			{
				Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
				if(Y <= 144)
				{
					Cb = (unsigned int)frame_buf[ix];
					if(Cb <= 133)
					{
						// ground: 98%
						(*uncertainty) = 2;
						return GROUND;
					}
					else
					{
						gradient = getGradient(frame_buf, x, y);
						if(gradient <= 2)
						{
							Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
							if(Y <= (maxY * 81) / 100)
							{
								Cb = (unsigned int)frame_buf[ix];
								if(Cb <= 143)
								{
									// ground: 87%
									(*uncertainty) = 13;
									return GROUND;
								}
								else
								{
									// sky: 69%
									(*uncertainty) = 31;
									return SKY;
								}
							}
							else
							{
								// sky: 77%
								(*uncertainty) = 23;
								return SKY;
							}
						}
						else
						{
							// ground: 91%
							(*uncertainty) = 9;
							return GROUND;
						}
					}
				}
				else
				{
					gradient = getGradient(frame_buf, x, y);
					if(gradient <= 3)
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 127)
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 125)
							{
								// sky: 84%
								(*uncertainty) = 16;
								return SKY;
							}
							else
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= 197)
								{
									// ground: 85%
									(*uncertainty) = 15;
									return GROUND;
								}
								else
								{
									gradient = getGradient(frame_buf, x, y);
									if(gradient <= 1)
									{
										// sky: 75%
										(*uncertainty) = 25;
										return SKY;
									}
									else
									{
										// ground: 70%
										(*uncertainty) = 30;
										return GROUND;
									}
								}
							}
						}
						else
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 1)
							{
								// sky: 86%
								(*uncertainty) = 14;
								return SKY;
							}
							else
							{
								Cr = (unsigned int)frame_buf[ix+2];
								if(Cr <= 123)
								{
									// sky: 83%
									(*uncertainty) = 17;
									return SKY;
								}
								else
								{
									Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
									if(Y <= 208)
									{
										Cb = (unsigned int)frame_buf[ix];
										if(Cb <= 130)
										{
											// ground: 71%
											(*uncertainty) = 29;
											return GROUND;
										}
										else
										{
											// sky: 76%
											(*uncertainty) = 24;
											return SKY;
										}
									}
									else
									{
										// sky: 77%
										(*uncertainty) = 23;
										return SKY;
									}
								}
							}
						}
					}
					else
					{
						Cb = (unsigned int)frame_buf[ix];
						if(Cb <= 129)
						{
							// ground: 91%
							(*uncertainty) = 9;
							return GROUND;
						}
						else
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 13)
							{
								Cb = (unsigned int)frame_buf[ix];
								if(Cb <= 136)
								{
									Cr = (unsigned int)frame_buf[ix+2];
									if(Cr <= 123)
									{
										// sky: 70%
										(*uncertainty) = 30;
										return SKY;
									}
									else
									{
										// ground: 74%
										(*uncertainty) = 26;
										return GROUND;
									}
								}
								else
								{
									// sky: 76%
									(*uncertainty) = 24;
									return SKY;
								}
							}
							else
							{
								// ground: 81%
								(*uncertainty) = 19;
								return GROUND;
							}
						}
					}
				}
			}
}


extern void segmentNoIllumination(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, YCO_threshold, dx, dy, gradient, relative_gradient, maxGradient;
	unsigned int ix, Y, Cb, Cr;

	// global variables, so that they are calculated / initialized only once:

	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++)
		{
			ix = image_index(x,y);

			YCO_threshold = (imgHeight * 41) / 100;
			if(y <= YCO_threshold)
			{
				gradient = getGradient(frame_buf, x, y);
				if(gradient <= 3)
				{
					Cr = (unsigned int)frame_buf[ix+2];
					if(Cr <= 113)
					{
						YCO_threshold = (imgHeight * 0) / 100;
						if(y <= YCO_threshold)
						{
							// ground: 100%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 0);
						}
						else
						{
							Cb = (unsigned int)frame_buf[ix];
							if(Cb <= 115)
							{
								// ground: 95%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 5);
							}
							else
							{
								// sky: 98%
								setUncertainty(frame_buf2, ix, 2);
							}
						}
					}
					else
					{
						Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
						if(Y <= 130)
						{
							Cb = (unsigned int)frame_buf[ix];
							if(Cb <= 136)
							{
								// ground: 86%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 14);
							}
							else
							{
								YCO_threshold = (imgHeight * 19) / 100;
								if(y <= YCO_threshold)
								{
									// sky: 91%
									setUncertainty(frame_buf2, ix, 9);
								}
								else
								{
									// ground: 69%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 31);
								}
							}
						}
						else
						{
							YCO_threshold = (imgHeight * 0) / 100;
							if(y <= YCO_threshold)
							{
								// ground: 100%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 0);
							}
							else
							{
								Cr = (unsigned int)frame_buf[ix+2];
								if(Cr <= 131)
								{
									// sky: 92%
									setUncertainty(frame_buf2, ix, 8);
								}
								else
								{
									Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
									if(Y <= 186)
									{
										// ground: 71%
										groundPixel(frame_buf, ix);
										setUncertainty(frame_buf2, ix, 29);
									}
									else
									{
										// sky: 85%
										setUncertainty(frame_buf2, ix, 15);
									}
								}
							}
						}
					}
				}
				else
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= 132)
					{
						Cr = (unsigned int)frame_buf[ix+2];
						if(Cr <= 115)
						{
							Cb = (unsigned int)frame_buf[ix];
							if(Cb <= 155)
							{
								// ground: 76%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 24);
							}
							else
							{
								// sky: 78%
								setUncertainty(frame_buf2, ix, 22);
							}
						}
						else
						{
							// ground: 92%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 8);
						}
					}
					else
					{
						Cr = (unsigned int)frame_buf[ix+2];
						if(Cr <= 123)
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 12)
							{
								// sky: 84%
								setUncertainty(frame_buf2, ix, 16);
							}
							else
							{
								YCO_threshold = (imgHeight * 20) / 100;
								if(y <= YCO_threshold)
								{
									// sky: 75%
									setUncertainty(frame_buf2, ix, 25);
								}
								else
								{
									// ground: 69%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 31);
								}
							}
						}
						else
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 7)
							{
								Cb = (unsigned int)frame_buf[ix];
								if(Cb <= 128)
								{
									// ground: 73%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 27);
								}
								else
								{
									// sky: 77%
									setUncertainty(frame_buf2, ix, 23);
								}
							}
							else
							{
								// ground: 82%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 18);
							}
						}
					}
				}
			}
			else
			{
				YCO_threshold = (imgHeight * 57) / 100;
				if(y <= YCO_threshold)
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= 132)
					{
						// ground: 96%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 4);
					}
					else
					{
						Cr = (unsigned int)frame_buf[ix+2];
						if(Cr <= 121)
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 11)
							{
								// sky: 81%
								setUncertainty(frame_buf2, ix, 19);
							}
							else
							{
								// ground: 81%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 19);
							}
						}
						else
						{
							// ground: 87%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 13);
						}
					}
				}
				else
				{
					// ground: 100%
					groundPixel(frame_buf, ix);
					setUncertainty(frame_buf2, ix, 0);
				}
			}
		}
	}
}

extern void segmentOnlyGradient(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, YCO_threshold, dx, dy, gradient, relative_gradient, maxGradient;
	unsigned int ix;

	// global variables, so that they are calculated / initialized only once:

	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++)
		{
			ix = image_index(x,y);

			YCO_threshold = (imgHeight * 41) / 100;
			if(y <= YCO_threshold)
			{
				gradient = getGradient(frame_buf, x, y);
				if(gradient <= 3)
				{
					YCO_threshold = (imgHeight * 28) / 100;
					if(y <= YCO_threshold)
					{
						YCO_threshold = (imgHeight * 0) / 100;
						if(y <= YCO_threshold)
						{
							// ground: 100%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 0);
						}
						else
						{
							// sky: 93%
							setUncertainty(frame_buf2, ix, 7);
						}
					}
					else
					{
						gradient = getGradient(frame_buf, x, y);
						if(gradient <= 1)
						{
							// sky: 85%
							setUncertainty(frame_buf2, ix, 15);
						}
						else
						{
							YCO_threshold = (imgHeight * 36) / 100;
							if(y <= YCO_threshold)
							{
								gradient = getGradient(frame_buf, x, y);
								if(gradient <= 3)
								{
									// sky: 73%
									setUncertainty(frame_buf2, ix, 27);
								}
								else
								{
									// ground: 68%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 32);
								}
							}
							else
							{
								// ground: 70%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 30);
							}
						}
					}
				}
				else
				{
					YCO_threshold = (imgHeight * 20) / 100;
					if(y <= YCO_threshold)
					{
						gradient = getGradient(frame_buf, x, y);
						if(gradient <= 8)
						{
							// sky: 75%
							setUncertainty(frame_buf2, ix, 25);
						}
						else
						{
							gradient = getGradient(frame_buf, x, y);
							if(gradient <= 16)
							{
								YCO_threshold = (imgHeight * 8) / 100;
								if(y <= YCO_threshold)
								{
									// sky: 68%
									setUncertainty(frame_buf2, ix, 32);
								}
								else
								{
									// ground: 73%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 27);
								}
							}
							else
							{
								// ground: 79%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 21);
							}
						}
					}
					else
					{
						// ground: 82%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 18);
					}
				}
			}
			else
			{
				YCO_threshold = (imgHeight * 57) / 100;
				if(y <= YCO_threshold)
				{
					gradient = getGradient(frame_buf, x, y);
					if(gradient <= 3)
					{
						gradient = getGradient(frame_buf, x, y);
						if(gradient <= 1)
						{
							YCO_threshold = (imgHeight * 50) / 100;
							if(y <= YCO_threshold)
							{
								// sky: 74%
								setUncertainty(frame_buf2, ix, 26);
							}
							else
							{
								// ground: 73%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 27);
							}
						}
						else
						{
							// ground: 78%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 22);
						}
					}
					else
					{
						// ground: 94%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 6);
					}
				}
				else
				{
					// ground: 100%
					groundPixel(frame_buf, ix);
					setUncertainty(frame_buf2, ix, 0);
				}
			}
		}
	}
}
*/

/*
void segmentSkyUncertainty(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, threshold, value;
	unsigned int ix, Y, U, V, maxY;
	
	// the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
	maxY = getMaximumY(frame_buf);
	
	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++) // we could divide imgHeight by 2 to speed things up
		{
			ix = image_index(x,y);
			
			threshold = (imgHeight * 41) / 100;
			if(y <= threshold) // high in the image
			{
				value = getGradient(frame_buf, x, y);
				if(value <= 3) // little gradient
				{
					U = (unsigned int)frame_buf[ix];
					
					if(U <= 137)
					{
						Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
						if(Y <= (maxY * 59) / 100) // not very bright
						{
							V = (unsigned int)frame_buf[ix + 2];
							
							if(V <= 143) // check this one: are the colors right?
							{
								threshold = (imgHeight * 29) / 100;
								if(y <= threshold)
								{
									if(value <= 1)
									{
										// sky
										// 70.0%
										setUncertainty(frame_buf2, ix, 30);
									}
									else
									{
										// ground
										// 77.6%
										groundPixel(frame_buf, ix);
										setUncertainty(frame_buf2, ix, 22);
									}
								}
								else
								{
									// ground:
									// 87.76%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 12);
								}
							}
							else
							{
								// sky:
								// 76,53%
								setUncertainty(frame_buf2, ix, 23);
							}
						}
						else
						{
							// sky
							// 87.37% certainty
							setUncertainty(frame_buf2, ix, 12);
						}
					}
					else
					{
						// sky
						// 93,45% certainty
						setUncertainty(frame_buf2, ix, 6);
					}
				}
				else
				{
					// more gradient
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 59) / 100) // not very bright
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 141)
						{
							// ground:
							// 92.0%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 8);
						}
						else
						{
							if(U <= 152)
							{
								threshold = (imgHeight * 21) / 100;
								if(y <= threshold)
								{
									// sky:
									// 67.7%
									setUncertainty(frame_buf2, ix, 32);
								}
								else
								{
									// ground:
									// 80.0%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 20);
								}
							}
							else
							{
								// sky
								// 74.5%
								setUncertainty(frame_buf2, ix, 25);
							}
						}
					}
					else
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 135)
						{
							if(value <= 7) // medium gradient:
							{
								if(Y <= (maxY * 75) / 100)
								{
									// ground:
									// 71.5%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 28);
								}
								else
								{
									//sky:
									// 74.1% certainty
									setUncertainty(frame_buf2, ix, 26);
								}
							}
							else
							{
								// high gradient:
								// ground
								// 78.6%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 21);
							}
						}
						else
						{
							// sky
							// 79.5%
							setUncertainty(frame_buf2, ix, 20);
						}
					}
					
				}
			}
			else
			{
				threshold = (imgHeight * 57) / 100;
				if(y <= threshold)
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 58) / 100)
					{
						// ground
						// 94.5%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 5);
					}
					else
					{
						value = getGradient(frame_buf, x, y);
						if(value <= 4)
						{
							V = (unsigned int)frame_buf[ix + 2];
							if(V <= 118)
							{
								// sky
								// 85.1%
								setUncertainty(frame_buf2, ix, 15);
							}
							else
							{
								threshold = (imgHeight * 47) / 100;
								if(y <= threshold)
								{
									// sky:
									// 70.3%
									setUncertainty(frame_buf2, ix, 29);
								}
								else
								{
									// ground:
									// 71.9%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 28);
								}
							}
						}
						else
						{
							// ground
							// 84.2%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 16);
						}
					}
				}
				else
				{
					// ground: 98,74%
					groundPixel(frame_buf, ix);
					setUncertainty(frame_buf2, ix, 1);
				}
			}
		}
	}
}

void segmentSky(unsigned char *frame_buf)
{
	// use a pre-defined tree to segment the image:
	// we may give a second image to store the uncertainties in the future:
	int x, y, threshold, value;
	unsigned int ix, Y, U, V, maxY;
	
	// the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
	maxY = getMaximumY(frame_buf);
	
	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++) // we could divide imgHeight by 2 to speed things up
		{
			ix = image_index(x,y);
			
			threshold = (imgHeight * 41) / 100;
			if(y <= threshold) // high in the image
			{
				value = getGradient(frame_buf, x, y);
				if(value <= 3) // little gradient
				{
					U = (unsigned int)frame_buf[ix];
					
					if(U <= 137)
					{
						Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
						if(Y <= (maxY * 59) / 100) // not very bright
						{
							V = (unsigned int)frame_buf[ix + 2];
							
							if(V <= 143) // check this one: are the colors right?
							{
								threshold = (imgHeight * 29) / 100;
								if(y <= threshold)
								{
									if(value <= 1)
									{
										// sky
										// 70.0%
									}
									else
									{
										// ground
										// 77.6%
										groundPixel(frame_buf, ix);
									}
								}
								else
								{
									// ground:
									// 87.76%
									groundPixel(frame_buf, ix);
								}
							}
							else
							{
								// sky:
								// 76,53%
							}
						}
						else
						{
							// sky
							// 87.37% certainty
						}
					}
					else
					{
						// sky
						// 93,45% certainty
					}
				}
				else
				{
					// more gradient
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 59) / 100) // not very bright
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 141)
						{
							// ground:
							// 92.0%
							groundPixel(frame_buf, ix);
						}
						else
						{
							if(U <= 152)
							{
								threshold = (imgHeight * 21) / 100;
								if(y <= threshold)
								{
									// sky:
									// 67.7%
								}
								else
								{
									// ground:
									// 80.0%
									groundPixel(frame_buf, ix);
								}
							}
							else
							{
								// sky
								// 74.5%
							}
						}
					}
					else
					{
						U = (unsigned int)frame_buf[ix];
						if(U <= 135)
						{
							if(value <= 7) // medium gradient:
							{
								if(Y <= (maxY * 75) / 100)
								{
									// ground:
									// 71.5%
									groundPixel(frame_buf, ix);
								}
								else
								{
									//sky:
									// 74.1% certainty
								}
							}
							else
							{
								// high gradient:
								// ground
								// 78.6%
								groundPixel(frame_buf, ix);
							}
						}
						else
						{
							// sky
							// 79.5%
						}
					}
					
				}
			}
			else
			{
				threshold = (imgHeight * 57) / 100;
				if(y <= threshold)
				{
					Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
					if(Y <= (maxY * 58) / 100)
					{
						// ground
						// 94.5%
						groundPixel(frame_buf, ix);
					}
					else
					{
						value = getGradient(frame_buf, x, y);
						if(value <= 4)
						{
							V = (unsigned int)frame_buf[ix + 2];
							if(V <= 118)
							{
								// sky
								// 85.1%
							}
							else
							{
								threshold = (imgHeight * 47) / 100;
								if(y <= threshold)
								{
									// sky:
									// 70.3%
								}
								else
								{
									// ground:
									// 71.9%
									groundPixel(frame_buf, ix);
								}
							}
						}
						else
						{
							// ground
							// 84.2%
							groundPixel(frame_buf, ix);
						}
					}
				}
				else
				{
					// ground: 98,74%
					groundPixel(frame_buf, ix);
				}
			}
		}
	}
}
*/

/*
extern void segmentPatch(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, YCO_threshold, patch_size, patch_mean, patch_texture;
	unsigned int ix, Y, Cb, Cr;

	// global variables, so that they are calculated / initialized only once:
	patch_size = 10;

	for(x = 0; x < imgWidth; x++)
	{
		for(y = 0; y < imgHeight; y++)
		{
			ix = image_index(x,y);

			YCO_threshold = (imgHeight * 39) / 100;
			if(y <= YCO_threshold)
			{
				patch_mean = getPatchMean(frame_buf, x, y, patch_size);
				if(patch_mean <= 131)
				{
					Cb = (unsigned int)frame_buf[ix];
					if(Cb <= 139)
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 1)
						{
							// sky: 73%
							setUncertainty(frame_buf2, ix, 27);
						}
						else
						{
							// ground: 94%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 6);
						}
					}
					else
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 1)
						{
							// sky: 92%
							setUncertainty(frame_buf2, ix, 8);
						}
						else
						{
							YCO_threshold = (imgHeight * 18) / 100;
							if(y <= YCO_threshold)
							{
								// sky: 73%
								setUncertainty(frame_buf2, ix, 27);
							}
							else
							{
								// ground: 82%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 18);
							}
						}
					}
				}
				else
				{
					patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
					if(patch_texture <= 3)
					{
						YCO_threshold = (imgHeight * 0) / 100;
						if(y <= YCO_threshold)
						{
							// ground: 100%
							groundPixel(frame_buf, ix);
							setUncertainty(frame_buf2, ix, 0);
						}
						else
						{
							// sky: 95%
							setUncertainty(frame_buf2, ix, 5);
						}
					}
					else
					{
						Cr = (unsigned int)frame_buf[ix+2];
						if(Cr <= 120)
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 109)
							{
								// sky: 81%
								setUncertainty(frame_buf2, ix, 19);
							}
							else
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= 128)
								{
									// ground: 74%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 26);
								}
								else
								{
									// sky: 74%
									setUncertainty(frame_buf2, ix, 26);
								}
							}
						}
						else
						{
							patch_mean = getPatchMean(frame_buf, x, y, patch_size);
							if(patch_mean <= 141)
							{
								// ground: 84%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 16);
							}
							else
							{
								YCO_threshold = (imgHeight * 26) / 100;
								if(y <= YCO_threshold)
								{
									patch_mean = getPatchMean(frame_buf, x, y, patch_size);
									if(patch_mean <= 147)
									{
										// ground: 69%
										groundPixel(frame_buf, ix);
										setUncertainty(frame_buf2, ix, 31);
									}
									else
									{
										// sky: 76%
										setUncertainty(frame_buf2, ix, 24);
									}
								}
								else
								{
									// ground: 75%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 25);
								}
							}
						}
					}
				}
			}
			else
			{
				patch_mean = getPatchMean(frame_buf, x, y, patch_size);
				if(patch_mean <= 131)
				{
					// ground: 100%
					groundPixel(frame_buf, ix);
					setUncertainty(frame_buf2, ix, 0);
				}
				else
				{
					YCO_threshold = (imgHeight * 60) / 100;
					if(y <= YCO_threshold)
					{
						patch_texture = getPatchTexture(frame_buf, x, y, patch_size);
						if(patch_texture <= 2)
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 121)
							{
								// sky: 83%
								setUncertainty(frame_buf2, ix, 17);
							}
							else
							{
								YCO_threshold = (imgHeight * 49) / 100;
								if(y <= YCO_threshold)
								{
									// sky: 72%
									setUncertainty(frame_buf2, ix, 28);
								}
								else
								{
									// ground: 76%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 24);
								}
							}
						}
						else
						{
							Cr = (unsigned int)frame_buf[ix+2];
							if(Cr <= 122)
							{
								Y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
								if(Y <= 139)
								{
									// ground: 84%
									groundPixel(frame_buf, ix);
									setUncertainty(frame_buf2, ix, 16);
								}
								else
								{
									Cr = (unsigned int)frame_buf[ix+2];
									if(Cr <= 117)
									{
										// sky: 73%
										setUncertainty(frame_buf2, ix, 27);
									}
									else
									{
										// ground: 72%
										groundPixel(frame_buf, ix);
										setUncertainty(frame_buf2, ix, 28);
									}
								}
							}
							else
							{
								// ground: 90%
								groundPixel(frame_buf, ix);
								setUncertainty(frame_buf2, ix, 10);
							}
						}
					}
					else
					{
						// ground: 99%
						groundPixel(frame_buf, ix);
						setUncertainty(frame_buf2, ix, 1);
					}
				}
			}
		}
	}
}
*/

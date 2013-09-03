#include "guido.h"
#include "colors.h"
#include <math.h>
#include <stdlib.h>     /* abs */
#include <stdio.h> /* printf */

// *****************
// INLINE FUNCTIONS:
// *****************

static inline void groundPixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0x00;
	frame_buf[ip+1] = 0x00;
	frame_buf[ip+2] = 0x00;
	frame_buf[ip+3] = 0x00;
}

static inline void redPixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0x00;
	frame_buf[ip+1] = 0x00;
	frame_buf[ip+2] = 0xff;
	frame_buf[ip+3] = 0x00;
}

static inline void blackDot(unsigned char *frame_buf, int x, int y)
{
	unsigned int ip;
	int xx, yy;
	for(xx = x-1; xx <= x+1; xx++)
	{
		for(yy = y-1; yy <= y+1; yy++)
		{
			if(xx >= 0 && xx < (int)imgWidth && yy >= 0 && yy < (int)imgHeight)
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

static inline void setUncertainty(unsigned char *frame_buf, unsigned int ip, unsigned int uncertainty) 
{
	// if(uncertainty > 255) uncertainty = 255;
	frame_buf[ip] = 127;
	frame_buf[ip+1] = uncertainty;
	frame_buf[ip+2] = 127;
	frame_buf[ip+3] = uncertainty;
}

static inline int isGroundPixel(unsigned char *frame_buf, unsigned int ip)
{
	if(frame_buf[ip] == 0x00 && frame_buf[ip+1] == 0x00 && frame_buf[ip+2] == 0x00 && 	frame_buf[ip+3] == 0x00) return 1;
	else return 0;
}

static inline void linePixel(unsigned char *frame_buf, unsigned int ip) 
{
	frame_buf[ip] = 0;
	frame_buf[ip+1] = 255;
	frame_buf[ip+2] = 255;
	frame_buf[ip+3] = 255;
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
	x = (x >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : x;	
	y = (y < half_patch_size) ? half_patch_size : y;
	y = (y >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : y;
	
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
	x = (x >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : x;	
	y = (y < half_patch_size) ? half_patch_size : y;
	y = (y >= (int)imgWidth - half_patch_size) ? (int)imgWidth - half_patch_size - 1 : y;
	
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
	else if(x >= (int)imgWidth - 1) min_x = (int)imgWidth - 2;
	else min_x = x - 1;
	if(y <= 0) min_y = 0;
	else if(y >= (int)imgHeight - 1) min_y = (int)imgHeight - 2;
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
	else if(x >= (int)imgWidth - 1) min_x = (int)imgWidth - 2;
	else min_x = x - 1;
	if(y <= 0) min_y = 0;
	else if(y >= (int)imgHeight - 1) min_y = (int)imgHeight - 2;
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
	if(x >= 0 && x < (int)imgWidth && y >= 0 && y < (int)imgHeight)
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
	
		if(x < (int)imgWidth - 1)
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
	
		if(y < (int)imgHeight - 1)
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

	if(x >= 0 && x < (int)imgWidth && y >= 0 && y < (int)imgHeight)
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



void segmentSkyUncertainty2(unsigned char *frame_buf, unsigned char *frame_buf2)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, threshold, value;
	unsigned int ix, Y, U, V, maxY;
	
	// the maximal illuminance is used in almost all sub-branches, so it is better to calculate it immediately once:
	maxY = getMaximumY(frame_buf);
		
	for(x = 0; x < (int)imgWidth; x++)
	{
		for(y = 0; y < (int)imgHeight; y++) // we could divide imgHeight by 2 to speed things up
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
	unsigned int ix, Y, Cr;

	// global variables, so that they are calculated / initialized only once:
	maxY = getMaximumY(frame_buf);
	patch_size = 10;

	for(x = 0; x < (int)imgWidth; x++)
	{
		for(y = 0; y < (int)imgHeight; y++)
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
								if(Y <= (unsigned int)(maxY * 60) / 100)
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

extern void segment_no_yco_AdjustTree(unsigned char *frame_buf, unsigned char *frame_buf2, int adjust_factor)
{
	// use a pre-defined tree to segment the image:
	// the second buffer (image) stores the uncertainties between 0 and 100.
	int x, y, maxY, adjust_rel_Y, patch_size, patch_texture, adjust_patch_texture, adjust_Cr, FD_YCV, adjust_FD_YCV, FD_CV, adjust_FD_CV;
	unsigned int ix, Y, Cr;

	// global variables, so that they are calculated / initialized only once:
	maxY = getMaximumY(frame_buf);
	patch_size = 10;

	// variables for adjusting thresholds:
	adjust_Cr = adjust_factor * 3;
	adjust_rel_Y = adjust_factor * -5;
	adjust_patch_texture = adjust_factor * 2;
	adjust_FD_YCV = adjust_factor * -48;
	adjust_FD_CV = adjust_factor * -48;


	for(x = 0; x < (int)imgWidth; x++)
	{
		for(y = 0; y < (int)imgHeight; y++)
		{
			ix = image_index(x,y);

			FD_YCV = get_FD_YCV(frame_buf, x, y);
			if(FD_YCV <= 58 + adjust_FD_YCV)
			{
				Cr = (unsigned int)frame_buf[ix+2];
				if(Cr <= 150 + (unsigned int)adjust_Cr)
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
							if(Cr <= 126 + (unsigned int)adjust_Cr)
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
								if(Y <= (unsigned int)(maxY * 66) / 100 + adjust_rel_Y)
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
void skyseg_interface_i(unsigned char *frame_buf, unsigned char *frame_buf2, char adjust_factor, unsigned int counter) {
//	case 'n': // with adjustable tree:
		
		if(adjust_factor < 3)
		{
			if(adjust_factor == 0) adjust_factor = -10;
			if(adjust_factor == 1) adjust_factor = -5;
			if(adjust_factor == 2) adjust_factor = -2;
		}
		else if(adjust_factor == 3)
		{
			adjust_factor = 0;
		}
		else
		{
			if(adjust_factor >= 4 && adjust_factor <= 6) adjust_factor -= 3;
			if(adjust_factor == 7) adjust_factor = 5;
			if(adjust_factor == 8) adjust_factor = 8;
			if(adjust_factor == 9) adjust_factor = 11;
			if(adjust_factor == 10) adjust_factor = 15;
		}

		
		unsigned int print_frequency = 5;
		
		//get_state_from_autopilot(&state); //TODO: change
		segment_no_yco_AdjustTree((unsigned char *)frame_buf, (unsigned char *)frame_buf2, adjust_factor);
	    // determine the amount of obstacle per orientation segment
	    getObstacles(obstacles, N_BINS, (unsigned char *)frame_buf, &max_bin, &bin_total, MAX_BIN_VALUE);
		//send_obstacles_to_autopilot(max_bin, bin_total, obstacles, N_BINS); //TODO: change
		if(counter++ % print_frequency == 0)
		{
				printf("*od*"); // protocol start for obstacle info
				for(bin = 0; bin < N_BINS; bin++)
				{
				    printf("%d,", obstacles[bin]);
				}
		    	printf("u");
		    	// determine the amount of uncertainty in the segmentation per orientation segment
		    	getUncertainty(uncertainty, N_BINS, (unsigned char *)frame_buf2);
		    	for(bin = 0; bin < N_BINS; bin++)
		    	{
		    	    printf("%d,", uncertainty[bin]);
		    	}
		    	printf("s\n"); // protocol end
		}
		
	}
	*/
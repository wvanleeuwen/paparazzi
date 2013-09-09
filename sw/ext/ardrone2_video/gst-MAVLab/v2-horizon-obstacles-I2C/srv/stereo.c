/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  stereo.c - SVS (stereo vision system) functions
 *    Copyright (C) 2005-2009  Surveyor Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details (www.gnu.org/licenses)
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "stereo.h"
#include <cdefBF537.h>
#include "config.h"
#include "uart.h"
#include "print.h"
#include "xmodem.h"
#include "srv.h"
#include "malloc.h"
#include "string.h"

/* -----------------------------------------------------------------------------------------------------*/
/* stereo correspondence */
/* -----------------------------------------------------------------------------------------------------*/

void svs_master(unsigned short *outbuf16, unsigned short *inbuf16, int bufsize)
{
#ifdef SVS_PROFILE
    int t[6];
#endif
    int remainingBytes;
    unsigned short ix, dummy;

#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif

    *pPORTF_FER |= (PF14|PF13|PF12|PF11|PF10);  /* SLCK, MISO, MOSI perhipheral */
    *pPORT_MUX |= PJSE;  /* Enable PJ10 (SSEL2/3) PORT_MUX PJSE=1 */
    *pSPI_BAUD = 8;      /* 4, 8, 16 */
    *pSPI_FLG = FLS3;    /* set FLS3 (FLG3 not required with CHPA=0 */
    *pSPI_CTL = SPE|MSTR|CPOL|SIZE|EMISO|0x00;  /* we use CPHA=0 hardware control */
    SSYNC;

#ifdef SVS_PROFILE
    t[1] = readRTC();
#endif

    remainingBytes = bufsize;

    ix = (unsigned int)crc16_ccitt((void *)outbuf16, bufsize-8);
    outbuf16[(bufsize/2)-2] = ix; /* write CRC into buffer */

#ifdef SVS_PROFILE
    t[2] = readRTC();
#endif

    *pSPI_TDBR = *outbuf16++;
    dummy = *pSPI_RDBR;  /* slave will not have written yet */
    remainingBytes -= 2;
    SSYNC;

#ifdef SVS_PROFILE
    t[3] = readRTC();
#endif

    while (remainingBytes)
    {
        while ((*pSPI_STAT&SPIF) ==0 )
            ; /* ensure spi tranfer complete */
        while ((*pSPI_STAT&TXS) >0 )
            ;  /* ensure tx buffer empty */
        while ((*pSPI_STAT&RXS) ==0 )
            ; /* ensure rx buffer full */
        *pSPI_TDBR = *outbuf16++;
        SSYNC;
        if (remainingBytes == (bufsize-2))  /* first data is junk */
            dummy = *pSPI_RDBR;
        else
            *inbuf16++ = *pSPI_RDBR; /* read data from slave processor */
        SSYNC;
        remainingBytes -= 2;
    }

#ifdef SVS_PROFILE
    t[4] = readRTC();
#endif

    printf("##$X SPI Master - Ack = 0x%x\n\r", (unsigned int)dummy);
    *pSPI_CTL = 0x400;
    *pSPI_FLG = 0x0;
    *pSPI_BAUD = 0x0;
    SSYNC;

#ifdef SVS_PROFILE
    t[5] = readRTC();
#endif

#ifdef SVS_PROFILE
    printf("svs_master:\n\r");
    printf("    step 0  %d\n\r", t[1]-t[0]);
    printf("    step 1  %d\n\r", t[2]-t[1]);
    printf("    step 2  %d\n\r", t[3]-t[2]);
    printf("    step 3  %d\n\r", t[4]-t[3]);
    printf("    step 4  %d\n\r", t[5]-t[4]);
#endif
}

void svs_slave(
    unsigned short *inbuf16,
    unsigned short *outbuf16,
    int bufsize)
{
    int remainingBytes;
    unsigned short *bufsave;

    bufsave = inbuf16;
    *pPORTF_FER    |= (PF14|PF13|PF12|PF11|PF10);    /* SPISS select PF14 input as slave, */
    /* MOSI PF11 enabled (note shouldn't need PF10 as that's the flash memory) */
    *pPORT_MUX    |= PJSE;     /* Enable PJ10 SSEL2 & 3 PORT_MUX PJSE=1  not required as we're slave.. */
    *pSPI_BAUD    = 8;         /* 4, 8, 16 */
    *pSPI_CTL     = SPE|CPOL|SIZE|EMISO|0x00; /* tc on read */
    SSYNC;

    remainingBytes = bufsize;

    //ix = (unsigned int)crc16_ccitt((void *)buf16, bufsize-8);
    //buf16[(bufsize/2)-2] = ix; /* write CRC into buffer */

    while (remainingBytes)
    {
        while ( (*pSPI_STAT&SPIF) == 0 )
            ; /* ensure spif transfer complete */
        while ( (*pSPI_STAT&RXS) == 0  )
            ; /* ensure rx buffer full */

        *pSPI_TDBR = *outbuf16++;
        SSYNC;
        *inbuf16++ = *pSPI_RDBR; /* read full 16 bits in one */
        remainingBytes -= 2;
        while ( (remainingBytes>0) && ((*pSPI_STAT&TXS) > 0)  )
            ; /* ensure tx buffer empty */
    };

    *pSPI_CTL = 0x400;
    *pSPI_FLG = 0x0;
    *pSPI_BAUD = 0x0;
    SSYNC;
}

/* structure storing data for this camera */
svs_data_struct *svs_data;

/* structure storing data for this camera on the previous frame*/
svs_data_struct *svs_data_previous;

/* structure storing data received from the opposite camera */
svs_data_struct *svs_data_received;

/* buffer which stores sliding sum */
int *row_sum;

/* buffer used to find peaks in edge space */
unsigned int *row_peaks;

/* array stores matching probabilities (prob,x,y,disp) */
unsigned int *svs_matches;

/* used during filtering */
unsigned char *valid_quadrants;

/* array used to store a disparity histogram */
int* disparity_histogram_plane;
int* disparity_plane_fit;

/* number of detected planes found during the filtering step */
int no_of_planes;
int* plane;

/* maps raw image pixels to rectified pixels */
int *calibration_map;

/* array storing the number of features detected on each column */
unsigned short int* features_per_col;

/* array storing y coordinates of detected features */
short int* feature_y;

/* used during segmentation */
int* curr_ID;
int* curr_ID_hits;

extern unsigned int imgWidth, imgHeight;
extern int svs_calibration_offset_x, svs_calibration_offset_y;
extern int svs_centre_of_disortion_x, svs_centre_of_disortion_y;
extern int svs_scale_num, svs_scale_denom;
extern int svs_coeff_degree;
extern long* svs_coeff;
extern int svs_width, svs_height;

/* previous image width.  This is used to detect resolution changes */
int svs_prev_width;

/* array used to estimate footline of objects on the ground plane */
unsigned short int** footline;

/* distances to the footline in mm */
unsigned short int* footline_dist_mm;

#ifdef SVS_FOOTLINE
/* lines fitted to the footline points */
unsigned short int** footline_slope;
#endif

/* whether mapping was previously enabled */
int prev_svs_enable_mapping;

/* offsets of pixels to be compared within the patch region
 * arranged into a rectangular structure */
const int pixel_offsets[] =
{
    -2,-4,  -1,-4,         1,-4,  2,-4,
    -5,-2,  -4,-2,  -3,-2,  -2,-2,  -1,-2,  0,-2,  1,-2,  2,-2,  3,-2,  4,-2,  5,-2,
    -5, 2,  -4, 2,  -3, 2,  -2, 2,  -1, 2,  0, 2,  1, 2,  2, 2,  3, 2,  4, 2,  5, 2,
    -2, 4,  -1, 4,         1, 4,  2, 4
};


/* lookup table used for counting the number of set bits */
const unsigned char BitsSetTable256[] =
{
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

/* Updates sliding sums and edge response values along a single row or column
 * Returns the mean luminance along the row or column */
int svs_update_sums(
    int cols, /* if non-zero we're dealing with columns not rows */
    int i, /* row or column index */
    unsigned char* rectified_frame_buf, /* image data */
    int segment) { /* if non zero update low contrast areas used for segmentation */

    int w, j, x, y, idx, max, sum = 0, mean = 0;

    if (cols == 0) {
        /* compute sums along the row */
        y = i;
        idx = (int)imgWidth * y;
        max = (int)imgWidth;
        w = max*2;

        sum = rectified_frame_buf[idx];
        row_sum[0] = sum;
        for (x = 1; x < max; x++, idx++) {
            sum += rectified_frame_buf[idx] +
                + rectified_frame_buf[idx - w] +
                + rectified_frame_buf[idx + w];
            row_sum[x] = sum;
        }
    } else {
        /* compute sums along the column */
        idx = i;
        x = i;
        max = (int) imgHeight;

        sum = rectified_frame_buf[idx];
        row_sum[0] = sum;
        for (y = 1; y < max; y++, idx+=(int)imgWidth) {
            sum += rectified_frame_buf[idx];
            row_sum[y] = sum;
        }
    }

    /* row mean luminance */
    mean = row_sum[max - 1] / (max * 2);

    /* compute peaks */
    int p0, p1, p2;
    for (j = 5; j < max - 5; j++) {
        sum = row_sum[j];
        /* edge using 1 pixel radius */
        p0 = (sum - row_sum[j - 1]) - (row_sum[j + 1] - sum);
        if (p0 < 0) p0 = -p0;

        /* edge using 3 pixel radius */
        p1 = (sum - row_sum[j - 3]) - (row_sum[j + 3] - sum);
        if (p1 < 0) p1 = -p1;

        /* edge using 5 pixel radius */
        p2 = (sum - row_sum[j - 5]) - (row_sum[j + 5] - sum);
        if (p2 < 0) p2 = -p2;

        /* overall edge response */
        row_peaks[j] = p0*8 + p1*2 + p2;
    }

    return (mean);
}


/* performs non-maximal suppression on the given row or column */
void svs_non_max(
    int cols, /* if non-zero we're dealing with columns not rows */
    int inhibition_radius, /* radius for non-maximal suppression */
    unsigned int min_response) { /* minimum threshold as a percent in the range 0-200 */

    int i, r, max, max2;
    unsigned int v;

    /* average response */
    unsigned int av_peaks = 0;
    max = (int)imgWidth;
    if (cols != 0) max = (int)imgHeight;
    max2 = max - inhibition_radius;
    max -= 4;
    for (i = 4; i < max; i++) {
        av_peaks += row_peaks[i];
    }

    /* adjust the threshold */
    av_peaks = av_peaks * min_response / (100 * (max - 4));

    for (i = 4; i < max2; i++) {
        if (row_peaks[i] > av_peaks) {
            v = row_peaks[i];
            if (v > 0) {
                for (r = 1; r < inhibition_radius; r++) {
                    if (row_peaks[i + r] < v) {
                        row_peaks[i + r] = 0;
                    } else {
                        row_peaks[i] = 0;
                        break;
                    }
                }
            }
        } else {
            row_peaks[i] = 0;
        }
    }
}

/* creates a binary descriptor for a feature at the given coordinate
   which can subsequently be used for matching */
int svs_compute_descriptor(
    int px,
    int py,
    unsigned char* rectified_frame_buf,
    int no_of_features,
    int row_mean)
{

    unsigned char bit_count = 0;
    int pixel_offset_idx, ix, bit;
    int meanval = 0;
    unsigned int desc = 0;

    /* find the mean luminance for the patch */
    for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS*2; pixel_offset_idx += 2)
    {
        ix = rectified_frame_buf[pixindex2((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
        meanval += rectified_frame_buf[ix];
    }
    meanval /= SVS_DESCRIPTOR_PIXELS;

    /* binarise */
    bit = 1;
    for (pixel_offset_idx = 0; pixel_offset_idx < SVS_DESCRIPTOR_PIXELS*2; pixel_offset_idx += 2, bit *= 2)
    {
        ix = rectified_frame_buf[pixindex2((px + pixel_offsets[pixel_offset_idx]), (py + pixel_offsets[pixel_offset_idx + 1]))];
        if (rectified_frame_buf[ix] > meanval)
        {
            desc |= bit;
            bit_count++;
        }
    }

    if ((bit_count > 3) &&
            (bit_count < SVS_DESCRIPTOR_PIXELS-3))
    {
        /* adjust the patch luminance relative to the mean
         * luminance for the entire row.  This helps to ensure
         * that comparisons between left and right images are
         * fair even if there are exposure/illumination differences. */
        meanval = meanval - row_mean + 127;
        if (meanval < 0)
            meanval = 0;
        if (meanval > 255)
            meanval = 255;
            
		meanval /= 16;
		if (meanval > 15) meanval = 15;
		svs_data->mean[no_of_features] = (unsigned char)meanval;
        svs_data->descriptor[no_of_features] = desc;
        return(0);
    }
    else
    {
        /* probably just noise */
        return(-1);
    }
}

/* returns a set of vertically oriented features suitable for stereo matching */
int svs_get_features_vertical(
    unsigned char* rectified_frame_buf,  /* image data */
    int inhibition_radius,               /* radius for non-maximal supression */
    unsigned int minimum_response,       /* minimum threshold */
    int calibration_offset_x,            /* calibration x offset in pixels */
    int calibration_offset_y,            /* calibration y offset in pixels */
    int segment)                         /* if non zero update low contrast areas used for segmentation */
{

    unsigned short int no_of_feats;
	int x, y, row_mean, start_x;
	int prev_x, mid_x, grad;    
    int no_of_features = 0;
    int row_idx = 0;

    memset((void*)(svs_data->features_per_row), '\0', imgHeight/SVS_VERTICAL_SAMPLING * sizeof(unsigned short));

    start_x = imgWidth - 15;
    if ((int)imgWidth - inhibition_radius - 1 < start_x)
        start_x = (int)imgWidth - inhibition_radius - 1;

    for (y = 4 + calibration_offset_y; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING)
    {

        /* reset number of features on the row */
        no_of_feats = 0;

        if ((y >= 4) && (y <= (int)imgHeight - 4))
        {

            row_mean = svs_update_sums(0, y, rectified_frame_buf, segment);
            svs_non_max(0, inhibition_radius, minimum_response);

            /* store the features */
            prev_x = start_x;
            for (x = start_x; x > 15; x--)
            {
                if (row_peaks[x] > 0)
                {

                    if (svs_compute_descriptor(
                                x, y, rectified_frame_buf, no_of_features, row_mean) == 0)
                    {

						mid_x = prev_x + ((x - prev_x)/2);
						if (x != prev_x) {
							grad =
								((row_sum[mid_x] - row_sum[x]) -
								(row_sum[prev_x] - row_sum[mid_x])) /
								((prev_x - x)*4);
							/* limit gradient into a 4 bit range */
							if (grad < -7) grad = -7;
							if (grad > 7) grad = 7;
							/* pack the value into the upper 4 bits */
							svs_data->mean[no_of_features] |= 
							    ((unsigned char)(grad + 8) << 4);						    
						}
							
                        svs_data->feature_x[no_of_features++] = (short int)(x + calibration_offset_x);
                        no_of_feats++;
                        if (no_of_features == SVS_MAX_FEATURES)
                        {
                            y = imgHeight;
                            printf("stereo feature buffer full\n");
                            break;
                        }
                    }
                }
            }
        }

        svs_data->features_per_row[row_idx++] = no_of_feats;
    }

    svs_data->no_of_features = no_of_features;

#ifdef SVS_VERBOSE
    printf("%d vertically oriented edge features located\n", no_of_features);
#endif

    return(no_of_features);
}

/* returns a set of horizontally oriented features */
int svs_get_features_horizontal(
    unsigned char* rectified_frame_buf, /* image data */
    int inhibition_radius, /* radius for non-maximal supression */
    unsigned int minimum_response, /* minimum threshold */
    int calibration_offset_x, int calibration_offset_y,
    int segment) /* if non zero update low contrast areas used for segmentation */
{
    unsigned short int no_of_feats;
    int x, y, row_mean, start_y, i, idx;
    int no_of_features = 0;
    int col_idx = 0;

    /* create arrays */
    if (features_per_col == 0) {
        features_per_col = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING*sizeof(unsigned short int));
        feature_y = (short int*)malloc(SVS_MAX_FEATURES*sizeof(short int));
        footline = (unsigned short int**)malloc(2*sizeof(unsigned short int*));
        for (i = 0; i < 2; i++) {
            footline[i] = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
        }
    }

    int ground_y=(int)imgHeight;
    if (svs_enable_ground_priors) {
        ground_y = (int)imgHeight - 1 - (svs_ground_y_percent*(int)imgHeight/100);
        for (i = 0; i < 2; i++) {
            memset((void*)(footline[i]), '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING
                   * sizeof(unsigned short));
        }
    }

    memset((void*)features_per_col, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING
           * sizeof(unsigned short));

    start_y = (int)imgHeight - 15;
    if ((int)imgHeight - inhibition_radius - 1 < start_y)
        start_y = (int)imgHeight - inhibition_radius - 1;

    for (x = 4 + calibration_offset_x; x < (int)imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING) {

        /* reset number of features on the row */
        no_of_feats = 0;

        if ((x >= 4) && (x <= (int)imgWidth - 4)) {

            row_mean = svs_update_sums(1, x, rectified_frame_buf, segment);
            svs_non_max(1, inhibition_radius, minimum_response);

            /* store the features */
            for (y = start_y; y > 15; y--) {
                if (row_peaks[y] > 0) {

                    if (svs_enable_ground_priors) {
                        if (y >= ground_y) {
                            idx = x / SVS_HORIZONTAL_SAMPLING;
                            footline[1][idx] = footline[0][idx];
                            footline[0][idx] = y;
                        }
                    }

                    feature_y[no_of_features++] = (short int) (y + calibration_offset_y);
                    no_of_feats++;
                    if (no_of_features == SVS_MAX_FEATURES) {
                        x = (int)imgWidth;
                        printf("stereo feature buffer full\n");
                        break;
                    }
                }
            }
        }

        features_per_col[col_idx++] = no_of_feats;
    }

#ifdef SVS_VERBOSE
    printf("%d horizontally oriented edge features located\n", no_of_features);
#endif

    return (no_of_features);
}

#ifdef SVS_FOOTLINE
void svs_footline_slope()
{
    unsigned short int x,x2,xx,y,y2,yy;
    unsigned short int max = (int)imgWidth / SVS_HORIZONTAL_SAMPLING;
    unsigned short int baseline = max * 20 / 100;
    if (baseline < 3) baseline = 3;
    unsigned short int min_slope = 10;
    const int max_variance = 6;
    int diff, tot_diff, hits, idx, i;
    int hits_thresh = max * 30 / 100;

    /* clear the variance values */    
    for (idx = 0; idx < 3; idx++) {
        footline_slope[idx][0] = (unsigned short int)9999;
    }
    
    for (i = 0; i < 2; i++) {
        for (x = 0; x < max-baseline; x++) {
            if (footline[i][x] > 0) {
                x2 = x + baseline;
                if (x2 < max) {
                    if (footline[i][x2] > 0) {
                        y = footline[i][x];
                        y2 = footline[i][x2];
                        
                        /* check this line to see how well it fits to the data */
                        tot_diff = 0;
                        hits = 0;
                        for (xx = 0; xx < (unsigned short int)imgWidth; xx++) {
                            if (footline[i][xx] > 0) {
                                yy = y + ((xx - x) * (y2 - y) / (x2 - x));
                                diff = (int)yy - (int)footline[i][xx];
                                if ((diff > -max_variance) && 
                                    (diff < max_variance)) {                            
                                    tot_diff += diff*diff;
                                    hits++;
                                }
                                else {
                                    if (footline[1 - i][xx] > 0) {
                                        diff = (int)yy - (int)footline[1 - i][xx];
                                        if ((diff > -max_variance) && 
                                            (diff < max_variance)) {                            
                                            tot_diff += diff*diff;
                                            hits++;
                                        }
                                    }
                                }
                            }
                            else {
                                if (footline[1 - i][xx] > 0) {
                                    yy = y + ((xx - x) * (y2 - y) / (x2 - x));
                                    diff = (int)yy - (int)footline[1 - i][xx];
                                    if ((diff > -max_variance) && 
                                        (diff < max_variance)) {                            
                                        tot_diff += diff*diff;
                                        hits++;
                                    }
                                }
                            }
                        }
                        
                        if (hits > hits_thresh) {
                            tot_diff /= hits;                        
                            idx = 0;
                            if (y2 < y - min_slope) {
                                /* slope_left */                        
                                idx = 0;
                                
                            }
                            else {
                                if (y2 > y + min_slope) {
                                    /* slope right */
                                    idx = 1;
                                }
                                else {
                                    /* horizontal */
                                    idx = 2;
                                }
                            }
                            
                            if ((footline_slope[idx][0] == (unsigned short int)9999) ||
                                (tot_diff < (int)footline_slope[idx][0])) {
                                footline_slope[idx][0] = (unsigned short int)tot_diff;
                                footline_slope[idx][1] = x*SVS_HORIZONTAL_SAMPLING;
                                footline_slope[idx][2] = y;
                                footline_slope[idx][3] = x2*SVS_HORIZONTAL_SAMPLING;
                                footline_slope[idx][4] = y2;
                            }
                        }
                        
                    }
                }
            }
        }
    }
}
#endif

/* sends feature data from the right camera board to the left */
void svs_send_features()
{
    /* create checksum */
    svs_data->crc = 0;
    svs_data->received_bit = 1;
    svs_data->crc = (unsigned short)crc16_ccitt((void *)svs_data, sizeof(svs_data));

    /* copy data into the send buffer */
    memcpy2(
        (void*)(FLASH_BUFFER+16384),
        (void*)svs_data,
        16384);

#ifdef SVS_VERBOSE
    printf("svs slave waiting to send features\n");
#endif
    /* send it */
    svs_slave(
        (unsigned short *)FLASH_BUFFER,
        (unsigned short *)(FLASH_BUFFER+16384),
        16384);

#ifdef SVS_VERBOSE
    printf("%d features sent\n", svs_data->no_of_features);
#endif
}

/* left camera board receives feature data from the right camera board */
int svs_receive_features()
{
#ifdef SVS_PROFILE
    int t[3];
#endif
    unsigned short crc, crc_received;
    int features_received = 0;

#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif
    svs_master(
        (unsigned short *)FLASH_BUFFER,
        (unsigned short *)(FLASH_BUFFER+16384),
        16384);
#ifdef SVS_PROFILE
    t[1] = readRTC();
#endif
    svs_data_received = (svs_data_struct*)(FLASH_BUFFER+16384);

    if (svs_data_received->received_bit != 0) {

        /* run checksum */
        crc_received = svs_data_received->crc;
        svs_data_received->crc = 0;
        crc = (unsigned short)crc16_ccitt((void *)svs_data_received, sizeof(svs_data_received));

#ifdef SVS_PROFILE
        t[2] = readRTC();
#endif
        if (crc == crc_received) {
            features_received = svs_data_received->no_of_features;
        }
        else {
            printf("Checksum invalid\n");
            features_received = -2;
        }
    }
    else {
#ifdef SVS_PROFILE
        t[2] = 0;
#endif
        printf("Received bit not set\n");
        features_received = -1;
    }

#ifdef SVS_VERBOSE
    printf("%d features received\n", svs_data_received->no_of_features);
#endif
#ifdef SVS_PROFILE
    printf("svs_receive_features:\n\r");
    printf("    svs_master  %d\n\r", t[1]-t[0]);
    printf("    crc16_ccitt %d\n\r", t[2]-t[1]);
#endif

    return(features_received);
}

/* reads camera calibration parameters from flash sector 03 */
int svs_read_calib_params()
{
    int coeff, is_valid=1;
    read_user_sector(SVS_SECTOR);
    int* buf = (int *)FLASH_BUFFER;
    svs_calibration_offset_x = buf[0];
    svs_calibration_offset_y = buf[1];
    svs_centre_of_disortion_x = imgWidth / 2;
    svs_centre_of_disortion_y = (int)imgHeight / 2;
    if ((buf[2] > 0) && (buf[3] > 0) &&
            (buf[2] < 3000) && (buf[3] < 3000)) {
        svs_centre_of_disortion_x = buf[2];
        svs_centre_of_disortion_y = buf[3];
    }
    else {
        is_valid = 0;
    }
    svs_scale_num = 1;
    svs_scale_denom = 1;
    svs_coeff[0] = 0;
    svs_width = 320;
    svs_height = 240;
    if ((buf[4] > 0) && (buf[5] > 0)) {
        svs_scale_num = buf[4];
        svs_scale_denom = buf[5];
        svs_coeff_degree = buf[6];
        if ((svs_coeff_degree > 5) || (svs_coeff_degree < 3)) svs_coeff_degree = 3;
        for (coeff = 1; coeff <= svs_coeff_degree; coeff++) {
            svs_coeff[coeff] = buf[6+coeff];
        }
        if ((buf[7+svs_coeff_degree] > 100) &&
                (buf[7+svs_coeff_degree] <= SVS_MAX_IMAGE_WIDTH)) {
            svs_width = buf[7+svs_coeff_degree];
            svs_height = buf[8+svs_coeff_degree];
        }
    }
    else {
        is_valid = 0;
        svs_coeff_degree = 3;
        svs_coeff[1] = 10954961;
        svs_coeff[2] = -10710;
        svs_coeff[3] = -6;
    }

#ifdef SVS_VERBOSE
    if (is_valid != 0) {
        printf("offset: %d %d\n", svs_calibration_offset_x, svs_calibration_offset_y);
        printf("centre: %d %d\n", svs_centre_of_disortion_x, svs_centre_of_disortion_y);
        printf("scale:  %d/%d\n", svs_scale_num, svs_scale_denom);
        printf("coefficients:  ");
        for (coeff = 1; coeff <= svs_coeff_degree; coeff++) {
            printf("%d ", svs_coeff[coeff]);
        }
        printf("\n");
    }
    else {
        printf("Calibration parameters were not found\n");
        int i;
        for (i = 0; i < 9; i++) {
            printf("buffer %d: %d\n", i, buf[i]);
        }
    }
#endif

    /* create the calibration map using the parameters */
    svs_make_map(
        (long)svs_centre_of_disortion_x,
        (long)svs_centre_of_disortion_y,
        svs_coeff,
        svs_coeff_degree,
        svs_scale_num, svs_scale_denom);

    return(is_valid);
}


/* updates a set of features suitable for stereo matching */
int svs_grab(
    int calibration_offset_x,  /* calibration x offset in pixels */
    int calibration_offset_y,  /* calibration y offset in pixels */
    int left_camera,           /* if non-zero this is the left camera */
    int show_feats)            /* if non-zero then show features */
{
#ifdef SVS_PROFILE
    int t[8];
#endif
    int no_of_feats;
    /* some default values */
    const int inhibition_radius = 16;
    const unsigned int minimum_response = 300;

#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif

    /* clear received data bit */
    svs_data->received_bit = 0;

    /* grab new frame */
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, (unsigned char *)FRAME_BUF, imgWidth, imgHeight);

#ifdef SVS_PROFILE
    t[1] = readRTC();
#endif

    /* if the image resolution has changed recompute the calibration map */
    if (svs_prev_width != svs_width) {
        svs_read_calib_params();
        svs_prev_width = svs_width;
    }

    /* rectify the image */
    svs_rectify((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3);

#ifdef SVS_PROFILE
    t[2] = readRTC();
#endif

    /* copy rectified back to the original */
    memcpy2((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, imgWidth * imgHeight * 4);

#ifdef SVS_PROFILE
    t[3] = readRTC();
#endif

    /* convert to YUV */
    move_yuv422_to_planar((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, imgWidth, imgHeight);

#ifdef SVS_PROFILE
    t[4] = readRTC();
#endif

    /* compute edge features */
    no_of_feats = svs_get_features_vertical((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response, calibration_offset_x, calibration_offset_y, left_camera);

#ifdef SVS_PROFILE
    t[5] = readRTC();
    t[6] = 0;
#endif

    if ((left_camera != 0) && (svs_enable_horizontal)) {
        svs_get_features_horizontal((unsigned char *)FRAME_BUF3, inhibition_radius, minimum_response,     calibration_offset_x, calibration_offset_y, left_camera);
#ifdef SVS_PROFILE
        t[6] = readRTC();
#endif
    }

#ifdef SVS_PROFILE
    t[7] = 0;
#endif
    if (show_feats != 0) {
        /* display the features */
        svs_show_features((unsigned char *)FRAME_BUF, no_of_feats);
#ifdef SVS_PROFILE
        t[7] = readRTC();
#endif
    }

#ifdef SVS_PROFILE
    printf("svs_grab:\n\r");
    printf("    move_image                  %d\n\r", t[1]-t[0]);
    printf("    svs_rectify                 %d\n\r", t[2]-t[1]);
    printf("    memcpy2                      %d\n\r", t[3]-t[2]);
    printf("    move_yuv422_to_planar       %d\n\r", t[4]-t[3]);
    printf("    svs_get_features_vertical   %d\n\r", t[5]-t[4]);
    printf("    svs_get_features_horizontal %d\n\r", t[6]-t[5]);
    if (show_feats != 0) printf("    svs_show_features           %d\n\r", t[7]-t[6]);
#endif
    return(no_of_feats);
}

/* Match features from this camera with features from the opposite one.
 * It is assumed that matching is performed on the left camera CPU */
int svs_match(
    int ideal_no_of_matches,          /* ideal number of matches to be returned */
    int max_disparity_percent,        /* max disparity as a percent of image width */
    int learnDesc,                    /* descriptor match weight */
    int learnLuma,                    /* luminance match weight */
    int learnDisp,                    /* disparity weight */
    int learnGrad,                    /* horizontal gradient weight */
    int groundPrior,                  /* prior weight for ground plane */
    int use_priors)                   /* if non-zero then use priors, assuming time between frames is small */
{
    int x, xL=0, xR, L, R, y, no_of_feats=0, no_of_feats_left;
    int no_of_feats_right, row, col, bit, disp_diff=0;
    int luma_diff, min_disp=0, max_disp=0, max_disp_pixels;
    int meanL, meanR, disp, fL=0, fR=0, bestR=0;
    unsigned int descL, descLanti, descR, desc_match;
    unsigned int correlation, anticorrelation, total, n;
    unsigned int match_prob, best_prob;
    int max, curr_idx, search_idx, winner_idx=0;
    int no_of_possible_matches = 0, matches = 0;
    int itt, idx, prev_matches;
	int grad_diff0, gradL0, grad_diff1=0, gradL1=0, grad_anti;
    int p, pmax=3, prev_matches_idx, prev_right_x, right_x;
	int min_left, min_right, disp_left=0, disp_left_x=0;
	int disp_right=0, disp_right_x=0, dx, dy, x2, dist;

    unsigned int meandescL, meandescR;
    short meandesc[SVS_DESCRIPTOR_PIXELS];

    /* initialise arrays used for matching
     * note that we only need to do this on the left camera */
    if (svs_matches == 0) init_svs_matching_data();

    /* convert max disparity from percent to pixels */
    max_disp_pixels = max_disparity_percent * imgWidth / 100;
    min_disp = -10;
    max_disp = max_disp_pixels;

#ifdef SVS_FOOTLINE
    /* fit lines to the footline points */
    svs_footline_slope();
#endif

    /* find ground plane */
    int ground_prior=0;
    int ground_y = (int)imgHeight * svs_ground_y_percent/100;
    int footline_y=0;
    int ground_y_sloped=0, ground_height_sloped=0;
    int obstaclePrior = 20;
    int half_width = (int)imgWidth/2;
    svs_ground_plane();

    row = 0;
    for (y = 4; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING, row++)
    {

        /* number of features on left and right rows */
        no_of_feats_left = svs_data->features_per_row[row];
        no_of_feats_right = svs_data_received->features_per_row[row];

        /* compute mean descriptor for the left row
         * this will be used to create eigendescriptors */
        meandescL = 0;
        memset((void *)&meandesc, '\0', (SVS_DESCRIPTOR_PIXELS)* sizeof(short));
        for (L = 0; L < no_of_feats_left; L++)
        {
            descL = svs_data->descriptor[fL + L];
            n = 1;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
            {
                if (descL & n)
                    meandesc[bit]++;
                else
                    meandesc[bit]--;
            }
        }
        n = 1;
        for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
        {
            if (meandesc[bit] >= 0)
                meandescL |= n;
        }

        /* compute mean descriptor for the right row
         * this will be used to create eigendescriptors */
        meandescR = 0;
        memset((void *)&meandesc, '\0', (SVS_DESCRIPTOR_PIXELS)* sizeof(short));
        for (R = 0; R < no_of_feats_right; R++)
        {
            descR = svs_data_received->descriptor[fR + R];
            n = 1;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
            {
                if (descR & n)
                    meandesc[bit]++;
                else
                    meandesc[bit]--;
            }
        }
        n = 1;
        for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++, n *= 2)
        {
            if (meandesc[bit] > 0)
                meandescR |= n;
        }

        prev_matches_idx = no_of_possible_matches;

        /* features along the row in the left camera */
        for (L = 0; L < no_of_feats_left; L++)
        {

            /* x coordinate of the feature in the left camera */
            xL = svs_data->feature_x[fL + L];

            if (use_priors != 0) {
                if (svs_enable_ground_priors) {
                    /* Here svs_ground_slope_percent is a simple way of representing the
                       roll angle of the robot, without using any floating point maths.
                       Positive slope corresponds to a positive (clockwise) roll angle.
                       The slope is the number of pixels of vertical displacement of
                       the horizon at the far right of the image, relative to the centre
                       of the image, and is expressed as a percentage of the image height */
                    ground_y_sloped = ground_y + ((half_width - xL) * svs_ground_slope_percent / 100);
                    ground_height_sloped = (int)imgHeight - 1 - ground_y_sloped;
                    footline_y = footline[0][xL / SVS_HORIZONTAL_SAMPLING];
                    ground_prior = (footline_y - ground_y_sloped) * max_disp_pixels / ground_height_sloped;
                }
            }

            /* mean luminance and eigendescriptor for the left camera feature */
            meanL = svs_data->mean[fL + L] & 15;
            descL = svs_data->descriptor[fL + L] & meandescL;

            /* invert bits of the descriptor for anti-correlation matching */
            n = descL;
            descLanti = 0;
            for (bit = 0; bit < SVS_DESCRIPTOR_PIXELS; bit++)
            {
                /* Shift result vector to higher significance. */
                descLanti <<= 1;
                /* Get least significant input bit. */
                descLanti |= n & 1;
                /* Shift input vector to lower significance. */
                n >>= 1;
            }

            total = 0;

			gradL0 = svs_data->mean[fL + L] >> 4;
			if (L < no_of_feats_left - 1) {
			    gradL1 = svs_data->mean[fL + L + 1] >> 4;
			}			

            /* features along the row in the right camera */
            for (R = 0; R < no_of_feats_right; R++)
            {
                /* set matching score to zero */
                row_peaks[R] = 0;

				if (((gradL0 >= 8) &&
					((int)(svs_data_received->mean[fR + R] >> 4) >= 8)) ||
					((gradL0 < 8) &&
					((int)(svs_data_received->mean[fR + R] >> 4) < 8))) {

                    /* x coordinate of the feature in the right camera */
                    xR = svs_data_received->feature_x[fR + R];

                    /* compute disparity */
                    disp = xL - xR;

                    /* is the disparity within range? */
                    if ((disp >= min_disp) && (disp < max_disp))
                    {
                        if (disp < 0)
                            disp = 0;

                        /* mean luminance for the right camera feature */
                        meanR = svs_data_received->mean[fR + R] & 15;

                        /* is the mean luminance similar? */
                        luma_diff = meanR - meanL;

                        /* right camera feature eigendescriptor */
                        descR = svs_data_received->descriptor[fR + R] & meandescR;

                        /* bitwise descriptor correlation match */
                        desc_match = descL & descR;

                        /* count the number of correlation bits */
                        correlation =
                            BitsSetTable256[desc_match & 0xff] +
                            BitsSetTable256[(desc_match >> 8) & 0xff] +
                            BitsSetTable256[(desc_match >> 16) & 0xff] +
                            BitsSetTable256[desc_match >> 24];

                        /* bitwise descriptor anti-correlation match */
                        desc_match = descLanti & descR;

                        /* count the number of anti-correlation bits */
                        anticorrelation =
                            BitsSetTable256[desc_match & 0xff] +
                            BitsSetTable256[(desc_match >> 8) & 0xff] +
                            BitsSetTable256[(desc_match >> 16) & 0xff] +
                            BitsSetTable256[desc_match >> 24];

    					grad_anti = (15 - gradL0) - (int)(svs_data_received->mean[fR + R] >> 4);
    					if (grad_anti < 0) grad_anti = -grad_anti;

    					grad_diff0 = gradL0 - (int)(svs_data_received->mean[fR + R] >> 4);
    					if (grad_diff0 < 0) grad_diff0 = -grad_diff0;
    					grad_diff0 = (15 - grad_diff0) - (15 - grad_anti);

    					if (R < no_of_feats_right - 1) {
    						grad_anti = (15 - gradL1) - (int)(svs_data_received->mean[fR + R + 1] >> 4);
    						if (grad_anti < 0) grad_anti = -grad_anti;

    						grad_diff1 = gradL1 - (int)(svs_data_received->mean[fR + R + 1] >> 4);
    					    if (grad_diff1 < 0) grad_diff1 = -grad_diff1;
    					    grad_diff1 = (15 - grad_diff1) - (15 - grad_anti);
    					}

                        if (luma_diff < 0)
                            luma_diff = -luma_diff;
                        int score =
                            10000 +
                            (max_disp * learnDisp) +
    						(grad_diff0 * learnGrad) +
    						(grad_diff1 * learnGrad) +
                            (((int)correlation + (int)(SVS_DESCRIPTOR_PIXELS - anticorrelation)) * learnDesc) -
                            (luma_diff * learnLuma) -
                            (disp * learnDisp);

                        if (use_priors) {
                            /* bias for ground_plane */
                            if (svs_enable_ground_priors) {
                                if (y > footline_y) {
                                    /* below the footline - bias towards ground plane */
                                    disp_diff = disp - ((y - ground_y_sloped) * max_disp_pixels / ground_height_sloped);
                                    if (disp_diff < 0) disp_diff = -disp_diff;
                                    score -= disp_diff * groundPrior;
                                }
                                else {
                                    if (ground_prior > 0) {
                                        /* above the footline - bias towards obstacle*/
                                        disp_diff = disp - ground_prior;
                                        if (disp_diff < 0) disp_diff = -disp_diff;
                                        score -= disp_diff * obstaclePrior;
                                    }
                                }
                            }
                        }

                        if (score < 0)
                            score = 0;

                        /* store overall matching score */
                        row_peaks[R] = (unsigned int)score;
                        total += row_peaks[R];
                    
                    }
                    else
                    {
                        if ((disp < min_disp) && (disp > -max_disp))
                        {
                            row_peaks[R] = (unsigned int)((max_disp - disp) * learnDisp);
                            total += row_peaks[R];
                        }
                    }
                }
            }

            /* non-zero total matching score */
            if (total > 0)
            {

				/* several candidate disparities per feature
				   observing the principle of least commitment */
				for (p = 0; p < pmax; p++) {
				
                    /* convert matching scores to probabilities */
                    best_prob = 0;
                    for (R = 0; R < no_of_feats_right; R++)
                    {
                        if (row_peaks[R] > 0)
                        {
                            match_prob = row_peaks[R] * 1000 / total;
                            if (match_prob > best_prob)
                            {
                                best_prob = match_prob;
                                bestR = R;
                            }
                        }
                    }

                    if ((best_prob > 0) &&
                        (best_prob < 1000) &&
                        (no_of_possible_matches < SVS_MAX_FEATURES))
                    {

                        /* x coordinate of the feature in the right camera */
                        xR = svs_data_received->feature_x[fR + bestR];

                        /* possible disparity */
                        disp = xL - xR;

                        if (disp >= min_disp)
                        {
                            if (disp < 0)
                                disp = 0;
                            /* add the best result to the list of possible matches */
                            svs_matches[no_of_possible_matches*4] = best_prob;
                            svs_matches[no_of_possible_matches*4 + 1] = (unsigned int)xL;
                            svs_matches[no_of_possible_matches*4 + 2] = (unsigned int)y;
                            svs_matches[no_of_possible_matches*4 + 3] = (unsigned int)disp;
							if (p > 0) {
								svs_matches[no_of_possible_matches * 4 + 1] += imgWidth;
							}
							no_of_possible_matches++;
							row_peaks[bestR] = 0;
                        }
                    }
                
                }
            }
        }

		/* apply ordering constraint within the right image */
		prev_right_x = 0;
		for (idx = no_of_possible_matches-1; idx >= prev_matches_idx; idx--) {
			right_x = (int)svs_matches[idx*5 + 1] + (int)svs_matches[idx*5 + 3];
			if (right_x < prev_right_x) {
			    /* set probability to zero if rays cross (occlusion) */
				if (svs_matches[(idx+1) * 5] >= svs_matches[idx * 5]) {
				    svs_matches[idx * 5] = 0;
				}
				else {
					svs_matches[(idx+1) * 5] = 0;
					prev_right_x = right_x;
				}
			}
			else {
			    prev_right_x = right_x;
			}
		}

        /* increment feature indexes */
        fL += no_of_feats_left;
        fR += no_of_feats_right;
    }

    if (no_of_possible_matches > 20)
    {
        /* filter the results */
        svs_filter_plane(no_of_possible_matches, max_disp);

        /* sort matches in descending order of probability */
        if (no_of_possible_matches < ideal_no_of_matches)
        {
            ideal_no_of_matches = no_of_possible_matches;
        }
        curr_idx = 0;
        search_idx = 0;
        for (matches = 0; matches < ideal_no_of_matches; matches++, curr_idx += 4)
        {

            match_prob =  svs_matches[curr_idx];
            if (match_prob > 0) 
                winner_idx = curr_idx;
            else
                winner_idx = -1;

            search_idx = curr_idx + 4;
            max = no_of_possible_matches * 4;
            while (search_idx < max)
            {
                if (svs_matches[search_idx] > match_prob)
                {
                    match_prob = svs_matches[search_idx];
                    winner_idx = search_idx;
                }
                search_idx += 4;
            }
            if (winner_idx > -1)
            {

                /* swap */
                best_prob = svs_matches[winner_idx];
                xL = svs_matches[winner_idx + 1];
                y = svs_matches[winner_idx + 2];
                disp = svs_matches[winner_idx + 3];

                svs_matches[winner_idx] = svs_matches[curr_idx];
                svs_matches[winner_idx + 1] = svs_matches[curr_idx + 1];
                svs_matches[winner_idx + 2] = svs_matches[curr_idx + 2];
                svs_matches[winner_idx + 3] = svs_matches[curr_idx + 3];

                svs_matches[curr_idx] = best_prob;
                svs_matches[curr_idx + 1] = xL;
                svs_matches[curr_idx + 2] = y;
                svs_matches[curr_idx + 3] = disp;

                /* update your priors */
				if (svs_matches[winner_idx + 1] >= imgWidth) {
					svs_matches[winner_idx + 1] -= imgWidth;
				}
            }

            if (svs_matches[curr_idx] == 0)
            {
                break;
            }
        }

        /* attempt to assign disparities to horizontally oriented features */
        if ((svs_enable_horizontal != 0) &&
                (features_per_col != 0)) {
            memset((void*)valid_quadrants, '\0', SVS_MAX_MATCHES * sizeof(unsigned char));
            itt = 0;
            prev_matches = matches;
            for (itt = 0; itt < 10; itt++) {
                fL = 0;
                col = 0;
                for (x = 4; x < (int)imgWidth - 4; x += SVS_HORIZONTAL_SAMPLING, col++) {

                    no_of_feats = features_per_col[col];

                    // features along the row in the left camera
                    for (L = 0; L < no_of_feats; L++) {

                        if (valid_quadrants[fL + L] == 0) {
                            // y coordinate of the feature in the left camera
                            y = feature_y[fL + L];
                            row = y / SVS_VERTICAL_SAMPLING;

    						min_left = 99999;
    						min_right = 99999;
    						for (idx = prev_matches-1; idx >= 0; idx--) {
    							if (svs_matches[idx*4] > 0) {
    								x2 = (int)svs_matches[idx*4+1];
    								dx = x - x2;
    								dy = y - (int)svs_matches[idx*4+2];
                                    if ((dy > -5) && (dy < 5)) {
        								if (dy < 0)
        									dist = -dy;
        								else
        									dist = dy;

        								if (dx > 0) {
        									dist += dx;
        									if ((dist < min_left) &&
        										((int)svs_matches[idx*4+3] < max_disp_pixels)) {
        										min_left = dist;
        										disp_left = (int)svs_matches[idx*4+3];
        										disp_left_x = x2;
        									}
        								}
        								else {
        									dist += -dx;
        									if ((dist < min_right) &&
        										((int)svs_matches[idx*4+3] < max_disp_pixels)) {
        										min_right = dist;
        										disp_right = (int)svs_matches[idx*4+3];
        										disp_right_x = x2;
        									}
        								}
                                    }
    							}
    						}

    						if ((min_left != 99999) &&
    							(min_right != 99999)) {

    							disp =
    								disp_left + ((x - disp_left_x) * (disp_right - disp_left) /
    							    (disp_right_x - disp_left_x));

                                if ((disp > 0) &&
                                        (matches < SVS_MAX_MATCHES)) {
                                    curr_idx = matches * 4;
                                    svs_matches[curr_idx] = 500;
                                    svs_matches[curr_idx + 1] = x;
                                    svs_matches[curr_idx + 2] = y;
                                    svs_matches[curr_idx + 3] = disp;
                                    matches++;

                                    valid_quadrants[fL + L] = 1;
                                }
                            }
                        }
                    }
                    fL += no_of_feats;
                }
                if (prev_matches == matches) break;
                prev_matches = matches;
            }
        }
    }

#ifdef SVS_VERBOSE
    printf("%d stereo matches\n", matches);
#endif

    return(matches);
}

/* filtering function removes noise by fitting planes to the disparities
   and disguarding outliers */
void svs_filter_plane(
    int no_of_possible_matches, /* the number of stereo matches */
    int max_disparity_pixels) /*maximum disparity in pixels */
{
    int i, hf, hist_max, w = SVS_FILTER_SAMPLING, w2, n, horizontal = 0;
    unsigned int x, y, disp, tx = 0, ty = 0, bx = 0, by = 0;
    int hist_thresh, hist_mean, hist_mean_hits, mass, disp2;
    int min_ww, max_ww, m, ww, d;
    int ww0, ww1, disp0, disp1, cww, dww, ddisp;
    no_of_planes = 0;

    /* clear quadrants */
    memset(valid_quadrants, 0, no_of_possible_matches * sizeof(unsigned char));

    /* create disparity histograms within different
     * zones of the image */
    for (hf = 0; hf < 11; hf++) {

        switch (hf) {

            // overall horizontal
        case 0: {
            tx = 0;
            ty = 0;
            bx = imgWidth;
            by = imgHeight;
            horizontal = 1;
            w = bx;
            break;
        }
        // overall vertical
        case 1: {
            tx = 0;
            ty = 0;
            bx = imgWidth;
            by = imgHeight;
            horizontal = 0;
            w = by;
            break;
        }
        // left hemifield 1
        case 2: {
            tx = 0;
            ty = 0;
            bx = imgWidth / 3;
            by = imgHeight;
            horizontal = 1;
            w = bx;
            break;
        }
        // left hemifield 2
        case 3: {
            tx = 0;
            ty = 0;
            bx = imgWidth / 2;
            by = imgHeight;
            horizontal = 1;
            w = bx;
            break;
        }

        // centre field (vertical)
        case 4: {
            tx = imgWidth / 3;
            bx = imgWidth * 2 / 3;
            w = imgHeight;
            horizontal = 0;
            break;
        }
        // centre above field
        case 5: {
            tx = imgWidth / 3;
            ty = 0;
            bx = imgWidth * 2 / 3;
            by = imgHeight / 2;
            w = by;
            horizontal = 0;
            break;
        }
        // centre below field
        case 6: {
            tx = imgWidth / 3;
            ty = imgHeight / 2;
            bx = imgWidth * 2 / 3;
            by = imgHeight;
            w = ty;
            horizontal = 0;
            break;
        }
        // right hemifield 0
        case 7: {
            tx = imgWidth * 2 / 3;
            ty = 0;
            bx = imgWidth;
            by = imgHeight;
            horizontal = 1;
            break;
        }
        // right hemifield 1
        case 8: {
            tx = imgWidth / 2;
            bx = imgWidth;
            w = tx;
            break;
        }
        // upper hemifield
        case 9: {
            tx = 0;
            ty = 0;
            bx = imgWidth;
            by = imgHeight / 2;
            horizontal = 0;
            w = by;
            break;
        }
        // lower hemifield
        case 10: {
            ty = by;
            by = imgHeight;
            horizontal = 0;
            break;
        }
        }

        /* clear the histogram */
        w2 = w / SVS_FILTER_SAMPLING;
        if (w2 < 1)
            w2 = 1;
        memset((void*) disparity_histogram_plane, '\0', w2
               * max_disparity_pixels * sizeof(int));
        memset((void*) disparity_plane_fit, '\0', w2 * sizeof(int));
        hist_max = 0;

        /* update the disparity histogram */
        n = 0;
        for (i = no_of_possible_matches-1; i >= 0; i--) {
            x = svs_matches[i * 4 + 1];
            if ((x > tx) && (x < bx)) {
                y = svs_matches[i * 4 + 2];
                if ((y > ty) && (y < by)) {
                    disp = svs_matches[i * 4 + 3];
                    if ((int) disp < max_disparity_pixels) {
                        if (horizontal != 0) {
                            n = (((x - tx) / SVS_FILTER_SAMPLING)
                                 * max_disparity_pixels) + disp;
                        } else {
                            n = (((y - ty) / SVS_FILTER_SAMPLING)
                                 * max_disparity_pixels) + disp;
                        }
                        disparity_histogram_plane[n]++;
                        if (disparity_histogram_plane[n] > hist_max)
                            hist_max = disparity_histogram_plane[n];
                    }
                }
            }
        }

        /* find peak disparities along a range of positions */
        hist_thresh = hist_max / 4;
        hist_mean = 0;
        hist_mean_hits = 0;
        disp2 = 0;
        min_ww = w2;
        max_ww = 0;
        for (ww = 0; ww < (int) w2; ww++) {
            mass = 0;
            disp2 = 0;
            for (d = 1; d < max_disparity_pixels - 1; d++) {
                n = ww * max_disparity_pixels + d;
                if (disparity_histogram_plane[n] > hist_thresh) {
                    m = disparity_histogram_plane[n]
                        + disparity_histogram_plane[n - 1]
                        + disparity_histogram_plane[n + 1];
                    mass += m;
                    disp2 += m * d;
                }
                if (disparity_histogram_plane[n] > 0) {
                    hist_mean += disparity_histogram_plane[n];
                    hist_mean_hits++;
                }
            }
            if (mass > 0) {
                // peak disparity at this position
                disparity_plane_fit[ww] = disp2 / mass;
                if (min_ww == (int) w2)
                    min_ww = ww;
                if (ww > max_ww)
                    max_ww = ww;
            }
        }
        if (hist_mean_hits > 0)
            hist_mean /= hist_mean_hits;

        /* fit a line to the disparity values */
        ww0 = 0;
        ww1 = 0;
        disp0 = 0;
        disp1 = 0;
        int hits0,hits1;
        if (max_ww >= min_ww) {
            cww = min_ww + ((max_ww - min_ww) / 2);
            hits0 = 0;
            hits1 = 0;
            for (ww = min_ww; ww <= max_ww; ww++) {
                if (ww < cww) {
                    disp0 += disparity_plane_fit[ww];
                    ww0 += ww;
                    hits0++;
                } else {
                    disp1 += disparity_plane_fit[ww];
                    ww1 += ww;
                    hits1++;
                }
            }
            if (hits0 > 0) {
                disp0 /= hits0;
                ww0 /= hits0;
            }
            if (hits1 > 0) {
                disp1 /= hits1;
                ww1 /= hits1;
            }
        }
        dww = ww1 - ww0;
        ddisp = disp1 - disp0;

        /* find inliers */
        int plane_tx = imgWidth;
        int plane_ty = 0;
        int plane_bx = imgHeight;
        int plane_by = 0;
        int plane_disp0 = 0;
        int plane_disp1 = 0;
        int hits = 0;
        for (i = no_of_possible_matches-1; i >= 0; i--) {
            x = svs_matches[i * 4 + 1];
            if ((x > tx) && (x < bx)) {
                y = svs_matches[i * 4 + 2];
                if ((y > ty) && (y < by)) {
                    disp = svs_matches[i * 4 + 3];

                    if (horizontal != 0) {
                        ww = (x - tx) / SVS_FILTER_SAMPLING;
                        n = ww * max_disparity_pixels + disp;
                    } else {
                        ww = (y - ty) / SVS_FILTER_SAMPLING;
                        n = ww * max_disparity_pixels + disp;
                    }

                    if (dww > 0) {
                        disp2 = disp0 + ((ww - ww0) * ddisp / dww);
                    }
                    else {
                        disp2 = disp0;
                    }

                    /* check how far this is from the plane */
                    if (((int)disp >= disp2-2) &&
                            ((int)disp <= disp2+2) &&
                            ((int)disp < max_disparity_pixels)) {

                        /* inlier detected - this disparity lies along the plane */
                        valid_quadrants[i]++;
                        hits++;
                        
                        /* keep note of the bounds of the plane */
                        if (x < plane_tx) {
                            plane_tx = x;
                            if (horizontal == 1) plane_disp0 = disp2;
                        }
                        if (y < plane_ty) {
                            plane_ty = y;
                            if (horizontal == 0) plane_disp0 = disp2;
                        }
                        if (x > plane_bx) {
                            plane_bx = x;
                            if (horizontal == 1) plane_disp1 = disp2;
                        }
                        if (y > plane_by) {
                            plane_by = y;
                            if (horizontal == 0) plane_disp1 = disp2;
                        }
                    }
                }
            }
        }
        if (hits > 5) {
            /* add a detected plane */
            plane[no_of_planes*9+0]=plane_tx;
            plane[no_of_planes*9+1]=plane_ty;
            plane[no_of_planes*9+2]=plane_bx;
            plane[no_of_planes*9+3]=plane_by;
            plane[no_of_planes*9+4]=horizontal;
            plane[no_of_planes*9+5]=plane_disp0;
            plane[no_of_planes*9+6]=plane_disp1;
            plane[no_of_planes*9+7]=hits;
            plane[no_of_planes*9+8]=0;
            no_of_planes++;
        }
    }

    /* deal with the outliers */
    for (i = no_of_possible_matches-1; i >= 0; i--) {
        if (valid_quadrants[i] == 0) {
        
            /* by default set outlier probability to zero,
               which eliminates it from further enquiries */
            svs_matches[i * 4] = 0;
        }
    }
}

/* takes the raw image and returns a rectified version */
void svs_rectify(
    unsigned char* raw_image,     /* raw image grabbed from camera (4 bytes per pixel) */
    unsigned char* rectified_frame_buf) /* returned rectified image (4 bytes per pixel) */
{
    int i, max, idx0, idx1;

    if (calibration_map != 0) {

        max = (int)(imgWidth * imgHeight * 2)-2;
        for (i = max; i >= 0; i-=2) {
            idx0 = i & 0xFFFFFFFC;
            idx1 = calibration_map[idx0];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0++] = raw_image[idx1++];
            rectified_frame_buf[idx0] = raw_image[idx1];
        }

    }
    else {
        printf("No calibration map has been defined\n");
    }
}

/* initialises arrays */
void init_svs() {
    calibration_map = (int *)malloc(SVS_MAX_IMAGE_WIDTH*SVS_MAX_IMAGE_HEIGHT*4);
    row_peaks = (unsigned int *)malloc(SVS_MAX_IMAGE_WIDTH*4);
    row_sum = (int*)malloc(SVS_MAX_IMAGE_WIDTH*4);
    plane = (int*)malloc(11*9*4);
    svs_data = (svs_data_struct *)malloc(sizeof(svs_data_struct));
    svs_matches = 0;
    features_per_col = 0;
    feature_y = 0;
    svs_enable_horizontal = 1;

    svs_coeff = (long*)malloc(8 * sizeof(long));
    svs_read_calib_params();
    svs_prev_width = svs_width;
    
#ifdef SVS_FOOTLINE    
    int i;
    footline_slope = (unsigned short int**)malloc(3*sizeof(unsigned short int*));
    for (i = 0; i < 3; i++) {
        footline_slope[i] = (unsigned short int*)malloc(5*4);
    }
#endif
}

/* initialises arrays associated with matching
 * This only needs to be done on the left camera */
void init_svs_matching_data() {
    /* size of the image on the CCD/CMOS sensor
       OV9655 4.17mm
       OV7725 (low light sensor) 3.98mm */
    svs_sensor_width_mmx100 = 417; // TODO possible autodetect
    svs_ground_y_percent = 50; // ground plane as a percent of the image height
    svs_enable_ground_priors = 1;
    svs_ground_slope_percent = 0;
    svs_right_turn_percent = 0;
    svs_turn_tollerance_percent = 10;
#ifdef SVS_MAPPING_BY_DEFAULT
    svs_enable_mapping = 1;
#else
    svs_enable_mapping = 0;
#endif
    prev_svs_enable_mapping = 0;

    svs_data_previous = (svs_data_struct *)malloc(sizeof(svs_data_struct));
    memset((void*)svs_data_previous->features_per_row, '\0', SVS_MAX_IMAGE_HEIGHT/SVS_VERTICAL_SAMPLING*sizeof(unsigned short int));

    footline_dist_mm = (unsigned short int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));
    memset((void*)footline_dist_mm, '\0', SVS_MAX_IMAGE_WIDTH / SVS_HORIZONTAL_SAMPLING * sizeof(unsigned short int));

    svs_matches = (unsigned int *)malloc(SVS_MAX_MATCHES*16);
    disparity_histogram_plane = (int*)malloc((SVS_MAX_IMAGE_WIDTH/SVS_FILTER_SAMPLING)*(SVS_MAX_IMAGE_WIDTH / 2)*sizeof(int));
    disparity_plane_fit = (int*)malloc(SVS_MAX_IMAGE_WIDTH / SVS_FILTER_SAMPLING * sizeof(int));
    valid_quadrants = (unsigned char*)malloc(SVS_MAX_MATCHES);

}

/* shows edge features */
void svs_show_features(
    unsigned char *outbuf,  /* output image.  Note that this is YUVY */
    int no_of_feats) {      /* number of detected features */

    int n, f, x, y, row = 0;
    int feats_remaining = svs_data->features_per_row[row];

    for (f = 0; f < no_of_feats; f++, feats_remaining--) {

        x = (int)svs_data->feature_x[f] - svs_calibration_offset_x;
        y = 4 + (row * SVS_VERTICAL_SAMPLING) + svs_calibration_offset_y;

        if ((y > 0) &&
                (y < (int)imgHeight-1)) {

            /* draw edge */
            n = pixindex(x, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x, y-1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x, y-2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x, y+1);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x, y+2);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x-1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x+1, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x-2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
            n = pixindex(x+2, y);
            outbuf[n++] = 84;
            outbuf[n++] = 72;
            outbuf[n] = 255;
        }

        /* move to the next row */
        if (feats_remaining <= 0) {
            row++;
            feats_remaining = svs_data->features_per_row[row];
        }
    }
}

/* calculates a distance for each point on the footline */
void svs_footline_update(
    int max_disparity_percent)
{
    const int range_const = SVS_FOCAL_LENGTH_MMx100 * SVS_BASELINE_MM;
    int max = (int)imgWidth / (int)SVS_HORIZONTAL_SAMPLING;
    int i, x, y, disp, disp_mmx100, y_mm;
    int ground_y = (int)imgHeight * svs_ground_y_percent/100;
    int ground_y_sloped=0, ground_height_sloped=0;
    int half_width = (int)imgWidth/2;
    int max_disp_pixels = max_disparity_percent * (int)imgWidth / 100;
    int forward_movement_mm = 0;
    int forward_movement_hits = 0;

    for (i = 0; i < max; i++) {
        x = i * SVS_HORIZONTAL_SAMPLING;
        y = footline[0][i];
        ground_y_sloped = ground_y + ((half_width - x) * svs_ground_slope_percent / 100);
        ground_height_sloped = (int)imgHeight - 1 - ground_y_sloped;
        disp = (y - ground_y_sloped) * max_disp_pixels / ground_height_sloped;
        disp_mmx100 = disp * svs_sensor_width_mmx100 / (int)imgWidth;
        if (disp_mmx100 > 0) {
            // get position of the feature in space
            y_mm = range_const / disp_mmx100;

            if (footline_dist_mm[i] != 0) {
                forward_movement_mm += y_mm - footline_dist_mm[i];
                forward_movement_hits++;
            }
            footline_dist_mm[i] = y_mm;
        }
        else {
            footline_dist_mm[i] = 0;
        }
    }
    if (forward_movement_hits > 0) {
        forward_movement_mm /= forward_movement_hits;
        //printf("forward movement %d mm\n", forward_movement_mm);
    }
}

/* finds the ground plane */
void svs_ground_plane()
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    if (svs_enable_ground_priors) {
        int i, j, x, y, diff, prev_y = 0, prev_i=0;

        /* Remove points which have a large vertical difference.
           Successive points with small vertical difference are likely
           to correspond to real borders rather than carpet patterns, etc*/
        int max_diff = (int)imgHeight / 30;
        for (i = 0; i < (int)imgWidth / SVS_HORIZONTAL_SAMPLING; i++) {
            x = i * SVS_HORIZONTAL_SAMPLING;
            y = footline[0][i];
            if (y != 0) {
                if (prev_y != 0) {
                    diff = y - prev_y;
                    if (diff < 0) diff = -diff;
                    if (diff > max_diff) {
                        if (y < prev_y) {
                            footline[0][prev_i] = 0;
                        }
                        else {
                            footline[0][i] = 0;
                        }
                    }
                }
                prev_i = i;
                prev_y = y;
            }
        }

        /* fill in missing data to create a complete ground plane */
        prev_i = 0;
        prev_y = 0;
        int max = (int)imgWidth / SVS_HORIZONTAL_SAMPLING;
        for (i = 0; i < max; i++) {
            x = i * SVS_HORIZONTAL_SAMPLING;
            y = footline[0][i];
            if (y != 0) {
                if (prev_y == 0) prev_y = y;
                for (j = prev_i; j < i; j++) {
                    footline[0][j] = prev_y + ((j - prev_i) * (y - prev_y) / (i - prev_i));
                }
                prev_y = y;
                prev_i = i;
            }
        }
        for (j = prev_i; j < max; j++) {
            footline[0][j] = prev_y;
        }
    }
#ifdef SVS_PROFILE
    printf("svs_ground_plane %d mS\n", readRTC() - t);
#endif
}

#ifdef SVS_FOOTLINE
/* shows footline */
void svs_show_footline(
    unsigned char *outbuf)  /* output image.  Note that this is YUVY */
{
    int i, j, x, y, xx, yy, x2, y2, n;

    for (i = 0; i < 3; i++) {
        if (footline_slope[i][0] != (unsigned short int)9999) {
            x = (int)footline_slope[i][1];
            y = (int)footline_slope[i][2];
            x2 = (int)footline_slope[i][3];
            y2 = (int)footline_slope[i][4];
            
            //for (xx = x; xx < x2; xx++) {
            for (xx = 0; xx < (int)imgWidth; xx++) {
                yy = y + ((xx - x) * (y2 - y) / (x2 - x));
                n = pixindex(xx, yy);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(xx, yy-1);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
            }
        }
    }

    for (j = 0; j < 2; j++) {
        for (i = 0; i < (int)imgWidth / SVS_HORIZONTAL_SAMPLING; i++) {
            x = i * SVS_HORIZONTAL_SAMPLING;
            y = footline[j][i];
            if (y > 0) {
                /* draw edge */
                n = pixindex(x, y);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x, y-1);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x, y-2);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x, y+1);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x, y+2);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x-1, y);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x+1, y);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x-2, y);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
                n = pixindex(x+2, y);
                outbuf[n++] = 84;
                outbuf[n++] = 72;
                outbuf[n] = 255;
            }
        }
    }
}
#endif

/* shows stereo matches as blended spots */
void svs_show_matches(
    unsigned char *outbuf,  /* output image.  Note that this is YUVY */
    int no_of_matches) {    /* number of stereo matches */

    int x, y, xx, yy, i, dx, dy, dist_sqr, n;
    int radius, radius_sqr;

    for (i = 0; i < no_of_matches*4; i += 4) {
        if (svs_matches[i] > 0) {
            x = svs_matches[i + 1];
            y = svs_matches[i + 2];
            radius = 1 + (svs_matches[i + 3] / 6);

            if ((x - radius >= 0) &&
                    (x + radius < (int)imgWidth) &&
                    (y - radius >= 0) &&
                    (y + radius < (int)imgHeight)) {

                radius_sqr = radius*radius;

                for (yy = y - radius; yy <= y + radius; yy++) {
                    dy = yy - y;
                    for (xx = x - radius; xx <= x + radius; xx++, n += 3) {
                        dx = xx - x;
                        dist_sqr = dx*dx + dy*dy;
                        if (dist_sqr <= radius_sqr) {
                            n = pixindex(xx, yy);
                            outbuf[n] = (unsigned char)((outbuf[n] + 84) / 2);
                            outbuf[n+1] = (unsigned char)((outbuf[n + 1] + 72) / 2);
                            outbuf[n+2] = (unsigned char)((outbuf[n + 2] + 255) / 2);
                        }
                    }
                }
            }
        }
    }

#ifdef SVS_SHOW_STEERING
    int steer_y = (int)imgHeight*9/10;
    int steer_width = 10;
    int steer_length = 50;
    int half_width = (int)imgWidth/2;
    int w;
    switch (svs_steer)
    {
    case -1: { /* left */
        for (x = half_width-steer_length; x < half_width; x++) {
            w = (x - (half_width-steer_length)) * steer_width / steer_length;
            for (y = steer_y - w; y < steer_y + w; y++) {
                if (y < (int)imgHeight) {
                    n = pixindex(x, y);
                    outbuf[n] = (unsigned char)84;
                    outbuf[n+1] = (unsigned char)72;
                    outbuf[n+2] = (unsigned char)255;
                }
            }
        }
        break;
    }
    case 0: { /* ahead */
        for (y = steer_y - steer_length; y < steer_y; y++) {
            w = (y - (steer_y - steer_length)) * steer_width / steer_length;
            for (x = half_width-w; x < half_width+w; x++) {
                n = pixindex(x, y);
                outbuf[n] = (unsigned char)84;
                outbuf[n+1] = (unsigned char)72;
                outbuf[n+2] = (unsigned char)255;
            }
        }
        break;
    }
    case 1: { /* right */
        for (x = half_width; x < half_width+steer_length; x++) {
            w = steer_width - ((x - half_width) * steer_width / steer_length);
            for (y = steer_y - w; y < steer_y + w; y++) {
                if (y < (int)imgHeight) {
                    n = pixindex(x, y);
                    outbuf[n] = (unsigned char)84;
                    outbuf[n+1] = (unsigned char)72;
                    outbuf[n+2] = (unsigned char)255;
                }
            }
        }
        break;
    }
    }
#endif
}

/* computes the calibration map */
void svs_make_map(
    long centre_of_distortion_x, /* centre of distortion x coordinate in pixels xSVS_MULT */
    long centre_of_distortion_y, /* centre of distortion y coordinate in pixels xSVS_MULT */
    long* coeff,                 /* three lens distortion polynomial coefficients xSVS_MULT_COEFF */
    int degree,                  /* number of polynomial coefficients */
    long scale_num,              /* scaling numerator */
    long scale_denom) {          /* scaling denominator */

#ifdef SVS_VERBOSE
    printf("Computing calibration map...");
#endif

    long v, powr, radial_dist_rectified, radial_dist_original;
    long i, x, y, dx, dy;
    long n, n2, x2, y2;
    long ww = svs_width;
    long hh = svs_height;
    long half_width = ww / 2;
    long half_height = hh / 2;
    scale_denom *= SVS_MULT;
    for (x = 0; x < ww; x++) {

        dx = (x*SVS_MULT) - centre_of_distortion_x;

        for (y = 0; y < hh; y++) {

            dy = (y*SVS_MULT) - centre_of_distortion_y;

            v = dx*dx + dy*dy;
            /* integer square root */
            for (radial_dist_rectified=0;
                    v >= (2*radial_dist_rectified)+1;
                    v -= (2*radial_dist_rectified++)+1);

            if (radial_dist_rectified >= 0) {

                /* for each polynomial coefficient */
                radial_dist_original = 0;
                powr = 1;
                for (i = 0; i <= degree; i++, powr *= radial_dist_rectified) {
                    radial_dist_original += coeff[i] * powr;
                }

                if (radial_dist_original > 0) {

                    radial_dist_original /= SVS_MULT_COEFF;

                    x2 = centre_of_distortion_x + (dx * radial_dist_original / radial_dist_rectified);
                    x2 = (x2 - (half_width*SVS_MULT)) * scale_num / scale_denom;
                    y2 = centre_of_distortion_y + (dy * radial_dist_original / radial_dist_rectified);
                    y2 = (y2 - (half_height*SVS_MULT)) * scale_num / scale_denom;

                    x2 += half_width;
                    y2 += half_height;

                    if ((x2 > -1) && (x2 < ww) &&
                            (y2 > -1) && (y2 < hh)) {

                        if (ww != imgWidth) {
                            /* if the resolution at which the cameras were calibrated
                             * is different from the current resolution */
                            x = x*imgWidth / ww;
                            y = y*imgHeight / hh;
                            x2 = x2*imgWidth / ww;
                            y2 = y2*imgHeight / hh;
                        }
                        n = pixindex(x, y);
                        n2 = pixindex(x2, y2);

                        calibration_map[(int)n] = (int)n2;
                    }
                }
            }
        }
    }
#ifdef SVS_VERBOSE
    printf("Done\n");
#endif
}

/* send the disparity data */
void svs_send_disparities
(
    int no_of_matches) {
    unsigned char ch, *cp;
    unsigned short *disp;
    unsigned int i,n;

    if (no_of_matches > 0) {
        int disp_data_size = 4*sizeof(unsigned short);
        int data_size = disp_data_size * no_of_matches;
        char* dispHead = (char*)DISP_BUF;
        strcpy(dispHead, "##DSP");

        /* write header */
        for (i=0; i<5; i++) {
            while (*pPORTHIO & 0x0001)  // hardware flow control
                continue;
            putchar(dispHead[i]);
        }

        sprintf(dispHead, "%d    ", no_of_matches);

        /* write number of matches */
        for (i=0; i<4; i++) {
            while (*pPORTHIO & 0x0001)  // hardware flow control
                continue;
            putchar(dispHead[i]);
        }

        /* write the data */
        disp = (unsigned short *)DISP_BUF;
        n=0;
        for (i=0;i<no_of_matches;i++) {
            disp[n++] = (unsigned short)svs_matches[i*4];
            disp[n++] = (unsigned short)svs_matches[i*4+1];
            disp[n++] = (unsigned short)svs_matches[i*4+2];
            disp[n++] = (unsigned short)svs_matches[i*4+3];
        }

        cp = (unsigned char *)DISP_BUF;
        for (i=0; i<data_size; i++)
            putchar(*cp++);

        while (getchar(&ch)) {
            // flush input
            continue;
        }
    }
}

/* main stereo update */
void svs_stereo(int send_disparities)
{
#ifdef SVS_PROFILE
    int t[4];
#endif
    int matches = 0;
    *pPORTHIO |= 0x0100;  // set stereo sync flag high

#ifdef SVS_PROFILE
    t[0] = readRTC();
#endif
    svs_grab(svs_calibration_offset_x, svs_calibration_offset_y, master, 0);
    if (master) {
#ifdef SVS_PROFILE
        t[1] = readRTC();
#endif
        if (svs_receive_features() > -1) {
            int ideal_no_of_matches = 200;
            int max_disparity_percent = 40;
            /* descriptor match weight */
            int learnDesc = 90;
            /* luminance match weight */
            int learnLuma = 35;
            /* disparity weight */
            int learnDisp = 1;            
            /* priors*/
            int use_priors = 1;
            int groundPrior = 200;
            /* gradient */
            int learnGrad = 10;

#ifdef SVS_PROFILE
            t[2] = readRTC();
#endif
            matches = svs_match(
                          ideal_no_of_matches,
                          max_disparity_percent,
                          learnDesc,
                          learnLuma,
                          learnDisp,
                          learnGrad,
                          groundPrior,
                          use_priors);

#ifdef SVS_PROFILE
            t[3] = readRTC();
            printf("svs_grab             %d\n\r", t[1]-t[0]);
            printf("svs_receive_features %d\n\r", t[2]-t[1]);
            printf("svs_match            %d\n\r", t[3]-t[2]);
#endif
            if (send_disparities != 0) svs_send_disparities(matches);

#ifdef SVS_ENABLE_MAPPING
            /* mapping */
            if (svs_enable_mapping != 0) {
                if (prev_svs_enable_mapping == 0) init_map();
                svs_footline_update(max_disparity_percent);
                update_pose(svs_data, svs_data_previous);
                if (robot_moving == 0) {
                    map_recenter();
                    map_update(matches, max_disparity_percent, svs_matches, footline);
                }
            }
#endif
        }

        // swap data buffers to enable visual odometry between successive frames */
        svs_data_struct *temp = svs_data;
        svs_data = svs_data_previous;
        svs_data_previous = temp;

        svs_show_matches((unsigned char *)FRAME_BUF, matches);
        //svs_show_footline((unsigned char *)FRAME_BUF);

#ifdef SVS_ENABLE_MAPPING
        if (svs_enable_mapping != 0) {
            if (prev_svs_enable_mapping == 0) init_map();
            show_map((unsigned char *)FRAME_BUF);
        }
        prev_svs_enable_mapping = svs_enable_mapping;
#endif

        *pPORTHIO &= 0xFEFF;  // set stereo sync flag low
    }
}


/* -----------------------------------------------------------------------------------------------------*/
/* mapping */
/* -----------------------------------------------------------------------------------------------------*/


#ifdef SVS_ENABLE_MAPPING

/* grid map with occupancy represented as log odds */
int* map_occupancy;

/* cartesian coordinates of observed features for each map position */
int* map_coords;

/* robot position and speed */
int robot_x_mm, robot_y_mm;
int robot_speed_mmsec, robot_orientation_degrees;

/* center position of the map */
int map_x_mm, map_y_mm;

/* distance moved from the previous position */
int dist_moved_mm, prev_dist_moved_mm;

/* min and max occupancy values */
int min_occupancy, max_occupancy;

/* index numbers for the start of each ray model */
const int sensmodel_indexes = 131;
int sensmodelindex[] = {0,0,136,363,645,904,1057,1154,1222,1272,1311,1342,1367,1388,1406,1421,1435,1447,1458,1467,1476,1484,1491,1497,1503,1508,1513,1518,1522,1526,1530,1534,1537,1540,1543,1546,1549,1552,1554,1556,1558,1560,1562,1564,1566,1568,1570,1572,1574,1575,1576,1577,1578,1579,1580,1581,1582,1583,1584,1585,1586,1587,1588,1589,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590,1590};

/* sensor model, computed for baseline = 107mm, grid cell size = 40mm */
int sensmodel[] = {
    1,1,1,1,2,2,2,2,2,2,2,2,2,3,4,4,4,4,4,5,5,5,5,5,7,7,7,7,7,8,8,8,8,8,12,12,12,12,12,14,14,14,14,14,18,19,19,19,19,21,22,22,22,22,26,30,30,29,29,31,34,34,33,33,37,44,43,43,43,44,49,49,48,48,50,62,62,61,61,61,68,68,68,67,67,84,84,84,83,83,91,92,91,91,91,108,112,111,111,110,118,121,120,120,119,134,144,143,143,142,148,154,153,153,152,164,181,180,180,179,183,192,192,191,190,198,223,222,221,220,222,235,234,233,232,233,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,3,3,4,4,4,4,4,5,6,6,6,6,6,8,8,8,7,8,10,10,10,10,10,12,12,12,12,12,15,15,15,15,14,17,18,18,18,17,20,21,21,21,21,23,25,25,24,24,27,29,28,28,28,30,33,32,32,32,34,37,37,36,36,38,41,41,41,41,41,46,45,45,45,45,50,50,50,49,49,55,54,54,54,54,59,59,59,58,58,62,63,63,63,62,66,67,67,67,66,70,71,71,71,70,73,75,75,74,74,76,79,78,78,78,79,82,82,81,81,82,85,84,84,84,84,88,87,87,86,86,90,90,89,89,88,92,92,91,91,90,93,93,93,93,92,95,95,94,94,94,96,96,96,95,95,96,97,97,96,96,97,98,97,97,96,97,98,97,97,97,97,98,97,97,97,97,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,2,2,2,2,3,3,4,4,4,4,6,6,6,6,7,8,9,9,9,9,13,13,13,13,14,16,17,18,18,18,22,23,23,23,24,27,28,29,29,29,34,36,36,36,37,40,42,43,43,42,46,50,49,49,50,52,56,56,56,55,57,63,62,62,62,63,68,67,67,67,67,73,72,72,71,71,76,76,75,75,74,78,78,78,77,76,80,81,79,79,78,80,80,80,79,78,80,82,80,79,79,79,79,78,78,76,77,79,77,76,76,75,74,74,73,71,72,74,72,71,71,70,68,67,67,65,65,67,66,64,64,63,60,60,60,58,57,59,58,57,56,56,53,52,52,51,50,50,50,49,49,48,46,44,44,44,43,43,43,42,41,41,39,37,37,37,36,36,36,35,34,34,33,31,31,31,30,29,30,29,28,28,28,25,25,25,24,24,24,24,23,23,23,21,20,20,20,20,20,19,19,19,19,17,16,16,16,16,16,16,15,15,15,14,13,13,13,13,13,12,12,12,12,12,10,10,10,10,10,10,10,10,10,9,8,8,8,8,8,8,8,8,8,8,7,6,6,6,6,6,6,6,6,6,5,
    2,2,3,3,3,5,5,5,7,7,8,9,9,12,12,14,15,16,20,19,22,24,24,29,29,31,35,34,40,41,43,47,47,53,54,56,61,61,66,68,69,75,75,79,82,82,89,88,91,95,94,100,100,102,105,105,109,109,110,113,113,115,115,116,118,118,119,119,118,120,120,119,119,118,119,118,117,116,115,115,115,113,111,110,109,109,107,104,103,102,101,100,95,95,94,93,92,86,86,85,84,83,77,77,75,74,74,68,67,66,65,65,60,59,58,57,56,52,50,50,49,48,45,43,43,41,41,38,36,36,35,35,33,30,30,29,29,27,25,25,24,24,23,21,21,20,20,19,17,17,16,16,16,14,14,13,13,13,11,11,11,11,11,9,9,9,9,9,7,7,7,7,7,6,6,6,5,5,5,4,4,4,4,4,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    2,2,3,3,4,5,7,7,9,10,13,14,17,19,23,25,29,32,37,40,44,50,55,60,65,72,76,82,90,96,98,109,113,120,123,133,135,142,148,155,154,163,165,171,170,177,176,180,181,185,181,183,183,184,182,180,179,177,176,173,170,167,163,160,158,151,147,144,141,134,131,128,120,117,115,109,103,101,97,90,87,85,78,75,73,69,63,62,60,54,51,50,46,43,41,40,35,34,33,30,27,27,25,22,21,21,17,17,17,15,14,13,12,11,10,10,9,8,8,7,6,6,6,5,5,4,4,3,3,3,3,2,2,2,2,2,2,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,
    3,3,5,6,8,11,13,18,22,26,34,38,50,56,67,77,86,105,111,129,139,154,173,179,201,208,223,239,242,260,262,272,279,280,284,283,284,283,281,272,268,262,254,249,233,226,216,203,197,181,172,163,148,142,129,119,114,99,95,86,77,73,62,58,53,46,44,37,34,31,26,25,21,19,17,14,13,11,10,9,7,7,6,5,5,3,3,3,2,2,1,1,1,1,1,0,0,
    3,5,6,11,15,20,30,41,49,67,85,104,125,154,173,203,237,266,282,320,338,360,384,392,398,408,410,407,398,382,371,355,335,315,290,267,244,227,198,180,158,143,121,111,90,82,67,61,47,44,33,31,23,22,15,14,10,10,6,6,4,4,2,2,1,1,1,1,
    4,7,11,17,29,45,63,84,115,158,204,245,287,348,398,454,483,510,541,554,558,547,520,493,467,423,393,332,295,259,217,193,145,126,101,81,69,47,40,29,23,18,12,10,6,5,4,2,2,1,
    5,9,18,32,55,86,126,184,260,342,434,502,577,656,702,728,709,688,654,594,527,449,376,305,253,197,141,110,84,55,42,30,18,13,9,5,3,2,1,
    5,12,26,53,99,167,266,374,505,640,768,873,898,893,862,786,677,556,441,343,241,174,121,80,51,30,19,11,6,3,1,
    6,17,42,95,187,327,499,685,883,1045,1136,1111,989,851,680,515,363,232,144,82,48,26,13,6,3,
    7,25,70,167,323,554,867,1155,1340,1339,1209,1006,750,515,316,175,93,46,21,9,3,
    9,34,106,265,597,999,1309,1559,1568,1341,998,600,326,164,73,29,10,3,
    10,47,155,480,954,1534,1863,1761,1419,934,492,220,83,29,8,
    12,63,271,747,1482,2114,2123,1564,946,452,159,46,11,2,
    14,80,433,1164,2159,2509,1912,1117,432,138,33,4,
    16,129,646,1827,2733,2455,1481,530,153,21,3,
    20,204,999,2419,3105,2170,841,211,24,
    22,278,1412,3159,3180,1579,314,49,3,
    23,399,2117,3944,2615,794,100,4,
    27,540,2812,4198,2033,363,23,
    28,774,3717,4135,1225,114,
    39,1098,4532,3650,649,28,
    53,1559,5217,2900,265,
    70,2052,5701,2057,116,
    99,2701,6087,1085,26,
    125,3322,5806,740,
    165,4162,5382,288,
    200,5506,4153,139,
    104,5932,3922,40,
    273,6561,3138,
    342,7817,1837,
    411,8188,1399,
    483,8853,662,
    405,9149,445,
    892,8503,603,
    1390,8558,
    1888,8073,
    2368,7606,
    2447,7551,
    2917,7081,
    2994,7004,
    1734,8263,
    4679,5320,
    5082,4917,
    5812,4187,
    6033,3966,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000,
    10000
};

int SinLookup[] = {
    0,174,348,523,697,871,1045,1218,1391,1564,1736,1908,2079,2249,2419,2588,2756,2923,3090,3255,3420,3583,3746,3907,4067,4226,4383,4539,4694,4848,5000,5150,5299,5446,5591,5735,5877,6018,6156,6293,6427,6560,6691,6819,6946,7071,            7193,7313,7431,7547,7660,7771,7880,7986,8090,8191,8290,8386,8480,8571,8660,8746,8829,8910,8987,9063,9135,9205,9271,9335,9396,9455,9510,9563,9612,9659,9702,9743,9781,9816,9848,9876,9902,9925,9945,9961,9975,9986,9993,9998,9999,            9998,9993,9986,9975,9961,9945,9925,9902,9876,9848,9816,9781,9743,9702,9659,9612,9563,9510,9455,9396,9335,9271,9205,9135,9063,8987,8910,8829,8746,8660,8571,8480,8386,8290,8191,8090,7986,7880,7771,7660,7547,7431,7313,7193,7071,           6946,6819,6691,6560,6427,6293,6156,6018,5877,5735,5591,5446,5299,5150,5000,4848,4694,4539,4383,4226,4067,3907,3746,3583,3420,3255,3090,2923,2756,2588,2419,2249,2079,1908,1736,1564,1391,1218,1045,871,697,523,348,174,0,            -174,-348,-523,-697,-871,-1045,-1218,-1391,-1564,-1736,-1908,-2079,-2249,-2419,-2588,-2756,-2923,-3090,-3255,-3420,-3583,-3746,-3907,-4067,-4226,-4383,-4539,-4694,-4848,-5000,-5150,-5299,-5446,-5591,-5735,-5877,-6018,-6156,-6293,-6427,-6560,-6691,-6819,-6946,-7071,            -7193,-7313,-7431,-7547,-7660,-7771,-7880,-7986,-8090,-8191,-8290,-8386,-8480,-8571,-8660,-8746,-8829,-8910,-8987,-9063,-9135,-9205,-9271,-9335,-9396,-9455,-9510,-9563,-9612,-9659,-9702,-9743,-9781,-9816,-9848,-9876,-9902,-9925,-9945,-9961,-9975,-9986,-9993,-9998,-9999,           -9998,-9993,-9986,-9975,-9961,-9945,-9925,-9902,-9876,-9848,-9816,-9781,-9743,-9702,-9659,-9612,-9563,-9510,-9455,-9396,-9335,-9271,-9205,-9135,-9063,-8987,-8910,-8829,-8746,-8660,-8571,-8480,-8386,-8290,-8191,-8090,-7986,-7880,-7771,-7660,-7547,-7431,-7313,-7193,-7071,            -6946,-6819,-6691,-6560,-6427,-6293,-6156,-6018,-5877,-5735,-5591,-5446,-5299,-5150,-4999,-4848,-4694,-4539,-4383,-4226,-4067,-3907,-3746,-3583,-3420,-3255,-3090,-2923,-2756,-2588,-2419,-2249,-2079,-1908,-1736,-1564,-1391,-1218,-1045,-871,-697,-523,-348,-174,
};

/* initialises arrays associated with mapping */
void init_map() {
    int i;

    map_occupancy = (int*)malloc(MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 4);
    map_coords = (int*)malloc(MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2 * sizeof(int));
    memset((void*)map_occupancy, '\0', MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 4);
    for (i = (MAP_WIDTH_CELLS * MAP_WIDTH_CELLS * 2)-2; i >= 0; i-=2) map_coords[i] = EMPTY_CELL;
    robot_x_mm = 0;
    robot_y_mm = 0;
    map_x_mm = 0;
    map_y_mm = 0;
    robot_speed_mmsec = 0;
    robot_orientation_degrees = 0;
    robot_moving = 0;
    dist_moved_mm = 0;
    prev_dist_moved_mm = 0;
    max_occupancy = 2147473647;
    min_occupancy = -max_occupancy;
}

/* visual equivalent of horizontal scan matching */
void scan_match(
    int right_turn_pixels,
    int tollerance,
    svs_data_struct *svs_data,
    svs_data_struct *svs_data_previous)
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    int i, j, best_dx, x, expected_x, x_prev, y, no_of_feats, no_of_feats_prev, n, f, f_prev;
    int dx, dx2, min_dx, state, av_dx=0, hits=0;
    n = 0;
    f = 0;
    f_prev = 0;
    for (y = 4; y < (int)imgHeight - 4; y += SVS_VERTICAL_SAMPLING, n++)
    {
        no_of_feats = svs_data->features_per_row[n];
        no_of_feats_prev = svs_data_previous->features_per_row[n];
        // current edge features
        for (i = 0; i < no_of_feats; i++) {
            x = svs_data_previous->feature_x[f + i];
            min_dx = 9999;
            best_dx = 9999;
            state = 0;
            expected_x = x - right_turn_pixels;
            // previous edge features
            for (j = 0; j < no_of_feats_prev; j++) {
                x_prev = svs_data_previous->feature_x[f_prev + j];
                dx = x_prev - expected_x;
                if ((dx > -tollerance) && (dx < tollerance)) {
                    // within tollerance
                    state = 1;
                    if (dx >= 0)
                        dx2 = dx;
                    else
                        dx2 = -dx;
                    if (dx2 < min_dx) {
                        min_dx = dx2;
                        best_dx = x - x_prev;
                        if (min_dx < 4) break;
                    }
                }
                else {
                    // outside of tollerance
                    if (state == 1) break;
                }
            }
            if (best_dx != 9999) {
                av_dx += best_dx;
                hits++;
            }

        }
        f += no_of_feats;
        f_prev += no_of_feats_prev;
    }

    if (hits > 0) {
        av_dx /= hits;
    }
    else {
        av_dx = right_turn_pixels;
    }

    // update orientation of the robot
    robot_orientation_degrees += av_dx * SVS_FOV_DEGREES / ((int)imgWidth*2);
    if (robot_orientation_degrees < 0) robot_orientation_degrees += 360;
    if (robot_orientation_degrees >= 360) robot_orientation_degrees -= 360;

#ifdef SVS_PROFILE
    printf("scan_match %d mS\n", readRTC() - t);
#endif
}

/* re-centers the grid map */
void map_recenter()
{
#ifdef SVS_PROFILE
    int t;
#endif

    /* compute change in translation */
    int dx = robot_x_mm - map_x_mm;
    int dy = robot_y_mm - map_y_mm;

    if ((dx < -MAP_CELL_SIZE_MM) || (dx > MAP_CELL_SIZE_MM) ||
            (dy < -MAP_CELL_SIZE_MM) || (dy > MAP_CELL_SIZE_MM)) {

#ifdef SVS_PROFILE
        t = readRTC();
#endif

        int n, i, offset, cell_x_mm, cell_y_mm, cell_x, cell_y;
        int* buffer_occupancy = (int*)FRAME_BUF3;
        int* buffer_coords = (int*)FRAME_BUF4;

        /* clear the buffer */
        const int array_length = MAP_WIDTH_CELLS * MAP_WIDTH_CELLS;
        for (i = array_length*2-2; i >= 0; i-=2) buffer_coords[i] = EMPTY_CELL;
        memset((void*)buffer_occupancy, '\0', array_length*4);

        /* update map cells */
        offset = MAP_WIDTH_CELLS/2;
        for (i = 0; i < array_length; i++) {
            if (map_coords[i*2] != EMPTY_CELL) {

                /* update cell position */
                cell_x_mm = map_coords[i*2];
                cell_y_mm = map_coords[i*2 + 1];

                cell_x = offset + ((((cell_x_mm - map_x_mm - dx)+1)*2) / (MAP_CELL_SIZE_MM*2));
                cell_y = offset + ((((cell_y_mm - map_y_mm - dy)+1)*2) / (MAP_CELL_SIZE_MM*2));
                n = cell_y * MAP_WIDTH_CELLS + cell_x;
                if ((cell_x >= 0) && (cell_y >= 0) &&
                        (cell_x < MAP_WIDTH_CELLS) && (cell_y < MAP_WIDTH_CELLS)) {

                    if (buffer_coords[n*2] == EMPTY_CELL) {
                        /* new cell */
                        buffer_occupancy[n] = map_occupancy[i];
                        buffer_coords[n*2] = cell_x_mm;
                        buffer_coords[n*2+1] = cell_y_mm;
                    }
                    else {
                        /* merge cells */
                        buffer_occupancy[n] += map_occupancy[i];
                        buffer_coords[n*2] += (cell_x_mm - buffer_coords[n*2])/2;
                        buffer_coords[n*2+1] += (cell_y_mm - buffer_coords[n*2+1])/2;
                    }
                }
            }
        }

        /* copy buffer back to the map */
        memcpy2((void*)map_coords, (void*)buffer_coords, array_length*8);
        memcpy2((void*)map_occupancy, (void*)buffer_occupancy, array_length*4);

        /* set the new map centre position */
        map_x_mm = robot_x_mm;
        map_y_mm = robot_y_mm;

        prev_dist_moved_mm = dist_moved_mm;

#ifdef SVS_PROFILE
        printf("map_recenter %d mS\n", readRTC() - t);
#endif
    }
}

/* update the estimated pose of the robot */
void update_pose(
    svs_data_struct *svs_data,
    svs_data_struct *svs_data_previous)
{
    /* estimate the change in orientation of the robot */
    scan_match(
        svs_right_turn_percent * (int)imgWidth / 100,
        svs_turn_tollerance_percent * (int)imgWidth / 100,
        svs_data, svs_data_previous);
    svs_right_turn_percent = 0;

    /* update the position/pose estimate */
    if (robot_moving != 0) {
        int pwm = (lspeed + rspeed)/2;
        if (pwm < 0) pwm = -pwm;
        const int reference_variance0 = 7;
        const int reference_speed0 = 95;
        const int reference_pwm0 = 19;
        const int reference_variance1 = 22;
        const int reference_speed1 = 265;
        const int reference_pwm1 = 31;
        int robot_speed_mmsec =
            reference_speed0 +
            ((pwm - reference_pwm0) *
             (reference_speed1 - reference_speed0) /
             (reference_pwm1 - reference_pwm0));
        int speed_variance_mmsec =
            reference_variance0 +
            ((pwm - reference_pwm0) *
             (reference_variance1 - reference_variance0) /
             (reference_pwm1 - reference_pwm0));
        int time_elapsed_mS = readRTC() - move_start_time;
        int dist_mm_min = time_elapsed_mS * (robot_speed_mmsec - speed_variance_mmsec) / 1000;
        int dist_mm_max = time_elapsed_mS * (robot_speed_mmsec + speed_variance_mmsec) / 1000;
        if (lspeed + rspeed < 0) {
            dist_mm_min = -dist_mm_min;
            dist_mm_max = -dist_mm_max;
        }

        dist_moved_mm = ((dist_mm_min + dist_mm_max)/2) - prev_dist_moved_mm;

        int rot = robot_orientation_degrees;

        robot_x_mm = map_x_mm + (dist_moved_mm*SinLookup[rot]/10000);
        int CosVal = 90 - rot;
        if (CosVal < 0) CosVal += 360;
        robot_y_mm = map_y_mm + (dist_moved_mm*SinLookup[CosVal]/10000);
    }
    else {
        prev_dist_moved_mm = 0;
    }
}

/* updates a grid map */
void map_update(
    int no_of_matches,
    int max_disparity_percent,
    unsigned int* svs_matches,
    unsigned short int** footline)
{
#ifdef SVS_PROFILE
    int t = readRTC();
#endif

    int i, rot, x, y, xx, yy, disp;
    int x_mm, y_mm, curr_x_mm, curr_y_mm, x_rotated_mm, y_rotated_mm, disp_mmx100;
    int n, SinVal, CosVal;
    const int range_const = SVS_FOCAL_LENGTH_MMx100 * SVS_BASELINE_MM;
    int half_width = (int)imgWidth/2;
    int dx = robot_x_mm - map_x_mm;
    int dy = robot_y_mm - map_y_mm;
    const int centre_cell = MAP_WIDTH_CELLS/2;
    int distill_grid = 0;

    int on_ground_plane;
    for (i = 0; i < no_of_matches*4; i += 4) {
        if (svs_matches[i] > 0) {  // match probability > 0
            x = svs_matches[i + 1];
            y = svs_matches[i + 2];
            if (y < footline[0][x / SVS_HORIZONTAL_SAMPLING])
                on_ground_plane = 0;
            else
                on_ground_plane = 1;

            disp = svs_matches[i + 3];
            disp_mmx100 = disp * svs_sensor_width_mmx100 / (int)imgWidth;
            if (disp_mmx100 > 0) {
                // get position of the feature in space
                int range_mm = range_const / disp_mmx100;

                curr_y_mm = range_mm;
                int rot_off;
                int max_rot_off = (int)SVS_FOV_DEGREES/2;

                rot_off = (x - half_width) * (int)SVS_FOV_DEGREES / (int)imgWidth;
                if ((rot_off > -max_rot_off) && (rot_off < max_rot_off)) {
                    if (rot_off < 0) rot_off += 360;
                    curr_x_mm = SinLookup[rot_off] * curr_y_mm / (int)10000;

                    rot = -robot_orientation_degrees;
                    if (rot < 0) rot += 360;
                    if (rot >= 360) rot -= 360;

                    SinVal = SinLookup[rot];
                    CosVal = 90 - rot;
                    if (CosVal < 0) CosVal += 360;
                    CosVal = SinLookup[CosVal];

                    // rotate by the orientation of the robot
                    x_rotated_mm = (curr_x_mm*CosVal - curr_y_mm*SinVal) / (int)10000;
                    y_rotated_mm = (curr_x_mm*SinVal + curr_y_mm*CosVal) / (int)10000;

                    int x_cell = ((x_rotated_mm+dx+1)*2) / (MAP_CELL_SIZE_MM*2);
                    int y_cell = ((y_rotated_mm+dy+1)*2) / (MAP_CELL_SIZE_MM*2);

                    if ((x_cell > -centre_cell) && (x_cell < centre_cell) &&
                            (y_cell > -centre_cell) && (y_cell < centre_cell)) {

                        int abs_x_cell = x_cell;
                        if (abs_x_cell < 0) abs_x_cell = -abs_x_cell;
                        int abs_y_cell = y_cell;
                        if (abs_y_cell < 0) abs_y_cell = -abs_y_cell;

                        /* vacancy */
                        int cells = range_mm / MAP_CELL_SIZE_MM;
                        if (cells < 1) cells = 1;
                        int prob_average = 10000 / cells;
                        int prob = prob_average * 120 / 100;
                        int prob_decr = prob_average * 40 / (100 * cells);
                        if (abs_x_cell > abs_y_cell) {
                            for (x = 0; x <= abs_x_cell; x++) {
                                y = x * y_cell / abs_x_cell;
                                xx = x;
                                if (x_cell < 0) xx = -x;

                                x_mm = robot_x_mm + (x*x_rotated_mm/abs_x_cell);
                                y_mm = robot_y_mm + (x*y_rotated_mm/abs_x_cell);

                                n = (((y + centre_cell) * MAP_WIDTH_CELLS) + (xx + centre_cell));
                                if (map_coords[n*2] == EMPTY_CELL) {
                                    map_coords[n*2] = x_mm;
                                    map_coords[n*2+1] = y_mm;
                                }
                                else {
                                    map_coords[n*2] += (x_mm - map_coords[n*2])/2;
                                    map_coords[n*2+1] += (y_mm - map_coords[n*2+1])/2;
                                }
                                if ((map_occupancy[n] > min_occupancy) &&
                                        (map_occupancy[n] < max_occupancy)) {
                                    // increment vacancy
                                    map_occupancy[n] -= prob;
                                    prob-=prob_decr;
                                    if (prob < 1) prob = 1;
                                }
                                else {
                                    distill_grid = 1;
                                }
                            }
                        }
                        else {
                            for (y = 0; y < abs_y_cell; y++) {
                                x = y * x_cell / abs_y_cell;
                                yy = y;
                                if (y_cell < 0) yy = -y;

                                x_mm = robot_x_mm + (y*x_rotated_mm/abs_y_cell);
                                y_mm = robot_y_mm + (y*y_rotated_mm/abs_y_cell);

                                n = (((yy + centre_cell) * MAP_WIDTH_CELLS) + (x + centre_cell));
                                if (map_coords[n*2] == EMPTY_CELL) {
                                    map_coords[n*2] = x_mm;
                                    map_coords[n*2+1] = y_mm;
                                }
                                else {
                                    map_coords[n*2] += (x_mm - map_coords[n*2])/2;
                                    map_coords[n*2+1] += (y_mm - map_coords[n*2+1])/2;
                                }
                                if ((map_occupancy[n] > min_occupancy) &&
                                        (map_occupancy[n] < max_occupancy)) {
                                    // increment vacancy
                                    map_occupancy[n] -= prob;
                                    prob-=prob_decr;
                                    if (prob < 1) prob = 1;
                                }
                                else {
                                    distill_grid = 1;
                                }
                            }
                        }

                        if ((on_ground_plane == 0) &&
                                (range_mm > MAP_MIN_RANGE_MM)) {
                            // occupancy
                            int tail_length = 0;
                            int tail_index = disp;
                            if (disp+1 >= sensmodel_indexes) tail_index = sensmodel_indexes-2;
                            tail_length = sensmodelindex[tail_index+1] - sensmodelindex[tail_index];
                            int idx = sensmodelindex[disp];

                            if (abs_x_cell > abs_y_cell) {
                                for (x = abs_x_cell; x < abs_x_cell+tail_length; x++, idx++) {
                                    y = x * y_cell / abs_x_cell;
                                    xx = x;
                                    if (x_cell < 0) xx = -x;

                                    x_mm = robot_x_mm + (x*x_rotated_mm/abs_x_cell);
                                    y_mm = robot_y_mm + (x*y_rotated_mm/abs_x_cell);

                                    n = (((y + centre_cell) * MAP_WIDTH_CELLS) + (xx + centre_cell));
                                    if ((y+centre_cell >= 0) && (xx+centre_cell >= 0) &&
                                            (y+centre_cell < MAP_WIDTH_CELLS) && (xx+centre_cell < MAP_WIDTH_CELLS)) {
                                        if (map_coords[n*2] == EMPTY_CELL) {
                                            map_coords[n*2] = x_mm;
                                            map_coords[n*2+1] = y_mm;
                                        }
                                        else {
                                            map_coords[n*2] += (x_mm - map_coords[n*2])/2;
                                            map_coords[n*2+1] += (y_mm - map_coords[n*2+1])/2;
                                        }
                                        if ((map_occupancy[n] > min_occupancy) &&
                                                (map_occupancy[n] < max_occupancy)) {
                                            map_occupancy[n] += sensmodel[idx];
                                        }
                                        else {
                                            distill_grid = 1;
                                        }
                                    }
                                }
                            }
                            else {
                                for (y = abs_y_cell; y <= abs_y_cell+tail_length; y++, idx++) {
                                    x = y * x_cell / abs_y_cell;
                                    yy = y;
                                    if (y_cell < 0) yy = -y;

                                    x_mm = robot_x_mm + (y*x_rotated_mm/abs_y_cell);
                                    y_mm = robot_y_mm + (y*y_rotated_mm/abs_y_cell);

                                    n = (((yy + centre_cell) * MAP_WIDTH_CELLS) + (x + centre_cell));
                                    if ((yy+centre_cell >= 0) && (x+centre_cell >= 0) &&
                                            (yy+centre_cell < MAP_WIDTH_CELLS) && (x+centre_cell < MAP_WIDTH_CELLS)) {
                                        if (map_coords[n*2] == EMPTY_CELL) {
                                            map_coords[n*2] = x_mm;
                                            map_coords[n*2+1] = y_mm;
                                        }
                                        else {
                                            map_coords[n*2] += (x_mm - map_coords[n*2])/2;
                                            map_coords[n*2+1] += (y_mm - map_coords[n*2+1])/2;
                                        }
                                        if ((map_occupancy[n] > min_occupancy) &&
                                                (map_occupancy[n] < max_occupancy)) {
                                            // increment occupancy
                                            map_occupancy[n] += sensmodel[idx];
                                        }
                                        else {
                                            distill_grid = 1;
                                        }
                                    }
                                }
                            }

                        }
                    }

                }
            }
        }
    }

    /* if the occupancy values become saturated then divide them down */
    if (distill_grid == 1) {
        n = MAP_WIDTH_CELLS * MAP_WIDTH_CELLS - 1;
        for (i = ((MAP_WIDTH_CELLS * MAP_WIDTH_CELLS - 1)*2)-2; i >= 0; i-=2, n--) {
            if (map_coords[i] != EMPTY_CELL) {
                map_occupancy[n] /= 1024;
            }
        }
    }

#ifdef SVS_PROFILE
    printf("map_update %d mS\n", readRTC() - t);
#endif
}

/* shows a map oriented with the robot */
void show_map(
    unsigned char* outbuf)
{
    int x,y,xx,yy,n,n2,logodds,prob;
    int occupancy_thresh=0, vacancy_thresh=0;
    const int divisor = 3;
    int ww = (int)imgWidth/divisor;
    int hh = (int)imgHeight/divisor;

    /* find average occupancy and vacancy */
    unsigned int average_occupancy = 0;
    unsigned int average_vacancy = 0;
    unsigned int occupancy_hits = 0;
    unsigned int vacancy_hits = 0;
    n2 = MAP_WIDTH_CELLS*MAP_WIDTH_CELLS - 1;
    for (n = (MAP_WIDTH_CELLS*MAP_WIDTH_CELLS*2) - 2; n >= 0; n-=2, n2--) {
        if (map_coords[n] != EMPTY_CELL) {
            logodds = map_occupancy[n2];
            if (logodds >= 0) {
                prob = 100000 - (100000 / (1 + (logodds/65536)));
                average_occupancy += (unsigned int)(prob/64);
                occupancy_hits++;
            }
            else {
                prob = 100000 - (100000 / (1 - (logodds/16384)));
                average_vacancy += (unsigned int)prob;
                vacancy_hits++;
            }
        }
    }
    if (occupancy_hits > 0) {
        average_occupancy /= occupancy_hits;
        /* set the occupancy threshold */
        occupancy_thresh = (int)average_occupancy*64;
    }
    if (vacancy_hits > 0) {
        average_vacancy /= vacancy_hits;
        /* set the vacancy threshold */
        vacancy_thresh = (int)average_vacancy * 80 / 100;
    }

    for (y = 0; y < hh; y++) {
        yy = MAP_WIDTH_CELLS - 1 - (y * MAP_WIDTH_CELLS / hh);
        for (x = 0; x < ww; x++) {
            xx = x * MAP_WIDTH_CELLS / ww;
            n = pixindex(x, y);
            n2 = (yy * MAP_WIDTH_CELLS) + xx;

            if (map_coords[n2*2] != EMPTY_CELL) {
                logodds = map_occupancy[n2];
                if (logodds >= 0)
                    prob = 100000 - (100000 / (1 + (logodds/65536)));
                else
                    prob = 100000 - (100000 / (1 - (logodds/16384)));

                if ((logodds < 0) && (prob > vacancy_thresh)) {
                    /* vacant */
                    outbuf[n] = 255;
                    outbuf[n+1] = 44;
                    outbuf[n+2] = 84;
                    outbuf[n+3] = 21;
                }
                else {
                    if ((logodds >= 0) && (prob > occupancy_thresh)) {
                        /* occupied */
                        outbuf[n] = 84;
                        outbuf[n+1] = 72;
                        outbuf[n+2] = 255;
                        outbuf[n+3] = 0;
                    }
                    else {
                        /* not enough confidence to determine
                           whether this is occupied or vacant */
                        outbuf[n] = 128;
                        outbuf[n+1] = 0;
                        outbuf[n+2] = 128;
                        outbuf[n+3] = 0;
                    }
                }
            }
            else {
                /* terra incognita */
                outbuf[n] = 128;
                outbuf[n+1] = 0;
                outbuf[n+2] = 128;
                outbuf[n+3] = 0;
            }
        }
    }

}

#endif


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  colors.c - image processing routines for the SRV-1 Blackfin robot.
 *    Copyright (C) 2007-2009  Surveyor Corporation
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

#include "colors.h"

extern unsigned int imgWidth, imgHeight;

unsigned int ymax[MAX_COLORS], ymin[MAX_COLORS], umax[MAX_COLORS], umin[MAX_COLORS], vmax[MAX_COLORS], vmin[MAX_COLORS];
unsigned int blobx1[MAX_BLOBS], blobx2[MAX_BLOBS], bloby1[MAX_BLOBS], bloby2[MAX_BLOBS], blobcnt[MAX_BLOBS], blobix[MAX_BLOBS];
unsigned int hist0[256], hist1[256], mean[3];
unsigned int x_sum[MAX_IMHEIGHT], y_sum[MAX_IMWIDTH];

/*
void init_colors() {
    unsigned int ii;
    
    for(ii = 0; ii<MAX_COLORS; ii++) {
        ymax[ii] = 0;
        ymin[ii] = 0;
        umax[ii] = 0;
        umin[ii] = 0;
        vmax[ii] = 0;
        vmin[ii] = 0;
    }
}*/

unsigned int vpix(unsigned char *frame_buf, unsigned int xx, unsigned int yy) {
        unsigned int ix;
        ix = index(xx,yy); 
        return    ((unsigned int)frame_buf[ix] << 24) +    // returns UYVY packed into 32-bit word
                        ((unsigned int)frame_buf[ix+1] << 16) +
                        ((unsigned int)frame_buf[ix+2] << 8) +
                        (unsigned int)frame_buf[ix+3];
}


void findCenter(unsigned char *frame_buf, unsigned int ii, int * coords) 
{
    unsigned int ix, xx, yy, y, u, v, total, half_total, tmp_x, tmp_y;

    unsigned int y1, y2, u1, u2, v1, v2;
    y1 = ymin[ii];
    y2 = ymax[ii];
    u1 = umin[ii];
    u2 = umax[ii];
    v1 = vmin[ii];
    v2 = vmax[ii];

    // empty x_sum and y_sum that will be used to find the center
    total = 0;
    for(ix = 0; ix < imgWidth; ix++)
    {
	y_sum[ix] = 0; // sum for image columns
	if(ix < imgHeight) x_sum[ix] = 0; // sum for image rows 
    }

//    for (ix=0; ix<(imgWidth*imgHeight*2); ix+=4){ // 40; ix+=4){ //
    for(xx = 0; xx < imgWidth; xx++) {
	for(yy = 0; yy < imgHeight; yy++) {
	ix = index(xx,yy);
        y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
        u = (unsigned int)frame_buf[ix];
        v = (unsigned int)frame_buf[ix+2];

        if ((y > y1) && (y < y2) && (u > u1) && (u < u2) && (v > v1) && (v < v2))
	{
	    total++;
	    // increment x_sum and y_sum with one
	    //x_sum[y_from_index(ix)]++;
	    //y_sum[x_from_index(ix)]++;
	    x_sum[yy]++;
	    y_sum[xx]++;
	}
    }   
    } // added for xx, yy
/*    printf("Total = %d\n\r", total);
    half_total = 0;
    for(ix = 0; ix < imgHeight; ix++) 
    {
	half_total += x_sum[ix];
	printf("%d,", x_sum[ix]);
    }
    printf("total x_sum = %d\n\r", half_total);
    half_total = 0;
    for(ix = 0; ix < imgHeight; ix++) 
    {
	half_total += y_sum[ix];
	printf("%d,", y_sum[ix]);
    }
    printf("total y_sum = %d\n\r", half_total);
*/

    coords[0] = -1; // x_center
    coords[1] = -1; // y_center

    tmp_x = 0; tmp_y = 0;
    half_total = total / 2;
    for(ix = 0; ix < imgWidth; ix++)
    {
	if(coords[0] > -1 && coords[1] > -1) 
	{
	    break;
	}
	else
	{
	    if(coords[1] < 0 && ix < imgHeight)
	    {
		tmp_x += x_sum[ix];
		if(tmp_x >= half_total) 
		{
		    coords[1] = ix;
		}
	    }
	    if(coords[0] < 0)
	    {
            	tmp_y += y_sum[ix];
	    	if(tmp_y >= half_total)
	    	{
		    coords[0] = ix;
	    	}
	    }
	}
    }
    // printf("fC: c[0] = %d, c[1] = %d\n\r", coords[0], coords[1]);
}

// return number of blobs found that match the search color
unsigned int vblob(unsigned char *frame_buf, unsigned char *blob_buf, unsigned int ii) {
    unsigned int jj, ix, xx, yy, y, u, v, count, bottom, top, tmp;
    unsigned int maxx, maxy;
    unsigned char *bbp, ctmp;
    int itmp, jtmp;
    unsigned int y1, y2, u1, u2, v1, v2;
    
    y1 = ymin[ii];
    y2 = ymax[ii];
    u1 = umin[ii];
    u2 = umax[ii];
    v1 = vmin[ii];
    v2 = vmax[ii];
    
    bbp = blob_buf;
    for (ix=0; ix<imgWidth*imgHeight; ix++)
        *bbp++ = 0;

    /* tag all pixels in blob_buf[]    
         matching = 1  
         no color match = 0
       thus all matching pixels will belong to blob #1 */
    bbp = blob_buf;
    for (ix=0; ix<(imgWidth*imgHeight*2); ix+=4) {
        y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
        u = (unsigned int)frame_buf[ix];
        v = (unsigned int)frame_buf[ix+2];

        if ((y >= y1) && (y <= y2) && (u >= u1) && (u <= u2) && (v >= v1) && (v <= v2))
            *bbp = 1;
        bbp += 2;
    }

    ix = imgWidth * (imgHeight-1);
    for (xx=0; xx<imgWidth; xx++) {
        blob_buf[xx] = 0;
        blob_buf[xx+ix] = 0;
    }
    ix = imgWidth-1;
    for (yy=0; yy<imgHeight; yy++) {
        blob_buf[yy*imgWidth] = 0;
        blob_buf[yy*imgWidth + ix] = 0;
    }
    
    /* clear out orphan pixels */
    itmp = 0;
    jtmp = 0;
    for (ix=0; ix<(imgWidth*imgHeight); ix+=2) {
        if (blob_buf[ix]) {
            itmp++;
            ctmp = 
                blob_buf[(ix-imgWidth)-2] +
                blob_buf[ix-imgWidth] +
                blob_buf[(ix-imgWidth)+2] +
                blob_buf[ix-2] +
                blob_buf[ix+2] +
                blob_buf[(ix+imgWidth)-2] +
                blob_buf[ix+imgWidth] +
                blob_buf[(ix+imgWidth)+2];
            if (ctmp < 4) {
                jtmp++;
                blob_buf[ix] = 0;
            }
        }
    }
    //printf("cleared %d out of %d matching pixels\r\n", jtmp, itmp);

    /* clear out orphan pixels */
    itmp = 0;
    jtmp = 0;
    for (ix=0; ix<(imgWidth*imgHeight); ix+=2) {
        if (blob_buf[ix]) {
            itmp++;
            ctmp = 
                blob_buf[(ix-imgWidth)-2] +
                blob_buf[ix-imgWidth] +
                blob_buf[(ix-imgWidth)+2] +
                blob_buf[ix-2] +
                blob_buf[ix+2] +
                blob_buf[(ix+imgWidth)-2] +
                blob_buf[ix+imgWidth] +
                blob_buf[(ix+imgWidth)+2];
            if (ctmp < 4) {
                jtmp++;
                blob_buf[ix] = 0;
            }
        }
    }
    //printf("cleared %d out of %d matching pixels\r\n", jtmp, itmp);

    maxx = imgWidth;
    maxy = imgHeight;

    for (jj=0; jj<MAX_BLOBS; jj++) {
        blobcnt[jj] = 0;
        blobx1[jj] = maxx;
        blobx2[jj] = 0;
        bloby1[jj] = maxy;
        bloby2[jj] = 0;
    }
        
    jj = 0;    // jj indicates the current blob being processed
    for (xx=0; xx<maxx; xx+=2) {
        count = 0;
        bottom = maxy;
        top = 0;
        for (yy=0; yy<maxy; yy++) {
            ix = xx + yy*imgWidth;
            if (blob_buf[ix]) {
                count++;
                if (bottom > yy)
                    bottom = yy;
                if (top < yy)
                    top = yy;
            }
        }
        if (count) {
            if (bloby1[jj] > bottom)
                bloby1[jj] = bottom;
            if (bloby2[jj] < top)
                bloby2[jj] = top;
            if (blobx1[jj] > xx)
                blobx1[jj] = xx;
            if (blobx2[jj] < xx)
                blobx2[jj] = xx;
            blobcnt[jj] += count;
        } else {
            if (blobcnt[jj])    // move to next blob if a gap is found
                jj++;
            if (jj > (MAX_BLOBS-2))
                goto blobbreak;
        }
    }
blobbreak:     // now sort blobs by size, largest to smallest pixel count
    for (xx=0; xx<=jj; xx++) {
        if (blobcnt[xx] == 0)    // no more blobs, so exit
            return xx;
        for (yy=xx; yy<=jj; yy++) {
            if (blobcnt[yy] == 0)
                break;
            if (blobcnt[xx] < blobcnt[yy]) {
                tmp = blobcnt[xx];
                blobcnt[xx] = blobcnt[yy];
                blobcnt[yy] = tmp;
                tmp = blobx1[xx];
                blobx1[xx] = blobx1[yy];
                blobx1[yy] = tmp;
                tmp = blobx2[xx];
                blobx2[xx] = blobx2[yy];
                blobx2[yy] = tmp;
                tmp = bloby1[xx];
                bloby1[xx] = bloby1[yy];
                bloby1[yy] = tmp;
                tmp = bloby2[xx];
                bloby2[xx] = bloby2[yy];
                bloby2[yy] = tmp;
            }
        }
    }
    return xx;
}

// histogram function - 
//  hist0[] holds frequency of u|v combination  
//  hist1[] holds average luminance corresponding each u|v combination

void vhist(unsigned char *frame_buf) {
    unsigned int ix, iy, xx, yy, y1, u1, v1;

    for (ix=0; ix<256; ix++) {  
        hist0[ix] = 0;  // accumulator 
        hist1[ix] = 0;  
    }
    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            iy = (u1 & 0xF0) + (v1 >> 4);
            hist0[iy]++;
            hist1[iy] += y1;
        }
    }
    for (ix=0; ix<256; ix++)
        if (hist1[ix])
            hist1[ix] /= hist0[ix];  // normalize by number of hits
}

/* mean color function - computes mean value for Y, U and V
   mean[0] = Y mean, mean[1] = U mean, mean[2] = V mean */
void vmean(unsigned char *frame_buf) {
    unsigned int ix, xx, yy, y1, u1, v1;
    unsigned int my, mu, mv;
    my = mu = mv = 0;

    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);  // yx, uv, vx range from 0-63 (yuv value divided by 4)
            y1 = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            u1 = ((unsigned int)frame_buf[ix]);
            v1 = ((unsigned int)frame_buf[ix+2]);
            my += y1;
            mu += u1;
            mv += v1;
        }
    }
    mean[0] = ((my*2) / imgWidth) / imgHeight;
    mean[1] = ((mu*2) / imgWidth) / imgHeight;
    mean[2] = ((mv*2) / imgWidth) / imgHeight;
}

void color_segment(unsigned char *frame_buf) {
    unsigned int ix, xx, yy, y, u, v, clr;
    unsigned int ymid[MAX_COLORS], umid[MAX_COLORS], vmid[MAX_COLORS];
    
    for (ix=0; ix<MAX_COLORS; ix++) {
        ymid[ix] = (ymax[ix] + ymin[ix]) >> 1;
        umid[ix] = (umax[ix] + umin[ix]) >> 1;
        vmid[ix] = (vmax[ix] + vmin[ix]) >> 1;
    }
    for (xx=0; xx<imgWidth; xx+=2) {   
        for (yy=0; yy<imgHeight; yy++) {
            ix = index(xx,yy);
            y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            //y = (unsigned int)frame_buf[ix+1];
            u = (unsigned int)frame_buf[ix];
            v = (unsigned int)frame_buf[ix+2];
            for (clr=0; clr<MAX_COLORS; clr++) {
                if (ymax[clr] == 0)    // skip this color if not defined
                    continue;
                if ((y >= ymin[clr])
                  && (y <= ymax[clr]) 
                  && (u >= umin[clr]) 
                  && (u <= umax[clr]) 
                  && (v >= vmin[clr]) 
                  && (v <= vmax[clr])) {
                    frame_buf[ix+1] = frame_buf[ix+3] = ymid[clr];
                    frame_buf[ix] = umid[clr];
                    frame_buf[ix+2] = vmid[clr];
                    break;
                }
            }
            if (clr == MAX_COLORS) {  // if no match, black out the pixel
                frame_buf[ix+1] = frame_buf[ix+3] = 0;
                frame_buf[ix] = frame_buf[ix+2] = 128;
            }
        }
    }
}

/* count number of pixels matching 'clr' bin in range [x1,y1] to [x2,y2] */
unsigned int vfind(unsigned char *frame_buf, unsigned int clr, 
                   unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2) {
    unsigned int ix, xx, yy, y, u, v, count;
    
    count = 0;
    for (xx=x1; xx<x2; xx+=2) {   
        for (yy=y1; yy<y2; yy++) {
            ix = index(xx,yy);
            y = (((unsigned int)frame_buf[ix+1] + (unsigned int)frame_buf[ix+3])) >> 1;
            //y = (unsigned int)frame_buf[ix+1];
            u = (unsigned int)frame_buf[ix];
            v = (unsigned int)frame_buf[ix+2];
            if ((y >= ymin[clr])
              && (y <= ymax[clr]) 
              && (u >= umin[clr]) 
              && (u <= umax[clr]) 
              && (v >= vmin[clr]) 
              && (v <= vmax[clr])) {
                count++;
            }
        }
    }
    return count;
}

void edge_detect(unsigned char *inbuf, unsigned char *outbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op;
    unsigned int *ip1, *op1;
    unsigned int gx, gy;
    
    skip = imgWidth*2;
    for (xx=2; xx<imgWidth-2; xx+=2) {   
        for (yy=1; yy<imgHeight-1; yy++) {
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;
            op = outbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            if (((int)gx > thresh) || ((int)gy > thresh)) {
                *op = 128;
                *(op+2) = 128;
                *(op+1) = 255;
                *(op+3) = 255;
            } else {
                *op = (*(ip) & 0x80) | 0x40;
                *(op+2) = (*(ip+2) & 0x80) | 0x40;
                *(op+1) = (*(ip+1) & 0xC0) | 0x20;
                *(op+3) = (*(ip+3) & 0xC0) | 0x20;
            }
        }
    }

    op1 = (unsigned int *)outbuf;
    ip1 = (unsigned int *)inbuf;
    for (ix=0; ix<(imgWidth*imgHeight>>1); ix++)
        ip1[ix] = op1[ix];
}

unsigned int vscan(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           unsigned int columns, unsigned int *outvect) {
    int x, y;
    unsigned int ix, hits;
    unsigned char *pp;
    
    svs_segcode(outbuf, inbuf, thresh);  // find edge pixels

    hits = 0;
    pp = outbuf;
    for (ix=0; ix<columns; ix++)    // initialize output vector
        outvect[ix] = imgHeight;
        
    for (y=0; y<(int)imgHeight; y++) {  // now search from top to bottom, noting each hit in the appropriate column
        for (x=0; x<(int)imgWidth; x+=2) {  // note that edge detect used full UYVY per pixel position
            if (*pp & 0xC0) {   // look for edge hit 
                outvect[((x * columns) / imgWidth)] = imgHeight - y;
                hits++;
            }
            pp++;
        }
    }
    return hits;
}

/* search for image horizon.  similar to vscan(), but search is top-to-bottom rather than bottom-to-top */
unsigned int vhorizon(unsigned char *outbuf, unsigned char *inbuf, int thresh, 
           int columns, unsigned int *outvect, int *slope, int *intercept, int filter) {
    int x, y;
    int ix, hits;
    unsigned char *pp;
    static int sfilter[10], ifilter[10];
    
    if (filter > 10) filter = 10;  // max number of filter taps
    if (filter < 1) filter = 1;
    
    svs_segcode(outbuf, inbuf, thresh);  // find edge pixels

    hits = 0;
    for (ix=0; ix<columns; ix++)    // initialize output vector
        outvect[ix] = 0;
        
    for (y=imgHeight-1; y>=0; y--) {  // now search from top to bottom, noting each hit in the appropriate column
        pp = outbuf + (y * imgWidth / 2);
        for (x=0; x<(int)imgWidth; x+=2) {  // note that edge detect used full UYVY per pixel position
            if (*pp & 0xC0) {   // look for edge hit 
                outvect[((x * columns) / imgWidth)] = y;
                hits++;
            }
            pp++;
        }
    }
    /*int sx, sy, sxx, sxy, dx, delta;
    sx = sy = sxx = sxy = 0;
    dx = imgWidth / columns;
    for (ix=0; ix<columns; ix++) {
        x = ix*dx + dx/2;
        y = (int)outvect[ix];
        sx += x;
        sy += y;
        sxx += x*x;
        sxy += x*y;
    }
    delta = columns*sxx - sx*sx;
    *slope = (1000 * (columns*sxy - sx*sx))/ delta;  // slope is scaled by 1000
    *intercept = (sxx*sy - sx*sxy) / delta; */
    int sx, sy, stt, sts, t, dx;
    sx = sy = stt = sts = 0;
    dx = imgWidth / columns;
    for (ix = 0; ix < columns; ix++) {
        sx += ix*dx + dx/2;
        sy += (int)outvect[ix];
    }
    for (ix = 0; ix < columns; ix++) {
        t = ix*dx + dx/2 - sx/columns;
        stt += t*t;
        sts += t*(int)outvect[ix];
    }
	*slope = (1000*sts)/stt;
	*intercept = (sy*1000 - sx*(*slope))/(columns*1000);
    if (*intercept > ((int)imgHeight-1)) *intercept = (int)imgHeight-1;
    if (*intercept < 0) *intercept = 0;

    if (filter > 1) {
        for (ix=filter; ix>1; ix--) {  // push old values to make room for latest
            sfilter[ix-1] = sfilter[ix-2];
            ifilter[ix-1] = ifilter[ix-2];
        }
        sfilter[0] = *slope;
        ifilter[0] = *intercept;
        sx = sy = 0;
        for (ix=0; ix<filter; ix++) {  // now average the data
            sx += sfilter[ix];
            sy += ifilter[ix];
        }
        *slope = sx / filter;
        *intercept = sy / filter;
    }
    //printf("vhorizon:  slope = %d   intercept = %d\r\n", *slope, *intercept);
    return hits;
}

void svs_segcode(unsigned char *outbuf, unsigned char *inbuf, int thresh) {
    unsigned int ix, xx, yy, y2, u2, v2, skip;
    unsigned char *ip, *op, cc;
    unsigned int gx, gy;
    
    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    
    skip = imgWidth*2;
    op = outbuf;
    for (yy=0; yy<imgHeight; yy++) {
        for (xx=0; xx<imgWidth; xx+=2) {   
            if ((xx < 2) || (xx >= imgWidth-2) || (yy < 1) || (yy >= imgHeight-1)) {
                *op++ = 0;
                continue;
            }
            gx = gy = 0;
            ix = index(xx, yy);
            ip = inbuf + ix;

            y2 = *(ip+5) + *(ip+7) - *(ip-3) - *(ip-1);
            u2 = *(ip+4) - *(ip-4);
            v2 = *(ip+6) - *(ip-2);
            gy = ((y2*y2)>>2) + u2*u2 + v2*v2;
            
            y2 = *(ip+1+skip) + *(ip+3+skip) - *(ip+1-skip) - *(ip+3-skip);
            u2 = *(ip+skip) - *(ip-skip); 
            v2 = *(ip+2+skip) - *(ip+2-skip); 
            gx = ((y2*y2)>>2) + u2*u2 + v2*v2;
 
            cc = ((*ip >> 2) & 0x38)       // threshold U to 0x38 position
               + ((*(ip+2) >> 5) & 0x07);  // threshold V to 0x07 position
            if ((int)gx > thresh)  
                cc |= 0x80;               // add 0x80 flag if this is edge pixel
            if ((int)gy > thresh)  
                cc |= 0x40;               // add 0x40 flag if this is edge pixel
            *op++ = cc;
        }
    }
}

void svs_segview(unsigned char *inbuf, unsigned char *outbuf) {
    unsigned int ix;
    unsigned char *ip, *op;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    
    ip = inbuf;
    op = outbuf;
    for (ix=0; ix<imgWidth*imgHeight; ix+=2) {   
        if (*ip & 0xC0) {      // is this an edge pixel ?
            *(op+1) = *(op+3) = 0xFF;
            *op = *(op+2) = 0x80;
        } else {
            *(op+1) = *(op+3) = 0xA0;
            *op = ((*ip & 0x38) << 2) + 0x10;
            *(op+2) = ((*ip & 0x07) << 5) + 0x10;
        }
        op += 4;
        ip++;
    }
}

/* display vector as red pixels (YUV = 72 84 255) */
void addvect(unsigned char *outbuf, unsigned int columns, unsigned int *vect)
{
    unsigned int xx, yy, ix;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    for (xx=0; xx<imgWidth; xx++) {
        yy = (imgHeight) - (vect[(xx * columns) / imgWidth]);
        ix = index(xx, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 72;
        outbuf[ix] = 84;
        outbuf[ix+2] = 255;
    }
}

/* display line as red pixels (red YUV = 72 84 255) (yellow YUV = 194 18 145)
   note that slope is scaled up by 1000 */
void addline(unsigned char *outbuf, int slope, int intercept)
{
    int xx, yy, ix;

    if (imgWidth > 640)   // buffer size limits this function to 640x480 resolution
        return;
    for (xx=0; xx<(int)imgWidth; xx+=2) {
        yy = ((slope * xx) / 1000) + intercept; 
        if (yy > ((int)imgHeight-1)) yy = (int)imgHeight - 1;
        if (yy < 0) yy = 0; 
        ix = index(xx, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 72;
        outbuf[ix] = 84;
        outbuf[ix+2] = 255;
    }
}

/* display a box as red pixels (YUV = 72 84 255) or yellow pixels (YUV = 194 18 145) */
void addbox(unsigned char *outbuf, unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2)
{
    unsigned int xx, yy, ix;

    for (xx=x1; xx<=x2; xx+=2) {
        ix = index(xx, y1);
        outbuf[ix+1] =  outbuf[ix+3] = 194;
        outbuf[ix] = 18;
        outbuf[ix+2] = 145;
        ix = index(xx, y2);
        outbuf[ix+1] =  outbuf[ix+3] = 194;
        outbuf[ix] = 18;
        outbuf[ix+2] = 145;
    }
    for (yy=y1; yy<=y2; yy++) {
        ix = index(x1, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 194;
        outbuf[ix] = 18;
        outbuf[ix+2] = 145;
        ix = index(x2, yy);
        outbuf[ix+1] =  outbuf[ix+3] = 194;
        outbuf[ix] = 18;
        outbuf[ix+2] = 145;
    }
}





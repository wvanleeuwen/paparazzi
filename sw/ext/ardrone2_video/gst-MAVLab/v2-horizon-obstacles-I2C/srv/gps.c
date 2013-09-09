/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  gps.c - navigation functions for the SRV-1 / SVS Blackfin controllers
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

#include "gps.h"
#include "print.h"
#include "malloc.h"
#include "string.h"
#include "srv.h"
#include "uart.h"
#include "i2c.h"

/* typedef struct gps_data {
    int lat;
    int lon;
    int alt;
    int fix;
    int sat;
    int utc;
}; 

#define METERS_PER_DEGREE 111319.5
#define FEET_PER_DEGREE  365221
#define MILES_PER_DEGREE 69.17   // equatorial  (68.939 polar)

$GPGGA,173415.400,3514.5974,N,12037.2028,W,1,8,1.18,84.6,M,-30.8,M,,*50
$GPGGA,hhmmss.sss,ddmm.mmmm,n,dddmm.mmmm,e,q,ss,y.y,a.a,z,g.g,z,t.t,iii*CC
    where:
        GPGGA        : GPS fixed data identifier
        hhmmss.ss    : Coordinated Universal Time (UTC), also known as GMT
        ddmm.mmmm,n  : Latitude in degrees, minutes and cardinal sign
        dddmm.mmmm,e : Longitude in degrees, minutes and cardinal sign
        q            : Quality of fix.  1 = there is a fix
        ss           : Number of satellites used
        y.y          : Horizontal precision
        a.a,M        : GPS antenna altitude in meters
        g.g,M        : geoidal separation in meters
        t.t          : age of the differential correction data
        iiii         : Differential station's ID
        *CC          : Checksum 
*/

extern unsigned int uart1_flag;
struct gps_data gps_gga;
static int ublox = 0;
static int locosys = 0;

void gps_show() {
    if (!gps_parse())
        printf("no response from gps\r\n");
    if (ublox)
        printf("##gps: i2c\r\n");
    else
        printf("##gps: uart\r\n");
    printf("gps lat: %d\r\n", gps_gga.lat);
    printf("gps lon: %d\r\n", gps_gga.lon);
    printf("gps alt: %d\r\n", gps_gga.alt);
    printf("gps fix: %d\r\n", gps_gga.fix);
    printf("gps sat: %d\r\n", gps_gga.sat);
    printf("gps utc: %d\r\n", gps_gga.utc);
}

unsigned char read_ublox() {
    unsigned char idat[2];
    int ix;

    for (ix=0; ix<10; ix++) {  
        idat[0] = 0;  
        i2cslowread(0x42, (unsigned char *)idat, 1, SCCB_ON);
        if (idat[0] != 0xff)   // if there's no data, i2cread() reads 0xff
            return idat[0];
        delayUS(100);
    }
    return 0xff;  // no data
}

int gps_parse() {
    unsigned char buf[100];
    unsigned char ch;
    unsigned int t0, sum, pow10, div10;
    int i1, i2, ilast, ix;
    int latdeg, londeg, latmin, lonmin;
    
    i1 = i2 = ilast = 0;  // to get rid of compiler warnings

    if ((ublox == 0) && (locosys == 0)) {  // check first for ublox on i2c channel 042
        for (ix=0; ix<250; ix++) {
            ch = read_ublox();
            delayUS(500);
            if (ch != 0xff) {
                ublox = 1;
                break;
            }
        }
        if (!ublox) {
            init_uart1(9600);
            uart1_flag = 1;
            locosys = 1;
        }
    }

    t0 = readRTC();  // set up for 1-sec timeout
    while (1) {
        if ((readRTC() - t0) > 1000) { // check for timeout
            gps_gga.fix = 0;
            gps_gga.sat = 0;
            return 0;
        }
        if (ublox) {
            ch = read_ublox();
            if (ch == 0xff)
                continue;
        } else {
            if (!uart1GetChar(&ch)) 
                continue;
        }
        if (ch != '$')   // look for "$GPGGA," header
            continue;
        for (i1=0; i1<6; i1++) { 
            if (ublox)
                buf[i1] = read_ublox();
            else
                buf[i1] = uart1GetCh();
        }
        if ((buf[2] != 'G') || (buf[3] != 'G') || (buf[4] != 'A'))
            continue;
        for (i1=0; i1<100; i1++) {
            if (ublox)
                buf[i1] = read_ublox();
            else
                buf[i1] = uart1GetCh();

            if (buf[i1] == '\r') {
                buf[i1] = 0;
                ilast = i1;
                break;
            }
        }
        //printf("$GPGGA,%s\r\n", buf);

        // i1 = start of search, i2 = end of search (comma), ilast = end of buffer

        // parse utc
        i1 = 0;
        sum = 0;
        pow10 = 1;
        div10 = 0;
        for (ix=0; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.') {
                div10 = 1;
                continue;
            }
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
            div10 *= 10;
        }
        div10 = pow10 / div10;
        gps_gga.utc = sum / div10;
        
        // parse lat
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        div10 = 0;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.') {
                div10 = 1;
                continue;
            }
            sum += pow10 * (buf[ix] & 0x0F);
            if (ix>i1) {  // to prevent overflow
                pow10 *= 10;
                div10 *= 10;
            }
        }
        div10 = pow10 / div10;
        latdeg = sum / (div10*100);
        latmin = sum - (latdeg*div10*100);
        latmin = (latmin * 100) / 60;  // convert to decimal minutes
        gps_gga.lat = (latdeg*div10*100) + latmin;
        if (div10 > 10000)
            gps_gga.lat /= (div10 / 10000);  // normalize lat minutes to 6 decimal places
            
        // skip N/S field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == 'S')
                gps_gga.lat = -gps_gga.lat;
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse lon
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        div10 = 0;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.') {
                div10 = 1;
                continue;
            }
            sum += pow10 * (buf[ix] & 0x0F);
            if (ix>i1) {  // to prevent overflow
                pow10 *= 10;
                div10 *= 10;
            }
        }
        div10 = pow10 / div10;
        londeg = sum / (div10*100);
        lonmin = sum - (londeg*div10*100);
        lonmin = (lonmin * 100) / 60;  // convert to decimal minutes
        gps_gga.lon = (londeg*div10*100) + lonmin;        
        if (div10 > 10000)
            gps_gga.lon /= (div10 / 10000);  // normalize lon minutes to 6 decimal places

        // skip E/W field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == 'W')
                gps_gga.lon = -gps_gga.lon;
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse fix
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        gps_gga.fix = buf[i2-1] & 0x0F;
        
        // parse satellites
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.sat = sum;
        
        // skip horz-precision field
        i1 = i2+1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }

        // parse alt
        i1 = i2+1;
        sum = 0;
        pow10 = 1;
        for (ix=i1; ix<ilast; ix++) {
            if (buf[ix] == ',') {
                i2 = ix;
                break;
            }
        }
        for (ix=(i2-1); ix>=i1; ix--) {
            if (buf[ix] == '.')
                continue;
            sum += pow10 * (buf[ix] & 0x0F);
            pow10 *= 10;
        }
        gps_gga.alt = sum / 10;
        
        return 1;
    }
}

static int cosine[] = {
10000, 9998, 9994, 9986, 9976, 9962, 9945, 9925, 9903, 9877, 
 9848, 9816, 9781, 9744, 9703, 9659, 9613, 9563, 9511, 9455, 
 9397, 9336, 9272, 9205, 9135, 9063, 8988, 8910, 8829, 8746, 
 8660, 8572, 8480, 8387, 8290, 8192, 8090, 7986, 7880, 7771, 
 7660, 7547, 7431, 7314, 7193, 7071, 6947, 6820, 6691, 6561, 
 6428, 6293, 6157, 6018, 5878, 5736, 5592, 5446, 5299, 5150, 
 5000, 4848, 4695, 4540, 4384, 4226, 4067, 3907, 3746, 3584, 
 3420, 3256, 3090, 2924, 2756, 2588, 2419, 2250, 2079, 1908, 
 1736, 1564, 1392, 1219, 1045,  872,  698,  523,  349,  175, 
    0 
};

int sin(int ix)
{
    while (ix < 0)
        ix = ix + 360;
    while (ix >= 360)
        ix = ix - 360;
    if (ix < 90)  return cosine[90-ix] / 10;
    if (ix < 180) return cosine[ix-90] / 10;
    if (ix < 270) return -cosine[270-ix] / 10;
    if (ix < 360) return -cosine[ix-270] / 10;
    return 0;
}

int cos(int ix)
{
    while (ix < 0)
        ix = ix + 360;
    while (ix >= 360)
        ix = ix - 360;
    if (ix < 90)  return cosine[ix] / 10;
    if (ix < 180) return -cosine[180-ix] / 10;
    if (ix < 270) return -cosine[ix-180] / 10;
    if (ix < 360) return cosine[360-ix] / 10;
    return 0;
}

int tan(int ix)
{
    while (ix < 0)
        ix = ix + 360;
    while (ix >= 360)
        ix = ix - 360;
    if (ix == 90)  return 9999;
    if (ix == 270) return -9999;
    if (ix < 90)   return (1000 * cosine[90-ix]) / cosine[ix];
    if (ix < 180)  return -(1000 * cosine[ix-90]) / cosine[180-ix];
    if (ix < 270)  return (1000 * cosine[270-ix]) / cosine[ix-180];
    if (ix < 360)  return -(1000 * cosine[ix-270]) / cosine[360-ix];
    return 0;
}

int asin(int y, int hyp)
{
    int quot, sgn, ix;
    if ((y > hyp) || (y == 0))
        return 0;
    sgn = hyp * y;
    if (hyp < 0) 
        hyp = -hyp;
    if (y < 0)
        y = -y;
    quot = (y * 10000) / hyp;
    if (quot > 9999)
        quot = 9999;
    for (ix=0; ix<90; ix++)
        if ((quot < cosine[ix]) && (quot >= cosine[ix+1]))
            break;
    if (sgn < 0)
        return -(90-ix);
    else
        return 90-ix;
}

int acos(int x, int hyp)
{
    int quot, sgn, ix;
    if (x > hyp) 
        return 0;
    if (x == 0) {
        if (hyp < 0)
            return -90;
        else
            return 90;
        return 0;
    }
    sgn = hyp * x;
    if (hyp < 0) 
        hyp = -hyp;
    if (x < 0)
        x = -x;
    quot = (x * 10000) / hyp;
    if (quot > 9999)
        quot = 9999;
    for (ix=0; ix<90; ix++)
        if ((quot < cosine[ix]) && (quot >= cosine[ix+1]))
            break;
    if (sgn < 0)
        return -ix;
    else
        return ix;
}

int atan(int y, int x)
{
    int angle, flip, t, xy;

    if (x < 0) x = -x;
    if (y < 0) y = -y;
    flip = 0;  if (x < y) { flip = 1; t = x;  x = y;  y = t; }
    if (x == 0) return 90;
    
    xy = (y*1000)/x;
    angle = (360 * xy) / (6283 + ((((1764 * xy)/ 1000) * xy) / 1000));
    if (flip) angle = 90 - angle;
    return angle;
} 

unsigned int isqrt(unsigned int val) {
    unsigned int temp, g=0, b = 0x8000, bshft = 15;
    do {
        if (val >= (temp = (((g << 1) + b)<<bshft--))) {
           g += b;
           val -= temp;
        }
    } while (b >>= 1);
    return g;
}

/* calculates heading between two gps sets of gps coordinates
   heading angle corresponds to compass points (N = 0-deg) */
int gps_head(int lat1, int lon1, int lat2, int lon2)
{
    int x, y, sy, sx, ll, ang;
    
    sx = sy = 0;  // sign bits

    y = lat2 - lat1;
    if (y < 0) { sy = 1; y = -y; }

    ll = lat1;  
    if (ll < 0) 
        ll = -lat1;
    x = ((lon2 - lon1) * cos(ll / 1000000)) / 1000;
    if (x < 0) { sx = 1; x = -x; }
    
    ang = atan(y, x);
    if ((sx==0) && (sy==0))
        ang = 90 - ang;
    if ((sx==0) && (sy==1))
        ang = 90 + ang;
    if ((sx==1) && (sy==1))
        ang = 270 - ang;
    if ((sx==1) && (sy==0))
        ang = 270 + ang;
    return ang;
}

/* calculates distance between two gps coordinates in meters */
int gps_dist(int lat1, int lon1, int lat2, int lon2)
{
    int x, y, ll;
    
    ll = lat1;  
    if (ll < 0) 
        ll = -lat1;
    y = lat2 - lat1;
    if (y < 0) y = -y;
    x = ((lon2 - lon1) * cos(ll / 1000000)) / 1000;
    if (x < 0) x = -x;
    if ((x > 10000) || (y > 10000)) {
        x = x / 10000;
        y = y / 10000;
        return isqrt(x*x + y*y) * 1113;   
    } else {
        return (isqrt(x*x + y*y) * 1113) / 10000;   
    }
}


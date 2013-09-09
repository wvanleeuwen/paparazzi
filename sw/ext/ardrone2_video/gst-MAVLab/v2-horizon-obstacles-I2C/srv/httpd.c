/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  httpd.c - HTTP GET and POST functions for the SRV-1 / SVS 
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

#include <cdefBF537.h>
#include "srv.h"
#include "uart.h"
#include "string.h"
#include "print.h"
#include "malloc.h"
#include "jpeg.h"
#include "debug.h"
#include "stdlib.h"
#include "stm_m25p32.h"
#include "gps.h"
#include "colors.h"
#include "i2c.h"

extern int picoc(char *);

#define REQBUF_SIZE             4096
#define INLINEIMGBUF_SIZE       (64 * 1024)

#define FLASH_SECTOR_SIZE       (64 * 1024)
#define FLASH_SECTORS_FIRST     4
#define FLASH_SECTORS_LAST      62
#define FLASH_SECTORS_UNIT      2

#define BOOT_LOADER_SECTORS     2
#define BOOT_LOADER_MIN_SIZE    (16 * 1024)
#define BOOT_LOADER_MAX_SIZE    (BOOT_LOADER_SECTORS * FLASH_SECTOR_SIZE)

char adminHtml[] =
#include "www/admin.html.c";

static char robotJpg[] = "/robot.jpg";
static char robotCgi[] = "/robot.cgi?";
static char adminPath[] = "/admin";

static char *fileNames[] = {  // index from file name to flash sector:  
    "/00.html",  // sector 10-11
    "/01.html",  // sector 12-13
    "/02.html",  // sector 14-15
    "/03.html",  // sector 16-17
    "/04.html",  // sector 18-19
    "/05.html",  // sector 20-21
    "/06.html",  // sector 22-23
    "/07.html",  // sector 24-25
    "/08.html",  // sector 26-27
    "/09.html"   // sector 28-29
};

#define CONNECTION_HEADER   "Connection: Close\r\n"       // better in IE, Safari

static char cgiBody[] = "ok\r\n";
static char body404[] = "File not found\r\n";
static char body501[] = "Request format not supported\r\n";
static char resultCode404[] = "HTTP/1.1 404 File not found";
static char resultCode501[] = "HTTP/1.1 501 Request format not supported";

// Get the value of an HTTP header as a string
static  BOOL    getHdrString (              // Returns TRUE if header found
char *          hdrs,                       // String containing all headers
char *          hdrName,                    // Header name to find, including ": " delimiter
char *          value,                      // String buffer containing returned value. Returns an empty string
                                            //  (value[0] = 0) if header not found
int             valMaxChars)                // Size of value buffer in characters. If header value exceeds this size,
                                            //  FALSE is returned
{
    int valChars = 0;
    int hdrNameLen = strlen (hdrName);

    value[0] = 0;   // Return empty string if header not found

    // Scan lines in the headers
    while (*hdrs != 0) {
        // If header name matches, return the value
        if (strncmp (hdrs, hdrName, hdrNameLen) == 0) {
            hdrs += hdrNameLen;
            while (*hdrs != 0  &&  *hdrs != '\r'  &&  *hdrs != '\n') {
                if (valChars == valMaxChars - 1)
                    return FALSE;
                value[valChars++] = *hdrs++;
            }
            value[valChars] = 0;
            return TRUE;        
        }

        // Find the beginning of the next line
        else {
            while (*hdrs != 0  &&  *hdrs != '\r'  &&  *hdrs != '\n')
                ++hdrs;
            if (*hdrs != 0)
                while (*hdrs == '\r'  ||  *hdrs == '\n')
                    ++hdrs;
        }
    }
    return FALSE;
}


/* Get the value of an HTTP header as a decimal int */
static  BOOL    getHdrDecimal (             // Returns TRUE if header found
char *          hdrs,                       // String containing all headers
char *          hdrName,                    // Header name to find, including ": " delimiter
int *           value)                      // Out: returned header value. Unchanged if header not found
{
    char valStr[16];
    if (!getHdrString (hdrs, hdrName, valStr, countof (valStr)))
        return FALSE;
    *value = atoi (valStr);
    return TRUE;
}


/* Get the value of an HTTP "name=value" parameter from a header string or url parameter list
   Handles both header strings like:
      Content-Disposition: form-data; name="file1"; filename="Terrier1.jpg"
   and URL parameters like:
      path?name=file1&filename=Terrier1.jpg
   Removes any quotes surrounding value.*/
static  BOOL    getParam (      // Returns TRUE if paramName found; FALSE if not found or valMaxChars too small
char *          params,         // URL/header parameter string
char *          paramName,      // Name of parameter, including equals sign, as in "name="
char *          value,          // Out: value. Returns empty string if header not found or buffer too small.
int             valMaxChars)    // In: maximum size of value buffer, including null terminator
{
    value[0] = 0;
    char * targ = strstr (params, paramName);
    if (targ == NULL) {
        return FALSE;
    }

    BOOL quoted = FALSE;
    targ += strlen (paramName);
    if (*targ == '"') {
        quoted = TRUE;
        ++targ;
    }

    int valChars = 0;
    while (TRUE)
    {
        char c = *targ++;

        if (valChars >= valMaxChars - 1)
        {
            value[0] = 0;
            return FALSE;
        }
        if (c == 0)
            break;
        if (quoted)
        {
            if (c == '"'  ||  c == '\r'  ||  c == '\n')
                break;
        }
        else
        {
            if (c == ';'  ||  c == ' '  ||  c == '&'  ||  c == '\r'  ||  c == '\n')
                break;
        }
        value[valChars++] = c;
    }
    value[valChars] = 0;
    return TRUE;
}


int clean_buffer(char *buf) {
    char *cp;
    
    for (cp = (buf+0x1FFFF); cp > buf; cp--) {  // sweep buffer from end clearing out garbage characters
        if (*cp == '>') { // final html character
            cp[1] = '\r';
            cp[2] = '\n';
            return cp - buf + 3;
        }
        else
            *cp = 0;
    }
    return 0;
}


/* Build the response body for the admin page*/
char *  adminBodyBuilder (      // Returns malloc-ed body string. Caller must free this. NULL if alloc error
char *  statusMsg,              // Status message string to display. NULL if none
char *  statusColor,            // Status message color as HTML color, e.g., "#ff0000". NULL if default
int *   contentLength)          // Out: length of response body
{
    static char statusMsgPat[] =   "$$statusMsg$$";
    static char statusColorPat[] = "$$statusColor$$";
    static char versionPat[] =     "$$version$$";
    static char defStatusColor[] = "#ffffff";

    *contentLength = countof (adminHtml) - 1
                        + (statusMsg == NULL ? 0 : strlen (statusMsg))
                        + (statusColor == NULL ? countof(defStatusColor) - 1 : strlen (statusColor))
                        + strlen ((char *) version_string);
    char * body = malloc (*contentLength + 1);
    memcpy ((unsigned char *) body, (unsigned char *) adminHtml, countof (adminHtml));
    if (body != NULL) {
        strReplace (body, *contentLength, statusMsgPat, statusMsg != NULL ? statusMsg : "");
        strReplace (body, *contentLength, statusColorPat, statusColor != NULL ? statusColor : defStatusColor);
        strReplace (body, *contentLength, versionPat, (char *) version_string);
        *contentLength -= countof (statusMsgPat) - 1 + countof (statusColorPat) - 1 + countof (versionPat) - 1;
    }
    return body;
}


void    httpd_request (char firstChar)
{
    char *reqBuf = (char *)HTTP_BUFFER; 
    char *cgiResponse = (char *)HTTP_BUFFER2; 

    int ret, t0;
    char ch, *cp;

    // request fields
    char * method;
    char * path;
    char * protocol;
    char * headers;

    // response fields
    char * body = NULL;
    BOOL freeBody = FALSE;
    char * resultCode = "HTTP/1.1 200 OK";
    char * contentType = "text/html";
    int contentLength = 0, flashContentLength = 0, new_content = 0;

    BOOL deferredReset = FALSE;

    // command variables
    unsigned int channel;
    unsigned char i2c_device, i2c_data[16];
    unsigned char ch1, ch2, ch3, ch4;
    unsigned int x1, x2;
    unsigned int ix, iy, i1, i2;
    unsigned int ulo[4], uhi[4], vlo[4], vhi[4];
    short x, y, z;
    int vect[16];  // used by vscan()
    
    // Receive the request and headers
    reqBuf[0] = firstChar;

    // GET method
    //if (strcmp (method, "GET") == 0) {
    if (reqBuf[0] == 'G') {
        ret = 1;
        t0 = readRTC();
        while (readRTC() - t0 < 1000  &&  ret < REQBUF_SIZE) {
            if (getchar((unsigned char *) &ch))
            {
                char pch = reqBuf[ret - 1];     // ret always >= 1
                reqBuf[ret++] = ch;
                       // Read to the end of the headers: handle any permutation of empty line EOL sequences
                       // Apparently some clients just terminate lines with LF
                if ((ret >= 4  &&  strncmp (reqBuf + ret - 4, "\r\n\r\n", 4) == 0)  ||
                    (pch == 0x0d  &&  ch == 0x0d)  ||  (pch == 0x0a  &&  ch == 0x0a))
                    break;
            }
        }
        reqBuf[ret] = 0;
        reqBuf[ret+1] = 0;  // for benefit of serial_out_httpbuffer(), which has to handle nulls inserted by strtok()

        // Parse the request fields
        method = strtok(reqBuf, " ");
        path = strtok(0, " ");
        protocol = strtok(0, "\r\n");
        headers = protocol + strlen (protocol) + 1;
        while (*headers != 0  &&  (*headers == '\r'  ||  *headers == '\n'))
            ++headers;        

        if (!method || !path || !protocol) 
            goto exit;

        // Camera image binary - robot.jpg
        if (strncmp(path, robotJpg, countof (robotJpg) - 1) == 0) {
            grab_frame();
            led1_on();
            body = (char *) JPEG_BUF;
            contentLength = encode_image((unsigned char *)FRAME_BUF, (unsigned char *) body, quality,
                                        FOUR_TWO_TWO, imgWidth, imgHeight)
                            - (unsigned char *) body;
            contentType = "image/jpeg";
        }

        // Robot control - robot.cgi  
        // drive options - m=SRV1, x=SRV-4WD, s=ESC, t=tilt, l=laser
        else if (strncmp(path, robotCgi, countof(robotCgi) - 1) == 0) {
            char * params = path + countof (robotCgi) - 1;
            new_content = 0;
            cp = (char *)HTTP_BUFFER2;  // clear the response buffer
            for (ix=0; ix<HTTP_BUFFER2_SIZE; ix++)
                *cp++ = 0;
            switch (params[0]) {
                case 'V':  // version string
                    body = cgiResponse;  // use HTTP_BUFFER2
                    sprintf(body, "%s\r\n", version_string);
                    contentLength = strlen((char *)body);
                    contentType = "text/plain";
                    new_content = 1;
                    break;
                case '$': 
                    switch (params[1]) {
                        case '!':   // Blackfin reset
                            deferredReset = TRUE;   // defer until after we send the response
                            break;
                        case 'A':  // SRV-NAV or RCM A/D
                            channel = ((unsigned int)(params[2] & 0x0F) * 10) + (unsigned int)(params[3] & 0x0F);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%04d\r\n", analog(channel));
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'a':  // SRV-4WD A/D
                            channel = (unsigned int)(params[2] & 0x0F);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%04d\r\n", analog_4wd(channel));
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'c':  // SRV-NAV compass
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%04d\r\n", read_compass3x(&x, &y, &z));
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'e':  // SRV-4WD motor encoder
                            channel = (unsigned int)(params[2] & 0x0F);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%04d\r\n", encoder_4wd(channel));
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'g':  // gps
                            body = cgiResponse;  // use HTTP_BUFFER2
                            gps_parse();
                            sprintf(body, "gps lat: %d\r\ngps lon: %d\r\ngps alt: %d\r\ngps fix: %d\r\ngps sat: %d\r\ngps utc: %d\r\n", 
                                gps_gga.lat, gps_gga.lon, gps_gga.alt, gps_gga.fix, gps_gga.sat, gps_gga.utc);
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                       case 'T':  // SRV-NAV tilt
                            ch1 = 0;
                            switch((unsigned int)(params[2] & 0x0F)) {  // channel 1 = x, 2 = y, 3 = z 
                                case 1:  // x axis
                                    ch1 = 0x28;
                                    break;  
                                case 2:  // y axis
                                    ch1 = 0x2A;
                                    break;  
                                case 3:  // z axis
                                    ch1 = 0x2C;
                                    break;  
                            }
                            if (ch1) {
                                i2c_data[0] = ch1;
                                i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
                                ix = (unsigned int)i2c_data[0];
                                i2c_data[0] = ch1 + 1;
                                delayUS(1000);
                                i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
                                ix += (unsigned int)i2c_data[0] << 8;
                                body = cgiResponse;  // use HTTP_BUFFER2
                                sprintf(body, "%04d\r\n", ix);
                                contentLength = strlen((char *)body);
                                contentType = "text/plain";
                                new_content = 1;
                            }
                            break;
                    }
                    break;
                case 'a':
                    camera_reset(160);
                    break; 
                case 'b':
                    camera_reset(320);
                    break; 
                case 'c':
                    camera_reset(640);
                    break; 
                case 'd':
                    camera_reset(1280);
                    break; 
                case 'g':  
                    switch (params[1]) {
                        case '0':
                            frame_diff_flag = 1;
                            grab_reference_frame();
                            break;
                        case '1':
                            segmentation_flag = 1;
                            break;
                        case '2':
                            edge_detect_flag = 1;
                            edge_thresh = 3200;
                            break;
                        case '3':
                            horizon_detect_flag = 1;
                            edge_thresh = 3200;
                            break;
                        case '4':
                            obstacle_detect_flag = 1;
                            edge_thresh = 3200;
                            break;
                        #ifdef STEREO
                        case '5':
                            if (master)
                                stereo_processing_flag = 1;
                            break;
                        #endif /* STEREO */
                        case '6':
                            blob_display_flag = 1;
                            blob_display_num = params[2] & 0x0F;
                            break;
                        default:  // no match - turn them all off
                            frame_diff_flag = 0;
                            segmentation_flag = 0;
                            edge_detect_flag = 0;
                            horizon_detect_flag = 0;
                            obstacle_detect_flag = 0;
                            blob_display_flag = 0;
                            #ifdef STEREO
                            stereo_processing_flag = 0;
                            #endif /* STEREO */
                            break;
                    }
                    break;
                case 'q':  // set JPEG quality 1 - 8 (1 is highest)
                    if ((params[1]<'1') || (params[1]>'8'))  // command out of range ?
                        break;
                    quality = params[1] & 0x0F;
                    break; 
                case 'i': 
                    switch (params[1]) {
                        case 'r':  // single byte I2C read
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%02x\r\n", i2c_data[0]);
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'R':  // double byte (short) I2C read
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "%04x\r\n", ((i2c_data[0] << 8) + i2c_data[1]));
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'w':  // single byte I2C write
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[6]);
                            i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                            break;
                        case 'W':  // double byte (short) I2C write
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[6]);
                            i2c_data[2] = (unsigned char)atoi_b16(&params[8]);
                            i2cwritex(i2c_device, (unsigned char *)i2c_data, 3, SCCB_ON);
                            break;
                        case 'd':  // dual channel single byte I2C write
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[6]);
                            i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                            delayUS(1000);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[8]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[10]);
                            i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
                            break;
                        case 'D':  // dual channel double byte (short) I2C write
                            i2c_device = (unsigned char)atoi_b16(&params[2]);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[4]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[6]);
                            i2c_data[2] = (unsigned char)atoi_b16(&params[8]);
                            i2cwritex(i2c_device, (unsigned char *)i2c_data, 3, SCCB_ON);
                            delayUS(1000);
                            i2c_data[0] = (unsigned char)atoi_b16(&params[10]);
                            i2c_data[1] = (unsigned char)atoi_b16(&params[12]);
                            i2c_data[2] = (unsigned char)atoi_b16(&params[14]);
                            i2cwritex(i2c_device, (unsigned char *)i2c_data, 3, SCCB_ON);
                            break;
                    }
                    break;
                case 'v':  // process colors
                    //    va = enable/disable AGC / AWB / AEC camera controls
                    //    vb = find blobs matching color bin 
                    //    vc = set color bin ranges
                    //    vm = mean colors
                    //    vp = sample individual pixel
                    //    vr = recall color bin ranges
                    //    vs = scan for edges
                    //    vt = set edge detect threshold (0000-9999, default is 3200)
                    //    vz = zero all color settings
                    switch (params[1]) {
                        case 'a':  //    va = enable/disable AGC(4) / AWB(2) / AEC(1) camera controls
                                   //    va7 = AGC+AWB+AEC on   va0 = AGC+AWB+AEC off
                            ix = params[2] & 0x07;
                            i2c_data[0] = 0x13;
                            i2c_data[1] = 0xC0 + ix;
                            i2cwrite(0x30, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV9655
                            i2cwrite(0x21, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV7725
                            break;
                        case 'b':  //    vb = find blobs for a given color
                            ch1 = params[2];
                            ch2 = ch1;
                            if (ch1 > '9')
                                ch1 = (ch1 & 0x0F) + 9;
                            else
                                ch1 &= 0x0F;
                            grab_frame();
                            ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, ch1);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            if (ix == 0xFFFFFFFF) {
                                sprintf(body, "##vb%c -1\r\n", ch2);
                                contentLength = strlen((char *)body);
                                break;  // too many blobs found
                            }
                            for (iy=0; iy<ix; iy++) {
                                sprintf(&body[strlen((char *)body)], " %d - %d %d %d %d  \r\n", 
                                    blobcnt[iy], blobx1[iy], blobx2[iy], bloby1[iy], bloby2[iy]);
                            }
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'c':  //    vc = set colors
                            ix = (unsigned int)params[2];
                            if (ix > '9')
                                ix = (ix & 0x0F) + 9;
                            else
                                ix &= 0x0F;
                            ch1 = params[3] & 0x0F;
                            ch2 = params[4] & 0x0F;
                            ch3 = params[4] & 0x0F;
                            ymin[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            ch1 = params[6] & 0x0F;
                            ch2 = params[7] & 0x0F;
                            ch3 = params[8] & 0x0F;
                            ymax[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            ch1 = params[9] & 0x0F;
                            ch2 = params[10] & 0x0F;
                            ch3 = params[11] & 0x0F;
                            umin[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            ch1 = params[12] & 0x0F;
                            ch2 = params[13] & 0x0F;
                            ch3 = params[14] & 0x0F;
                            umax[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            ch1 = params[15] & 0x0F;
                            ch2 = params[16] & 0x0F;
                            ch3 = params[17] & 0x0F;
                            vmin[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            ch1 = params[18] & 0x0F;
                            ch2 = params[19] & 0x0F;
                            ch3 = params[20] & 0x0F;
                            vmax[ix] = ch1 * 100 + ch2 * 10  + ch3;
                            break;
                        case 'm':  //    vm = mean colors
                            grab_frame();
                            vmean((unsigned char *)FRAME_BUF);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "vmean %d %d %d\r\n", mean[0], mean[1], mean[2]);
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'p':  //    vp = sample individual pixel, print YUV value
                            ch1 = params[2] & 0x0F;
                            ch2 = params[3] & 0x0F;
                            ch3 = params[4] & 0x0F;
                            ch4 = params[5] & 0x0F;
                            i1 = ch1*1000 + ch2*100 + ch3*10 + ch4;
                            ch1 = params[6] & 0x0F;
                            ch2 = params[7] & 0x0F;
                            ch3 = params[8] & 0x0F;
                            ch4 = params[9] & 0x0F;
                            i2 = ch1*1000 + ch2*100 + ch3*10 + ch4;
                            grab_frame();
                            ix = vpix((unsigned char *)FRAME_BUF, i1, i2);
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "pix %d %d %d\r\n",
                                ((ix>>16) & 0x000000FF),  // Y1
                                ((ix>>24) & 0x000000FF),  // U
                                ((ix>>8) & 0x000000FF));   // V
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 'r':  //    vr = recall colors
                            ix = (unsigned int)params[2];
                            if (ix > '9')
                                ix = (ix & 0x0F) + 9;
                            else
                                ix &= 0x0F;
                            body = cgiResponse;  // use HTTP_BUFFER2
                            sprintf(body, "vr %d %d %d %d %d %d %d\r\n",
                               ix, ymin[ix], ymax[ix], umin[ix], umax[ix], vmin[ix], vmax[ix]);
                            contentLength = strlen((char *)body);
                            contentType = "text/plain";
                            new_content = 1;
                            break;
                        case 's':  //    vs = scan for edges 
                            x1 = (unsigned int)params[2] & 0x0F;  // get number of columns to use
                            grab_frame();
                            ix = vscan((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, (unsigned int)x1, (unsigned int *)&vect[0]);
                            printf("vscan = %d ", ix);
                            for (i1=0; i1<x1; i1++)
                                printf("%4d ", vect[i1]);
                            printf("\r\n");
                            break;
                        case 't':  //    vt = set edge detect threshold (0000-9999, default is 3200)
                            ch1 = params[2] & 0x0F;
                            ch2 = params[3] & 0x0F;
                            ch3 = params[4] & 0x0F;
                            ch4 = params[5] & 0x0F;
                            edge_thresh = ch1*1000 + ch2*100 + ch3*10 + ch4;
                            break;
                        case 'z':  //    vz = clear or segment colors
                            switch (params[2] & 0x0F) {
                                case 0:
                                    for(ix = 0; ix<MAX_COLORS; ix++) 
                                        ymin[ix] = ymax[ix] = umin[ix] = umax[ix] = vmin[ix] = vmax[ix] = 0;
                                    break;
                                case 1:
                                    for(ix = 0; ix<MAX_COLORS; ix++) {
                                        ymin[ix] = (ix / 4) * 64;
                                        ymax[ix] = ymin[ix] + 63;
                                        umin[ix] = (ix & 0x02) * 64;
                                        umax[ix] = umin[ix] + 127;
                                        vmin[ix] = (ix & 0x01) * 128;
                                        vmax[ix] = vmin[ix] + 127;
                                    }
                                    break;
                                case 2:
                                    for(ix = 0; ix<MAX_COLORS; ix++) {
                                        ymin[ix] = 0;
                                        ymax[ix] = 255;
                                        umin[ix] = (ix >> 2) * 64;
                                        umax[ix] = umin[ix] + 63;
                                        vmin[ix] = (ix & 0x03) * 64;
                                        vmax[ix] = vmin[ix] + 63;
                                    }
                                    break;
                                case 3:
                                    ulo[0]=0; ulo[1]=96; ulo[2]=128; ulo[3]=160;
                                    uhi[0]=ulo[1]-1; uhi[1]=ulo[2]-1; uhi[2]=ulo[3]-1; uhi[3]=255;
                                    vlo[0]=0; vlo[1]=96; vlo[2]=128; vlo[3]=160;
                                    vhi[0]=vlo[1]-1; vhi[1]=vlo[2]-1; vhi[2]=vlo[3]-1; vhi[3]=255;
                                    for(ix = 0; ix<MAX_COLORS; ix++) {
                                        i1 = ix >> 2;
                                        i2 = ix & 0x03;
                                        ymin[ix] = 0;
                                        ymax[ix] = 255;
                                        umin[ix] = ulo[i1];
                                        umax[ix] = uhi[i1];
                                        vmin[ix] = vlo[i2];
                                        vmax[ix] = vhi[i2];
                                    }
                                    break;
                                case 4:
                                    for(ix = 0; ix<MAX_COLORS; ix++) {
                                        ymin[ix] = ix << 4;
                                        ymax[ix] = ymin[ix] + 15;
                                        umin[ix] = 0;
                                        umax[ix] = 255;
                                        vmin[ix] = 0;
                                        vmax[ix] = 255;
                                    }
                                    break;
                            }
                            break;
                    }
                    break;
                case 't':  // time in millisecs since last reset
                    body = cgiResponse;  // use HTTP_BUFFER2
                    sprintf(body, "%08d\r\n", readRTC());
                    contentLength = strlen((char *)body);
                    contentType = "text/plain";
                    new_content = 1;
                    break;
                case 'l':  // l1 = lasers on, l0 = lasers off
                    if (params[1] == '1')
                        *pPORTHIO |= 0x0280;
                    else
                        *pPORTHIO &= 0xFD7F;
                    break;
                case 'm':  // SRV-1 Robot motor drive (PWM)
                    if ((params[1]<'0') || (params[1]>'9') || (params[2]<'0') || (params[2]>'9')  // out of range 
                            || (params[3]<'0') || (params[3]>'9') || (params[4]<'0') || (params[4]>'9'))  
                        break;
                    if (!pwm1_init) {
                        initPWM();
                        pwm1_init = 1;
                        pwm1_mode = PWM_PWM;
                        base_speed = 40;
                        lspeed = rspeed = 0;
                    }
                    lspeed = ((int)atoi_b10(&params[1]) * 2) - 100;
                    rspeed = ((int)atoi_b10(&params[3]) * 2) - 100;
                    setPWM(lspeed, rspeed);
                    break;
                case 'x':  // SRV-4WD motor controller
                    if ((params[1]<'0') || (params[1]>'9') || (params[2]<'0') || (params[2]>'9')  // out of range ?
                            || (params[3]<'0') || (params[3]>'9') || (params[4]<'0') || (params[4]>'9')) 
                        break;
                    if (xwd_init == 0) {
                        xwd_init = 1;
                        init_uart1(115200);
                        delayMS(10);
                    }
                    x1 = (((int)atoi_b10(&params[1]) * 5) / 2) - 125;
                    x2 = (((int)atoi_b10(&params[3]) * 5) / 2) - 125;
                    uart1SendChar('x');
                    uart1SendChar((char)x1);
                    uart1SendChar((char)x2);
                    while (uart1GetChar(&ch))  // flush the receive buffer
                        continue;
                    break;
                case 'S': // Sabertooth or equivalent PPM servo-based motor control
                    if ((params[1]<'0') || (params[1]>'9') || (params[2]<'0') || (params[2]>'9')  // out of range ?
                            || (params[3]<'0') || (params[3]>'9') || (params[4]<'0') || (params[4]>'9'))  
                        break;
                    if (!pwm1_init) {
                        initPPM1();
                        pwm1_init = 1;
                        pwm1_mode = PWM_PPM;
                    }
                    x1 = (int)atoi_b10(&params[1]);
                    x2 = (int)atoi_b10(&params[3]);
                    setPPM1((char)x1, (char)x2);
                    break;
                case 's': // servo controls for timers 6 / 7
                    if ((params[1]<'0') || (params[1]>'9') || (params[2]<'0') || (params[2]>'9')  // out of range ?
                            || (params[3]<'0') || (params[3]>'9') || (params[4]<'0') || (params[4]>'9'))  
                        break;
                    if (!pwm2_init) {
                        initPPM2();
                        pwm2_init = 1;
                        pwm2_mode = PWM_PPM;
                    }
                    x1 = (int)atoi_b10(&params[1]);
                    x2 = (int)atoi_b10(&params[3]);
                    setPPM2((char)x1, (char)x2);
                    break;
            }
            
            if (new_content == 0) {  // if command processing didn't generate new content
                body = cgiBody;
                contentLength = countof (cgiBody) - 1;
                contentType = "text/plain";
            }
        }

        // admin - built-in administration page
        else if (strncmp(path, adminPath, countof(adminPath) - 1) == 0) {
            body = adminBodyBuilder (NULL, NULL, &contentLength);
            freeBody = TRUE;
        }

        // Look up HTML from flash
        else {
            int fileIndex = countof(fileNames);     // assume no match

            // Look up the name
            if ((strcmp(path, "/") == 0) || (strcmp(path, "/index.html") == 0))
                fileIndex = 0;
            else {
                for (fileIndex = 0; fileIndex < countof(fileNames); ++fileIndex) 
                    if (strcmp(path, fileNames[fileIndex]) == 0)
                        break;
            }

            // If it's found, pull it out of flash
            if (fileIndex < countof(fileNames)) {
                char * cp;

                read_double_sector ((fileIndex*2) + 10, 1);  // set quiet flag
                body = cp = (char *) FLASH_BUFFER;
                flashContentLength = contentLength = clean_buffer(cp);  // last character of html file should be '>'.  clean out anything else
            }

            // Not found - 404
            else {
                body = body404;
                resultCode = resultCode404;
                contentLength = countof (body404) - 1;
            }
        }
    } // if GET method


    // POST method
    //else if (strcmp (method, "POST") == 0) {
    else if (reqBuf[0] == 'P') {
        ret = 1;
        t0 = readRTC();
        while (readRTC() - t0 < 1000  &&  ret < HTTP_BUFFER_SIZE) {
            if (getchar((unsigned char *) &ch)) {
                reqBuf[ret++] = ch;
                       // POST should end with a boundary terminated by "--"
                //if (ret >= 4  &&  strncmp (reqBuf + ret - 4, "--\r\n", 4) == 0)
                    //break;
                t0 = readRTC();
            }
        }
        reqBuf[ret] = 0;
        reqBuf[ret+1] = 0;  // for benefit of serial_out_httpbuffer(), which has to handle nulls inserted by strtok()

        // Parse the request fields
        method = strtok(reqBuf, " ");
        path = strtok(0, " ");
        protocol = strtok(0, "\r\n");
        headers = protocol + strlen (protocol) + 1;
        while (*headers != 0  &&  (*headers == '\r'  ||  *headers == '\n'))
            ++headers;        

        if (!method || !path || !protocol) 
            goto exit;

        char *reqBody = (char *)HTTP_BUFFER;
        int reqBodyCount = 0;
        int reqContentLength = -1;  // assume no Content-Length header
        static char reqContentType[128];

        // Assume error
        BOOL error = TRUE;
        body = body501;
        contentLength = countof (body501) - 1;
        resultCode = resultCode501;

        // Receive the request body
        getHdrDecimal (headers, "Content-Length: ", &reqContentLength);   // get Content-Length
        getHdrString (headers, "Content-Type: ", reqContentType, countof (reqContentType));

        for (ix=0; ix<ret; ix++) {
            if ((reqBuf[ix] == '\r') &&
              (reqBuf[ix+1] == '\n') &&
              (reqBuf[ix+2] == '\r') &&
              (reqBuf[ix+3] == '\n')) {
                ix += 4;
                while ((ix < ret) && (reqBuf[ix] != '-'))  // search for boundary start
                    ix++;
                reqBody = &reqBuf[ix];
                reqBodyCount = ret - ix;
                break;
            }
        }

        // admin -- Post to administration page
        if (strncmp(path, adminPath, countof(adminPath) - 1) == 0) {
            static char type[64];
            static char boundary[128];
            int boundaryLen;

            // Get the content type and boundary parameter
            type[0] = 0;
            char * typeEnd = strchr (reqContentType, ';');
            if (typeEnd != NULL) {
                strncpy (type, reqContentType, min (typeEnd - reqContentType, countof (type)));
                type[countof (type) - 1] = 0;
            }
            strcpy (boundary, "--");
            getParam (reqContentType, "boundary=", boundary + 2, countof (boundary) - 2);
            boundaryLen = strlen (boundary);

            // If everything is in order for an upload,
            if (reqContentLength  &&  strcmp (type, "multipart/form-data") == 0  &&  
                    boundaryLen > 0  &&  contentLength <= 0x00020000) {   // currently a 128kB limit on transfers

                // Init POST body fields
                char *fileBodyStart = NULL;
                int fileBodyLength = 0;
                enum { undefined, toSectors, toBootLoader } uploadDest = undefined;
                int sectorStart = -1;
                BOOL confirmBootLoader = FALSE;

                // Parse sections of the body
                if (strncmp (reqBody, boundary, boundaryLen) == 0)    // if boundary at start of body
                {
                    char * secStart = NULL;
                    char * secEnd = NULL;
                    char * secBody;
                    int bodyIndex = 0;

                    while (TRUE)
                    {
                        // Find the next boundary. If there was a prev boundary, process this section
                        secEnd = strnstr (reqBody + bodyIndex, boundary, reqBodyCount - bodyIndex);
                        if (secEnd == NULL)
                            break;
                        secEnd -= 2;    // back up to CRLF preceding boundary
                    
                        if (secStart != NULL  &&
                            (secBody = strnstr (secStart, "\r\n\r\n", secEnd - secStart)) != NULL)
                        {
                            static char contDisp[128], nameParam[32];
                            *secBody = 0;   // terminate section headers
                            secBody += 4;

                            // Get Content-Disposition section header and the field name for this section
                            if (getHdrString (secStart, "Content-Disposition: ", contDisp, countof (contDisp)) &&
                                getParam (contDisp, "name=", nameParam, countof (nameParam))) {
                                static char val[32];

                                // fileItem1: the file being uploaded
                                if (strcmp (nameParam, "fileItem1") == 0) {
                                    fileBodyStart = secBody;
                                    fileBodyLength = secEnd - secBody;
                                }

                                // uploadRadios: the upload destination
                                else if (strcmp (nameParam, "uploadRadios") == 0) {
                                    static char toSectorsStr[] = "toSectors", toBootLoaderStr[] = "toBootLoader";
                                    if (strncmp (secBody, toSectorsStr, countof (toSectorsStr) - 1) == 0)
                                        uploadDest = toSectors;
                                    else if (strncmp (secBody, toBootLoaderStr, countof (toBootLoaderStr) - 1) == 0)
                                        uploadDest = toBootLoader;                                           
                                }

                                // sectorStart
                                else if (strcmp (nameParam, "sectorStart") == 0  &&  secEnd - secBody > 0) {
                                    strncpy (val, secBody, min (countof (val), secEnd - secBody));
                                    val[countof (val) - 1] = 0;
                                    sectorStart = atoi (val);
                                    if (sectorStart < FLASH_SECTORS_FIRST  ||  sectorStart > FLASH_SECTORS_LAST)
                                        sectorStart = -1;
                                }

                                // confirmBootLoader
                                else if (strcmp (nameParam, "confirmBootLoader") == 0) {
                                    if (strncmp (secBody, "on", 2) == 0  ||  strncmp (secBody, "1", 1) == 0)
                                        confirmBootLoader = TRUE;
                                }
                            }                                    
                        } // if section to process
                        
                        secStart = secEnd + boundaryLen + 2;   // start of the next section is after end of last

                        // If we're at the end boundary, we're done
                        if (secStart[0] == '-'  &&  secStart[1] == '-')
                            break;
                        secStart += 2; // skip CRLFs after boundary
                        bodyIndex = secStart - reqBody;
                    } // while parsing request body
                } // if first boundary

                 // Validate the upload, then flash the fugger
                static char resultMsg[128];
                resultMsg[0] = 0;
                char * resultColor = "#ffa0a0";     // assume error: red
                if (reqBodyCount != reqContentLength)
                    sprintf (resultMsg, "bodyCount (%d) != Content-Length (%d)", reqBodyCount, reqContentLength);
                else if (fileBodyStart == NULL  ||  fileBodyLength == 0)
                    sprintf (resultMsg, "No file received");
                else if (uploadDest == undefined)
                    sprintf (resultMsg, "No upload destination specified");

                else if (uploadDest == toSectors  &&  sectorStart == -1)
                    sprintf (resultMsg, "Valid starting sector not specified");
                else if (uploadDest == toSectors  &&  fileBodyLength > FLASH_SECTORS_UNIT * FLASH_SECTOR_SIZE)
                    sprintf (resultMsg, "File size (%d) too big for %d sectors", fileBodyLength, FLASH_SECTORS_UNIT);

                else if (uploadDest == toBootLoader  &&  !confirmBootLoader)
                    sprintf (resultMsg, "No boot loader confirmation");
                else if (uploadDest == toBootLoader  &&  
                         (fileBodyLength < BOOT_LOADER_MIN_SIZE  ||  fileBodyLength > BOOT_LOADER_MAX_SIZE))
                    sprintf (resultMsg, "Boot loader file size (%d) too big or too small", fileBodyLength);
                else if (uploadDest == toBootLoader  &&
                         getUnaligned32 (fileBodyStart) != 0xFFA00000  &&  
                         getUnaligned32 (fileBodyStart) != 0xFF800000) {
                    sprintf (resultMsg, "Boot loader first-word signature (0x%08x) wrong", 
                                getUnaligned32 (fileBodyStart));
                }
                else {
                    // Set destination and flash write size based on parameters
                    int flashAddress, bytesToWrite;
                    if (uploadDest == toSectors)
                    {
                        flashAddress = sectorStart * FLASH_SECTOR_SIZE;
                        bytesToWrite = FLASH_SECTORS_UNIT * FLASH_SECTOR_SIZE;
                    }
                    else
                    {
                        flashAddress = BOOT_SECTOR;
                        bytesToWrite = BOOT_LOADER_SECTORS * FLASH_SECTOR_SIZE;
                    }

                    // Move the file down to the beginning of FLASH_BUFFER and pad the sectors with zeroes
                    memmove ((void *) FLASH_BUFFER, fileBodyStart, fileBodyLength);
                    memset ((void *) (FLASH_BUFFER + fileBodyLength), 0, bytesToWrite - fileBodyLength);
#if 1
                    if (spi_write (flashAddress, (unsigned char *) FLASH_BUFFER,
                                    (unsigned char *) FLASH_BUFFER + bytesToWrite, bytesToWrite) != bytesToWrite)
                    {
                        sprintf (resultMsg, "Flash write failed. Error code lost in lower-level code. :)");
                        error = FALSE;
                    }
                    else
#endif
                    {
                        resultColor = "#c0ffc0";        // green
                        if (uploadDest == toSectors)
                            sprintf (resultMsg, "%d bytes uploaded to sector %d", fileBodyLength, sectorStart);
                        else
                            sprintf (resultMsg, "%d bytes uploaded to boot loader", fileBodyLength);
                    }
                }

                // Form our response
                body = adminBodyBuilder (resultMsg, resultColor, &contentLength);
                freeBody = TRUE;

                // Unless we got a valid file, clear the flash buffer
                if (error)
                    memset ((void *) FLASH_BUFFER, 0, FLASH_BUFFER_SIZE);
            } // if starting boundary
        } // if uploadFlash
    } // if POST method


    // Unknown method
    else {
        body = body501;
        contentLength = countof (body501) - 1;
        resultCode = resultCode501;
    }

    // Send response headers
    printf ("%s\r\n"
            "Content-Type: %s\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            CONNECTION_HEADER
            "Content-Length: %d\r\n"
            "\r\n",
            resultCode,
            contentType,
            contentLength);

    // Send response body
#if 0
    if (insertInlineImg) {
        putchars ((unsigned char *) body, inlineTagPtr - body);
        putchars ((unsigned char *) inlineImgBuf, inlineImgLength);
        putchars ((unsigned char *) inlineTagPtr + sizeof (inlineImgTag) - 1, 
                    flashContentLength - (inlineTagPtr - body - sizeof (inlineImgTag) + 1));
    }
    else
#endif
    {
        if (body != NULL)
            putchars ((unsigned char *) body, contentLength);
    }

    // If the response body was malloc-ed, free it
    if (freeBody  &&  body != NULL)
        free (body);

    // If a reset request came in, reset
    if (deferredReset)
    {
        reset_cpu();
    }

exit:
    return;
}


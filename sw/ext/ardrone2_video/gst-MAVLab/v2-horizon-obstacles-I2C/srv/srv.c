/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  srv.c - routines to interface with the SRV-1 Blackfin robot.
 *    modified from main.c - main control loop for SRV-1 robot
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
#include "config.h"
#include "uart.h"
#include "i2c.h"
#include "ov9655.h"
#include "ov7725.h"
#include "camera.h"
#include "jpeg.h"
#include "xmodem.h"
#include "stm_m25p32.h"
#include "font8x8.h"
#include "colors.h"
#include "malloc.h"
#include "edit.h"
#include "print.h"
#include "string.h"
#include "neural.h"
#include "sdcard.h"
#include "gps.h"

#ifdef STEREO
#include "stereo.h"
#endif

#include "srv.h"

void _motionvect(unsigned char *, unsigned char *, char *, char *, int, int, int);
extern int picoc(char *);

/* Size of frame */
unsigned int imgWidth, imgHeight;

/* stereo vision globals */
#ifdef STEREO
int svs_calibration_offset_x, svs_calibration_offset_y;
int svs_centre_of_disortion_x, svs_centre_of_disortion_y;
int svs_scale_num, svs_scale_denom, svs_coeff_degree;
unsigned int stereo_processing_flag, stereo_sync_flag;
long* svs_coeff;
int svs_right_turn_percent;
int svs_turn_tollerance_percent;
int svs_sensor_width_mmx100;
int svs_width, svs_height;
int svs_enable_horizontal;
int svs_ground_y_percent;
int svs_ground_slope_percent;
int svs_enable_ground_priors;
int svs_disp_left, svs_disp_right, svs_steer;
unsigned int enable_stereo_flag;
int svs_enable_mapping;
#endif /* STEREO */

/* motor move times */
int move_start_time, move_stop_time, move_time_mS;
int robot_moving;

/* Version */
unsigned char version_string[] = "SRV-1 Blackfin w/picoC 1.0 "  __TIME__ " - " __DATE__;

/* Frame count output string */
unsigned char frame[] = "000-deg 000-f 000-d 000-l 000-r";
//unsigned char frame[] = "frame     ";

/* Camera globals */
unsigned int quality, framecount, ix, overlay_flag;
unsigned int segmentation_flag, edge_detect_flag, frame_diff_flag, horizon_detect_flag;
unsigned int obstacle_detect_flag;
unsigned int blob_display_flag;
unsigned int blob_display_num;
unsigned int edge_thresh;
unsigned int invert_flag;
unsigned char *output_start, *output_end; /* Framebuffer addresses */
unsigned int image_size; /* JPEG image size */
char imgHead[11]; /* image frame header for I command */

/* Motor globals */
int lspeed, rspeed, lcount, rcount, lspeed2, rspeed2, base_speed, base_speed2, err1;
int pwm1_mode, pwm2_mode, pwm1_init, pwm2_init, xwd_init;
int encoder_flag;

/* IMU globals */
int x_acc, x_acc0, x_center;
int y_acc, y_acc0, y_center;
int compass_init, compass_continuous_calibration, analog_init, tilt_init;
short cxmin, cymin, cxmax, cymax, czmin, czmax;

/* Failsafe globals */
int failsafe_mode = 0;
int lfailsafe, rfailsafe;
int failsafe_clock;

/* Sonar globals */
int sonar_data[5], sonar_flag = 0;

/* random number generator globals */
unsigned int rand_seed = 0x55555555;

/* General globals */
unsigned char *cp;
unsigned int i, j; // Loop counter.
unsigned int master;  // SVS master or slave ?
unsigned int uart1_flag = 0;

void init_io() {
    *pPORTGIO_DIR = 0x0300;   // LEDs (PG8 and PG9)
    *pPORTH_FER = 0x0000;     // set for GPIO
    *pPORTHIO_DIR |= 0x0040;  // set PORTH6 to output for serial flow control
    *pPORTHIO = 0x0000;       // set output low 
    *pPORTHIO_INEN |= 0x000D; // enable inputs: Matchport RTS0 (H0), battery (H2), master/slave (H3)
    *pPORTHIO_DIR |= 0x0380;  // set up lasers - note that GPIO-H8 is used for SD SPI select on RCM board
    //*pPORTHIO |= 0x0100;      // set GPIO-H8 high in case it's used for SD SPI select 

    #ifdef STEREO
    if (*pPORTHIO & 0x0008) {  // check SVS master/slave bit
        master = 0;
        *pPORTHIO_DIR &= 0xFEFF;
        *pPORTHIO_INEN |= 0x0100; // enable GPIO-H8 as input on slave for stereo sync
        stereo_sync_flag = *pPORTHIO & 0x0100;
    } else {
        master = 1;
        *pPORTHIO_DIR |= 0x0100;  // set GPIO-H8 as output on master for stereo sync        
    } 
    #endif /* STEREO */

    pwm1_mode = PWM_OFF;
    pwm2_mode = PWM_OFF;
    pwm1_init = 0;
    pwm2_init = 0;
    xwd_init = 0;
    analog_init = 0;
    tilt_init = 0;
    sonar_flag = 0;
    edge_detect_flag = 0;
    horizon_detect_flag = 0;
    edge_thresh = 3200;
    obstacle_detect_flag = 0;
    segmentation_flag = 0;
    blob_display_flag = 0;
    invert_flag = 0;
    encoder_flag = 0;

    /* compass calibration */
    cxmin = cymin = 9999;
    cxmax = cymax = -9999;
    compass_init = 0;
    compass_continuous_calibration = 1;
    
    #ifdef STEREO
    stereo_processing_flag = 0;
    #endif /* STEREO */
}

/* reset CPU */
void reset_cpu() {
    asm(
    "p0.l = 0x0100; "
    "p0.h = 0xFFC0; "
    "r0.l = 0x0007; "
    "w[p0] = r0; "
    "ssync; "
    "p0.l = 0x0100; "
    "p0.h = 0xFFC0; "
    "r0.l = 0x0000; "
    "w[p0] = r0; "
    "ssync; "
    "raise 1; ");                        
}

/* clear SDRAM */
void clear_sdram() {
  for (cp=(unsigned char *)0x00100000; cp<(unsigned char *)0x02000000; cp++) {
    *cp = 0;
  }
}

void show_stack_ptr() {
    int x = 0;
    asm("%0 = SP;" : "=r"(x) : "0"(x));
    printf("stack_ptr = 0x%x\r\n", x);
    return;
}

unsigned int stack_remaining() {
    unsigned int x = 0;
    asm("%0 = SP" : "=r"(x) : "0"(x));
    return (x - (unsigned int)STACK_BOTTOM);
}

void show_heap_ptr() {
    printf("heap_ptr  = 0x%x\r\n", (int)heap_ptr);
}

/* SRV-1 Firmware Version Request
   Serial protocol char: V */
void serial_out_version () {
    printf("##Version - %s", version_string);
    #ifdef STEREO
    if (master)  // check master/slave bit
        printf(" (stereo master)");     
    else
        printf(" (stereo slave)");     
    #endif /* STEREO */
    printf("\r\n");
}

/* Get current time
   Serial protocol char: t */
void serial_out_time () {
    printf("##time - millisecs:  %d\r\n", readRTC());
}

/* load flash sector 4 into flash buffer on startup.  
   If "autorun" is found at beginning of buffer, launch picoC */
void check_for_autorun() {
    char *cp;
    int ix;
    
    printf("##checking for autorun() in flash sect#4 ... ");
    spi_read(FLASH_SECTOR, (unsigned char *)FLASH_BUFFER, 0x00010000);  // read flash sector #4
    cp = (char *)FLASH_BUFFER;
    if (strncmp("autorun", cp, 7) == 0) {
        printf("autorun() found - launching picoC\r\n");
        picoc((char *)FLASH_BUFFER);
    } else {
        printf("no autorun() found\r\n");
        for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00010000); ix++)  // clear FLASH_BUFFER
            *((unsigned char *)ix) = 0;
    }
}

/* Dump flash buffer to serial
   Serial protocol char: z-d */
void serial_out_flashbuffer () {
    printf("##zdump: \r\n");
    cp = (unsigned char *)FLASH_BUFFER;
    for (i=0; i<0x10000; i++) {
        if (*cp == 0)
            return;
        if (*cp == 0x0A)
            putchar(0x0D);
        putchar(*cp++);
    }
}

/* Dump http buffer to serial
   Serial protocol char: z-d */
void serial_out_httpbuffer () {
    printf("##zhttp: \r\n");
    cp = (unsigned char *)HTTP_BUFFER;
    for (i=0; i<0x10000; i++) {
        if ((*cp == 0) && (*(cp+1) == 0)) // skips the 0's that were inserted by strtok()
            return;
        if (*cp == 0x0A)
            putchar(0x0D);
        putchar(*cp++);
    }
}

/* Turn lasers on
   Serial protocol char: l */
void lasers_on () {
    *pPORTHIO |= 0x0280;
    printf("#l");
}

/* Turn lasers off
   Serial protocol char: L */
void lasers_off () {
    *pPORTHIO &= 0xFD7F;
    printf("#L");
}

/* Show laser range
   Serial protocol char: R */
void show_laser_range(int flag) {
    printf("##Range(cm) = %d\r\n", laser_range(flag));
}

/* Compute laser range 
    turn off lasers
    stop motors
    delay 250ms
    grab reference frame
    do right laser, then left laser
        turn on laser
        delay 250ms
        grab frame
        turn lasers off 
        compute difference
        set color #16
        use adaptive threshold to filter blobs
        range (inches) = imgWidth*2 / vblob(16)
    compare results, return best measurement
*/
unsigned int laser_range(int dflag) {
    unsigned int ix, rrange, lrange, rconf, lconf;
    
    rrange = lrange = 9999;
    rconf = lconf = 0;    // confidence measure
    
    *pPORTHIO &= 0xFC7F;    // lasers off
    if (pwm1_mode == PWM_PWM) setPWM(0, 0);    // motors off
    else setPPM1(50, 50);
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab reference frame
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
    *pPORTHIO |= 0x0200;      // right laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("right blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\r\n",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx1[0] < (imgWidth/2)) // make certain that blob is on right
            continue;
        break;
    }
    rrange = (6*imgWidth) / (blobx2[0]+blobx1[0]-imgWidth+1); // right blob
    rconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));

    *pPORTHIO |= 0x0080;      // left laser on
    delayMS(250);
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    compute_frame_diff((unsigned char *)FRAME_BUF,             // compute difference        
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    *pPORTHIO &= 0xFC7F;    // lasers off
    umin[16] = 80; umax[16]=144; vmin[16] = 100; vmax[16]=255; ymax[16]=255;   // set color bin #16
    for(ymin[16]=200; ymin[16]>0; ymin[16]-=10) {
        vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, 16);  // use the brightest blob
        if (dflag)
            printf("left blobs: ymin=%d   %d %d %d %d %d  %d %d %d %d %d  %d %d %d %d %d\r\n",
              ymin[16], 
              blobcnt[0], blobx1[0], blobx2[0], bloby1[0], bloby2[0],
              blobcnt[1], blobx1[1], blobx2[1], bloby1[1], bloby2[1],
              blobcnt[2], blobx1[2], blobx2[2], bloby1[2], bloby2[2]);
        ix = blobcnt[0];
        if (!ix)
            continue;
        if (blobx2[0] > (imgWidth/2)) // make certain that blob is on left
            continue;
        break;
    }
    lrange = (6*imgWidth) / (imgWidth-blobx2[0]-blobx1[0]+ 1); // left blob
    lconf = (100 * ix) / ((blobx2[0]-blobx1[0]+1) * (bloby2[0]-bloby1[0]+1));
    
    if (dflag)
        printf("lconf %d lrange %d rconf %d rrange %d\r\n", lconf, lrange, rconf, rrange);
    if ((lrange==9999) && (rrange==9999))
        return 9999;
    if (lconf > rconf)
        return lrange;
    return rrange;
}

void check_battery() { // 'D' command
    if (*pPORTHIO & 0x0004)
        printf("##D - low battery voltage detected\r\n");
    else
        printf("##D - battery voltage okay\r\n");
}

void led0_on() {
    *pPORTGIO = 0x0100;  // turn on LED0
}

void led1_on() {
    *pPORTGIO = 0x0200;  // turn on LED1
}

/* init LIS3LV02DQ 3-axis tilt sensor with i2c device id 0x1D */
void init_tilt()
{
    unsigned char i2c_data[3], device_id;
    
    device_id = 0x1D;
    i2c_data[0] = 0x20;
    i2c_data[1] = 0x87;
    i2cwrite(device_id, (unsigned char *)i2c_data, 1, SCCB_ON);
    delayMS(10);
    tilt_init = 1;
}

unsigned int tilt(unsigned int channel)
{
    unsigned char i2c_data[2], ch1;
    unsigned int ix;
    
    if (!tilt_init)
        init_tilt();
    switch(channel) {
        case 1:  // x axis
            ch1 = 0x28;
            break;  
        case 2:  // y axis
            ch1 = 0x2A;
            break;  
        case 3:  // z axis
            ch1 = 0x2C;
            break;  
        default:
            return 0;  // invalid channel
    }
    i2c_data[0] = ch1;
    i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
    ix = (unsigned int)i2c_data[0];
    i2c_data[0] = ch1 + 1;
    delayUS(1000);
    i2cread(0x1D, (unsigned char *)i2c_data, 1, SCCB_ON);
    ix += (unsigned int)i2c_data[0] << 8;
    return ix;
}

void read_tilt()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$T%d %4d\r\n", channel, tilt(channel));
}

void show_compass2x()
{
    unsigned char i2c_data[3];
    unsigned int ix;

    i2c_data[0] = 0x41;  // read compass twice to clear last reading
    i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
    delayUS(20000);
    i2c_data[0] = 0x41;
    i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
    ix = (((unsigned int)i2c_data[0] << 8) + i2c_data[1]) / 10;
    printf("##$C %3d\r\n", ix);
}

void show_compass3x() {
    short x, y, z, head;
    
    head = read_compass3x(&x, &y, &z);
    printf("##c heading=%d  x=%d y=%d z=%d  xmin=%d xmax=%d ymin=%d ymax=%d\r\n", 
       head, x, y, z, cxmin, cxmax, cymin, cymax);
}

short read_compass3x(short *x, short *y, short *z) {
    unsigned char i2c_data[12];
    short i;
    unsigned char addr;
    short xx, yy, sy, sx, ang;
    
    // HMC5843
    addr = 0x1E;
    if (compass_init == 0) {
        i2c_data[0] = 0x00; i2c_data[1] = 0x20; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_ON);
        i2c_data[0] = 0x01; i2c_data[1] = 0x20; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_ON);
        i2c_data[0] = 0x02; i2c_data[1] = 0x00; i2cwrite(addr, (unsigned char *)i2c_data, 2, SCCB_ON);
        delayMS(10);
        compass_init = 1;
    }
    for(i = 0; i < 6; i++) i2c_data[i] = 0x00;
    i2c_data[0] = 0x03;
    i2cread(addr, (unsigned char *)i2c_data, 6, SCCB_ON);
    *x = ((short) (i2c_data[0] * 256 + i2c_data[1]));
    *y = ((short) (i2c_data[2] * 256 + i2c_data[3]));
    *z = ((short) (i2c_data[4] * 256 + i2c_data[5]));
    if (compass_continuous_calibration) {  // this is turned off by compassxcal() C function or $y console function
        if (cxmin > *x)
            cxmin = *x;
        if (cymin > *y)
            cymin = *y;
        if (cxmax < *x)
            cxmax = *x;
        if (cymax < *y)
            cymax = *y;
    }

    /* now compute heading */
    sx = sy = 0;  // sign bits

    yy = *y - (cymax + cymin) / 2;
    if (yy < 0) { sy = 1; yy = -yy; }

    xx = *x - (cxmax + cxmin) / 2;
    if (xx < 0) { sx = 1; xx = -xx; }
    
    ang = (short)atan((int)yy, (int)xx);
    if ((sx==0) && (sy==0))
        ang = 90 - ang;
    if ((sx==0) && (sy==1))
        ang = 90 + ang;
    if ((sx==1) && (sy==1))
        ang = 270 - ang;
    if ((sx==1) && (sy==0))
        ang = 270 + ang;
    i = 360 - ang;  // compass angles go in opposite direction of trig angles
    i += 90;  // shift by 90-deg for compass placement
    if (i >= 360)
        i -= 360;
    return (i); 
}

/* init all 3 possible AD7998 A/D's */
void init_analog() {
    unsigned char i2c_data[3], device_id;
    unsigned int ix;
    
    for (ix=0; ix<3; ix++) {
        switch (ix) {
            case 0:
                device_id = 0x20;
                break;
            case 1:
                device_id = 0x23;
                break;
            case 2:
                device_id = 0x24;
                break;
        }
        // set timer register 3
        i2c_data[0] = 0x03;
        i2c_data[1] = 0x01;
        i2cwrite(device_id, (unsigned char *)i2c_data, 1, SCCB_ON);
        delayMS(10);
        // set analog channel 
        i2c_data[0] = 0x02;
        i2c_data[1] = 0x0F;
        i2c_data[2] = 0xF0;
        i2cwritex(device_id, (unsigned char *)i2c_data, 3, SCCB_ON);
        delayMS(10);
    }
    analog_init = 1;
}

/* read AD7998 - channels 01-08, 11-17 or 21-27 */
unsigned int analog(unsigned int ix)
{
    unsigned char i2c_data[3], device_id;
    unsigned int channel;
    unsigned char mask1[] = { 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04, 0x08 };
    unsigned char mask2[] = { 0x10, 0x20, 0x40, 0x80, 0x00, 0x00, 0x00, 0x00 };
    
    if (!analog_init)
        init_analog();
    // decide which i2c device based on channel range
    if ((ix<1) || (ix>28))
        return 0xFFFF;  // invalid channel
    device_id = 0;
    switch (ix / 10) {
        case 0:
            device_id = 0x20;  // channels 1-8
            break;
        case 1:
            device_id = 0x23;  // channels 11-18
            break;
        case 2:
            device_id = 0x24;  // channels 21-28
            break;
    }
    channel = ix % 10;
    if ((channel<1) || (channel>8))
        return 0xFFFF;  // invalid channel
    
    // set timer register 3
    i2c_data[0] = 0x03;
    i2c_data[1] = 0x01;
    i2cwrite(device_id, (unsigned char *)i2c_data, 1, SCCB_ON);

    // set analog channel 
    i2c_data[0] = 0x02;
    i2c_data[1] = mask1[channel-1];
    i2c_data[2] = mask2[channel-1];
    i2cwritex(device_id, (unsigned char *)i2c_data, 3, SCCB_ON);

    // small delay
    delayUS(1000);

    // read data
    i2c_data[0] = 0x00;
    i2cread(device_id, (unsigned char *)i2c_data, 2, SCCB_ON);
    ix = (((i2c_data[0] & 0x0F) << 8) + i2c_data[1]);
    return ix;
}

void read_analog()
{
    unsigned int channel;
    channel = ((unsigned int)(getch() & 0x0F) * 10) + (unsigned int)(getch() & 0x0F);
    printf("##$A%d %04d\r\n", channel, analog(channel));
}

unsigned int analog_4wd(unsigned int ix)
{
    int t0, ii, val;
    unsigned char ch;
    
    if (xwd_init == 0) {
        xwd_init = 1;
        init_uart1(115200);
        delayMS(10);
    }
    uart1SendChar('a');
    uart1SendChar((char)(ix + 0x30));

    ii = 1000;
    val = 0;
    while (ii) {
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if ((ch < '0') || (ch > '9'))
            continue;
        val += (unsigned int)(ch & 0x0F) * ii;        
        ii /= 10;
    }
    while (1) {  // flush the UART1 receive buffer
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if (ch == '\n')
            break;
    }
    return val;
}

void read_analog_4wd()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$a%d %04d\r\n", channel, analog_4wd(channel));
}

/* use GPIO H10 (pin 27), H11 (pin 28), H12 (pin 29), H13 (pin 30) as sonar inputs -
    GPIO H1 (pin 18) is used to trigger the sonar reading (low-to-high transition) */
void init_sonar() {  
    *pPORTHIO_INEN &= 0xC3FF;
    *pPORTHIO_INEN |= 0x3C00;  // enable H27, H28, H29, H30 as inputs
    *pPORTHIO_DIR |= 0x0002;  // set up sonar trigger
    *pPORTHIO &= 0xFFFD;       // force H1 low
    initTMR4();
}

void ping_sonar() {
    sonar();
    printf("##ping %d %d %d %d\r\n", sonar_data[1], sonar_data[2], sonar_data[3], sonar_data[4]);
}

void sonar() {
    int t0, t1, t2, t3, t4, x1, x2, x3, x4, imask;
    
    if (!sonar_flag) {
        sonar_flag = 1;
        init_sonar();
    }
    
    imask = *pPORTHIO & 0x3C00;
    *pPORTHIO |= 0x0002;       // force H1 high to trigger sonars
    t0 = readRTC();
    t1 = t2 = t3 = t4 = *pTIMER4_COUNTER;
    x1 = x2 = x3 = x4 = 0;
    
    while ((readRTC() - t0) < 40) {
        if ((*pPORTHIO & 0x0400)) 
            x1 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x0800)) 
            x2 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x1000)) 
            x3 = *pTIMER4_COUNTER;
        if ((*pPORTHIO & 0x2000)) 
            x4 = *pTIMER4_COUNTER;
    }

    *pPORTHIO &= 0xFFFD;       // force H1 low to disable sonar
    if (imask & 0x0400) {
        x1 = 0;
    }else {
        if (x1 < t1)
            t1 -= PERIPHERAL_CLOCK;
        x1 = (x1 - t1) / 200;
    }
    if (imask & 0x0800) {
        x2 = 0;
    } else {
        if (x2 < t2)
            t2 -= PERIPHERAL_CLOCK;
        x2 = (x2 - t2) / 200;
    }
    if (imask & 0x1000) {
        x3 = 0;
    } else {
        if (x3 < t3)
            t3 -= PERIPHERAL_CLOCK;
        x3 = (x3 - t3) / 200;
    }
    if (imask & 0x2000) {
        x4 = 0;
    } else {
        if (x4 < t4)
            t1 -= PERIPHERAL_CLOCK;
        x4 = (x4 - t4) / 200;
    }

    sonar_data[0] = sonar_data[1] = x1;  // should fix this - we are counting 1-4, not 0-3
    sonar_data[2] = x2;
    sonar_data[3] = x3;
    sonar_data[4] = x4;
}

void enable_frame_diff() {
    frame_diff_flag = 1;
    grab_reference_frame();
    printf("##g0");
}

void enable_segmentation() {
    segmentation_flag = 1;
    printf("##g1");
}

void enable_edge_detect() {
    edge_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g2");
}

void enable_horizon_detect() {
    horizon_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g3");
}

void enable_obstacle_detect() {
    obstacle_detect_flag = 1;
    edge_thresh = 3200;
    printf("##g4");
}

void enable_blob_display() {
    unsigned int ix;
    char cbuf[2];
    
    cbuf[0] = getch();
    cbuf[1] = 0;
    ix = atoi(cbuf);  // find out which color bin to use with vblob()
    printf("##g6 bin# %x\r\n", ix);
    blob_display_flag = 1;
    blob_display_num = ix;
}

#ifdef STEREO
void enable_stereo_processing() {
    if (master) {
        stereo_processing_flag = 1;
        printf("##g5");
    }
}

unsigned int check_stereo_sync() {
    return (*pPORTHIO & 0x0100);
}
#endif /* STEREO */

void set_edge_thresh () {
    unsigned char ch;
    ch = getch();
    edge_thresh = (unsigned int)(ch & 0x0f) * 800;
    printf("#T");
}

void disable_frame_diff() {  // disables frame differencing, edge detect and color segmentation
    frame_diff_flag = 0;
    segmentation_flag = 0;
    edge_detect_flag = 0;
    horizon_detect_flag = 0;
    obstacle_detect_flag = 0;
    blob_display_flag = 0;
    #ifdef STEREO
    stereo_processing_flag = 0;
    #endif /* STEREO */
    printf("#g_");
}

void grab_frame () {
    unsigned int vect[16];
    int slope, intercept;
    unsigned int ix, ii;
    
    #ifdef STEREO
    if (stereo_processing_flag != 0) {
        svs_stereo(0);
        return;
    }
    #endif /* STEREO */

    if (invert_flag)
        move_inverted((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab and flip new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    else
        move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    if (frame_diff_flag) {
        compute_frame_diff((unsigned char *)FRAME_BUF, 
                (unsigned char *)FRAME_BUF2, imgWidth, imgHeight);
    } else if (segmentation_flag) {
        color_segment((unsigned char *)FRAME_BUF);
    } else if (edge_detect_flag) {
        svs_segcode((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh);
        svs_segview((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF);
    } else if (horizon_detect_flag) {
        vhorizon((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 
                16, &vect[0], &slope, &intercept, 5);
        addline((unsigned char *)FRAME_BUF, slope, intercept);
    } else if (obstacle_detect_flag) {
        vscan((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 16, &vect[0]);
        addvect((unsigned char *)FRAME_BUF, 16, &vect[0]);
    } else if (blob_display_flag) {
        ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, blob_display_num);
        if (ix > 7)  // only show 8 largest blobs
            ix = 7;
        for (ii=0; ii<ix; ii++) 
            addbox((unsigned char *)FRAME_BUF, blobx1[ii], blobx2[ii], bloby1[ii], bloby2[ii]);
    }
}


void grab_reference_frame () {
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, 
            (unsigned char *)FRAME_BUF2, imgWidth, imgHeight); 
}

void motion_vect_test (int srange) {  
    char hvect[300], vvect[300];
    int ix, iy, hb, vb;
    
    if (srange < 1) srange = 1;
    if (srange > 3) srange = 3;
    hb = imgWidth / 16;
    vb = imgHeight / 16;
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    copy_image((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, imgWidth, imgHeight);
    move_yuv422_to_planar((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF4, imgWidth, imgHeight);
    _motionvect((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, 
        vvect, hvect, (int)imgWidth, (int)imgHeight, 3);
    for (iy=0; iy<vb; iy++) {
        for (ix=0; ix<hb; ix++) {
            printf("%2d %2d  ", hvect[iy*hb + ix], vvect[iy*hb + ix]);
        }
        printf("\r\n");
    }
}

void motion_vect80x64 () {  
    char hvect[20], vvect[20], hsum[20], vsum[20];
    int ix, iy;
    
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    copy_image((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, 80, 192);
    scale_image_to_80x64_planar ((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF4, imgWidth, imgHeight);
    _motionvect((unsigned char *)FRAME_BUF4, (unsigned char *)FRAME_BUF3, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] = hvect[ix];
        vsum[ix] = vvect[ix];
    }
    _motionvect((unsigned char *)FRAME_BUF4+5120, (unsigned char *)FRAME_BUF3+5120, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] += hvect[ix];
        vsum[ix] += vvect[ix];
    }
    _motionvect((unsigned char *)FRAME_BUF4+10240, (unsigned char *)FRAME_BUF3+10240, vvect, hvect, 80, 64, 3);
    for (ix=0; ix<20; ix++) {
        hsum[ix] += hvect[ix];
        vsum[ix] += vvect[ix];
    }
    for (iy=0; iy<4; iy++) {
        for (ix=0; ix<5; ix++) {
            printf("%2d %2d  ", hsum[iy*5 + ix], vsum[iy*5 + ix]);
        }
        printf("\r\n");
    }
}

/*  compute frame difference between two frames 
     U and V are computed by U1 + 128 - U2
     Y is computed as abs(Y1 - Y2) 
     fcur is current frame
     fref is reference frame*/
void compute_frame_diff(unsigned char *fcur, unsigned char *fref, int w1, int h1) {
    int ix, ipix;
    
    ipix = w1*h1*2;
    for (ix=0; ix<ipix; ix+=2) {
        fcur[ix] = (unsigned char)((unsigned int)fcur[ix] - (unsigned int)fref[ix] + 128);
        if (fcur[ix+1] < fref[ix+1])
            fcur[ix+1] = fref[ix+1] - fcur[ix+1];
        else
            fcur[ix+1] = fcur[ix+1] - fref[ix+1];
    }
}

/* JPEG compress and send frame captured by grab_frame()
   Serial protocol char: I */
void send_frame () {
    unsigned char i2c_data[2];
    unsigned char ch;
    unsigned int ix;
    
    if (overlay_flag) {
        //frame[9] = (framecount % 10) + 0x30;
        //frame[8] = ((framecount/10)% 10) + 0x30;
        //frame[7] = ((framecount/100)% 10) + 0x30;

        i2c_data[0] = 0x41;  // read compass twice to clear last reading
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        i2c_data[0] = 0x41;
        i2cread(0x22, (unsigned char *)i2c_data, 2, SCCB_ON);
        ix = ((i2c_data[0] << 8) + i2c_data[1]) / 10;

        frame[2] = (ix % 10) + 0x30;
        frame[1] = ((ix/10)% 10) + 0x30;
        frame[0] = ((ix/100)% 10) + 0x30;

        sonar();
        ix = sonar_data[1] / 100;
        frame[10] = (ix % 10) + 0x30;
        frame[9] = ((ix/10)% 10) + 0x30;
        frame[8] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[2] / 100;
        frame[16] = (ix % 10) + 0x30;
        frame[15] = ((ix/10)% 10) + 0x30;
        frame[14] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[3] / 100;
        frame[22] = (ix % 10) + 0x30;
        frame[21] = ((ix/10)% 10) + 0x30;
        frame[20] = ((ix/100)% 10) + 0x30;
        ix = sonar_data[4] / 100;
        frame[28] = (ix % 10) + 0x30;
        frame[27] = ((ix/10)% 10) + 0x30;
        frame[26] = ((ix/100)% 10) + 0x30;
        
        set_caption(frame, imgWidth);
    }
    output_start = (unsigned char *)JPEG_BUF;
    output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    image_size = (unsigned int)(output_end - output_start);

    led1_on();
    framecount++;

    imgHead[6] = (unsigned char)(image_size & 0x000000FF);
    imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
    imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
    imgHead[9] = 0x00;
    for (i=0; i<10; i++) {
        while (*pPORTHIO & 0x0001)  // hardware flow control
            continue;
        putchar(imgHead[i]);
    }
    cp = (unsigned char *)JPEG_BUF;
    for (i=0; i<image_size; i++) 
        putchar(*cp++);

    while (getchar(&ch)) {
        // flush input 
        continue;
    }
}

void send_80x64planar () {
    unsigned char ch, *cp;
    unsigned int i;
    
    move_image((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2,  // grab new frame
            (unsigned char *)FRAME_BUF, imgWidth, imgHeight); 
    scale_image_to_80x64_planar ((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF2, 
            imgWidth, imgHeight);

    led1_on();

    printf("P5\n80\n192\n255\n");  // send pgm header
    cp = (unsigned char *)FRAME_BUF2;
    for (i=0; i<80*64*3; i++) 
        putchar(*cp++);

    while (getchar(&ch)) {
        // flush input 
        continue;
    }
}

/* Turn image overlay on.
   Serial protocol char: o */
void overlay_on () {
    overlay_flag = 1;
    printf("#o");
}


/* Turn image overlay off.
   Serial protocol char: O */
void overlay_off () {
    overlay_flag = 0;
    printf("#O");
}

/* Camera initial setup */
void camera_setup () {
    int ix;
    
    /* Initialise camera-related globals */
    framecount = 0;
    overlay_flag = 0;
    quality = 4; // Default JPEG quality - range is 1-8 (1 is highest)
    frame_diff_flag = 0;
    segmentation_flag = 0;
    imgWidth = 320;
    imgHeight = 240;
    strcpy(imgHead, "##IMJ5    ");
    
    for (ix=0; ix<3; ix++) {
        delayMS(100);
        i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
    }
    for (ix=0; ix<3; ix++) {
        delayMS(100);
        i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
    }
    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

void invert_video() {  // flip video for upside-down camera
    invert_flag = 1;
    i2cwrite(0x21, ov7725_invert, sizeof(ov7725_invert)>>1, SCCB_ON);  // flip UV on OV7725
    i2cwrite(0x30, ov9655_invert, sizeof(ov9655_invert)>>1, SCCB_ON);  // flip UV on OV9655
    printf("#y");
}

void restore_video() {  // restore normal video orientation
    invert_flag = 0;
    i2cwrite(0x21, ov7725_restore, sizeof(ov7725_restore)>>1, SCCB_ON); // restore UV on OV7725
    i2cwrite(0x30, ov9655_restore, sizeof(ov9655_restore)>>1, SCCB_ON); // restore UV on OV9655
    printf("#Y");
}

/* Refactored out, code to reset the camera after a frame size change. */
void camera_reset (unsigned int width) {
    if (width == 160) {
        imgWidth = width;
        imgHeight = 120;
        strcpy(imgHead, "##IMJ3    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qqvga, sizeof(ov7725_qqvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qqvga, sizeof(ov9655_qqvga)>>1, SCCB_ON);
        printf("#a");
    } else if (width == 320) {
        imgWidth = width;
        imgHeight = 240;
        strcpy(imgHead, "##IMJ5    ");
        camera_stop();
        i2cwrite(0x21, ov7725_qvga, sizeof(ov7725_qvga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_qvga, sizeof(ov9655_qvga)>>1, SCCB_ON);
        printf("#b");
    } else if (width == 640) {
        imgWidth = width;
        imgHeight = 480;
        strcpy(imgHead, "##IMJ7    ");
        camera_stop();
        i2cwrite(0x21, ov7725_vga, sizeof(ov7725_vga)>>1, SCCB_ON);
        i2cwrite(0x30, ov9655_vga, sizeof(ov9655_vga)>>1, SCCB_ON);
        printf("#c");
    } else if (width == 1280) {
        imgWidth = width;
        imgHeight = 1024;
        strcpy(imgHead, "##IMJ9    ");
        camera_stop();
        i2cwrite(0x30, ov9655_sxga, sizeof(ov9655_sxga)>>1, SCCB_ON);
        printf("#d");
    }
    camera_init((unsigned char *)DMA_BUF1, (unsigned char *)DMA_BUF2, imgWidth, imgHeight);
    camera_start();
}

/* Change image quality.
   Serial protocol char: q */
void change_image_quality () {
    unsigned char ch;
    ch = getch();
    quality = (unsigned int)(ch & 0x0f);
    if (quality < 1) {
        quality = 1;
    } else if (quality > 8) {
        quality = 8;
    }
    printf("##quality - %c\r\n", ch);
}

// write caption string of up to 40 characters to frame buffer 
void set_caption(unsigned char *str, unsigned int width) {
    unsigned char *fbuf, *fcur, *str1, cc;
    int len, ix, iy, iz, w2;
    
    w2 = width * 2;
    str1 = str;
    
    for (len=0; len<40 && *str1++; len++);          // find string length
    fbuf = FRAME_BUF + (unsigned char *)((width * 17) - (len * 8));  // point to 1st char
    
    for (ix=0; ix<len; ix++) {
        fcur = fbuf;
        for (iy=0; iy< 8; iy++) {
            cc = font8x8[str[ix]*8 + iy];
            for (iz=0; iz<8; iz++) {
                if (cc & fontmask[iz]) {
                    fcur[0] = 0x80;
                    fcur[1] = 0xff;
                }
                fcur += 2;
            }
            fcur += (width * 2) - 16;          // move to next line
        }    
        fbuf += 16;  // move to next char position
    }
}


void move_image(unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height) 
{
    unsigned char *src;
    unsigned short *isrc, *idst;
    int ix;
        
    if (*pDMA0_CURR_ADDR < (void *)src2)
        src = src2;
    else
        src = src1;
    
    isrc = (unsigned short *)src;
    idst = (unsigned short *)dst;
    for (ix = 0; ix < (width * height); ix++)
        *idst++ = *isrc++;
}

void move_inverted(unsigned char *src1, unsigned char *src2, unsigned char *dst, unsigned int width, unsigned int height) 
{
    unsigned char *src;
    unsigned short *isrc, *idst;
    int ix;
        
    if (*pDMA0_CURR_ADDR < (void *)src2)
        src = src2;
    else
        src = src1;
    
    isrc = (unsigned short *)src;
    idst = (unsigned short *)(dst + imgWidth*imgHeight*2 - 2);
    for (ix = 0; ix < (width * height); ix++)
        *idst-- = *isrc++;
}

/* copy frame buffer 
   - works whether pixels are interleaved or planar
*/
void copy_image(unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height) 
{
    int ix, xy;
    unsigned short *isrc, *idst;
        
    xy = width * height;
    isrc = (unsigned short *)src;
    idst = (unsigned short *)dst;
    for (ix=0; ix<xy; ix++)
        *idst++ = *isrc++;
}


/* move YUV422 interleaved pixels to separate Y, U and V planar buffers 
   - incoming pixels are UYVY
   - Y buffer is twice the size of U or V buffer 
*/
void move_yuv422_to_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height)
{
    unsigned char *py, *pu, *pv;
    int ix, xy, xy2;
    
    xy = width * height;
    xy2 = xy / 2;
    py = dst;
    pu = dst + xy;
    pv = pu + xy2;
    
    for (ix=0; ix<xy2; ix++) {
        *pu++ = *src++;
        *py++ = *src++;
        *pv++ = *src++;
        *py++ = *src++;
    }
}

/* scale YUV422 interleaved pixels to separate 80x64 Y, U and V planar buffers 
   - incoming pixels are UYVY, so Y pixels will be averaged
*/
void scale_image_to_80x64_planar (unsigned char *src, unsigned char *dst, unsigned int width, unsigned int height)
{
    unsigned char *py, *pu, *pv;
    int ix, iy, xskip, yskip; 
    unsigned int y1, y2;
    
    xskip = (width / 40) - 4;
    yskip = ((height / 60) - 1) * width * 2;
    py = dst;
    pu = dst + 5120;  // 80*64
    pv = pu + 5120;
    
    for (iy=0; iy<64; iy++) {
        if ((iy==1) || (iy==2) || (iy==62) || (iy==63)) {  // duplicate first 2 and last 2 lines 
            for (ix=0; ix<80; ix++) {                      // to fill out 60 lines to 64
                *pu = *(pu-80);
                *pv = *(pv-80);
                *py = *(py-80);
                py++; pu++; pv++;                
            }
            continue;
        }
        for (ix=0; ix<80; ix++) {
            *pu++ = *src++;
            y1 = (unsigned int)*src++;
            *pv++ = *src++;
            y2 = (unsigned int)*src++;
            *py++ = (unsigned char)((y1+y2)>>1);
            src += xskip;
        }
        src += yskip;
    }
}


/* XModem Receive.
   Serial protocol char: X */
void xmodem_receive () {
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + FLASH_BUFFER_SIZE); ix++)
        *((unsigned char *)ix) = 0;   // clear the read buffer
      err1 = xmodemReceive((unsigned char *)FLASH_BUFFER, FLASH_BUFFER_SIZE);
      if (err1 < 0) {
          printf("##Xmodem receive error: %d\r\n", err1);
      } else {
          printf("##Xmodem success. Count: %d\r\n", err1);
      }
}

void launch_editor() {
    edit((unsigned char *)FLASH_BUFFER);
}

/* Clear flash buffer
   Serial protocol char: z-c */
void clear_flash_buffer () {
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + FLASH_BUFFER_SIZE); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    printf("##zclear buffer\r\n");
}

/* crc flash buffer using crc16_ccitt()
   Serial protocol char: z-C */
void crc_flash_buffer () {
    unsigned int ix;
    ix = (unsigned int)crc16_ccitt((void *)FLASH_BUFFER, 0x0001fff8);  // don't count last 8 bytes
    printf("##zCRC: 0x%x\r\n", ix);
}

/* Read user flash sector into flash buffer
   Serial protocol char: z-r */
void read_user_flash () {
    int ix;
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00010000); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    ix = spi_read(FLASH_SECTOR, (unsigned char *)FLASH_BUFFER, 0x00010000);
    printf("##zread count: %d\r\n", ix);
}

void read_user_sector (int isec) {
    int ix;
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00010000); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    printf("##zRead ");
    if ((isec < 2) || (isec > 63)) {
        printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_read((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 0x00010000);
    printf (" - loaded %d bytes from flash sector %d\r\n", ix, isec);   
}

void read_double_sector (int isec, int quiet) {
    int ix;
    for (ix = FLASH_BUFFER; ix < (FLASH_BUFFER  + 0x00020000); ix++)
      *((unsigned char *)ix) = 0;   // clear the read buffer
    if (!quiet)
        printf("##zA ");
    if ((isec < 2) || (isec > 63)) {
        if (!quiet)
            printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_read((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 0x00020000);
    if (!quiet)
        printf (" - loaded %d bytes from flash sector %d->%d\r\n", ix, isec, isec+1);   
}

/* Write user flash sector from flash buffer
   Serial protocol char: z-w */
void write_user_flash () {
    int ix;
    ix = spi_write(FLASH_SECTOR, (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00010000), 0x00010000);
    printf("##zwrite count: %d\r\n", ix);
}

void write_user_sector (int isec) {
    int ix;
    printf("##zWrite ");
    if ((isec < 2) || (isec > 63)) {
        printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_write((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00010000), 0x00010000);
    printf (" - saved %d bytes to flash sector %d\r\n", ix, isec);   
}

void write_double_sector (int isec, int quiet) {  // save 128kB to consecutive sectors
    int ix;
    if (!quiet)
        printf("##zB ");
    if ((isec < 2) || (isec > 63)) {
        if (!quiet)
            printf(" - sector %d not accessible\r\n", isec);
        return;
    }
    ix = spi_write((isec * 0x00010000), (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00020000), 0x00020000);
    if (!quiet)
        printf (" - saved %d bytes to flash sectors %d->%d\r\n", ix, isec, isec+1);   
}

/* Write boot flash sectors (1-2) from flash buffer
   Serial protocol char: z-Z */
void write_boot_flash () {
    unsigned int *ip;
    int ix;
    
    ip = (unsigned int *)FLASH_BUFFER;  // look at first 4 bytes of LDR file - should be 0xFFA00000 or 0xFF800000
    if ((*ip != 0xFFA00000) && (*ip != 0xFF800000)) {
        printf("##zZ boot image - invalid header\r\n");
        return;
    }                        
    ix = spi_write(BOOT_SECTOR, (unsigned char *)FLASH_BUFFER, 
        (unsigned char *)(FLASH_BUFFER + 0x00020000), 0x00020000);
    printf("##zZ boot image write count: %d\r\n", ix);
}

/* Process i2c command:  
        irxy  - i2c read device x, register y, return '##ir value'
        iRxy  - i2c read device x, register y, return 2-byte '##iR value'
        iMxyz - i2c read device x, register y, count z, return z-byte '##iM values'
        iwxyz - i2c write device x, register y, value z, return '##iw'
        iWabcd - i2c write device a, data b, c, d, return '##ix'
   Serial protocol char: i */
void process_i2c() {
    unsigned char i2c_device, i2c_data[16], cx, count;
    
    switch ((unsigned char)getch()) {
        case 'r':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            printf("##ir%2x %d\r\n", i2c_device, i2c_data[0]);
            break;
        case 'R':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2cread(i2c_device, (unsigned char *)i2c_data, 2, SCCB_ON);
            printf("##iR%2x %d\r\n",i2c_device, (i2c_data[0] << 8) + i2c_data[1]);
            break;
        case 'M':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            count = (unsigned char)getch() & 0x0F;
            i2cread(i2c_device, (unsigned char *)i2c_data, (unsigned int)count, SCCB_ON);
            printf("##iM%2x  ", i2c_device);
            for (cx=0; cx<count; cx++) printf("%d ", i2c_data[cx]);
            printf("\r\n");
            break;
        case 'w':
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2cwrite(i2c_device, (unsigned char *)i2c_data, 1, SCCB_ON);
            printf("##iw%2x\r\n", i2c_device);
            break;
        case 'W':  // multi-write
            i2c_device = (unsigned char)getch();
            i2c_data[0] = (unsigned char)getch();
            i2c_data[1] = (unsigned char)getch();
            i2c_data[2] = (unsigned char)getch();
            i2cwritex(i2c_device, (unsigned char *)i2c_data, 3, SCCB_ON);
            printf("##iW%2x", i2c_device);
            break;
        default:
            return;
    }
}



/* Motor command, three character command string follows.
   Serial protocol char: M */
void motor_command() {
    unsigned int mdelay;
    if (!pwm1_init) {
        initPWM();
        pwm1_init = 1;
        pwm1_mode = PWM_PWM;
        base_speed = 40;
        lspeed = rspeed = 0;
    }
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    setPWM(lspeed, rspeed);
    if (mdelay) {
        delayMS(mdelay * 10);
        setPWM(0, 0);
        lspeed = 0;
        rspeed = 0;
    }
    printf("#M");
}

/* Motor command, takes three integers, lspeed, rspeed, and mudelay. */
void motor_command2(int ls, int rs, unsigned int mudelay) {
 /*   if (!pwm1_init) {
        initPPM1();
        pwm1_init = 1;
        pwm1_mode = PWM_PPM;
        base_speed = 40;
        lspeed = rspeed = 0;
    }
    lspeed = ls;
    rspeed = rs;
    setPPM1(lspeed, rspeed);
    if (mudelay) {
        delayUS(mudelay);
        setPPM1(0, 0);
        lspeed = 0;
        rspeed = 0;
    }
*/
    if (!pwm1_init) {
        initPWM();
        pwm1_init = 1;
        pwm1_mode = PWM_PWM;
        base_speed = 40;
        lspeed = rspeed = 0;
    }
    lspeed = ls;
    rspeed = rs;
    setPWM(lspeed, rspeed);
    if (mudelay) {
        delayUS(mudelay * 10);
        setPWM(0, 0);
        lspeed = 0;
        rspeed = 0;
    }
    //printf("%d,%d\r", ls, rs);
    // printf("#M");
}


/* Motor command for 2nd set of timers, three character command string follows.
   Serial protocol char: m */
void motor2_command() {
    unsigned int mdelay;
    if (!pwm2_init) {
        initPWM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PWM;
        base_speed2 = 40;
        lspeed2 = rspeed2 = 0;
    }
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    mdelay = (unsigned int)getch();
    setPWM2(lspeed2, rspeed2);
    if (mdelay) {
        delayMS(mdelay * 10);
        setPWM2(0, 0);
        lspeed2 = 0;
        rspeed2 = 0;
    }
    printf("#m");
}

/* Increase motor base speed
   Serial protocol char: + */
void motor_increase_base_speed() {
    base_speed += 3;
    if (base_speed > 95) {
        base_speed = 95;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 3);
        rspeed = check_bounds_0_100(rspeed + 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#+");
}

/* Decrease motor base speed
   Serial protocol char: - */
void motor_decrease_base_speed() {
    base_speed -= 3;
    if (base_speed < 0) {
        base_speed = 0;
    }
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 3);
        rspeed = check_bounds_0_100(rspeed - 3);
        setPPM1(lspeed, rspeed);
    }
    printf("#-");
}

void motor_trim_left() {
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed - 1);
        rspeed = check_bounds_0_100(rspeed + 1);
        setPPM1(lspeed, rspeed);
    }
}

void motor_trim_right() {
    if (pwm1_mode == PWM_PPM) {
        lspeed = check_bounds_0_100(lspeed + 1);
        rspeed = check_bounds_0_100(rspeed - 1);
        setPPM1(lspeed, rspeed);
    }
}

/* Take motor action */
void motor_action(unsigned char ch) {

    #ifdef STEREO
    if (ch == '.') svs_right_turn_percent = 21;
    if (ch == '0') svs_right_turn_percent = -21;
    #endif
    
    motor_set(ch, base_speed, &lspeed, &rspeed);
    printf("#%c", ch);
}

/* General motor control code */
void motor_set(unsigned char cc, int speed, int *ls, int *rs)  {
    int left_speed, right_speed;

    if (pwm1_mode != PWM_PWM) // only run the keypad commands in PWM mode
        return;
    
    /* record the time at which the motors started moving */
    if (cc != '5') {
        move_start_time = readRTC();
        robot_moving = 1;
    }
    
    left_speed = right_speed = 0;
    switch (cc) {
        case '7':     // drift left
            left_speed = speed-15;
            right_speed = speed+15;
            break;
        case '8':     // forward
            left_speed = speed; 
            right_speed = speed;
            break;
        case '9':     // drift right
            left_speed = speed+15;
            right_speed = speed-15;
            break;
        case '4':     // turn left
            left_speed = speed-30;
            right_speed = speed+30;
            break;
        case '5':        // stop
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            //printf("Move time at speed %d = %d mS\r\n", speed, move_time_mS);
            break;
        case '6':     // turn right
            left_speed = speed+30;
            right_speed = speed-30;
            break;
        case '1':     // back left
            left_speed = -(speed-30);
            right_speed = -(speed+30);
            break;
        case '2':     // back
            left_speed = -speed;
            right_speed = -speed;
            break;
        case '3':     // back right
            left_speed = -(speed+30);
            right_speed = -(speed-30);
            break;
        case '.':     // clockwise turn
            setPWM(70, -70);
            #ifdef STEREO
            delayMS(100);
            #else
            delayMS(200);
            #endif
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            break;
        case '0':     // counter clockwise turn
            setPWM(-70, 70);
            #ifdef STEREO
            delayMS(100);
            #else
            delayMS(200);
            #endif
            setPWM(0, 0);
            left_speed = 0;
            right_speed = 0;
            move_stop_time = readRTC();
            move_time_mS = move_stop_time - move_start_time;
            robot_moving = 0;
            break;
    }
    setPWM(left_speed, right_speed);
    *ls = left_speed;
    *rs = right_speed;
    return;
}

/* servo command, timers 2 and 3, two character command string follows.
   Serial protocol char: S */
void ppm1_command() {
    if (!pwm1_init) {
        initPPM1();
        pwm1_init = 1;
        pwm1_mode = PWM_PPM;
    }
    lspeed = (int)((signed char)getch());
    rspeed = (int)((signed char)getch());
    setPPM1(lspeed, rspeed);
    printf("#S");
}

/* servo command, timers 6 and 7, two character command string follows.
   Serial protocol char: s */
void ppm2_command() {
    if (!pwm2_init) {
        initPPM2();
        pwm2_init = 1;
        pwm2_mode = PWM_PPM;
    }
    lspeed2 = (int)((signed char)getch());
    rspeed2 = (int)((signed char)getch());
    setPPM2(lspeed2, rspeed2);
    printf("#s");
}

void initPWM() {
    // configure timers 2 and 3 for PWM (H-bridge interface)
    //*pPORT_MUX = 0;  // don't do this - it clobbers timers 6/7
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE = TIMEN2 | TIMEN3;
    *pPORTHIO_DIR |= 0x0030;  // set PORTH4 and PORTH5 to output for direction control
    *pPORTHIO &= 0xFFCF;      // set output low 
    //*pPORTHIO |= 0x0030;  
}

void initPWM2() {
    // configure timers 6 and 7 for PWM
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 1000;                // 1000Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100; 
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * 1) / 100;
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void initTMR4() {
    // configure timer 4
    *pPORTF_FER |= 0x0020;  // configure PF5 TMR4
    *pTIMER4_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER4_PERIOD = PERIPHERAL_CLOCK;  // should be counting a 122MHz rate
    *pTIMER4_WIDTH = PERIPHERAL_CLOCK;
    *pTIMER_ENABLE |= TIMEN4;
}

void initPPM1() {
    // configure timers 2 and 3
    *pPORTF_FER |= 0x00C0;  // configure PF6 and PF7 as TMR3 and TMR2
    *pTIMER2_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER3_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER2_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER3_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER_ENABLE |= TIMEN2 | TIMEN3;
}

void initPPM2() {
    // configure timers 6 and 7
    *pPORT_MUX |= 0x0010;   // note that this reassigns UART1 signals as timers
    *pPORTF_FER |= 0x000C;  // configure PF2 and PF3 as TMR7 and TMR6
    *pTIMER6_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER7_CONFIG = PULSE_HI | PWM_OUT | PERIOD_CNT;
    *pTIMER6_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER7_PERIOD = PERIPHERAL_CLOCK / 50;                // 50Hz
    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * 150) / 2000; // 1.5 millisec pulse
    *pTIMER_ENABLE |= TIMEN6 | TIMEN7;
}

void setPWM (int mleft, int mright) {
    if (mleft < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFEF);  // clear left direction bit
        mleft = -mleft;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFEF) | 0x0010;  // turn on left direction bit
    }
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright < 0) {
        *pPORTHIO = (*pPORTHIO & 0xFFDF);  // clear right direction bit
        mright = -mright;
    } else {
        *pPORTHIO = (*pPORTHIO & 0xFFDF) | 0x0020;  // turn on right direction bit
    }
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPWM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;

    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mleft) / 100;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 1000) * mright) / 100;
}

void setPPM1 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER2_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER3_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

void setPPM2 (int mleft, int mright) {
    if (mleft > 100)
        mleft = 100;
    if (mleft < 1)
        mleft = 1;
    if (mright > 100)
        mright = 100;
    if (mright < 1)
        mright = 1;

    *pTIMER6_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mleft)) / 2000;
    *pTIMER7_WIDTH = ((PERIPHERAL_CLOCK / 50) * (100 + mright)) / 2000;
}

int check_bounds_0_100(int ix) {
    if (ix > 100)
        ix = 100;
    if (ix < 0)
        ix= 0;
    return ix;
}

/* Initialise the Real-time Clock */
void initRTC() {
    *pRTC_ICTL = 0;  // disable interrupts
    SSYNC;
    *pRTC_PREN = 0;  // disable prescaler - clock counts at 32768 Hz
    SSYNC;
    *pRTC_STAT = 0;  // clear counter
    SSYNC;
}

/* Read the RTC counter, returns number of milliseconds since reset */
int readRTC() {     
    int i1, i2;
    i1 = *pRTC_STAT;
    i2 = (i1 & 0x0000003F) + (((i1 >> 6) & 0x0000003F) * 60) +  
        (((i1 >> 12) & 0x0000001F) * 3600) + (((i1 >> 17) & 0x00007FFF) * 86400);
    return (i2 / 33);  // converts tick count to milliseconds
                       //    32,768 / 32.77 = 1,000
}

/* Clear the RTC counter value */
void clearRTC() {
    *pRTC_STAT = 0;
    SSYNC;
}

void delayMS(int delay) {  // delay up to 100000 millisecs (100 secs)
    int i0;

    if ((delay < 0) || (delay > 100000))
        return;
    i0 = readRTC();
    while (readRTC() < (i0 + delay))
        continue;
}

void delayUS(int delay) {  // delay up to 100000 microseconds (.1 sec)
    // CORE_CLOCK (MASTER_CLOCK * VCO_MULTIPLIER / CCLK_DIVIDER) = 22,118,000 * 22
    // PERIPHERAL_CLOCK  (CORE_CLOCK / SCLK_DIVIDER) =  CORE_CLOCK / 4 = 121,649,000
    // *pTIMER4_PERIOD = PERIPHERAL_CLOCK, so TIMER4 should be counting a 121.649MHz rate
    int target, start;
    
    if ((delay < 0) || (delay > 100000))
        return;
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

void delayNS(int delay) {  // delay up to 100000 nanoseconds (.1 millisec)
    // minimum possible delay is approx 10ns
    int target, start;
    
    if ((delay < 10) || (delay > 100000))
        return;
    start = *pTIMER4_COUNTER;
    target = (((PERIPHERAL_CLOCK / 10000) * delay) / 100000) + start;
    
    if (target > PERIPHERAL_CLOCK) {  // wait for timer to wrap-around
        target -= PERIPHERAL_CLOCK;
        while (*pTIMER4_COUNTER > target)
            continue;
    }
    while (*pTIMER4_COUNTER < target)
        continue;
}

/* Enable failsafe - two character motor speeds follow
   Serial protocol char: F */
void enable_failsafe() {
    lfailsafe = (int)((signed char)getch());
        if (lfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            lfailsafe = 1;
    rfailsafe = (int)((signed char)getch());
        if (rfailsafe == 0)  // minimum PWM power setting is 0x01, not 0x00
            rfailsafe = 1;
    failsafe_mode = 1;
    printf("#F");
}

/* Disable failsafe - 
   Serial protocol char: f */
void disable_failsafe() {
    failsafe_mode = 0;
    printf("#f");
}

void reset_failsafe_clock() {
    failsafe_clock = readRTC();
}

void check_failsafe() {
    if (!failsafe_mode)
        return;
    if ((readRTC() - failsafe_clock) < 2000)  // 2 second timeout
        return;
    lspeed = lfailsafe;
    rspeed = rfailsafe;
    if (pwm1_mode == PWM_PWM)
        setPWM(lspeed, rspeed);
    if (pwm1_mode == PWM_PPM)
        setPPM1(lspeed, rspeed);
}

void process_colors() {
    unsigned char ch, ch1, ch2, ch3, ch4;
    unsigned int clr, x1, x2, y1, y2;
    unsigned int ix, iy, i1, i2, itot;
    unsigned int ulo[4], uhi[4], vlo[4], vhi[4];
    int vect[16];  // used by vscan()
    int slope, intercept;
    unsigned char i2c_data[2];
              // vision processing commands
                    //    va = enable/disable AGC / AWB / AEC camera controls
                    //    vb = find blobs matching color bin 
                    //    vc = set color bin ranges
                    //    vd = dump camera registers
                    //    vf = find pixels matching color bin
                    //    vh = histogram
                    //    vm = mean colors
                    //    vp = sample individual pixel
                    //    vr = recall color bin ranges
                    //    vs = scan for edges
                    //    vt = set edge detect threshold (0000-9999, default is 3200)
                    //    vz = zero all color settings
    ch = getch();
    switch (ch) {
        case 'a':  //    va = enable/disable AGC(4) / AWB(2) / AEC(1) camera controls
                   //    va7 = AGC+AWB+AEC on   va0 = AGC+AWB+AEC off
            ix = (unsigned int)getch() & 0x07;
            i2c_data[0] = 0x13;
            i2c_data[1] = 0xC0 + ix;
            i2cwrite(0x30, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV9655
            i2cwrite(0x21, (unsigned char *)i2c_data, 1, SCCB_ON);  // OV7725
            printf("##va%d\r\n", ix);
            break;
        case 'b':  //    vb = find blobs for a given color
            ch1 = getch();
            ch2 = ch1;
            if (ch1 > '9')
                ch1 = (ch1 & 0x0F) + 9;
            else
                ch1 &= 0x0F;
            grab_frame();
            ix = vblob((unsigned char *)FRAME_BUF, (unsigned char *)FRAME_BUF3, ch1);
            if (ix == 0xFFFFFFFF) {
                printf("##vb%c -1\r\n", ch2);
                break;  // too many blobs found
            }
            printf("##vb%c %d\r\n", ch2, ix);
            for (iy=0; iy<ix; iy++) {
                printf(" %d - %d %d %d %d  \r\n", 
                    blobcnt[iy], blobx1[iy], blobx2[iy], bloby1[iy], bloby2[iy]);
            }
            break;
        case 'c':  //    vc = set colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ymin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ymax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            umin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            umax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            vmin[ix] = ch1 * 100 + ch2 * 10  + ch3;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            vmax[ix] = ch1 * 100 + ch2 * 10  + ch3;
            printf("##vc %d\r\n", ix);
            break;
        case 'd':  //    vd = dump camera registers
            printf("##vdump\r\n");
            for(ix=0; ix<256; ix++) {
                i2c_data[0] = ix;
                i2cread(0x21, (unsigned char *)i2c_data, 1, SCCB_ON);
                printf("%x %x\r\n", ix, i2c_data[0]);
            }
            break;
        case 'f':  //    vf = find number of pixels in x1, x2, y1, y2 range matching color bin
            clr = getch() & 0x0F;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            x1 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            x2 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            y1 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            y2 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            grab_frame();
            printf("##vf %d\r\n", vfind((unsigned char *)FRAME_BUF, clr, x1, x2, y1, y2));
            break;
        case 'h':  //    vh = histogram
            grab_frame();
            vhist((unsigned char *)FRAME_BUF);
            printf("##vhist\r\n");
            iy = 0;
            itot = imgWidth * imgHeight / 2;
            printf("      0  16  32  48  64  80  96 112 128 144 160 176 192 208 224 240 (V-axis)\r\n");
            for (i1=0; i1<16; i1++) {
                printf("%d", i1*16);
                for (i2=0; i2<16; i2++) {
                    iy++;
                    ix = i1*16 + i2;
                    if (hist0[ix] > (itot>>2))
                        printf("****");
                    else if (hist0[ix] > (itot>>5))
                        printf(" ***");
                    else if (hist0[ix] > (itot>>8))
                        printf("  **");
                    else if (hist0[ix] > (itot>>11))
                        printf("   *");
                    else {
                        printf("    ");
                        iy--;
                    }
                }
                printf("\r\n");
            }
            printf("(U-axis)             %d regions\r\n", iy);
            break;
        case 'm':  //    vm = mean colors
            grab_frame();
            vmean((unsigned char *)FRAME_BUF);
            printf("##vmean %d %d %d\r\n", mean[0], mean[1], mean[2]);
            break;
        case 'p':  //    vp = sample individual pixel, print YUV value
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            i1 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            i2 = ch1*1000 + ch2*100 + ch3*10 + ch4;
            grab_frame();
            ix = vpix((unsigned char *)FRAME_BUF, i1, i2);
            printf("##vp %d %d %d\r\n",
                ((ix>>16) & 0x000000FF),  // Y1
                ((ix>>24) & 0x000000FF),  // U
                ((ix>>8) & 0x000000FF));   // V
            break;
        case 'r':  //    vr = recall colors
            ix = (unsigned int)getch();
            if (ix > '9')
                ix = (ix & 0x0F) + 9;
            else
                ix &= 0x0F;
            printf("##vr %d %d %d %d %d %d %d\r\n",
               ix, ymin[ix], ymax[ix], umin[ix], umax[ix], vmin[ix], vmax[ix]);
            break;
        case 's':  //    vs = scan for edges 
            x1 = (unsigned int)getch() & 0x0F;  // get number of columns to use
            grab_frame();
            ix = vscan((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, (unsigned int)x1, (unsigned int *)&vect[0]);
            printf("##vscan = %d ", ix);
            for (i1=0; i1<x1; i1++)
                printf("%4d ", vect[i1]);
            printf("\r\n");
            break;
        case 't':  //    vt = set edge detect threshold (0000-9999, default is 3200)
            ch1 = getch() & 0x0F;
            ch2 = getch() & 0x0F;
            ch3 = getch() & 0x0F;
            ch4 = getch() & 0x0F;
            edge_thresh = ch1*1000 + ch2*100 + ch3*10 + ch4;
            printf("##vthresh %d\r\n", edge_thresh);
            break;
        case 'u':  //    vu = scan for horizon, 
            x1 = (unsigned int)getch() & 0x0F;  // get number of columns to use
            grab_frame();
            ix = vhorizon((unsigned char *)SPI_BUFFER1, (unsigned char *)FRAME_BUF, edge_thresh, 
                        (unsigned int)x1, (unsigned int *)&vect[0], &slope, &intercept, 5);
            printf("##vhorizon = %d ", ix);
            for (i1=0; i1<x1; i1++)
                printf("%4d ", vect[i1]);
            printf("\r\n");
            break;
        case 'z':  //    vz = clear or segment colors
            ix = (unsigned int)getch() & 0x0F;
            printf("##vzero\r\n");
            switch (ix) {
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
}

void process_neuralnet() {
    unsigned char ch;
    unsigned int ix, i1, i2;
              // neural net processing commands
                    //    np = set pattern
                    //    nd = display pattern
                    //    ni = init network
                    //    nt = train for 10000 iterations
                    //    nx = test a pattern
                    //    nb = match blob to patterns
                    //    ng = create pattern from blob
    ch = getch();
    switch (ch) {
        case 'p':  //    np = set pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\r\n");
                break;
            }
            for (i1=0; i1<8; i1++)
                npattern[ix*8 + i1] = (ctoi(getch()) << 4) + ctoi(getch());
            printf("##np %d\r\n", ix);
            break;
        case 'd':  //    nd = display pattern
            ix = ctoi(getch());
            if (ix > NUM_NPATTERNS) {
                printf("##np - invalid index\r\n");
                break;
            }
            printf("##nd %d\r\n", ix);
            nndisplay(ix);
            break;
        case 'i':  //    ni = init network
            nninit_network();
            printf("##ni - init neural net\r\n");
            break;
        case 't':  //    nt = train network
            nntrain_network(10000);
            printf("##nt - train 10000 iterations\r\n");
            for (ix=0; ix<NUM_NPATTERNS; ix++) {
                nnset_pattern(ix);
                nncalculate_network();
                for (i1=0; i1<NUM_OUTPUT; i1++) 
                    printf(" %3d", N_OUT(i1)/10);
                printf("\r\n");
            }
            break;
        case 'x':  //    nx = test example pattern
            ix = 0;
            for (i1=0; i1<8; i1++) {   /// capture the test pattern and store in N_IN input neurons
                ch = (ctoi(getch()) << 4) + ctoi(getch());
                for (i2=0; i2<8; i2++) {
                    if (ch & nmask[i2])
                        N_IN(ix++) = 1024;
                    else
                        N_IN(ix++) = 0;
                }
            }
            nncalculate_network();
            printf("##nx\r\n");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\r\n");
            break;            
        case 'b':  //    nb = match blob to patterns
            ix = ctoi(getch());    // grab the blob #
            if (!blobcnt[ix]) { 
                printf("##nb - not a valid blob\r\n");
                break;
            }
            /* use data still in blob_buf[] (FRAME_BUF3)
               square the aspect ratio of x1, x2, y1, y2
               then subsample blob pixels to populate N_IN(0:63) with 0:1024 values
               then nncalculate_network() and display the N_OUT() results */
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[ix], blobx1[ix], blobx2[ix], 
                    bloby1[ix], bloby2[ix], imgWidth, imgHeight);
            nncalculate_network();
            printf("##nb\r\n");
            for (i1=0; i1<NUM_OUTPUT; i1++) 
                printf(" %3d", N_OUT(i1)/10);
            printf("\r\n");
            break;
        case 'g':  //     ng = create pattern from blob
            ix = ctoi(getch());    // grab the new pattern #
            if (!blobcnt[0]) { 
                printf("##ng - no blob to grab\r\n");
                break;
            }
            nnscale8x8((unsigned char *)FRAME_BUF3, blobix[0], blobx1[0], blobx2[0], 
                    bloby1[0], bloby2[0], imgWidth, imgHeight);
            nnpack8x8(ix);
            nndisplay(ix);
            break;
    }
}

/* use GPIO H14 and H15 for 2-channel wheel encoder inputs -
    H14 (pin 31) is left motor, H15 (pin 32) is right motor */
void init_encoders() {  
    if (encoder_flag)
        return;
    encoder_flag = 1;
    *pPORTHIO_INEN |= 0xC000;  // enable H14 and H15 as inputs
    *pPORTHIO_DIR &= 0x3FFF;   // set H14 and H15 as inputs
    initTMR4();
}

void read_encoders()
{
    encoders();
    printf("##$Encoders:  left = %d  right = %d\r\n", lcount, rcount);
}

/* read encoder pulses from GPIO-H14 and H15.  compute pulses per second, and
      pack left and right encoder counts into 32-bit unsigned */
unsigned int encoders() {
    int t0;
    unsigned int llast, rlast, lnew, rnew, ltime0, ltime1, rtime0, rtime1;
    
    init_encoders();

    t0 = readRTC();
    llast = *pPORTHIO & 0x4000;
    rlast = *pPORTHIO & 0x8000;
    ltime0 = ltime1 = rtime0 = rtime1 = 0;
    lcount = rcount = 0;
    
    while ((readRTC() - t0) < 40) {  
        lnew = *pPORTHIO & 0x4000;
        if (llast != lnew) {
            llast = lnew;
            lcount++;
            if (lcount == 1)
                ltime0 = *pTIMER4_COUNTER;
            if (lcount == 3)
                ltime1 = *pTIMER4_COUNTER;
        }
        rnew = *pPORTHIO & 0x8000;
        if (rlast != rnew) {
            rlast = rnew;
            rcount++;
            if (rcount == 1)
                rtime0 = *pTIMER4_COUNTER;
            if (rcount == 3)
                rtime1 = *pTIMER4_COUNTER;
        }
        if ((lcount>=3) && (rcount>=3))
            break;
    }
    if (!ltime1) ltime0 = 0;
    if (ltime1 < ltime0) ltime1 += PERIPHERAL_CLOCK;   // fix wraparound
    if (!rtime1) rtime0 = 0;
    if (rtime1 < rtime0) rtime1 += PERIPHERAL_CLOCK;   // fix wraparound

    ltime1 -= ltime0;  // compute pulse width
    if (ltime1) 
        lcount = PERIPHERAL_CLOCK / ltime1;  // compute pulses per second
    else 
        lcount = 0;

    rtime1 -= rtime0;  
    if (rtime1) 
        rcount = PERIPHERAL_CLOCK / rtime1;  // compute pulses per second
    else 
        rcount = 0;
    
    return ((unsigned int)lcount << 16) + (unsigned int)rcount;
}

unsigned int encoder_4wd(unsigned int ix)
{
    int t0, ii, val;
    unsigned char ch;
    
    if (xwd_init == 0) {
        xwd_init = 1;
        init_uart1(115200);
        delayMS(10);
    }
    uart1SendChar('e');
    uart1SendChar((char)(ix + 0x30));

    ii = 10000;  // range is 0 - 65535
    val = 0;
    while (ii) {
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if ((ch < '0') || (ch > '9'))
            continue;
        val += (unsigned int)(ch & 0x0F) * ii;        
        ii /= 10;
    }
    while (1) {  // flush the UART1 receive buffer
        t0 = readRTC();
        while (!uart1GetChar(&ch))
            if ((readRTC() - t0) > 10)  // 10msec timeout
                return 0;
        if (ch == '\n')
            break;
    }
    return val;
}

void read_encoder_4wd()
{
    unsigned int channel;
    channel = (unsigned int)(getch() & 0x0F);
    printf("##$e%d %05d\r\n", channel, encoder_4wd(channel));
}

void testSD() {
    unsigned int numsec;
    
    InitSD();
    printf("CardInit() returns %d\r\n", CardInit());
    printf("GetCardParams() returns %d     ", GetCardParams(&numsec));
    printf("%d sectors found\r\n", numsec);
    CloseSD();
}

/* pseudo-random number generator based on Galois linear feedback shift register
     taps: 32 22 2 1; characteristic polynomial:  x^32 + x^22 + x^2 + x^1 + 1  */
unsigned int rand() { 
    int ix, iy;
    if (rand_seed == 0x55555555) {  // initialize 
        iy = (readRTC() % 1000) + 1000;
        for (ix=0; ix<iy; ix++)
            rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    }
    for (ix=0; ix<19; ix++)  // use every 19th result
        rand_seed = (rand_seed >> 1) ^ (-(rand_seed & 0x00000001) & 0x80200003); 
    return (rand_seed);
}


/* JPEG compress and send frame captured by grab_frame()
   Serial protocol char: I */
void send_frame_nr(int im) {
    //unsigned char i2c_data[2];
    unsigned char ch;
    //unsigned int ix;

    output_start = (unsigned char *)JPEG_BUF;
    if(im == 1)
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    }
    else if(im == 2)
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF2, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    }
    else
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF3, output_start, quality, 
            FOUR_TWO_TWO, imgWidth, imgHeight); 
    }
    image_size = (unsigned int)(output_end - output_start);

    led1_on();
    framecount++;

    imgHead[6] = (unsigned char)(image_size & 0x000000FF);
    imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
    imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
    imgHead[9] = 0x00;
    for (i=0; i<10; i++) {
        while (*pPORTHIO & 0x0001)  // hardware flow control
            continue;
        putchar(imgHead[i]);
    }
    cp = (unsigned char *)JPEG_BUF;
    for (i=0; i<image_size; i++) 
        putchar(*cp++);

    while (getchar(&ch)) {
        // flush input 
        continue;
    }
}

void send_frame_nr_size (int im, int size, int width, int height)
{
    // unsigned char i2c_data[2];
    unsigned char ch;
    // unsigned int ix;

    output_start = (unsigned char *)JPEG_BUF;
    if(im == 1)
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF, output_start, quality, 
            FOUR_TWO_TWO, width, height); 
    }
    else if(im == 2)
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF2, output_start, quality, 
            FOUR_TWO_TWO, width, height); 
    }
    else
    {
    	output_end = encode_image((unsigned char *)FRAME_BUF3, output_start, quality, 
            FOUR_TWO_TWO, width, height); 
    }
    image_size = (unsigned int)(output_end - output_start);
	//printf("start = %d, end = %d, image size: %d\n\r", output_start, output_end, image_size);
    led1_on();
    framecount++;

    strcpy(imgHead, "##IMJ1    "); // 40 x 30 image
//    printf("width = %d, height = %d\n\r", width, height);
    imgHead[6] = (unsigned char)(image_size & 0x000000FF);
    imgHead[7] = (unsigned char)((image_size & 0x0000FF00) >> 8);
    imgHead[8] = (unsigned char)((image_size & 0x00FF0000) >> 16);
    imgHead[9] = 0x00;
    //printf(imgHead);

    for (i=0; i<10; i++) {
        putchar(imgHead[i]);
    }
    cp = (unsigned char *)JPEG_BUF;
    for (i=0; i<image_size; i++) {
        putchar(*cp++);
    }

    while (getchar(&ch)) {
        // flush input 
        continue;
    }
}

void send_thumbnail()
{
   // FRAME_BUF -> FRAME_BUF2
   // sub sampling
    unsigned char *frame_buf, *frame_buf2;
    unsigned int ix, inew;
    int x_new, y_new, xx, yy;
	int width;
	width = imgWidth / 4;
    frame_buf = (unsigned char*) FRAME_BUF;
    frame_buf2 = (unsigned char*) FRAME_BUF2;
    y_new = 0;
	x_new = 0;
    for(yy = 0; yy < imgHeight; yy += 4)
    {
		x_new = 0;
    	for (xx = 0; xx < imgWidth; xx += 4)
    	{
			ix = index(xx, yy);
			inew = ((y_new * width + x_new) * 2) & 0xFFFFFFFC;
        	frame_buf2[inew] = frame_buf[ix];
	        frame_buf2[inew + 1] = frame_buf[ix+1];
	        frame_buf2[inew + 2] = frame_buf[ix+2];
	        frame_buf2[inew + 3] = frame_buf[ix+3];
			x_new++;
		}
		y_new++;
    }
    // send FRAMEBUF2
    send_frame_nr_size(2, (x_new*y_new)*2, x_new, y_new);
    return;
}

void putThumbnailInFrame(unsigned char* source, unsigned char* destination, int* size_in_bytes, int* width, int* height, int step)
{
   // sub sampling
    unsigned int ix, inew;
    unsigned int x_new, y_new, xx, yy;
	(*width) = imgWidth / step;
	(*height) = imgHeight / step;

	//int printed = 0;
	//printf("step = %d, imgWidth = %d, imgHeight = %d\n\r", step, imgWidth, imgHeight);
	x_new = 0;
    y_new = 0;
    for(yy = 0; yy < imgHeight; yy += step)
    {
		x_new = 0;
    	for (xx = 0; xx < imgWidth; xx += step)
    	{
			ix = index(xx, yy);
			// inew = index(x_new, y_new); // is also based on imgWidth, but should be based on width
			inew = ((y_new * (*width) + x_new) * 2) & 0xFFFFFFFC;
			/*if(printed < 10)
			{
				printf("xx = %d, yy = %d, ix = %d, x_new = %d, y_new = %d, inew = %d\n\t", xx, yy, ix, x_new, y_new, inew);
				printed++;
			}
			*/
        	destination[inew] = source[ix];
	        destination[inew + 1] = source[ix+1];
	        destination[inew + 2] = source[ix+2];
	        destination[inew + 3] = source[ix+3];
			x_new++;
		}
		y_new++;
    }

    // send FRAMEBUF2
	(*size_in_bytes) = (x_new*y_new)*2;
	(*width) = x_new;
	(*height) = y_new;

    return;
}

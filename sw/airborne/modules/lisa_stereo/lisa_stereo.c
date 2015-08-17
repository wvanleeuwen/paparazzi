/*
 * Copyright (C) 2015 Kirk Scheper
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/lisa_stereo.c
 *  @brief interface to usb stereocam
 */

#include "lisa_stereo.h"
#include <stdlib.h>
//#include "std.h"
//#include "generated/modules.h"
//#include "firmwares/rotorcraft/guidance/guidance_h.h" // to set heading
//#include "led.h"
#include "mcu_periph/uart.h"
//#include "subsystems/datalink/downlink.h"
#include "subsystems/datalink/telemetry.h"


#define STEREO_PORT    UART1

#define __StereoLink(dev, _x) dev##_x
#define _StereoLink(dev, _x)  __StereoLink(dev, _x)
#define StereoLink(_x) _StereoLink(STEREO_PORT, _x)
#define StereoBuffer() StereoLink(ChAvailable())


typedef struct ImageProperties{
  uint32_t positionImageStart;
  uint16_t width;
  uint16_t height;
} ImageProperties;

uint8_t isEndOfImage(uint8_t*);
uint8_t isStartOfImage(uint8_t*);
ImageProperties get_image_properties(uint8_t *, uint32_t, uint32_t);
static uint8_t handleStereoPackage(void);

// pervasive local variables
ImageProperties imageProperties;

#define MSG_BUF_SIZE 128    // size of image buffer
uint8_t msg_buf[MSG_BUF_SIZE];    // buffer used to contain image without line endings

#define BUF_SIZE 16384    // size of circular buffer
uint8_t ser_read_buf[BUF_SIZE];     // circular buffer for incoming data
uint32_t insert_loc, extract_loc, img_start;   // place holders for buffer read and write

uint8_t send_data;

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means that this is the end of an image
 */
uint8_t isEndOfImage(uint8_t *stack){
  if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==171){
    return 1;
  }
  return 0;
}

/**
 * Checks if the sequence in the array is equal to 255-0-0-171,
 * as this means a new image is starting from here
 */
uint8_t isStartOfImage(uint8_t *stack){
  if (stack[0] == 255 && (stack[1] == 0) && (stack[2] == 0) && stack[3]==175){
    return 1;
  }
  return 0;
}

ImageProperties get_image_properties(uint8_t *raw, uint32_t img_start, uint32_t img_end){
    imageProperties = (ImageProperties){img_start,0,0};
    uint32_t i = img_start, startOfLine = 0;
    while( 1 ){
      // Check the first 3 bytes for the pattern 255-0-0, then check what special byte is encoded next
        if ((raw[i] == 255) && (raw[i + 1] == 0) && (raw[i + 2] == 0)){
            if (raw[i + 3] == 171){ // End of image
                break;
            }
            if (raw[i + 3] == 128) // Start of line
              startOfLine = i;

            if (raw[i + 3] == 218) // End of line
              imageProperties.height++;
        }
        i = (i + 1) % BUF_SIZE;
    }

    imageProperties.width = (i - startOfLine - 8 + BUF_SIZE)  % BUF_SIZE; // removed 8 for the indication bits at the start and end of line

    return imageProperties;
}

static uint8_t handleStereoPackage(void) {
    // read all data from serial buffer
    while (StereoLink(ChAvailable()))
    {
      ser_read_buf[insert_loc] = StereoLink(Getch());
      insert_loc = (insert_loc + 1) % BUF_SIZE;
    }
    
    // currently written to search for full image buffer, if found increments read location and returns immediately
    while((insert_loc - extract_loc + BUF_SIZE) % BUF_SIZE > 0)    // Check if we found the end of the image
    {
      if(isStartOfImage(ser_read_buf + extract_loc))
        img_start = extract_loc;
      else if(isEndOfImage(ser_read_buf + extract_loc)) // process image
      {
        // Find the properties of the image by iterating over the complete image
        imageProperties = get_image_properties(ser_read_buf, img_start, extract_loc);

        // Copy array to telemetry buf and remove all bytes that are indications of start and stop lines
        uint32_t i = img_start, j = 0, index = 0;
        while((extract_loc - i + BUF_SIZE) % BUF_SIZE > 0 ){
          if ((ser_read_buf[i] == 255) && (ser_read_buf[i + 1] == 0) && (ser_read_buf[i + 2] == 0) && ser_read_buf[i + 3] == 128){ // Start Of Line
              j = 0;
              i = (i + 4) % BUF_SIZE;   // step over
              while (j++ < imageProperties.width)
              {
                msg_buf[index++] = ser_read_buf[i];
                i = (i + 1) % BUF_SIZE;
              }
          }
          i = (i + 1) % BUF_SIZE;
        } // continue search for new line
        extract_loc = (extract_loc + 4) % BUF_SIZE; // step over end of image
        return 1; // full message read
      }
      extract_loc = (extract_loc + 1) % BUF_SIZE;
    }
    return 0;
}

extern void lisa_stereo_start(void){
  // initial local variables
  imageProperties = (ImageProperties){0,0,0};

  insert_loc = 0;
  extract_loc = 0;
  
  img_start = 0;

  send_data = 0;

  // initiate uart
  UART1Init();
}

extern void lisa_stereo_stop(void) {
}

extern void lisa_stereo_periodic(void) {
  if( handleStereoPackage() )
  {
    //send_data = (send_data +1) % 5;
    //if (send_data == 0)
      //DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, 1, image_buf);
      //image_buf[0] = 1;
  }
  send_data = (send_data + 1) % 128;
    if (send_data == 0)
  		DOWNLINK_SEND_STEREO_IMG(DefaultChannel, DefaultDevice, &insert_loc, &extract_loc, &img_start, &imageProperties.width, imageProperties.width, msg_buf);
}

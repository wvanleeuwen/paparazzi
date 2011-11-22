/*
 * $Id: temp_lm75.c $
 *
 * Copyright (C) 2010 Martin Mueller
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

#include "atmega48.h"

#include "led.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "messages.h"
#include "downlink.h"
#include "generated/periodic.h"


#define VISION_DATA_SIZE 20

uint8_t vision_data[VISION_DATA_SIZE] = { 10, 30, 20, 40,
					  30, 50, 40, 60,
					  50, 70, 60, 80,
					  50, 70, 60, 80,
					  50, 70, 60, 80,
                                        };


void vision_periodic( void )
{
  static uint8_t nr = 0;
  vision_data[nr] ++;
  nr ++;
  if (nr >= VISION_DATA_SIZE)
    nr = 0;

  RunOnceEvery(10,DOWNLINK_SEND_PAYLOAD(DefaultChannel, VISION_DATA_SIZE, vision_data));
}

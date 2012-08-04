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

/** \file temp_lm75.c
 *  \brief National LM75 I2C sensor interface
 *
 *   This reads the values for temperature from the National LM75 sensor through I2C.
 */


#include "atmega48.h"

#include "mcu_periph/i2c.h"
#include "led.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include "generated/periodic.h"
#include "estimator.h"


uint8_t to_atmega48[ATMEGA_TX_SIZE];
uint8_t from_atmega48[ATMEGA_RX_SIZE];


void atmega48_init(void)
{
  from_atmega48[0] = 10;
  from_atmega48[1] = 20;
  from_atmega48[2] = 30;
  from_atmega48[3] = 40;
  from_atmega48[4] = 50;
  from_atmega48[5] = 60;
}


void atmega48_periodic( void )
{
  from_atmega48[10] = to_atmega48[0];
  from_atmega48[11] = to_atmega48[1];
  RunOnceEvery(6,DOWNLINK_SEND_PAYLOAD(DefaultChannel, ATMEGA_RX_SIZE, from_atmega48));
}

void atmega48_event( void )
{
}

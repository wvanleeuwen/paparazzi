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

#include "mcu_periph/i2c.h"
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
#include "estimator.h"


struct i2c_transaction atmega48_trans;

#ifndef ATMEGA48_I2C_DEV
#define ATMEGA48_I2C_DEV i2c0
#endif


#ifndef ATMEGA48_SLAVE_ADDR
#define ATMEGA48_SLAVE_ADDR 0x66
#endif


uint8_t to_atmega48[ATMEGA_TX_SIZE];
uint8_t from_atmega48[ATMEGA_RX_SIZE];


void atmega48_init(void)
{
  atmega48_trans.status = I2CTransDone;
}


void atmega48_periodic( void )
{
  int i;
/*
  static uint8_t tel = 0;

  tel++;
  if (tel > 3)
  {
    tel = 0;
  }
  to_atmega48[0] = tel;
*/
  for (i=0;i<ATMEGA_TX_SIZE;i++)
  {
    atmega48_trans.buf[i] = to_atmega48[i];
  }
  I2CTransceive(ATMEGA48_I2C_DEV, atmega48_trans, ATMEGA48_SLAVE_ADDR, ATMEGA_TX_SIZE, ATMEGA_RX_SIZE);
}

void atmega48_event( void )
{
  int i;
  if (atmega48_trans.status == I2CTransSuccess)
  {
    for (i=0;i<ATMEGA_RX_SIZE;i++)
    {
      from_atmega48[i] = atmega48_trans.buf[i];
    }

    RunOnceEvery(6,DOWNLINK_SEND_PAYLOAD(DefaultChannel, ATMEGA_RX_SIZE, from_atmega48));
    atmega48_trans.status = I2CTransDone;
  }
}

/*
 * Copyright (C) 2009  Gautier Hattenberger
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

#include "fireswarm_payload.h"
#include "mcu_periph/uart.h"
#include "led.h"

#define FIRESWARM_PAYLOAD_POWER_LED  5

void fireswarm_payload_init(void)
{
  LED_INIT(FIRESWARM_PAYLOAD_POWER_LED);
}

void fireswarm_periodic(void)
{
  LED_TOGGLE(FIRESWARM_PAYLOAD_POWER_LED);
}

void fireswarm_event(void)
{
}


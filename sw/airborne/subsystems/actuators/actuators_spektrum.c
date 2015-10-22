/*
 * Copyright (C) 2013 The Paparazzi Team
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
 */

/** @file actuators_spektrum.c
 *  Actuators for spektrum output at ~11ms
 */

#include "subsystems/actuators.h"
#include "subsystems/actuators/actuators_spektrum.h"

#include "mcu_periph/uart.h"

struct ActuatorsSpektrum actuators_spektrum;

void actuators_spektrum_init(void)
{
  actuators_spektrum.device = &((ACTUATORS_SPEKTRUM_DEV).device);
}


void actuators_spektrum_set(void)
{
  static uint8_t cnt = 0;

  cnt++;
  // Only send every 11 ms
  if(cnt == 6) {
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 0x00); // number missed frames
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 0x12); // 7 channels, 11 bit, 11ms
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 0 << 3 | actuators_spektrum.cmds[0] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[0] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 1 << 3 | actuators_spektrum.cmds[1] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[1] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 2 << 3 | actuators_spektrum.cmds[2] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[2] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 3 << 3 | actuators_spektrum.cmds[3] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[3] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 4 << 3 | actuators_spektrum.cmds[4] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[4] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 5 << 3 | actuators_spektrum.cmds[5] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[5] & 0xFF);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, 6 << 3 | actuators_spektrum.cmds[6] >> 8);
    actuators_spektrum.device->put_byte(actuators_spektrum.device->periph, actuators_spektrum.cmds[6] & 0xFF);
    cnt = 0;
  }
}


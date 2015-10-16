/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file actuators_asctec_v2.h
 *  Actuators driver for Asctec v2 motor controllers.
 */

#ifndef ACTUATORS_SPEKTRUM_H
#define ACTUATORS_SPEKTRUM_H

#include "mcu_periph/uart.h"
#include "generated/airframe.h"


struct ActuatorsSpektrum {
  enum actuators_spektrum_cmd cmd;
  int32_t cmds[ACTUATORS_NB];
  struct i2c_transaction i2c_trans;
  volatile uint32_t nb_err;
};


extern struct ActuatorsSpektrum actuators_spektrum;

extern void actuators_asctec_v2_init(void);
extern void actuators_asctec_v2_set(void);

#define ActuatorSpektrumSet(_i, _v) { actuators_spektrum.cmds[_i] = _v; }
#define ActuatorsSpektrumInit() actuators_spektrum_init()
#define ActuatorsSpektrumCommit() actuators_spektrum_set()


#endif /* ACTUATORS_SPEKTRUM_H */

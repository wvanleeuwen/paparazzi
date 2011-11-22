/*
 * $Id: onboardcam_module.h 3079 2009-03-11 16:55:42Z gautier $
 *
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

/** \file onboardcam_module.h
 *
 * onboardcam module with blinking LEDs
 */

#ifndef onboardcam_MODULE_H
#define onboardcam_MODULE_H

#include <inttypes.h>
#include "subsystems/sensors/infrared.h"

// ADC Result
extern uint8_t onboardcam_mode;
extern uint8_t blackfin_mode;

// On Board Settings
extern uint8_t onboardcam_status;

//void downlink_onboardcam(uint8_t channel);
#define PERIODIC_SEND_IR_SENSORS_CAM(_chan) DOWNLINK_SEND_IR_SENSORS(_chan, &infrared.value.ir1, &infrared.value.ir2, &infrared.pitch, &infrared.roll, &infrared.top);

// Module Functions
void init_onboardcam(void);
void periodic_onboardcam(void);
void start_onboardcam(void);
void stop_onboardcam(void);

#endif

/*
 * Copyright (C) C. De Wagter
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/boards/opa_controller_ap.c"
 * @author C. De Wagter
 * Controlelr for OPA-AP board functionalities
 */

#include "modules/boards/opa_controller_ap.h"
#include "generated/airframe.h"
#include "mcu_periph/gpio.h"

bool opa_controller_ap_vision_power = false;
bool opa_controller_ap_ftd_disarm = false;

extern void opa_controller_ap_do_disarm(bool action) {
  opa_controller_ap_ftd_disarm = action;
}

void opa_controller_ap_init() {

  /* Enable Vision Power Control: Default Power Off */
  opa_controller_ap_vision_power = false;
  opa_controller_ap_ftd_disarm = false;

  gpio_setup_output(VISION_PWR, VISION_PWR_PIN);
  VISION_PWR_OFF(VISION_PWR, VISION_PWR_PIN);

}

void opa_controller_ap_periodic() {
  if (opa_controller_ap_vision_power) {
    VISION_PWR_ON(VISION_PWR, VISION_PWR_PIN);
  } else {
    VISION_PWR_OFF(VISION_PWR, VISION_PWR_PIN);
  }
}


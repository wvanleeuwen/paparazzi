/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/boards/opa_controller.c"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Controller for OPA board functionalities
 */

#include "modules/boards/opa_controller.h"
#include "generated/airframe.h"
#include "mcu_periph/gpio.h"

void opa_controller_init(void) {
  gpio_setup_input(BTN_ESTOP, BTN_ESTOP_PIN);
}

void opa_controller_periodic(void) {
  /* Check E-Stop */
  if(!gpio_get(BTN_ESTOP, BTN_ESTOP_PIN)) {
    MCU_PWR_OFF(MCU_PWR, MCU_PWR_PIN);
  }
}

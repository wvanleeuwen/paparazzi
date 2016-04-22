/*
 * Copyright (C) Kevin van Hecke
 *               2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.c"
 * @author Kevin van Hecke
 * Puts Spektrum in binding mode through software
 */

#include "modules/spektrum_soft_bind/spektrum_soft_bind_fbw.h"
#include "subsystems/intermcu/intermcu_fbw.h"
#include "mcu.h"
#include "subsystems/radio_control.h"
#include "mcu_periph/sys_time_arch.h"

#include "mcu_periph/gpio.h"

void spektrum_soft_bind_run(uint8_t val __attribute__((unused)))
{

#if USE_INTERMCU
  intermcu_send_spektrum_bind();
#endif

#if USE_RADIO_CONTROL
  spektrum_soft_bind();
#endif
}

/** The real binding */
static inline void spektrum_soft_bind(void)
{
  // Power cycle the radio's
  RADIO_CONTROL_POWER_OFF(RADIO_CONTROL_POWER, RADIO_CONTROL_POWER_PIN);
  sys_time_usleep(100000);
  RADIO_CONTROL_POWER_ON(RADIO_CONTROL_POWER, RADIO_CONTROL_POWER_PIN);

  // Try to bind and restart uart
  radio_control_spektrum_try_bind();
  radio_control_spektrum_uart_init();
}

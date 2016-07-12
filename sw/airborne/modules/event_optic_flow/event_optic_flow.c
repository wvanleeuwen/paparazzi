/*
 * Copyright (C) Bas Pijnacker Hordijk
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
 * @file "modules/event_optic_flow/event_optic_flow.c"
 * @author Bas Pijnacker Hordijk
 * Event based opticflow using DVS camera
 */

#include "event_optic_flow.h"
#include "mcu_periph/uart.h"
#include "subsystems/datalink/downlink.h"

#ifndef DVS_PORT
#error Please define uart port connected to the dvs event based camera. e.g <define name="DVS_PORT" value="uart0"/>
#endif

void event_optic_flow_init(void) {

}

void event_optic_flow_periodic(void) {
  /* read all data from uart */
  static uint8_t buffer[UART_RX_BUFFER_SIZE]; // local communication buffer
  static uint16_t buf_loc = 0;                // circular buffer index location

  while(uart_char_available(&DVS_PORT))
  {
    buffer[buf_loc] = uart_getch(&DVS_PORT);          // copy over incoming data
    buf_loc = (buf_loc + 1) % UART_RX_BUFFER_SIZE;    // implement circular buffer
  }
  // send to ground station for debugging
  DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, 32, &(buffer[(buf_loc-32)%UART_RX_BUFFER_SIZE]));
}

/*
 * Copyright (C) 2013  Christophe De Wagter
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

#include "mcu_periph/uart.h"

#include "fireswarm_communication.h"


#define __FireSwarmPayloadLink(dev, _x) dev##_x
#define _FireSwarmPayloadLink(dev, _x)  __FireSwarmPayloadLink(dev, _x)
#define FireSwarmPayloadLink(_x) _FireSwarmPayloadLink(FIRESWARM_LINK, _x)

#define FireSwarmPayloadBuffer() FireSwarmPayloadLink(ChAvailable())

/*
#define FireSwarmPayloadEvent(_msg_available_callback) { \
    if (GpsBuffer()) {                             \
      ReadGpsBuffer();                             \
    }                                              \
    if (gps_ubx.msg_available) {                   \
      gps_ubx_read_message();                      \
      gps_ubx_ucenter_event();                     \
      if (gps_ubx.msg_class == UBX_NAV_ID &&       \
          (gps_ubx.msg_id == UBX_NAV_VELNED_ID ||  \
           (gps_ubx.msg_id == UBX_NAV_SOL_ID &&    \
            gps_ubx.have_velned == 0))) {          \
        if (gps.fix == GPS_FIX_3D) {               \
          gps.last_fix_ticks = sys_time.nb_sec_rem; \
          gps.last_fix_time = sys_time.nb_sec;      \
        }                                          \
        _sol_available_callback();                 \
      }                                            \
      gps_ubx.msg_available = FALSE;               \
    }                                              \
  }
*/





void fireswarm_payload_link_init(void)
{
  UART3SetBaudrate(B57600);
}


uint8_t fsw_crc = 0;

void fireswarm_payload_link_start(void)
{
  fsw_crc = 0;
}

void fireswarm_payload_link_transmit(uint8_t* buff, int size)
{
  while (size-- > 0)
  {
    fsw_crc += *buff;
    UART3Transmit(*buff++);
  }
}

void fireswarm_payload_link_crc(void)
{
  UART3Transmit(fsw_crc);
}


int fireswarm_payload_link_has_data(void)
{
  return FireSwarmPayloadLink(ChAvailable());
}

char fireswarm_payload_link_get(void)
{
  return FireSwarmPayloadLink(Getch());
}

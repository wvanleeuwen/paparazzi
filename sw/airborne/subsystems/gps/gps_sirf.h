/*
 * Copyright (C) 2012 Freek van Tienen
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

#ifndef GPS_SIRF_H
#define GPS_SIRF_H

#include "std.h"

#define GPS_NB_CHANNELS 16
#define SIRF_MAXLEN 255

//Read states
#define UNINIT	0
#define GOT_A0	1
#define GOT_A2	2
#define GOT_B0	3

struct GpsSirf {
  bool_t msg_available;
  bool_t pos_available;
  char msg_buf[SIRF_MAXLEN];  ///< buffer for storing one nmea-line
  int msg_len;
  int read_state;
};

extern void gps_impl_init(void);

/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#define GpsEvent(_sol_available_callback) {        \
    if (GpsBuffer()) {                             \
      ReadGpsBuffer();                             \
    }                                              \
    if (gps_sirf.msg_available) {                  \
      sirf_parse_msg();				   \
      if (gps_sirf.pos_available) {		   \
        if (gps.fix == GPS_FIX_3D) {               \
          gps.last_fix_ticks = cpu_time_ticks;     \
          gps.last_fix_time = sys_time.nb_sec;        \
        }                                          \
        _sol_available_callback();                 \
      }                                            \
      gps_sirf.msg_available = FALSE;               \
    }                                              \
  }

#define ReadGpsBuffer() {					\
    while (GpsLink(ChAvailable())&&!gps_sirf.msg_available)	\
      sirf_parse_char(GpsLink(Getch()));			\
  }

extern void sirf_parse_char(uint8_t c);
extern void sirf_parse_msg(void);

#endif /* GPS_SIRF_H */

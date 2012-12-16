/*
 *
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


#include "subsystems/gps.h"

#include "led.h"

#if GPS_USE_LATLONG
/* currently needed to get nav_utm_zone0 */
#include "subsystems/navigation/common_nav.h"
#endif
#include "math/pprz_geodetic_float.h"

#include <inttypes.h>
#include "gps_sirf.h"

struct GpsSirf gps_sirf;

void gps_impl_init( void ) {
  gps_sirf.msg_available = FALSE;
  gps_sirf.pos_available = FALSE;
  gps_sirf.msg_len = 0;
  gps_sirf.read_state = 0;
}

void sirf_parse_char(uint8_t c) {
	switch(gps_sirf.read_state) {
	case UNINIT:
		if(c == 0xA0) {
			gps_sirf.msg_len = 0;
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.read_state = GOT_A0;
		}
		break;
	case GOT_A0:
		if(c == 0xA2) {
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.read_state = GOT_A2;
		}
		else
			goto restart;
		break;
	case GOT_A2:
		gps_sirf.msg_buf[gps_sirf.msg_len] = c;
		gps_sirf.msg_len++;
		if(c == 0xB0)
			gps_sirf.read_state = GOT_B0;
		break;
	case GOT_B0:
		if(c == 0xB3) {
			gps_sirf.msg_buf[gps_sirf.msg_len] = c;
			gps_sirf.msg_len++;
			gps_sirf.msg_available = TRUE;
		}
		else
			goto restart;
		break;
	}
	return;

	restart:
		gps_sirf.read_state = UNINIT;
}

void sirf_parse_41(void) {

}

void sirf_parse_2() {

}

void sirf_parse_msg(void) {
	if(gps_sirf.msg_len < 8)
		return;

	uint8_t message_id = gps_sirf.msg_buf[4];
	switch(message_id) {
	case 0x28:
		sirf_parse_41();
		break;
	case 0x02:
		sirf_parse_2();
		break;
	}
}

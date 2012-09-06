/*
 * $Id: link_mcu.c,v 1.18 2006/07/19 06:54:38 hecto Exp $
 *  
 * Copyright (C) 2003-2006  Pascal Brisset, Antoine Drouin
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

#include "link_mcu_usart.h"
#include "uart.h"
#include "ubx_protocol.h"
#include "gps_ubx.h"
#include "gps.h"
#include "commands.h"

void link_mcu_init( void )
{
  uart0_init();
}



struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;

inline void parse_mavpilot_msg( void );

#define UbxInitCheksum() { send_ck_a = send_ck_b = 0; }
#define UpdateChecksum(c) { send_ck_a += c; send_ck_b += send_ck_a; }
#define UbxTrailer() { GpsUartSend1(send_ck_a);  GpsUartSend1(send_ck_b); }

#define UbxSend1(c) { uint8_t i8=c; GpsUartSend1(i8); UpdateChecksum(i8); }
#define UbxSend2(c) { uint16_t i16=c; UbxSend1(i16&0xff); UbxSend1(i16 >> 8); }
#define UbxSend1ByAddr(x) { UbxSend1(*x); }
#define UbxSend2ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); }
#define UbxSend4ByAddr(x) { UbxSend1(*x); UbxSend1(*(x+1)); UbxSend1(*(x+2)); UbxSend1(*(x+3)); }

#define UbxHeader(nav_id, msg_id, len) { \
  GpsUartSend1(UBX_SYNC1); \
  GpsUartSend1(UBX_SYNC2); \
  UbxInitCheksum(); \
  UbxSend1(nav_id); \
  UbxSend1(msg_id); \
  UbxSend2(len); \
}


//static uint8_t  ubx_status;
//static uint16_t ubx_len;
//static uint8_t  ubx_msg_idx;
//static uint8_t ck_a, ck_b;
uint8_t send_ck_a, send_ck_b;

void parse_mavpilot_msg( void ) 
{
//  uint8_t s1;

  if (ubx_class == UBX_MAVPILOT_ID)
  {
    if (ubx_id == UBX_MAVPILOT_COMMAND_ID)
    {
      ap_state->commands[0] = ((pprz_t)UBX_MAVPILOT_COMMAND_S1(ubx_msg_buf));
      ap_state->commands[1] = ((pprz_t)UBX_MAVPILOT_COMMAND_S2(ubx_msg_buf));
      ap_state->commands[2] = ((pprz_t)UBX_MAVPILOT_COMMAND_S3(ubx_msg_buf));
      ap_state->commands[3] = ((pprz_t)UBX_MAVPILOT_COMMAND_S4(ubx_msg_buf));
      ap_state->commands[4] = ((pprz_t)UBX_MAVPILOT_COMMAND_S5(ubx_msg_buf));

      inter_mcu_received_ap = TRUE;
      //inter_mcu_received_fbw = TRUE;
    }
  }
}


static uint8_t SixtyHzCounter = 0;

// 60 Hz
void link_mcu_periodic_task( void )
{
  SixtyHzCounter++;
  if (SixtyHzCounter >= 15)
  {
    // 4 Hz
    SixtyHzCounter = 0;
    inter_mcu_fill_fbw_state(); /** Prepares the next message for AP */

    UbxSend_MAVPILOT_FBW(
/*	fbw_state->channels[0],
	fbw_state->channels[1],
	fbw_state->channels[2],
	fbw_state->channels[3],
	fbw_state->channels[4],
*/	fbw_state->ppm_cpt,
	fbw_state->status,
	fbw_state->nb_err,
	fbw_state->vsupply,
	fbw_state->current);
/*
};

struct ap_state {
  pprz_t commands[COMMANDS_NB];  
};

	);
*/
  }
}

void link_mcu_event_task( void ) {
  /* A message has been received */
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
  if (gps_msg_received) {
    parse_mavpilot_msg();
    gps_msg_received = FALSE;
  }
}
   

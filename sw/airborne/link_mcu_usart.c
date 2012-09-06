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

#define USE_UART0

#include "link_mcu_usart.h"
#include "mcu_periph/uart.h"

#include "commands.h"

//////////////////////////////////////////////////////////////////////////////////////////////
// LINK

#define INTERMCU_LINK Uart0

#define __InterMcuLink(dev, _x) dev##_x
#define _InterMcuLink(dev, _x)  __InterMcuLink(dev, _x)
#define InterMcuLink(_x) _InterMcuLink(INTERMCU_LINK, _x)

#define InterMcuBuffer() InterMcuLink(ChAvailable())

#define InterMcuUartSend1(c) InterMcuLink(Transmit(c))
#define InterMcuUartSetBaudrate(_a) InterMcuLink(SetBaudrate(_a))
#define InterMcuUartRunning InterMcuLink(TxRunning)
#define InterMcuUartSendMessage InterMcuLink(SendMessage)

//////////////////////////////////////////////////////////////////////////////////////////////
// PROTOCOL

#define INTERMCU_SYNC1 0xB5
#define INTERMCU_SYNC2 0x62

#define InterMcuInitCheksum() { intermcu_data.send_ck_a = intermcu_data.send_ck_b = 0; }
#define UpdateChecksum(c) { intermcu_data.send_ck_a += c; intermcu_data.send_ck_b += intermcu_data.send_ck_a; }
#define InterMcuTrailer() { InterMcuUartSend1(intermcu_data.send_ck_a);  InterMcuUartSend1(intermcu_data.send_ck_b); InterMcuUartSendMessage(); }

#define InterMcuSend1(c) { uint8_t i8=c; InterMcuUartSend1(i8); UpdateChecksum(i8); }
#define InterMcuSend2(c) { uint16_t i16=c; InterMcuSend1(i16&0xff); InterMcuSend1(i16 >> 8); }
#define InterMcuSend1ByAddr(x) { InterMcuSend1(*x); }
#define InterMcuSend2ByAddr(x) { InterMcuSend1(*x); InterMcuSend1(*(x+1)); }
#define InterMcuSend4ByAddr(x) { InterMcuSend1(*x); InterMcuSend1(*(x+1)); InterMcuSend1(*(x+2)); InterMcuSend1(*(x+3)); }

#define InterMcuHeader(nav_id, msg_id, len) {        \
    InterMcuUartSend1(INTERMCU_SYNC1);                    \
    InterMcuUartSend1(INTERMCU_SYNC2);                    \
    InterMcuInitCheksum();                           \
    InterMcuSend1(nav_id);                           \
    InterMcuSend1(msg_id);                           \
    InterMcuSend2(len);                              \
  }

//////////////////////////////////////////////////////////////////////////////////////////////
// MESSAGES

// class
#define MSG_INTERMCU_ID 100

#define MSG_INTERMCU_COMMAND_ID 0x05
#define MSG_INTERMCU_COMMAND_S1(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+0)|*((uint8_t*)_intermcu_payload+1+0)<<8)
#define MSG_INTERMCU_COMMAND_S2(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+2)|*((uint8_t*)_intermcu_payload+1+2)<<8)
#define MSG_INTERMCU_COMMAND_S3(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+4)|*((uint8_t*)_intermcu_payload+1+4)<<8)
#define MSG_INTERMCU_COMMAND_S4(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+6)|*((uint8_t*)_intermcu_payload+1+6)<<8)
#define MSG_INTERMCU_COMMAND_S5(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+8)|*((uint8_t*)_intermcu_payload+1+8)<<8)
#define MSG_INTERMCU_COMMAND_S6(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+10)|*((uint8_t*)_intermcu_payload+1+10)<<8)

#define InterMcuSend_INTERMCU_COMMAND(s1,s2,s3,s4,s5,s6) { \
  InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_COMMAND_ID, 12);\
  uint16_t _s1 = s1; InterMcuSend2ByAddr((uint8_t*)&_s1);\
  uint16_t _s2 = s2; InterMcuSend2ByAddr((uint8_t*)&_s2);\
  uint16_t _s3 = s3; InterMcuSend2ByAddr((uint8_t*)&_s3);\
  uint16_t _s4 = s4; InterMcuSend2ByAddr((uint8_t*)&_s4);\
  uint16_t _s5 = s5; InterMcuSend2ByAddr((uint8_t*)&_s5);\
  uint16_t _s6 = s6; InterMcuSend2ByAddr((uint8_t*)&_s6);\
  InterMcuTrailer();\
}

#define MSG_INTERMCU_FBW_ID 0x06
#define MSG_INTERMCU_FBW_MOD(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+0))
#define MSG_INTERMCU_FBW_STAT(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+1))
#define MSG_INTERMCU_FBW_ERR(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+2))
#define MSG_INTERMCU_FBW_VOLT(_intermcu_payload) (uint8_t)(*((uint8_t*)_intermcu_payload+3))
#define MSG_INTERMCU_FBW_CURRENT(_intermcu_payload) (uint16_t)(*((uint8_t*)_intermcu_payload+4)|*((uint8_t*)_intermcu_payload+1+4)<<8)

#define InterMcuSend_INTERMCU_FBW(mod,stat,err,volt,current) { \
  InterMcuHeader(MSG_INTERMCU_ID, MSG_INTERMCU_FBW_ID, 6);\
  uint8_t _mod = mod; InterMcuSend1ByAddr((uint8_t*)&_mod);\
  uint8_t _stat = stat; InterMcuSend1ByAddr((uint8_t*)&_stat);\
  uint8_t _err = err; InterMcuSend1ByAddr((uint8_t*)&_err);\
  uint8_t _volt = volt; InterMcuSend1ByAddr((uint8_t*)&_volt);\
  uint16_t _current = current; InterMcuSend2ByAddr((uint8_t*)&_current);\
  InterMcuTrailer();\
}

//////////////////////////////////////////////////////////////////////////////////////////////
// PARSER

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_CLASS     3
#define GOT_ID        4
#define GOT_LEN1      5
#define GOT_LEN2      6
#define GOT_PAYLOAD   7
#define GOT_CHECKSUM1 8


#define INTERMCU_MAX_PAYLOAD 255
struct InterMcuData {
  bool_t msg_available;
  uint8_t msg_buf[INTERMCU_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t send_ck_a, send_ck_b;
  uint8_t error_cnt;
};

struct InterMcuData intermcu_data;

/* INTERMCU parsing */
void intermcu_parse( uint8_t c );
void intermcu_parse( uint8_t c ) {
  if (intermcu_data.status < GOT_PAYLOAD) {
    intermcu_data.ck_a += c;
    intermcu_data.ck_b += intermcu_data.ck_a;
  }
  switch (intermcu_data.status) {
  case UNINIT:
    if (c == INTERMCU_SYNC1)
      intermcu_data.status++;
    break;
  case GOT_SYNC1:
    if (c != INTERMCU_SYNC2) {
      goto error;
    }
    intermcu_data.ck_a = 0;
    intermcu_data.ck_b = 0;
    intermcu_data.status++;
    break;
  case GOT_SYNC2:
    if (intermcu_data.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      goto error;
    }
    intermcu_data.msg_class = c;
    intermcu_data.status++;
    break;
  case GOT_CLASS:
    intermcu_data.msg_id = c;
    intermcu_data.status++;
    break;
  case GOT_ID:
    intermcu_data.len = c;
    intermcu_data.status++;
    break;
  case GOT_LEN1:
    intermcu_data.len |= (c<<8);
    if (intermcu_data.len > INTERMCU_MAX_PAYLOAD) {
      goto error;
    }
    intermcu_data.msg_idx = 0;
    intermcu_data.status++;
    break;
  case GOT_LEN2:
    intermcu_data.msg_buf[intermcu_data.msg_idx] = c;
    intermcu_data.msg_idx++;
    if (intermcu_data.msg_idx >= intermcu_data.len) {
      intermcu_data.status++;
    }
    break;
  case GOT_PAYLOAD:
    if (c != intermcu_data.ck_a) {
      goto error;
    }
    intermcu_data.status++;
    break;
  case GOT_CHECKSUM1:
    if (c != intermcu_data.ck_b) {
      goto error;
    }
    intermcu_data.msg_available = TRUE;
    goto restart;
    break;
  default:
    goto error;
  }
  return;
 error:
  intermcu_data.error_cnt++;
 restart:
  intermcu_data.status = UNINIT;
  return;
}



//////////////////////////////////////////////////////////////////////////////////////////////
// USER


struct link_mcu_msg link_mcu_from_ap_msg;
struct link_mcu_msg link_mcu_from_fbw_msg;

inline void parse_mavpilot_msg( void );

void link_mcu_init( void )
{
   intermcu_data.status = UNINIT;
   intermcu_data.msg_available = FALSE;
   intermcu_data.error_cnt = 0;
}

void parse_mavpilot_msg( void ) 
{
  if (intermcu_data.msg_class == MSG_INTERMCU_ID)
  {
    if (intermcu_data.msg_id == MSG_INTERMCU_COMMAND_ID)
    {
#if COMMANDS_NB > 6
#warning "INTERMCU UART CAN ONLY SEND 6 COMMANDS"
#endif

#if COMMANDS_NB > 0
      ap_state->commands[0] = ((pprz_t)MSG_INTERMCU_COMMAND_S1(intermcu_data.msg_buf));
#endif
#if COMMANDS_NB > 1
      ap_state->commands[1] = ((pprz_t)MSG_INTERMCU_COMMAND_S2(intermcu_data.msg_buf));
#endif
#if COMMANDS_NB > 2
      ap_state->commands[2] = ((pprz_t)MSG_INTERMCU_COMMAND_S3(intermcu_data.msg_buf));
#endif
#if COMMANDS_NB > 3
      ap_state->commands[3] = ((pprz_t)MSG_INTERMCU_COMMAND_S4(intermcu_data.msg_buf));
#endif
#if COMMANDS_NB > 4
      ap_state->commands[4] = ((pprz_t)MSG_INTERMCU_COMMAND_S5(intermcu_data.msg_buf));
#endif
#if COMMANDS_NB > 5
      ap_state->commands[5] = ((pprz_t)MSG_INTERMCU_COMMAND_S6(intermcu_data.msg_buf));
#endif

      inter_mcu_received_ap = TRUE;
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

    InterMcuSend_INTERMCU_FBW(
	fbw_state->ppm_cpt,
	fbw_state->status,
	fbw_state->nb_err,
	fbw_state->vsupply,
	fbw_state->current);
  }
}

void link_mcu_event_task( void ) {
  /* A message has been received */
  if (InterMcuBuffer()) {
    while (InterMcuLink(ChAvailable())&&!intermcu_data.msg_available)
      intermcu_parse(InterMcuLink(Getch()));
  }

  if (intermcu_data.msg_available) {
    parse_mavpilot_msg();
    intermcu_data.msg_available = FALSE;
  }
}
   

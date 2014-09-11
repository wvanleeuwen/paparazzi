/*
 * Copyright (C) 2014 OpenUAS
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

/**
 * @file protocol.h
 *
 * |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
 *
 * where checksum is computed over length and payload:
 * @code
 * ck_A = ck_B = length
 * for each byte b in payload
 *     ck_A += b;
 *     ck_b += ck_A;
 * @endcode
 */

#ifndef MORA_TRANSPORT_H
#define MORA_TRANSPORT_H

#include <inttypes.h>
#include "std.h"


/////////////////////////////////////////////////////////////////////
// MESSAGES

#define MORA_SHOOT          1

// nr, lat, lon, h, phi, theta, psi

#define MORA_BUFFER_EMPTY   2

// null

#define MORA_PAYLOAD        3

// 70 bytes

#define MORA_STATUS         4

// int cpu
// int threads
// int shots
// int extra


/////////////////////////////////////////////////////////////////////
// SENDING

extern uint8_t ck_a, ck_b;

#define STX  0x99

#define MoraSizeOf(_dev, _payload) (_payload+4)
#define MoraHeader(_dev, payload_len) {             \
  CameraLink(Transmit( STX));                       \
  uint8_t msg_len = MoraSizeOf(_dev, payload_len);  \
  CameraLink(Transmit(_dev, msg_len));              \
  ck_a = msg_len; ck_b = msg_len;                   \
}

#define MoraTrailer(_dev) {        \
  CameraLink(Transmit(ck_a));      \
  CameraLink(Transmit(ck_b));      \
}

#define MoraPutUint8( _byte) {     \
  ck_a += _byte;                   \
  ck_b += ck_a;                    \
  CameraLink(Transmit(_byte));     \
}

#define MoraPut1ByteByAddr(_dev, _byte) {  \
  uint8_t _x = *(_byte);                   \
  MoraPutUint8(_dev, _x);                  \
}

/////////////////////////////////////////////////////////////////////
// PARSING

struct mora_transport {
  // generic interface
  uint8_t payload[256];
  uint8_t error;
  uint8_t msg_received;
  uint8_t payload_len;
  // specific pprz transport variables
  uint8_t status;
  uint8_t payload_idx;
  uint8_t ck_a, ck_b;
};

extern struct mora_transport mora_protocol;

void parse_pprz(struct mora_transport * t, uint8_t c );

/*
static inline void pprz_parse_payload(struct pprz_transport * t) {
  uint8_t i;
  for(i = 0; i < t->trans.payload_len; i++)
    dl_buffer[i] = t->trans.payload[i];
  dl_msg_available = TRUE;
}


#define PprzBuffer(_dev) TransportLink(_dev,ChAvailable())
#define ReadPprzBuffer(_dev,_trans) { while (TransportLink(_dev,ChAvailable())&&!(_trans.trans.msg_received)) parse_pprz(&(_trans),TransportLink(_dev,Getch())); }
#define PprzCheckAndParse(_dev,_trans) {  \
  if (PprzBuffer(_dev)) {                 \
    ReadPprzBuffer(_dev,_trans);          \
    if (_trans.trans.msg_received) {      \
      pprz_parse_payload(&(_trans));      \
      _trans.trans.msg_received = FALSE;  \
    }                                     \
  }                                       \
}
*/

#endif


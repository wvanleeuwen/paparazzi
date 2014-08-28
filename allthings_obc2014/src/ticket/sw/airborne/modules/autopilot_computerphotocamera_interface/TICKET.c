/*
 * Copyright (C) OpenUAS
 *
 * This file is part of paparazzi

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

#include "modules/autopilot_computerphotocamera_interface/TICKET.h"

void ticket_init {}
void ticket_takephotonow {

  //Send command over serialbux
}

/*
 * Copyright (C) 2014 The Paparazzi Team
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


/** \file ticket.c
 *  \brief Interface with computer based photo camera though UART, I2C, CAN or SPI bus
 *
 *   Send Commands over bus
 */


#include "ticket.h"
#include "mcu_periph/uart.h"
//#include "mcu_periph/i2c.h"
//#include "mcu_periph/can.h"
//#include "mcu_periph/spi.h"

#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"
#include "subsystems/nav.h"
#include "generated/flight_plan.h"

// In I2C mode we can not inline this function:
void dc_send_command(uint8_t cmd)
{
  ticket_send(cmd);
}

bool_t ticket_just_sent_command = false;

void ticket_init(void)
{
  ticket_trans.status = UARTTransDone;
  dc_init();
}

void ticket_periodic (void)
{
  ticket_just_sent_command = False;
  dc_periodic_4Hz();

  // Request Status
  if (!ticket_just_sent_command)
  {
    ticket_send(DC_GET_STATUS);
  }
}



void ticket_send(uint8_t cmd)
{
  ticket_just_sent_command = True;

  // Send Command
  ticket_trans.buf[0] = cmd;
  uart_transceive(blabla);

  if (cmd == DC_SHOOT)
  {
    dc_send_shot_position();
  }
}

void ticket_event( void )
{
  if (ticket_trans.status == UARTTransSuccess)
  {
    unsigned char cam_ret[1];
    cam_ret[0] = ticket_trans.buf[0];
    RunOnceEvery(6,DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, 1, cam_ret ));
    ticket_trans.status = I2CTransDone;
  }
}

/**
 *

{
    int     year;       /**< Years since 1900
    int     mon;        *< Months since January - [0,11]
    int     day;        *< Day of the month - [1,31]
    int     hour;       *< Hours since midnight - [0,23]
    int     min;        *< Minutes after the hour - [0,59]
    int     sec;        *< Seconds after the minute - [0,59]
    int     hsec;       *< Hundredth part of second - [0,99]

*
 * nmeaINFO_of_UAV_from_UA DATA

typedef struct _nmeaINFO_of_UAV //Where data is stored to be saved via NMA out
{

  unsigned char ac_id; *< the Aircraft ID of the data
    int     smask;       *< Mask specifying types of packages to use
    nmeaDATETIME utc;        *< UTC of position
    int     sig;         *< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
    int     fix;         *< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
    double  PDOP;        *< Position Dilution Of Precision
    double  HDOP;        *< Horizontal Dilution Of Precision
    double  VDOP;        *< Vertical Dilution Of Precision
    double  lat;         *< Latitude in NDEG - +/-[degree][min].[sec/60]
    double  lon;         *< Longitude in NDEG - +/-[degree][min].[sec/60]
    double  elv;         *< Antenna altitude above/below mean sea level (geoid) in meters
    double  speed;       *< Speed over the ground in kilometers/hour
    double  direction;   *< Track angle in degrees True
    double  declination; *< Magnetic variation degrees (Easterly var. subtracts from true course)


PPRA DATa to send


#define SIDESLIP_F  5



Main Ticket

/// Test if attitudes are valid.
static inline bool_t stateIsAttitudeValid(void) {
  return (orienationCheckValid(&state.ned_to_body_orientation));
}
*/




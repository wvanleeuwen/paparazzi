/*
 * Copyright (C) OpenUAS
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
 */

/**
 * @file modules/digital_cam/ticket/TICKET.c
 * Control the camera via uart to chdk-ptp.
 * Retrieve thumbnails
 */

#include "TICKET.h"
#include "generated/airframe.h"

// Include Standard Camera Control Interface
#include "modules/digital_cam/dc.h"

// Telemetry
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "generated/periodic_telemetry.h"
#include BOARD_CONFIG

#ifdef SITL
#include "modules/digital_cam/candy/serial.h"
#endif

#define __CameraLink(dev, _x) dev##_x
#define _CameraLink(dev, _x)  __CameraLink(dev, _x)
#define CameraLink(_x) _CameraLink(CAMERA_LINK, _x)

#define CameraBuffer() CameraLink(ChAvailable())

void ticket_parse(char c);

#define ReadCameraBuffer() {                  \
    while (CameraLink(ChAvailable()))         \
      ticket_parse(CameraLink(Getch()));      \
  }

void ticket_event(void)
{
  ReadCameraBuffer();
}

char last_char = 0;

void ticket_parse(char c)
{
  last_char = c;
}

int ticket_thumbnails = 0;
#define THUMB_MSG_SIZE  (80-8-2)
static uint8_t thumb[THUMB_MSG_SIZE];

#if PERIODIC_TELEMETRY
static void send_thumbnails(void)
{
  static int cnt = 0;
  if (ticket_thumbnails > 0)
  {
    if (ticket_thumbnails == 1)
    {
      cnt++;
      if (cnt>1)
      {
        cnt = 0;
        return;
      }
    }
    for (int i=0;i<THUMB_MSG_SIZE;i++)
      thumb[i]=last_char;
    DOWNLINK_SEND_PAYLOAD(DefaultChannel, DefaultDevice, THUMB_MSG_SIZE, thumb);
  }
}
#endif

void ticket_init(void)
{
  // Call common DC init
  dc_init();
  ticket_thumbnails = 0;
  for (int i=0;i<THUMB_MSG_SIZE;i++)
    thumb[i] = 0;
#if PERIODIC_TELEMETRY
  register_periodic_telemetry(&telemetry_Ap, "PAYLOAD", send_thumbnails);
#endif

#ifdef SITL
  serial_init("/dev/ttyUSB0");
#endif
}

void ticket_periodic( void )
{
  // Common DC Periodic task
  dc_periodic_4Hz();
}


/* Command The Camera */
void dc_send_command(uint8_t cmd)
{
  switch (cmd)
  {
    case DC_SHOOT:
      CameraLink(Transmit(cmd));
      dc_send_shot_position();
      break;
    case DC_TALLER:
      CameraLink(Transmit(cmd));
      break;
    case DC_WIDER:
      CameraLink(Transmit(cmd));
      break;
    case DC_ON:
      CameraLink(Transmit(cmd));
      break;
    case DC_OFF:
      CameraLink(Transmit(cmd));
      break;
    default:
      break;
  }
}

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

void ticket_init(void)
{
  // Call common DC init
  dc_init();
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
      dc_send_shot_position();
      break;
    case DC_TALLER:
      break;
    case DC_WIDER:
      break;
    case DC_ON:
      break;
    case DC_OFF:
      break;
    default:
      break;
  }
}

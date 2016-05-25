/*
 * Copyright (C) 2016 Freek van Tienen <freek.v.tienen@gmail.com>
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

/** @file modules/telemetry/telemetry_intermcu_fbw.c
 *  @brief Telemetry through InterMCU
 */

#include "telemetry_intermcu.h"
#include "subsystems/intermcu.h"
#include "pprzlink/intermcu_msg.h"
#include "subsystems/datalink/telemetry.h"

/* Telemetry InterMCU throughput */
static struct telemetry_intermcu_t telemetry_intermcu;

/* InterMCU initialization */
void telemetry_intermcu_init(void)
{
  // Initialize transport structure
  short_transport_init(&telemetry_intermcu.trans);
}

/* InterMCU periodic handling of telemetry */
void telemetry_intermcu_periodic(void)
{

}

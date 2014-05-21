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

/** \file fireswarm_payload.h
 *
 * Interface with FireSwarm Payload Module
 */

#ifndef FIRESWARM_PAYLOAD_H
#define FIRESWARM_PAYLOAD_H

#include "std.h"
#include "paparazzi.h"

#include "AutoPilotProt.h"

extern AutoPilotMsgLanding    FireSwarmLanding;
extern AutoPilotMsgMode       FireSwarmMode;
extern AutoPilotMsgWayPoints  FireSwarmWaypoints;


void fireswarm_payload_init(void);
void fireswarm_periodic(void);
extern bool_t fireswarm_periodic_nav_init(void);
extern bool_t fireswarm_periodic_nav(void);
void fireswarm_event(void);

#endif

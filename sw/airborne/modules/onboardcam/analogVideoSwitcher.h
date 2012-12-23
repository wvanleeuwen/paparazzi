/*
 * $Id: $
 *
 * Copyright (C) 2012 Dirk Dokter
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

#ifndef ANALOG_VIDEO_SWITCH_H
#define ANALOG_VIDEO_SWITCH_H

#include "std.h"
#include "paparazzi.h"

#ifndef ANALOG_VIDEO_SWITCH_NUM_CAMS
#define ANALOG_VIDEO_SWITCH_NUM_CAMS 3
#endif

#define ANALOG_VIDEO_SWITCH_CMD_STEP ((MAX_PPRZ-(MIN_PPRZ)) / ANALOG_VIDEO_SWITCH_NUM_CAMS)

#ifndef ANALOG_VIDEO_SWITCH_RC_CHANNEL
#define ANALOG_VIDEO_SWITCH_RC_CHANNEL RADIO_EXTRA1
#endif

#ifndef ANALOG_VIDEO_SWITCH_RC_MINDIFF
#define ANALOG_VIDEO_SWITCH_RC_MINDIFF 200
#endif



extern void analogVideoSwitch_init(void);
extern void analogVideoSwitch_periodic(void);


#endif //ANALOG_VIDEO_SWITCH_H


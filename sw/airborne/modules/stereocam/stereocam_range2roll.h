/*
 * Copyright (C) C. DW
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/stereocam/stereocam_range2roll.h"
 * @author C. DW
 * Stereocam Range 2 Pitch/Roll angles
 */

#ifndef STEREOCAM_RANGE2ROLL_H
#define STEREOCAM_RANGE2ROLL_H


extern void range2roll_flightplan(void);

extern void range2roll_init(void);
extern void range2roll_periodic(void);

#endif


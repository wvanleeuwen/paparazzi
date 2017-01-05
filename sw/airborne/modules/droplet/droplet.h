/*
 * Copyright (C) Kirk Scheper
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
 * @file "modules/droplet/droplet.h"
 * @author Kirk Scheper
 * Droplet collision avoidance system
 */

#ifndef DROPLET_H
#define DROPLET_H

#include "stdint.h"

extern void droplet_init(void );
extern void droplet_periodic(void);
extern void run_droplet(uint32_t disparities_total, uint32_t disparities_high);
extern void run_droplet_low_texture(uint32_t disparities_high, uint32_t disparities_total, uint32_t histogram_obs,
    uint32_t count_disps_left, uint32_t count_disps_right);

extern uint16_t obst_thr_1;      // obstacle threshold for phase 1
extern uint16_t disp_thr_1;      // obstacle count minimum threshold for phase 1
extern float wall_following_trim;// yaw rate trim to force vehicle to follow wall

#endif

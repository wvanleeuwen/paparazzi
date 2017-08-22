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
 * @file "modules/imav2017/imav2017.h"
 * @author Kirk Scheper
 * 
 */

#ifndef IMAV2017_H
#define IMAV2017_H

#include <inttypes.h>

extern float gate_distance, gate_y_offset;

extern void imav2017_init(void);
extern void imav2017_set_gate(uint8_t quality, float w, float h,
    float psi, float theta, float depth);

#endif


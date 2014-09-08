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

/** @file modules/digital_cam/ticket/TICKET.h
 *  @brief Digital Camera Control Over UART
 *
 * Provides the control of the shutter and the zoom of a digital camera
 * through standard binary IOs of the board.
 *
 * The required initialization (dc_init()) and periodic (4Hz) process.
 */

#ifndef TICKET_H
#define TICKET_H

#include "modules/digital_cam/dc.h"

extern void ticket_init(void);

/** 4Hz Periodic */
extern void ticket_periodic(void);

extern int ticket_thumbnails;

#endif // GPIO_CAM_CTRL_H

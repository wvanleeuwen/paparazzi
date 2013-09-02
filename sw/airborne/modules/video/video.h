/*
 * Copyright (C) 2008-2010 The Paparazzi Team
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
 * @file subsystems/video.h
 * Attitude and Heading Reference System interface.
 */

#ifndef VIDEO_H
#define VIDEO_H

#include "std.h"

/** Attitude and Heading Reference System state */
struct Video {
  uint8_t status; ///< status of the VIDEO, VIDEO_UNINIT or VIDEO_RUNNING
};

/** global VIDEO state */
extern struct Video video;

/** VIDEO initialization. Called at startup.
 *  Needs to be implemented by each VIDEO algorithm.
 */
extern void video_init(void);
extern void video_start(void);
extern void video_stop(void);

/** Receive over tcp.
 *  Needs to be implemented by each VIDEO algorithm.
 */
extern void video_receive(void);


#endif /* VIDEO_H */

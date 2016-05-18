/*
 * Copyright (C) Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file boards/bebop/mt9f002.h
 *
 * Initialization and configuration of the MT9F002 CMOS Chip
 */

#ifndef MT9F002H
#define MT9F002H

/* Interface types for the MT9F002 connection */
enum mt9f002_interface {
  MT9F002_MIPI,     ///< MIPI type connection
  MT9F002_HiSPi,    ///< HiSPi type connection
  MT9F002_PARALLEL  ///< Parallel type connection
}

/* Main configuration structure */
struct mt9f002_t {
  enum mt9f002_interface interface;   ///< Interface used to connect

  uint16_t output_width;              ///< Output width
  uint16_t output_height;             ///< Output height
  float output_scale;                 ///< Output scale
  uint16_t scaled_width;              ///< Width after corrected scaling
  uint16_t scaled_height;             ///< Height after corrected scaling
  uint16_t offset_x;                  ///< Offset from left in pixels
  uint16_t offset_y;                  ///< Offset from top in pixels

  struct i2c_periph *i2c_periph;      ///< I2C peripheral used to communicate over
  struct i2c_transaction i2c_trans;   ///< I2C transaction for comminication with CMOS chip
};

void mt9f002_init(struct mt9f002_t *mt);

#endif /* MT9F002_H */

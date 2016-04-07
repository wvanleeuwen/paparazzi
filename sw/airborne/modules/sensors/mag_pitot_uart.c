/*
 * Copyright (C) C. De Wagter
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
 * @file "modules/sensors/mag_pitot_uart.c"
 * @author C. De Wagter
 * Remotely located magnetometer and pitot tube over uart (RS232) communication
 */

#include "modules/sensors/mag_pitot_uart.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"


struct Int32Vect3 mag;
bool_t mag_valid;


void mag_pitot_init() {
  mag.x = 0;
  mag.y = 0;
  mag.z = 0;
  mag_valid = FALSE;
}

void mag_pitot_event() {
  if (mag_valid) {
    uint32_t now_ts = get_sys_time_usec();

    VECT3_COPY(imu.mag_unscaled, mag);
    mag_valid = FALSE;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);

    // BARO_BOARD_SENDER_ID
  }
}



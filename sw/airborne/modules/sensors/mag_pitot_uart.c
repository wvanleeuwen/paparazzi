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

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"

#include "subsystems/abi.h"
#include "subsystems/imu.h"


// Variables
static struct link_device *mag_pitot_device = (&((MAG_PITOT_PORT).device));
static struct pprz_transport mag_pitot_transport;
static uint8_t mag_pitot_buf[128]  __attribute__((aligned));
static bool_t mag_pitot_msg_available = FALSE;




// Downlink
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#include BOARD_CONFIG

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void mag_pitot_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_IMU_MAG_RAW(trans, dev, AC_ID, &imu.mag_unscaled.x, &imu.mag_unscaled.y,
                             &imu.mag_unscaled.z);
}
#endif


void mag_pitot_init() {
  pprz_transport_init(&mag_pitot_transport);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG_RAW, mag_pitot_raw_downlink);
#endif

}


static inline void mag_pitot_parse_msg(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* Parse the mag-pitot message */
  uint8_t msg_id = mag_pitot_buf[1];
  switch (msg_id) {
  case DL_IMCU_REMOTE_MAG: {
    struct Int16Vect3 mag;

    mag.x = DL_IMCU_REMOTE_MAG_mag_x(mag_pitot_buf);
    mag.y = DL_IMCU_REMOTE_MAG_mag_y(mag_pitot_buf);
    mag.z = DL_IMCU_REMOTE_MAG_mag_z(mag_pitot_buf);

    // TODO: Rotate from Mag Axis to IMU Axis
    VECT3_COPY(imu.mag_unscaled, mag);

    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);

    break;
  }

  case DL_IMCU_REMOTE_BARO: {
    float abs = 1.0f * ((float)(DL_IMCU_REMOTE_BARO_pitot_tot(mag_pitot_buf)));
    float diff = 1.0f * DL_IMCU_REMOTE_BARO_pitot_stat(mag_pitot_buf);
    float temp = 1.0f * DL_IMCU_REMOTE_BARO_pitot_temp(mag_pitot_buf);

    AbiSendMsgBARO_ABS(BARO_BOARD_SENDER_ID, abs);
    AbiSendMsgBARO_DIFF(BARO_BOARD_SENDER_ID, diff);
    AbiSendMsgTEMPERATURE(BARO_BOARD_SENDER_ID, temp);
    break;
  }

    default:
      break;
  }


}



void mag_pitot_event() {

  pprz_check_and_parse(mag_pitot_device, &mag_pitot_transport, mag_pitot_buf, &mag_pitot_msg_available );

  if (mag_pitot_msg_available) {
    mag_pitot_parse_msg();
    mag_pitot_msg_available = FALSE;
  }

}



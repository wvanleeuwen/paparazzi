/*
 * Copyright (C) C. De Wagter
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/sensors/kalamos_uart.c"
 * @author C. De Wagter
 * Parrot Kalamos Nvidia tk1 stereo vision uart (RS232) communication
 */

#include "modules/sensors/kalamos_uart.h"

#include "pprzlink/pprz_transport.h"
#include "pprzlink/intermcu_msg.h"
#include "mcu_periph/uart.h"
#include "subsystems/abi.h"
#include "subsystems/imu.h"
#include "state.h"

 /* Main magneto structure */
static struct kalamos_t kalamos = {
  .device = (&((KALAMOS_PORT).device)),
  .msg_available = false
};
static uint8_t mp_msg_buf[128]  __attribute__((aligned));   ///< The message buffer for the Kalamos

struct  Kalamos2PPRZPackage k2p_package;

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

static void kalamos_raw_downlink(struct transport_tx *trans, struct link_device *dev)
{
  k2p_package.endl = k2p_package.height; //tmp?!?!?
  pprz_msg_send_DEBUG(trans, dev, AC_ID, sizeof(struct Kalamos2PPRZPackage), (unsigned char *) &k2p_package);
}
#endif

/* Initialize the Kalamos */
void kalamos_init() {
  // Initialize transport protocol
  pprz_transport_init(&kalamos.transport);


#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_DEBUG, kalamos_raw_downlink);
#endif
}

/* Parse the InterMCU message */
static inline void kalamos_parse_msg(void)
{

  /* Parse the kalamos message */
  uint8_t msg_id = mp_msg_buf[1];
  k2p_package.hoer[0] = 'h';

  switch (msg_id) {

  /* Got a magneto message */
  case DL_IMCU_DEBUG: {
    uint8_t size = DL_IMCU_DEBUG_msg_length(mp_msg_buf);
    uint8_t *msg = DL_IMCU_DEBUG_msg(mp_msg_buf);

    for(uint8_t i = 0; i < size; i++)
      k2p_package.hoer[i] = msg[i];

      // Send ABI message

    if (k2p_package.height > 1.0 && k2p_package.height < 30.0)
      AbiSendMsgAGL(AGL_SONAR_ADC_ID, k2p_package.height);




    break;
  }
    default:
      break;
  }
}

/* We need to wait for incomming messages */
void kalamos_event() {
  // Check if we got some message from the Kalamos
  pprz_check_and_parse(kalamos.device, &kalamos.transport, mp_msg_buf, &kalamos.msg_available);

  // If we have a message we should parse it
  if (kalamos.msg_available) {
    kalamos_parse_msg();
    kalamos.msg_available = false;
  }
}

void kalamos_periodic() {

  struct FloatEulers *att = stateGetNedToBodyEulers_f();

  struct PPRZ2KalamosPackage p2k_package;
  p2k_package.phi = att->phi;
  p2k_package.theta = att->theta;

  unsigned char hoer[] = "hoer";
  for(uint8_t i = 0; i < 4; i++)
      p2k_package.hoer[i] = hoer[i];


      // Send Telemetry report
DOWNLINK_SEND_SONAR(DefaultChannel, DefaultDevice, 0, &k2p_package.height);


 pprz_msg_send_IMCU_DEBUG(&(kalamos.transport.trans_tx), kalamos.device,
                                       1, sizeof(struct PPRZ2KalamosPackage), (unsigned char *)(&p2k_package));


}



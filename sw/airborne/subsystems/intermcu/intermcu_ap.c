/*
 * Copyright (C) 2015 The Paparazzi Team
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
 *
 */

/** @file subsystems/intermcu/intermcu_ap.c
 *  @brief Rotorcraft Inter-MCU on the autopilot
 */

#include "intermcu_ap.h"
#include "pprzlink/intermcu_msg.h"
#include "subsystems/radio_control.h"
#include "pprzlink/pprz_transport.h"
#include "mcu_periph/uart.h"

#include "subsystems/electrical.h"
#include "autopilot.h"

#if COMMANDS_NB > 8
#error "INTERMCU UART CAN ONLY SEND 8 COMMANDS OR THE UART WILL BE OVERFILLED"
#endif

// Used for communication
static struct link_device *intermcu_device = (&((INTERMCU_LINK).device));
static struct pprz_transport intermcu_transport;

/* Main interMCU defines */
struct intermcu_t inter_mcu;
static struct fbw_status_t fbw_status;
static inline void intermcu_parse_msg(void (*rc_frame_handler)(void));

#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"

/* Send FBW status */
static void send_status(struct transport_tx *trans, struct link_device *dev)
{
  pprz_msg_send_FBW_STATUS(trans, dev, AC_ID,
                           &fbw_status.rc_status, &fbw_status.frame_rate, &fbw_status.mode, &fbw_status.vsupply,
                           &fbw_status.current);
}
#endif

/* InterMCU initialization */
void intermcu_init(void)
{
  inter_mcu.msg_available = FALSE;
  inter_mcu.enabled = TRUE;
  pprz_transport_init(&intermcu_transport);

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FBW_STATUS, send_status);
#endif
}

/* Check for InterMCU loss */
void intermcu_periodic(void)
{
  /* Check for interMCU loss */
  if (inter_mcu.time_since_last_frame >= INTERMCU_LOST_CNT) {
    inter_mcu.status = INTERMCU_LOST;
  } else {
    inter_mcu.time_since_last_frame++;
  }
}

/* Enable or disable the communication of the InterMCU */
void intermcu_set_enabled(bool_t value)
{
  inter_mcu.enabled = value;
}

/* Send the actuators to the FBW */
void intermcu_set_actuators(pprz_t *command_values, uint8_t ap_mode __attribute__((unused)))
{
  if (inter_mcu.enabled) {
    pprz_msg_send_IMCU_COMMANDS(&(intermcu_transport.trans_tx), intermcu_device,
                                INTERMCU_AP, &autopilot_motors_on, COMMANDS_NB, command_values); //TODO: Append more status
  }
}

/* Send the spektrum Bind message */
void intermcu_send_spektrum_bind(void)
{
  if (inter_mcu.enabled) {
    pprz_msg_send_IMCU_SPEKTRUM_SOFT_BIND(&(intermcu_transport.trans_tx), intermcu_device, INTERMCU_AP);
  }
}

/* Parse incomming InterMCU messages */
#pragma GCC diagnostic ignored "-Wcast-align"
static inline void intermcu_parse_msg(void (*rc_frame_handler)(void))
{
  /* Parse the Inter MCU message */
  uint8_t msg_id = inter_mcu.msg_buf[1];
  switch (msg_id) {
    case DL_IMCU_RADIO_COMMANDS: {
      uint8_t i;
      uint8_t size = DL_IMCU_RADIO_COMMANDS_values_length(inter_mcu.msg_buf);
      inter_mcu.status = DL_IMCU_RADIO_COMMANDS_status(inter_mcu.msg_buf);
      for (i = 0; i < size; i++) {
        radio_control.values[i] = DL_IMCU_RADIO_COMMANDS_values(inter_mcu.msg_buf)[i];
      }

      radio_control.frame_cpt++;
      radio_control.time_since_last_frame = 0;
      radio_control.status = RC_OK;
      rc_frame_handler();
      break;
    }

    case DL_IMCU_FBW_STATUS: {
      fbw_status.rc_status = DL_IMCU_FBW_STATUS_rc_status(inter_mcu.msg_buf);
      fbw_status.frame_rate = DL_IMCU_FBW_STATUS_frame_rate(inter_mcu.msg_buf);
      fbw_status.mode = DL_IMCU_FBW_STATUS_mode(inter_mcu.msg_buf);
      fbw_status.vsupply = DL_IMCU_FBW_STATUS_vsupply(inter_mcu.msg_buf);
      fbw_status.current = DL_IMCU_FBW_STATUS_current(inter_mcu.msg_buf);
      break;
    }

    default:
      break;
  }
}
#pragma GCC diagnostic pop

/* Radio control event misused as InterMCU event for frame_handler */
void RadioControlEvent(void (*frame_handler)(void))
{
  /* Parse incoming bytes */
  if (inter_mcu.enabled) {
    pprz_check_and_parse(intermcu_device, &intermcu_transport, inter_mcu.msg_buf, &inter_mcu.msg_available);

    if(inter_mcu.msg_available) {
      intermcu_parse_msg(frame_handler);
      inter_mcu.msg_available = FALSE;
    }
  }
}

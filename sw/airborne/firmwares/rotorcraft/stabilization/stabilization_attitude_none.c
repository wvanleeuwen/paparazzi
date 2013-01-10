/*
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include "firmwares/rotorcraft/stabilization.h"

#include "subsystems/radio_control.h"
#include "generated/airframe.h"

struct Int32Rates stabilization_none_rc_cmd;

//STUB
struct Int32Eulers stabilization_att_sum_err;
int32_t stabilization_att_fb_cmd[COMMANDS_NB];
int32_t stabilization_att_ff_cmd[COMMANDS_NB];

void stabilization_attitude_init(void) {
	INT_RATES_ZERO(stabilization_none_rc_cmd);
}


void stabilization_attitude_read_rc(bool_t in_flight) {
	stabilization_none_rc_cmd.p = (int32_t)radio_control.values[RADIO_ROLL];
	stabilization_none_rc_cmd.q = (int32_t)radio_control.values[RADIO_PITCH];
	stabilization_none_rc_cmd.r = (int32_t)radio_control.values[RADIO_YAW];
}


void stabilization_attitude_enter(void) {
	INT_RATES_ZERO(stabilization_none_rc_cmd);
}

void stabilization_attitude_run(bool_t  in_flight __attribute__ ((unused))) {
	/* just directly pass rc commands through */
	stabilization_cmd[COMMAND_ROLL]  = stabilization_none_rc_cmd.p;
	stabilization_cmd[COMMAND_PITCH] = stabilization_none_rc_cmd.q;
	stabilization_cmd[COMMAND_YAW]   = stabilization_none_rc_cmd.r;
}

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
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "subsystems/radio_control.h"
#include "generated/airframe.h"

struct FloatEulers stab_att_sp_euler;

//STUB
struct FloatEulers stab_att_ref_euler;
struct FloatRates  stab_att_ref_rate;
struct FloatRates  stab_att_ref_accel;
struct FloatEulers stabilization_att_sum_err_eulers;
float stabilization_att_fb_cmd[COMMANDS_NB];
float stabilization_att_ff_cmd[COMMANDS_NB];

void stabilization_attitude_init(void) {
	FLOAT_EULERS_ZERO( stabilization_att_sum_err_eulers );
	FLOAT_EULERS_ZERO(stab_att_sp_euler);
	FLOAT_EULERS_ZERO(stab_att_ref_euler);
	FLOAT_RATES_ZERO(stab_att_ref_rate);
	FLOAT_RATES_ZERO(stab_att_ref_accel);
}


void stabilization_attitude_read_rc(bool_t in_flight) {
	//Read from RC
	stabilization_attitude_read_rc_setpoint_eulers_f(&stab_att_sp_euler, in_flight);
}


void stabilization_attitude_enter(void) {

}

void stabilization_attitude_run(bool_t  in_flight __attribute__ ((unused))) {
	/* just directly pass guidance commands */
	stabilization_cmd[COMMAND_ROLL]  = stab_att_sp_euler.phi;
	stabilization_cmd[COMMAND_PITCH] = stab_att_sp_euler.theta;
	stabilization_cmd[COMMAND_YAW]   = stab_att_sp_euler.psi;
}

void stabilization_attitude_ref_init(void) {

}

void stabilization_attitude_ref_update(void) {

}

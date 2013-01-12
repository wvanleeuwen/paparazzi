/*
 * Paparazzi $Id: fw_v_ctl.h 4232 2009-10-06 02:47:20Z vassilis $
 *
 * Copyright (C) 2006  Pascal Brisset, Antoine Drouin
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

/**
 *
 * fixed wing vertical control using total energy theory
 *
 */

#ifndef FW_V_CTL_ENERGY_H
#define FW_V_CTL_ENERGY_H

#include <inttypes.h>
#include "paparazzi.h"

// Flight Envelope
extern float v_ctl_vmin;
extern float v_ctl_vmax;
extern float v_ctl_vz_max;
extern float v_ctl_pot_energy_bounds;
extern float v_ctl_kin_energy_bounds;
extern float v_ctl_pitch_min;
extern float v_ctl_pitch_max;
extern float v_ctl_throttle_min;
extern float v_ctl_throttle_max;

// Setpoints
extern float v_ctl_auto_airspeed_setpoint;

// Gains
extern float v_ctl_energy_tot_i;
extern float v_ctl_energy_dis_i;
extern float v_ctl_energy_tot_p;
extern float v_ctl_energy_dis_p;
extern float v_ctl_energy_tot_d;
extern float v_ctl_energy_dis_d;


#endif /* FW_V_CTL_ENERGY_H */

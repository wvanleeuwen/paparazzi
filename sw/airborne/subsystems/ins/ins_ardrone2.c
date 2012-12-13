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

#include "subsystems/ins/ins_ardrone2.h"
#include "subsystems/ahrs.h"
#include "subsystems/gps.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "math/pprz_geodetic_int.h"

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif

/* TODO: Remove from messages */
int32_t ins_qfe;

/* gps transformed to LTP-NED  */
struct LtpDef_i  ins_ltp_def;
         bool_t  ins_ltp_initialised;
struct NedCoor_i ins_gps_pos_cm_ned;
struct NedCoor_i ins_gps_speed_cm_s_ned;

/* output                      */
struct NedCoor_i ins_ltp_pos;
struct NedCoor_i ins_ltp_speed;
struct NedCoor_i ins_ltp_accel;


void ins_init() {
#if USE_INS_NAV_INIT
  ins_ltp_initialised = TRUE;

  /** FIXME: should use the same code than MOVE_WP in firmwares/rotorcraft/datalink.c */
  struct LlaCoor_i llh_nav0; /* Height above the ellipsoid */
  llh_nav0.lat = INT32_RAD_OF_DEG(NAV_LAT0);
  llh_nav0.lon = INT32_RAD_OF_DEG(NAV_LON0);
  /* NAV_ALT0 = ground alt above msl, NAV_MSL0 = geoid-height (msl) over ellipsoid */
  llh_nav0.alt = NAV_ALT0 + NAV_MSL0;

  struct EcefCoor_i ecef_nav0;
  ecef_of_lla_i(&ecef_nav0, &llh_nav0);

  ltp_def_from_ecef_i(&ins_ltp_def, &ecef_nav0);
  ins_ltp_def.hmsl = NAV_ALT0;
  stateSetLocalOrigin_i(&ins_ltp_def);
#else
  ins_ltp_initialised  = FALSE;
#endif
  ins.vf_realign = FALSE;
  ins.hf_realign = FALSE;

  INT32_VECT3_ZERO(ins_ltp_pos);
  INT32_VECT3_ZERO(ins_ltp_speed);
  INT32_VECT3_ZERO(ins_ltp_accel);

  // TODO correct init
  ins.status = INS_RUNNING;

}

void ins_periodic( void ) {
}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {

}

void ins_realign_v(float z __attribute__ ((unused))) {

}

void ins_propagate() {

}

void ins_update_baro() {

}


void ins_update_gps(void) {
#if USE_GPS
  if (gps.fix == GPS_FIX_3D) {
    if (!ins_ltp_initialised) {
      ltp_def_from_ecef_i(&ins_ltp_def, &gps.ecef_pos);
      ins_ltp_def.lla.alt = gps.lla_pos.alt;
      ins_ltp_def.hmsl = gps.hmsl;
      ins_ltp_initialised = TRUE;
      stateSetLocalOrigin_i(&ins_ltp_def);
    }
    ned_of_ecef_point_i(&ins_gps_pos_cm_ned, &ins_ltp_def, &gps.ecef_pos);
    ned_of_ecef_vect_i(&ins_gps_speed_cm_s_ned, &ins_ltp_def, &gps.ecef_vel);

    INT32_VECT3_SCALE_2(ins_ltp_pos, ins_gps_pos_cm_ned, INT32_POS_OF_CM_NUM, INT32_POS_OF_CM_DEN);
    INT32_VECT3_SCALE_2(ins_ltp_speed, ins_gps_speed_cm_s_ned, INT32_SPEED_OF_CM_S_NUM, INT32_SPEED_OF_CM_S_DEN);

#if USE_GPS_LAG_HACK
    VECT2_COPY(d_pos, ins_ltp_speed);
    INT32_VECT2_RSHIFT(d_pos, d_pos, 11);
    VECT2_ADD(ins_ltp_pos, d_pos);
#endif

    INS_NED_TO_STATE();
  }
#endif /* USE_GPS */
}

void ins_update_sonar() {

}

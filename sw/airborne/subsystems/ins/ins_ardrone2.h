/*
 * Copyright (C) 2008-2012 The Paparazzi Team
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

#ifndef INS_INT_H
#define INS_INT_H

#include "subsystems/ins.h"
#include "std.h"
#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

// TODO integrate all internal state to the structure
///** Ins implementation state (fixed point) */
//struct InsInt {
//};
//
///** global INS state */
//extern struct InsInt ins_impl;

/* gps transformed to LTP-NED  */
extern struct LtpDef_i  ins_ltp_def;
extern          bool_t  ins_ltp_initialised;
extern struct NedCoor_i ins_gps_pos_cm_ned;
extern struct NedCoor_i ins_gps_speed_cm_s_ned;

/* barometer                   */
extern int32_t ins_qfe;
#if USE_VFF || USE_VFF_EXTENDED
extern int32_t ins_baro_alt;
extern bool_t  ins_baro_initialised;
#if USE_SONAR
extern bool_t  ins_update_on_agl; /* use sonar to update agl if available */
extern int32_t ins_sonar_offset;
#endif
#endif

/* output LTP NED               */
extern struct NedCoor_i ins_ltp_pos;
extern struct NedCoor_i ins_ltp_speed;
extern struct NedCoor_i ins_ltp_accel;
#if USE_HFF
/* horizontal gps transformed to NED in meters as float */
extern struct FloatVect2 ins_gps_pos_m_ned;
extern struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

/* copy position and speed to state interface */
#define INS_NED_TO_STATE() {             \
  stateSetPositionNed_i(&ins_ltp_pos);   \
  stateSetSpeedNed_i(&ins_ltp_speed);    \
  stateSetAccelNed_i(&ins_ltp_accel);    \
}

/* ARDRONE */
typedef float    float32_t;

typedef struct _matrix33_t
{
  float32_t m11;
  float32_t m12;
  float32_t m13;
  float32_t m21;
  float32_t m22;
  float32_t m23;
  float32_t m31;
  float32_t m32;
  float32_t m33;
} matrix33_t;

typedef struct _vector31_t {
  union {
    float32_t v[3];
    struct
    {
      float32_t x;
      float32_t y;
      float32_t z;
    };
  };
} vector31_t;

// Define constants for gyrometers handling
typedef enum {
  GYRO_X    = 0,
  GYRO_Y    = 1,
  GYRO_Z    = 2,
  NB_GYROS  = 3
} def_gyro_t;


// Define constants for accelerometers handling
typedef enum {
  ACC_X   = 0,
  ACC_Y   = 1,
  ACC_Z   = 2,
  NB_ACCS = 3
} def_acc_t;

typedef struct _velocities_t {
  float32_t x;
  float32_t y;
  float32_t z;
} velocities_t;

typedef enum _navdata_tag_t {
  NAVDATA_DEMO_TAG = 0,
  NAVDATA_TIME_TAG
} navdata_tag_t;

typedef struct _navdata_option_t {
  uint16_t  tag;
  uint16_t  size;
  uint8_t   data[1];
} __attribute__ ((packed)) navdata_option_t;

typedef struct _navdata_t {
  uint32_t    header;			/*!< Always set to NAVDATA_HEADER */
  uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
  uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
  uint32_t    vision_defined;

  navdata_option_t  options[1];
} __attribute__ ((packed)) navdata_t;

typedef struct _navdata_cks_t {
  uint16_t  tag;
  uint16_t  size;

  // Checksum for all navdatas (including options)
  uint32_t  cks;
} __attribute__ ((packed)) navdata_cks_t;

typedef struct _navdata_demo_t {
  uint16_t    tag;					  /*!< Navdata block ('option') identifier */
  uint16_t    size;					  /*!< set this to the size of this structure */

  uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
  uint32_t    vbat_flying_percentage; /*!< battery voltage filtered (mV) */

  float32_t   theta;                  /*!< UAV's pitch in milli-degrees */
  float32_t   phi;                    /*!< UAV's roll  in milli-degrees */
  float32_t   psi;                    /*!< UAV's yaw   in milli-degrees */

  int32_t     altitude;               /*!< UAV's altitude in centimeters */

  float32_t   vx;                     /*!< UAV's estimated linear velocity */
  float32_t   vy;                     /*!< UAV's estimated linear velocity */
  float32_t   vz;                     /*!< UAV's estimated linear velocity */

  uint32_t    num_frames;			  /*!< streamed frame index */ // Not used -> To integrate in video stage.

  // Camera parameters compute by detection
  matrix33_t  detection_camera_rot;   /*!<  Deprecated ! Don't use ! */
  vector31_t  detection_camera_trans; /*!<  Deprecated ! Don't use ! */
  uint32_t	  detection_tag_index;    /*!<  Deprecated ! Don't use ! */

  uint32_t	  detection_camera_type;  /*!<  Type of tag searched in detection */

  // Camera parameters compute by drone
  matrix33_t  drone_camera_rot;		  /*!<  Deprecated ! Don't use ! */
  vector31_t  drone_camera_trans;	  /*!<  Deprecated ! Don't use ! */
} __attribute__ ((packed)) navdata_demo_t;

#endif /* INS_INT_H */

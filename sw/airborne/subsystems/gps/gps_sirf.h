#ifndef GPS_SIRF_H
#define GPS_SIRF_H

#include "mcu_periph/uart.h"
#include "sirf_protocol.h"
#include "mcu_periph/sys_time.h"
         
#define GPS_SIRF_MAX_PAYLOAD 1024
struct GpsSirf {
  bool_t msg_available;
  uint8_t msg_buf[GPS_SIRF_MAX_PAYLOAD] __attribute__ ((aligned));
  uint8_t msg_id;
  uint8_t msg_class;

  uint8_t status;
  uint16_t len;
  uint8_t msg_idx;
  uint8_t ck_a, ck_b;
  uint8_t error_cnt;
  uint8_t error_last;
  
  uint8_t have_geo;
  uint8_t have_nav;
};


extern struct GpsSirf gps_sirf;  

extern void gps_sirf_read_message(void);
extern void gps_sirf_parse(uint8_t c);

/*
 * This part is used by the autopilot to read data from a uart
 */
#define __GpsLink(dev, _x) dev##_x
#define _GpsLink(dev, _x)  __GpsLink(dev, _x)
#define GpsLink(_x) _GpsLink(GPS_LINK, _x)

#define GpsBuffer() GpsLink(ChAvailable())

#define GpsEvent(_gps_uptodate_callback) {        \
  if (GpsBuffer()) {                              \
    ReadGpsBuffer();                              \
  }                                               \
  if (gps_ubx.msg_available) {                    \
    gps_sirf_read_message();                      \
    if ((gps_sirf.msg_id == SIRF_GEO_ID ||        \
      gps_sirf.msg_id == SIRF_NAV_ID) &&          \
      gps_sirf.have_geo && gps_sirf.have_nav  ) { \
      if (gps.fix == GPS_FIX_3D) {                \
        gps.last_fix_ticks = sys_time.nb_sec_rem; \
        gps.last_fix_time = sys_time.nb_sec;      \
      }                                           \
      uint8_t have_geo = 0;                       \
      uint8_t have_nav = 0;                       \
      _gps_uptodate_callback();                   \
    }                                             \
    gps_sirf.msg_available = FALSE;               \
  }                                               \
}

#define ReadGpsBuffer() {					                \
  while (GpsLink(ChAvailable())&&!gps_sirf.msg_available)	\
    gps_sirf_parse(GpsLink(Getch()));			            \
}


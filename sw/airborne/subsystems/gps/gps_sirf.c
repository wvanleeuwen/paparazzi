#include "subsystems/gps.h"
#include <math.h>

/* parser status */
#define UNINIT        0
#define GOT_SYNC1     1
#define GOT_SYNC2     2
#define GOT_LEN1      3
#define GOT_LEN2      4
#define GOT_ID        5
#define GOT_PAYLOAD   6
#define GOT_CHECKSUM1 7

/* last error type */
#define GPS_SIRF_ERR_NONE         0
#define GPS_SIRF_ERR_OVERRUN      1
#define GPS_SIRF_ERR_MSG_TOO_LONG 2
#define GPS_SIRF_ERR_CHECKSUM     3
#define GPS_SIRF_ERR_UNEXPECTED   4
#define GPS_SIRF_ERR_OUT_OF_SYNC  5

struct GpsSirf gps_sirf;

void gps_impl_init(void) {
   gps_mtk.status = UNINIT;
   gps_mtk.msg_available = FALSE;
   gps_mtk.error_cnt = 0;
   gps_mtk.error_last = GPS_MTK_ERR_NONE;
}

/* reads the payload that was stored in msg_buf by the parse method */
void gps_sirf_read_message(void) {
  if(gps_sirf.msg_id == SIRF_NAV_ID)  {
    gps.tow        = SIRF_NAV_ITOW(gps_SIRF.msg_buf) * 10;
    gps.week       = SIRF_NAV_week(gps_SIRF.msg_buf);
    gps.ecef_pos.x = SIRF_NAV_XPOS(gps_SIRF.msg_buf) * 100;
    gps.ecef_pos.y = SIRF_NAV_YPOS(gps_SIRF.msg_buf) * 100;
    gps.ecef_pos.z = SIRF_NAV_ZPOS(gps_SIRF.msg_buf) * 100;
    gps.ecef_vel.x = SIRF_NAV_XVEL(gps_SIRF.msg_buf) * 100 / 8;
    gps.ecef_vel.y = SIRF_NAV_YVEL(gps_SIRF.msg_buf) * 100 / 8;
    gps.ecef_vel.z = SIRF_NAV_ZVEL(gps_SIRF.msg_buf) * 100 / 8;
    gps.num_sv     = SIRF_NAV_SV(gps_SIRF.msg_buf);
    /* Fix isn't in the message so we derive it from the amount of SV's */
    if(gps.num_sv == 3) {
      gps.fix = GPS_FIX_2D;
    } else if(gps.num_sv > 3) {
      gps.fix = GPS_FIX_3D;
    } else {
      gps.fix = GPS_FIX_NONE;
    }
    /* No PDOP in the msg, so we use HDOP instead */
    gps.pdop       = SIRF_NAV_HDOP(gps_SIRF.msg_buf)* 100 / 5;

  } else if(gps_sirf.msg_id == SIRF_GEO_ID) {
    gps.lla_pos.lat = RadOfDeg(SIRF_GEO_LAT(gps_sirf.msg_buf));
    gps.lla_pos.lon = RadOfDeg(SIRF_GEO_LON(gps_sirf.msg_buf));
    gps.lla_pos.alt = SIRF_GEO_ALT(gps_sirf.msg_buf) * 10;
    gps.hmsl        = SIRF_GEO_HMSL(gps_sirf.msg_buf) * 10;
    gps.pacc        = (uint32_t)sqrt(SIRF_GEO_HPE(gps_sirf.msg_bug) * SIRF_GEO_HPE(gps_sirf.msg_buf) + SIRF_GEO_VPE(gps_sirf.msg.buf) * SIRF_GEO_VPE(gps_sirf.msg.buf));
    gps.gspeed      = SIRF_GEO_GSPEED(gps_sirf.msg_buf);
    gps.sacc        = SIRF_GEO_HVE(gps_sirf.gps.msg_buf);
    gps.course      = RadOfDeg(SIRF_GEO_COURSE(gps_sirf.msg_buf)) * 100000;
  }
}

/* SiRF parsing */
void gps_sirf_parse( uint8_t c ) {
     if (gps_sirf.status < GOT_PAYLOAD) {
    gps_sirf.ck_a += c;
    gps_sirf.ck_b += gps_SIRF.ck_a;
  }
  switch (gps_sirf.status) {
  /* try to read a start sequence */
  case UNINIT:
    if (c == SIRF_SYNC1)
      gps_sirf.status++;
    break;
  case GOT_SYNC1:
    if (c != SIRF_SYNC2) {
      gps_SIRF.error_last = GPS_SIRF_ERR_OUT_OF_SYNC;
      goto error;
    }
    gps_sirf.ck_a = 0;
    gps_sirf.ck_b = 0;
    gps_sirf.status++;
    break;
  /* Start sequence has been received -> reads the length of the payLoad */
  case GOT_SYNC2:
    if (gps_sirf.msg_available) {
      /* Previous message has not yet been parsed: discard this one */
      gps_sirf.error_last = GPS_SIRF_ERR_OVERRUN;
      goto error;
    }
    gps_sirf.len = c;
    gps_sirf.status++;
    break;
  case GOT_LEN1:
    gps_sirf.len |= (c<<8);
    if (gps_sirf.len > GPS_SIRF_MAX_PAYLOAD) {
      gps_sirf.error_last = GPS_SIRF_ERR_MSG_TOO_LONG;
      goto error;
    }
    gps_sirf.status++;
    break;
  case GOT_LEN2:
    gps_sirf.msg_idx = 0;
    gps_sirf.msg_id = c;
    gps_sirf.status++;
    break;
  /* Length of payload received -> reads id of the message */
  case GOT_ID:
    gps_sirf.msg_buf[gps_sirf.msg_idx] = c;
    gps_sirf.msg_idx++;
    if (gps_sirf.msg_idx >= gps_sirf.len) {
      gps_sirf.status++;
    }
    break;
  /* payload received -> reads checksum */
  case GOT_PAYLOAD:
    if (c != gps_sirf.ck_a) {
      gps_sirf.error_last = GPS_SIRF_ERR_CHECKSUM;
      goto error;
    }
    gps_SIRF.status++;
    break;
  case GOT_CHECKSUM1:
    if (c != gps_sirf.ck_b) {
      gps_sirf.error_last = GPS_SIRF_ERR_CHECKSUM;
      goto error;
    }
    gps_sirf.msg_available = TRUE;
    goto restart;
    break;
  default:
    gps_sirf.error_last = GPS_SIRF_ERR_UNEXPECTED;
    goto error;
  }
  return;
 error:
  gps_sirf.error_cnt++;
 restart:
  gps_sirf.status = UNINIT;
  return;
}
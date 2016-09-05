/*
 * Copyright (C) 2016 Wilco Vlenterie, Anand Sundaresan.
 * Contact: Anand Sundaresan <nomail@donotmailme.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

 /**
 * \file rtcm2ivy.c
 * \brief RTCM3 GPS packets to Ivy for DGPS and RTK
 *
 * This communicates with an RTCM3 GPS receiver like an
 * ublox M8P. This then forwards the Observed messages
 * over the Ivy bus to inject them for DGPS and RTK positioning.
 */

#include <glib.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <ctype.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <rtcm3.h>
#include <math/pprz_geodetic_float.h>

#include "std.h"
#include "serial_port.h"

/** Used variables **/
struct SerialPort *serial_port;
rtcm3_state_t rtcm3_state;
rtcm3_msg_callbacks_node_t rtcm3_1005_node;
rtcm3_msg_callbacks_node_t rtcm3_1077_node;
rtcm3_msg_callbacks_node_t rtcm3_1087_node;

/** Default values **/
uint8_t ac_id = 0;
char *serial_device   = "/dev/ttyACM0";
uint32_t serial_baud  = B9600;

/** Debugging options */
bool verbose = FALSE;
#define printf_debug    if(verbose == TRUE) printf

/** Ivy Bus default */
#ifdef __APPLE__
char *ivy_bus                   = "224.255.255.255";
#else
char *ivy_bus                   = "127.255.255.255:2010"; // 192.168.1.255   127.255.255.255
#endif

/*
 * Read bytes from the Piksi UART connection
 * This is a wrapper functions used in the librtcm3 library
 */
static uint32_t rtcm3_read(unsigned char (*buff)[], uint32_t n, void *context __attribute__((unused)))
{
  int ret = read(serial_port->fd, buff, n);
  if(ret > 0)
    return ret;
  else
    return 0;
}

static void ivy_send_message(uint8_t packet_id, uint8_t len, uint8_t msg[]) {
  char gps_packet[512], number[5];
  uint8_t i;

  snprintf(gps_packet, 512, "0 RTCM_INJECT %d %d %d", packet_id, len, msg[0]); //AC_ID
  //snprintf(gps_packet, 512, "datalink RTCM_INJECT %d %d", packet_id, msg[0]); //AC_ID
  for(i = 1; i < len; i++) {
    snprintf(number, 5, ",%d", msg[i]);
    strcat(gps_packet, number);
  }
  printf("%s\n\n", gps_packet);
  IvySendMsg("%s", gps_packet);
  printf_debug("Ivy send: %s\n", gps_packet);
}

/*
 * Callback for the OBS observation message to send it trough GPS_INJECT
 */
struct EcefCoor_f posEcef;
struct LlaCoor_f posLla;

static void rtcm3_1005_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
	printf("Sending 1005 message\n");
    u16 StaId      = RTCMgetbitu(msg, 24 + 12, 12);
    u8 ItRef       = RTCMgetbitu(msg, 24 + 24, 6);
    u8 indGPS      = RTCMgetbitu(msg, 24 + 30, 1);
    u8 indGlonass  = RTCMgetbitu(msg, 24 + 31, 1);
    u8 indGalileo  = RTCMgetbitu(msg, 24 + 32, 1);
    u8 indRefS     = RTCMgetbitu(msg, 24 + 33, 1);
    posEcef.x      = RTCMgetbits_38(msg, 24 + 34) * 0.0001;
    posEcef.y      = RTCMgetbits_38(msg, 24 + 74) * 0.0001;
    posEcef.z      = RTCMgetbits_38(msg, 24 + 114) * 0.0001;
    printf("x: %f, y: %f, z: %f\n", posEcef.x, posEcef.y, posEcef.z);
    lla_of_ecef_f(&posLla, &posEcef);
    printf("Lat: %f, Lon: %f, Alt: %f\n", posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
    // Send spoof gpsd message to GCS to plot groundstation position
    IvySendMsg("%s %s %s %f %f %f %f %f %f %f %f %f %f %f %d %f", "ground", "FLIGHT_PARAM", "GCS", 0.0, 0.0, 0.0, posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, 0.0, 0.0, posLla.alt, 0.0, 0.0, 0.0, 0,  0.0);
    // Send UBX_RTK_GROUNDSTATION message to GCS for RTK info
    IvySendMsg("%s %s %s %i %i %i %i %i %i %f %f %f", "ground", "UBX_RTK_GROUNDSTATION", "GCS", StaId, ItRef, indGPS, indGlonass, indGalileo, indRefS, posLla.lat / (2 * M_PI) * 360, posLla.lon / (2 * M_PI) * 360, posLla.alt);
    ivy_send_message(RTCM3_MSG_1005, len, msg);
  }
  printf_debug("Parsed OBS callback\n");
}

/*
 * Callback for the OBS observation message to send it trough GPS_INJECT
 */
static void rtcm3_1077_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
	printf("Sending 1077 message\n");
    ivy_send_message(RTCM3_MSG_1077, len, msg);
  }
  printf_debug("Parsed OBS callback\n");
}

/*
 * Callback for the OBS observation message to send it trough GPS_INJECT
 */
static void rtcm3_1087_callback(uint16_t sender_id __attribute__((unused)),
                                  uint8_t len,
                                  uint8_t msg[],
                                  void *context __attribute__((unused)))
{
  if(len > 0) {
	printf("Sending 1087 message\n");
    ivy_send_message(RTCM3_MSG_1087, len, msg);
  }
  printf_debug("Parsed OBS callback\n");
}

/**
 * Parse the tty data when bytes are available
 */
static gboolean parse_device_data(GIOChannel *chan, GIOCondition cond, gpointer data)
{
  rtcm3_process(&rtcm3_state, &rtcm3_read);
  return TRUE;
}

/** Print the program help */
void print_usage(int argc __attribute__((unused)), char ** argv) {
  static const char *usage =
    "Usage: %s [options]\n"
    " Options :\n"
    "   -h, --help                Display this help\n"
    "   -v, --verbose             Verbosity enabled\n\n"

    "   -d <device>               The GPS device(default: /dev/ttyUSB0)\n\n";
  fprintf(stderr, usage, argv[0]);
}

int main(int argc, char** argv)
{
  // Parse the options from cmdline
  char c;
  while ((c = getopt (argc, argv, "hvd:b:i:")) != EOF) {
    switch (c) {
    case 'h':
      print_usage(argc, argv);
      exit(EXIT_SUCCESS);
      break;
    case 'v':
      verbose = TRUE;
      break;
    case 'd':
      serial_device = optarg;
      break;
    case 'b':
      serial_baud = atoi(optarg);
      break;
    case 'i':
      ac_id = atoi(optarg);
      break;
    case '?':
      if (optopt == 'd' || optopt == 'b' || optopt == 'i')
        fprintf (stderr, "Option -%c requires an argument.\n", optopt);
      else if (isprint (optopt))
        fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
      print_usage(argc, argv);
      exit(EXIT_FAILURE);
    default:
      abort();
    }
  }

  // Create the Ivy Client
  GMainLoop *ml =  g_main_loop_new(NULL, FALSE);
  IvyInit("Paparazzi server", "Paparazzi server READY", 0, 0, 0, 0);
  IvyStart(ivy_bus);

  // Start the tty device
  printf_debug("Opening tty device %s...\n", serial_device);
  serial_port = serial_port_new();
  int ret = serial_port_open_raw(serial_port, serial_device, serial_baud);
  if (ret != 0) {
    fprintf(stderr, "Error opening %s code %d\n", serial_device, ret);
    serial_port_free(serial_port);
    exit(EXIT_FAILURE);
  }

  // Setup RTCM3 callbacks
  printf_debug("Setup RTCM3 callbacks...\n");
  rtcm3_state_init(&rtcm3_state);
  rtcm3_register_callback(&rtcm3_state, RTCM3_MSG_1005, &rtcm3_1005_callback, NULL, &rtcm3_1005_node);
  rtcm3_register_callback(&rtcm3_state, RTCM3_MSG_1077, &rtcm3_1077_callback, NULL, &rtcm3_1077_node);
  rtcm3_register_callback(&rtcm3_state, RTCM3_MSG_1087, &rtcm3_1087_callback, NULL, &rtcm3_1087_node);

  // Add IO watch for tty connection
  printf_debug("Adding IO watch...\n");
  GIOChannel *sk = g_io_channel_unix_new(serial_port->fd);
  g_io_add_watch(sk, G_IO_IN, parse_device_data, NULL);

  // Run the main loop
  printf_debug("Started rtcm2ivy for aircraft id %d!\n", ac_id);
  g_main_loop_run(ml);

  return 0;
}

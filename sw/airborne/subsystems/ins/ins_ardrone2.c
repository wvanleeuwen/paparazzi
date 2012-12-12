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

#include "subsystems/sensors/baro.h"
#include "subsystems/gps.h"

#include "generated/airframe.h"

#if USE_VFF
#include "subsystems/ins/vf_float.h"
#endif

#if USE_HFF
#include "subsystems/ins/hf_float.h"
#endif

#ifdef SITL
#include "nps_fdm.h"
#include <stdio.h>
#endif
#include <stdio.h>


#include "math/pprz_geodetic_int.h"
#include "math/pprz_algebra_float.h"

#include "generated/flight_plan.h"

/* gps transformed to LTP-NED  */
struct LtpDef_i  ins_ltp_def;
         bool_t  ins_ltp_initialised;
struct NedCoor_i ins_gps_pos_cm_ned;
struct NedCoor_i ins_gps_speed_cm_s_ned;
#if USE_HFF
/* horizontal gps transformed to NED in meters as float */
struct FloatVect2 ins_gps_pos_m_ned;
struct FloatVect2 ins_gps_speed_m_s_ned;
#endif

/* barometer                   */
int32_t ins_qfe;

/* output                      */
struct NedCoor_i ins_ltp_pos;
struct NedCoor_i ins_ltp_speed;
struct NedCoor_i ins_ltp_accel;

/* ARDRONE2 */
#include "subsystems/electrical.h"
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#define NAVDATA_PORT 5554
#define AT_PORT 5556
#define NAVDATA_BUFFER_SIZE 2048
#define WIFI_MYKONOS_IP "192.168.1.1"

int seq=1;
char msg[NAVDATA_BUFFER_SIZE];

int at_socket = -1, //sendto
		navdata_socket = -1; //recvfrom

struct sockaddr_in
pc_addr, //INADDR_ANY
drone_at, //send at addr
drone_nav, //send nav addr
from;


void ins_init() {
  //INIT ARDrone
	int32_t one = 1;

	if((at_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0){
		printf ("at_socket error: %s\n", strerror(errno));
	};

	if((navdata_socket = socket (AF_INET, SOCK_DGRAM, 0)) < 0){
		printf ("navdata_socket: %s\n", strerror(errno));
	};

	//for recvfrom
	pc_addr.sin_family = AF_INET;
	pc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	pc_addr.sin_port = htons(9800);

	//for sendto AT
	drone_at.sin_family = AF_INET;
	drone_at.sin_addr.s_addr = inet_addr(WIFI_MYKONOS_IP);
	drone_at.sin_port = htons(AT_PORT);

	//for sendto navadata init
	drone_nav.sin_family = AF_INET;
	drone_nav.sin_addr.s_addr = inet_addr(WIFI_MYKONOS_IP);
	drone_nav.sin_port = htons(NAVDATA_PORT);

	if(bind( navdata_socket, (struct sockaddr *)&pc_addr, sizeof(pc_addr)) < 0){
		printf ("bind: %s\n", strerror(errno));
	};

	//set unicast mode on
	sendto(navdata_socket, &one, 4, 0, (struct sockaddr *)&drone_nav, sizeof(drone_nav));


  ins_ltp_initialised = TRUE;
  // TODO correct init
  ins.status = INS_RUNNING;
}

void ins_periodic( void ) {
	int l,size;
	navdata_t* packet;
	navdata_demo_t* packet2;
	struct FloatEulers angles;
	struct NedCoor_i pos;
	struct NedCoor_i speed;

	//printf("test\n");

	size = recvfrom ( navdata_socket, &msg, NAVDATA_BUFFER_SIZE, 0x0, (struct sockaddr *)&from, (socklen_t *)&l);

	//printf("read %d data\n",size);
	packet = (navdata_t*) &msg;
	printf("Packet tag %i, %i, %x, %x\n", packet->sequence, packet->vision_defined, packet->options[0].tag, packet->options[0].size);
	if(packet->options[0].tag == 0) {
		packet2 = (navdata_demo_t*) &packet->options[0];
		//printf("Packet tag %f %f %f %d %d\n", packet2->theta, packet2->psi, packet2->phi, packet2->altitude, packet2->vbat_flying_percentage);
		angles.theta = packet2->theta/180000.*M_PI;
		angles.psi = packet2->psi/180000.*M_PI;
		angles.phi = -packet2->phi/180000.*M_PI;
		electrical.vsupply = packet2->vbat_flying_percentage;
		pos.x = 0;
		pos.y = 0;
		pos.z = packet2->altitude;
		speed.x = packet2->vx;
		speed.y = packet2->vy;
		speed.z = packet2->vz;
	}
	//printf("pos\n");
	stateSetNedToBodyEulers_f(&angles);
	stateSetPositionNed_i(&pos);
	stateSetSpeedNed_i(&speed);
}

void ins_realign_h(struct FloatVect2 pos __attribute__ ((unused)), struct FloatVect2 speed __attribute__ ((unused))) {
#if USE_HFF
  b2_hff_realign(pos, speed);
#endif /* USE_HFF */
}

void ins_realign_v(float z __attribute__ ((unused))) {
#if USE_VFF
  vff_realign(z);
#endif
}

void ins_update_baro() {

}


void ins_update_gps(void) {

}

void ins_update_sonar() {

}

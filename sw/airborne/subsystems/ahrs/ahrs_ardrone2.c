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

#include "ahrs_ardrone2.h"
#include "state.h"
#include "math/pprz_algebra_float.h"
#include "boards/ardrone2.h"
#include "boards/ardrone/packets.h"

/* ARDRONE2 */
#include "subsystems/electrical.h"
#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

struct AhrsARDrone ahrs_impl;

//AR Drone communication
unsigned char buffer[ARDRONE_NAVDATA_BUFFER_SIZE];
int at_socket = -1,
	navdata_socket = -1;
struct sockaddr_in pc_addr,
				   drone_at,
				   drone_nav,
				   from;

void ahrs_init(void) {
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
	drone_at.sin_addr.s_addr = inet_addr(ARDRONE_IP);
	drone_at.sin_port = htons(ARDRONE_AT_PORT);

	//for sendto navadata init
	drone_nav.sin_family = AF_INET;
	drone_nav.sin_addr.s_addr = inet_addr(ARDRONE_IP);
	drone_nav.sin_port = htons(ARDRONE_NAVDATA_PORT);

	if(bind(navdata_socket, (struct sockaddr *)&pc_addr, sizeof(pc_addr)) < 0){
		printf("bind: %s\n", strerror(errno));
	};

	//set unicast mode on
	int one = 1;
	sendto(navdata_socket, &one, 4, 0, (struct sockaddr *)&drone_nav, sizeof(drone_nav));
}

void ahrs_align(void) {

}

void ahrs_propagate(void) {
	int l,size;
	navdata_t* packet;
	navdata_demo_t* packet2;
	struct FloatEulers angles;
	struct NedCoor_i pos;
	struct NedCoor_i speed;

	//printf("test\n");

	size = recvfrom(navdata_socket, &buffer, ARDRONE_NAVDATA_BUFFER_SIZE, 0x0, (struct sockaddr *)&from, (socklen_t *)&l);

	//printf("read %d data\n",size);
	packet = (navdata_t*) &buffer;
	//printf("Packet tag %i, %i, %x, %x\n", packet->sequence, packet->vision_defined, packet->options[0].tag, packet->options[0].size);
	if(packet->options[0].tag == 0) {
		packet2 = (navdata_demo_t*) &packet->options[0];
		//printf("Packet tag %f %f %f %d %d\n", packet2->theta, packet2->psi, packet2->phi, packet2->altitude, packet2->vbat_flying_percentage);
		ahrs_impl.control_state = packet2->ctrl_state;
		ahrs_impl.eulers.phi = packet2->phi;
		ahrs_impl.eulers.theta = packet2->theta;
		ahrs_impl.eulers.psi = packet2->psi;
		ahrs_impl.speed.x = packet2->vx;
		ahrs_impl.speed.y = packet2->vy;
		ahrs_impl.speed.z = packet2->vz;
		ahrs_impl.altitude = packet2->altitude;
		ahrs_impl.battery = packet2->vbat_flying_percentage;

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

void ahrs_update_accel(void) {

}

void ahrs_update_mag(void) {

}

void ahrs_update_gps(void) {

}

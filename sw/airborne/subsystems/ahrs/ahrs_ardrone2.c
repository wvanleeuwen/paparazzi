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

int seq = 1;

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
	navdata_t* main_packet;
	struct FloatEulers angles;
	INT_EULERS_ZERO(angles);

	//printf("test\n");

	size = recvfrom(navdata_socket, &buffer, ARDRONE_NAVDATA_BUFFER_SIZE, 0x0, (struct sockaddr *)&from, (socklen_t *)&l);

	//printf("read %d data\n",size);
	main_packet = (navdata_t*) &buffer;
	//printf("Packet tag %i, %i, %x, %x\n", packet->sequence, packet->vision_defined, packet->options[0].tag, packet->options[0].size);

	navdata_option_t* navdata_option = (navdata_option_t*)&main_packet->options[0];
	bool_t full_read = FALSE;
	navdata_demo_t* navdata_demo;
	navdata_phys_measures_t* navdata_phys_measures;


	while(!full_read) {
		switch(navdata_option->tag) {
		case 0:
			navdata_demo = (navdata_demo_t*) navdata_option;
			//printf("Packet tag %f %f %f %d %d\n", navdata_demo->theta, navdata_demo->psi, navdata_demo->phi, navdata_demo->altitude, navdata_demo->vbat_flying_percentage);
			ahrs_impl.control_state = navdata_demo->ctrl_state >> 16;
			ahrs_impl.eulers.phi = navdata_demo->phi;
			ahrs_impl.eulers.theta = navdata_demo->theta;
			ahrs_impl.eulers.psi = navdata_demo->psi;
			ahrs_impl.speed.x = navdata_demo->vx / 100.0f;
			ahrs_impl.speed.y = navdata_demo->vy / 100.0f;
			ahrs_impl.speed.z = navdata_demo->vz / 100.0f;
			ahrs_impl.altitude = navdata_demo->altitude;
			ahrs_impl.battery = navdata_demo->vbat_flying_percentage;

			angles.theta = navdata_demo->theta/180000.*M_PI;
			angles.psi = navdata_demo->psi/180000.*M_PI;
			angles.phi = -navdata_demo->phi/180000.*M_PI;
			electrical.vsupply = navdata_demo->vbat_flying_percentage;
			//full_read = TRUE;
			//printf("Read: %d %d %d\n", &main_packet->options[0], navdata_option, &navdata_demo->drone_camera_trans);
			break;
		case 3:
			navdata_phys_measures = (navdata_phys_measures_t*) navdata_option;
			VECT3_SDIV(ahrs_impl.accel, navdata_phys_measures->phys_accs, 1000);
			//printf("DONE!");
			//full_read = TRUE;
			break;
		case 0xFFFF:
			full_read = TRUE;
			break;
		default:
			//printf("TAG: %d\n", navdata_option->tag);
			break;
		}

		//printf("Prev: %d\n", navdata_option);
		//printf("Size: %d, %d\n", navdata_option->size, ((int)navdata_option + navdata_option->size));
		navdata_option = (navdata_option_t*) ((int)navdata_option + navdata_option->size);
		//printf("Next: %d\n", navdata_option);
		//break;
	}
	//printf("pos\n");
	stateSetNedToBodyEulers_f(&angles);
}

void ahrs_update_accel(void) {

}

void ahrs_update_mag(void) {

}

void ahrs_update_gps(void) {

}

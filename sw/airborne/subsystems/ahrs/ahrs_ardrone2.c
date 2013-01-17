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
		printf("bind error: %s\n", strerror(errno));
	};

	//set unicast mode on
	int one = 1;
	sendto(navdata_socket, &one, 4, 0, (struct sockaddr *)&drone_nav, sizeof(drone_nav));
}

void ahrs_align(void) {

}

#define Invert2Bytes(x) ((x>>8) | (x<<8))
#define Invert4Bytes(x) ((x>>24) | ((x<<8) & 0x00FF0000) | ((x>>8) & 0x0000FF00) | (x<<24))

void ahrs_propagate(void) {
	//Recieve the main packet
	int l;
	recvfrom(navdata_socket, &buffer, ARDRONE_NAVDATA_BUFFER_SIZE, 0x0, (struct sockaddr *)&from, (socklen_t *)&l);
	navdata_t*main_packet = (navdata_t*) &buffer;

	//Init the option
	navdata_option_t* navdata_option = (navdata_option_t*)&main_packet->options[0];
	bool_t full_read = FALSE;

	//The possible packets
	navdata_demo_t* navdata_demo;
	navdata_phys_measures_t* navdata_phys_measures;

	//Read the navdata until packet is fully readed
	while(!full_read) {
		//Check the tag for the right option
		switch(navdata_option->tag) {
		case 0: //NAVDATA_DEMO
			navdata_demo = (navdata_demo_t*) navdata_option;

			//Set the AHRS state
			ahrs_impl.control_state = navdata_demo->ctrl_state >> 16;
			ahrs_impl.eulers.phi = navdata_demo->phi;
			ahrs_impl.eulers.theta = navdata_demo->theta;
			ahrs_impl.eulers.psi = navdata_demo->psi;
			ahrs_impl.speed.x = navdata_demo->vx / 1000;
			ahrs_impl.speed.y = navdata_demo->vy / 1000;
			ahrs_impl.speed.z = navdata_demo->vz / 1000;
			ahrs_impl.altitude = navdata_demo->altitude / 10;
			ahrs_impl.battery = navdata_demo->vbat_flying_percentage;

			//Set the ned to body eulers
			struct FloatEulers angles;
			angles.theta = navdata_demo->theta/180000.*M_PI;
			angles.psi = navdata_demo->psi/180000.*M_PI;
			angles.phi = -navdata_demo->phi/180000.*M_PI;
			stateSetNedToBodyEulers_f(&angles);

			//Update the electrical supply
			electrical.vsupply = navdata_demo->vbat_flying_percentage;
			break;
		case 3: //NAVDATA_PHYS_MEASURES
			navdata_phys_measures = (navdata_phys_measures_t*) navdata_option;

			//Set the AHRS accel state
			INT32_VECT3_SCALE_2(ahrs_impl.accel, navdata_phys_measures->phys_accs, 9.81, 1000)
			break;
		case 0xFFFF: //LAST OPTION
			full_read = TRUE;
			break;
		default:
			//printf("NAVDATA UNKNOWN TAG: %d\n", navdata_option->tag);
			break;
		}
		navdata_option = (navdata_option_t*) ((int)navdata_option + navdata_option->size);
	}

}

void ahrs_update_accel(void) {

}

void ahrs_update_mag(void) {

}

void ahrs_update_gps(void) {

}

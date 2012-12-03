/*
* Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with paparazzi; see the file COPYING. If not, write to
* the Free Software Foundation, 59 Temple Place - Suite 330,
* Boston, MA 02111-1307, USA.
*
*/

#include "subsystems/datalink/wifi.h"
#include "fms/fms_network.h"

#define LINK_HOST     "192.168.1.0"
#define LINK_PORT             4242
#define DATALINK_PORT         4243
#define FMS_NETWORK_BROADCAST TRUE

char udp_buffer[1496];
uint16_t udp_buffer_id;
uint8_t ck_a, ck_b;
struct FmsNetwork* network;

void wifi_init( void ) {
	network = network_new(LINK_HOST, LINK_PORT, DATALINK_PORT, FMS_NETWORK_BROADCAST);
	udp_buffer_id = 0;
}

void wifi_transmit( uint8_t data ) {
	udp_buffer[udp_buffer_id] = data;
	udp_buffer_id++;
}

void wifi_send( void ) {
	network_write(network, udp_buffer, udp_buffer_id);
	udp_buffer_id =0;
}

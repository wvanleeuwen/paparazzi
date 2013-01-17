
#include "boards/ardrone2.h"
#include "at_com.h"

#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>


int packet_seq = 1;										//Packet sequence number

int at_socket = -1,										//AT socket connection
	navdata_socket = -1;								//Navdata socket connection

struct sockaddr_in pc_addr,								//Own pc address
				   drone_at,							//Drone AT address
				   drone_nav,							//Drone nav address
				   from;								//From address

bool_t at_com_ready = FALSE;							//Status of the at communication

void at_com_send(char* command);

//Init the at_com
void init_at_com(void) {
	//Check if already initialized
	if(at_com_ready)
		return;

	//Create the at and navdata socket
	if((at_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("at_com: at_socket error (%s)\n", strerror(errno));
	}
	if((navdata_socket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("at_com: navdata_socket error (%s)\n", strerror(errno));
	}

	//For recvfrom
	pc_addr.sin_family = AF_INET;
	pc_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	pc_addr.sin_port = htons(9800);

	//For sendto AT
	drone_at.sin_family = AF_INET;
	drone_at.sin_addr.s_addr = inet_addr(ARDRONE_IP);
	drone_at.sin_port = htons(ARDRONE_AT_PORT);

	//For sendto navadata init
	drone_nav.sin_family = AF_INET;
	drone_nav.sin_addr.s_addr = inet_addr(ARDRONE_IP);
	drone_nav.sin_port = htons(ARDRONE_NAVDATA_PORT);

	//Bind the navdata socket
	if(bind(navdata_socket, (struct sockaddr *)&pc_addr, sizeof(pc_addr)) < 0) {
		printf("at_com: bind error (%s)\n", strerror(errno));
	}

	//Set unicast mode on
	int one = 1;
	sendto(navdata_socket, &one, 4, 0, (struct sockaddr *)&drone_nav, sizeof(drone_nav));

	//Set at_com to ready
	at_com_ready = TRUE;
}

//Recieve a navdata packet
void at_com_recieve_navdata(unsigned char* buffer) {
	int l;
	recvfrom(navdata_socket, buffer, ARDRONE_NAVDATA_BUFFER_SIZE, 0x0, (struct sockaddr *)&from, (socklen_t *)&l);
}

//Send an AT command
void at_com_send(char* command) {
	sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at));
}

//Send a Config
void at_com_send_config(char* key, char* value) {
	char command[256];
	sprintf(command, "AT*CONFIG=%d,\"%s\",\"%s\"\r", packet_seq++, key, value);
	at_com_send(command);
}

//Send a Flat trim
void at_com_send_ftrim(void) {
	char command[256];
	sprintf(command, "AT*FTRIM=%d\r", packet_seq++);
	at_com_send(command);
}

//Send a Ref
void at_com_send_ref(int bits) {
	char command[256];
	sprintf(command, "AT*REF=%d,%d\r", packet_seq++, bits | REF_DEFAULT);
	at_com_send(command);
}

//Send a Pcmd
void at_com_send_pcmd(int mode, float thrust, float roll, float pitch, float yaw) {
	int f_thrust, f_roll, f_pitch, f_yaw;
	char command[256];

	//Change the floats to ints(dereferencing)
	memcpy(&f_thrust, &thrust, sizeof thrust);
	memcpy(&f_roll, &roll, sizeof roll);
	memcpy(&f_pitch, &pitch, sizeof pitch);
	memcpy(&f_yaw, &yaw, sizeof yaw);

	sprintf(command, "AT*PCMD=%d,%d,%d,%d,%d,%d\r", packet_seq++, mode, f_roll, f_pitch, f_thrust, f_yaw);
	at_com_send(command);
}

//Send a Calib
void at_com_send_calib(int device) {
	char command[256];
	sprintf(command, "AT*CALIB=%d,%d\r", packet_seq++, device);
	at_com_send(command);
}

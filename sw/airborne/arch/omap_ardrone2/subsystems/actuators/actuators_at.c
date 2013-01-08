
#include "actuators_at.h"
#include "generated/airframe.h"
#include "subsystems/ahrs/ahrs_ardrone2.h"

#include <stdio.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>

void actuators_init(void) {
	char command[256];
	sprintf(command,"AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", seq++);
	sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );

	sprintf(command,"AT*FTRIM=%d\r", seq++);
	sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );
}

void actuators_set(pprz_t commands[]) {
	char command[256];
	float thrust = ((float)(commands[COMMAND_THRUST]-MAX_PPRZ/2) / (float)MAX_PPRZ)*2.0f;
	float roll = ((float)commands[COMMAND_ROLL] / (float)MAX_PPRZ) / 2.0f;
	float pitch = ((float)commands[COMMAND_PITCH] / (float)MAX_PPRZ) / 2.0f;
	float yaw = ((float)commands[COMMAND_YAW] / (float)MAX_PPRZ) / 2.0f;
	printf("Commands: %f\t%f\t%f\t%f\n", thrust, roll, pitch, yaw);

	if(thrust > 0 && ahrs_impl.control_state != 3 && ahrs_impl.control_state != 4) {
		sprintf(command,"AT*REF=%d,%d\r", seq++, 0x11540000 + 0x200);
		sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );
		//printf("Send start to drone\n");
	}

	if(thrust < -0.75) {
		sprintf(command,"AT*REF=%d,%d\r", seq++, 0x11540000);
		sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );
		//printf("Send stop to drone\n");
	}

	if(ahrs_impl.control_state == 3 || ahrs_impl.control_state == 4) {
		sprintf(command,"AT*PCMD=%d,%d,%d,%d,%d,%d\r", seq++, 0x1, *(int*)(&roll), *(int*)(&pitch), *(int*)(&thrust), *(int*)(&yaw));
		sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );
	}

	sprintf(command,"AT*CONFIG=%d,\"general:navdata_demo\",\"FALSE\"\r", seq++);
		sendto(at_socket, command, strlen(command), 0, (struct sockaddr*)&drone_at, sizeof(drone_at) );
}

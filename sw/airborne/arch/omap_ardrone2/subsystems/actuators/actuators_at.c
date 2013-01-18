
#include "subsystems/ahrs/ahrs_ardrone2.h"
#include "actuators_at.h"
#include "generated/airframe.h"
#include "boards/ardrone/at_com.h"

void actuators_init(void) {
	init_at_com();

	//Set navdata_demo to FALSE and flat trim the ar drone
	at_com_send_config("general:navdata_demo", "FALSE");
	at_com_send_ftrim();
}

void actuators_set(pprz_t commands[]) {
	//Calculate the thrus, roll, pitch and yaw from the PPRZ commands
	float thrust = ((float)(commands[COMMAND_THRUST]-MAX_PPRZ/2) / (float)MAX_PPRZ)*2.0f;
	float roll = ((float)commands[COMMAND_ROLL] / (float)MAX_PPRZ);
	float pitch = ((float)commands[COMMAND_PITCH] / (float)MAX_PPRZ);
	float yaw = ((float)commands[COMMAND_YAW] / (float)MAX_PPRZ);

	//Starting engine
	if(thrust > 0 && (ahrs_impl.control_state == CTRL_DEFAULT || ahrs_impl.control_state == CTRL_INIT || ahrs_impl.control_state == CTRL_LANDED))
		at_com_send_ref(REF_TAKEOFF);

	//Check emergency or stop engine
	if((ahrs_impl.state & ARDRONE_EMERGENCY_MASK) != 0)
		at_com_send_ref(REF_EMERGENCY);
	else if(thrust < -0.9 && !(ahrs_impl.control_state == CTRL_DEFAULT || ahrs_impl.control_state == CTRL_INIT || ahrs_impl.control_state == CTRL_LANDED))
		at_com_send_ref(0);

	//Calibration
	if((ahrs_impl.state & ARDRONE_MAGNETO_NEEDS_CALIB) != 0 && (ahrs_impl.control_state == CTRL_FLYING || ahrs_impl.control_state == CTRL_HOVERING))
		at_com_send_calib(0);

	//Moving
	if((ahrs_impl.state & ARDRONE_MAGNETO_NEEDS_CALIB) == 0 && (ahrs_impl.control_state == CTRL_FLYING || ahrs_impl.control_state == CTRL_HOVERING))
		at_com_send_pcmd(1, thrust, roll, pitch, yaw);

	//Keep alive (FIXME)
	at_com_send_config("general:navdata_demo", "FALSE");
}

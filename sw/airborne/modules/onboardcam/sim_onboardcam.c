#include BOARD_CONFIG
#include "onboardcam.h"
#include "mcu_periph/adc.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include "generated/periodic.h"

#include "subsystems/sensors/infrared.h"
#include "estimator.h"

#define IS_CAMERA 1

//static struct adc_buf buf_onboardcama;
uint16_t adc_onboardcama;
//static struct adc_buf buf_onboardcamb;
uint16_t adc_onboardcamb;
uint8_t onboardcam_status;
uint8_t onboardcam_mode;
uint8_t blackfin_mode = 0;

void init_onboardcam(void) {
  infrared_init();
  onboardcam_mode = IS_CAMERA;
  adc_onboardcama = 512.00f;
  adc_onboardcamb = 512.00f;


}

void periodic_onboardcam(void) {


	if ( onboardcam_mode == IS_CAMERA )
	{
		estimator_phi  = (adc_onboardcama-512.00f)/512.00f; //0=1pi 512=0 930=1pi
		estimator_theta  =(adc_onboardcamb-512.00f)/512.00f;
	}
	else	// Infrared ATT
	{
		infrared_update();
		estimator_update_state_infrared();
	}

//pogin1
//   float cam_pan = 0;
//   float pan_diff = 0;

//       pan_diff = adc_onboardcama;
//       pan_diff /=512.0f;
//       pan_diff -=1.0f;

//         cam_pan = MAX_PPRZ * (pan_diff / (RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL)));

//        cam_pan = TRIM_PPRZ(cam_pan);

//        #ifdef COMMAND_CAM_PAN
//          ap_state->commands[COMMAND_CAM_PAN] = cam_pan;
//        #endif


//poging2
//      h_ctl_disabled = TRUE;
//      float cmd_aileron = adc_onboardcama;
//      float cmd_elevator = adc_onboardcamb;

//      h_ctl_aileron_setpoint = TRIM_PPRZ(cmd_aileron);
//      h_ctl_elevator_setpoint = TRIM_PPRZ(cmd_elevator);

//      ap_state->commands[COMMAND_ROLL] = h_ctl_aileron_setpoint;
//      ap_state->commands[COMMAND_PITCH] = h_ctl_elevator_setpoint;


}


void start_onboardcam(void)
{
  onboardcam_status = 1;
}

void stop_onboardcam(void)
{
  onboardcam_status = 0;
  h_ctl_disabled = FALSE;
}



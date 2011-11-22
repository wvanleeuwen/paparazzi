#include BOARD_CONFIG
#include "onboardcam.h"
#include "mcu_periph/adc.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"

#ifndef ADC_CHANNEL_ONBOARDCAMA
#error "ON BOARD CAM needs ADC_CHANNEL_ONBOARDCAMA or ADC_CHANNEL_ONBOARDCAMB"
#endif

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "uart.h"
#include "messages.h"
#include "downlink.h"
#include "generated/periodic.h"
#include "estimator.h"

#define IS_CAMERA 1

static struct adc_buf buf_onboardcama;
uint16_t adc_onboardcama;

static struct adc_buf buf_onboardcamb;
uint16_t adc_onboardcamb;

uint8_t onboardcam_status;

uint8_t onboardcam_mode;
uint8_t blackfin_mode = 0;

void init_onboardcam(void)
{
  infrared_init();
  onboardcam_mode = IS_CAMERA;
  adc_buf_channel(ADC_CHANNEL_ONBOARDCAMA, &buf_onboardcama, ADC_CHANNEL_ONBOARDCAMA_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_ONBOARDCAMB, &buf_onboardcamb, ADC_CHANNEL_ONBOARDCAMB_NB_SAMPLES);
  LED_INIT(1);
  LED_INIT(5);
}

void periodic_onboardcam(void)
{
#ifndef SITL
  adc_onboardcama = (buf_onboardcama.sum / buf_onboardcama.av_nb_sample); //gaat van 0-3 vold dus 3*1024/3,3 0 tot 930
  adc_onboardcamb = (buf_onboardcamb.sum / buf_onboardcamb.av_nb_sample);
//adc_onboardcama = 1.0f;
//adc_onboardcamb = 2.0f;
#endif



	if ( onboardcam_mode == IS_CAMERA )
	{
		infrared_update(); //for mesages
		estimator_phi  = (adc_onboardcama-512.00f)/512.00f; //0=1rad 512=0 930=1rad
		estimator_theta  =(adc_onboardcamb-512.00f)/512.00f;
//		ir_roll_neutral = 0.00f;
//		ir_pitch_neutral = 0.00f;
	}
	else	// Infrared ATT
	{
		infrared_update();
		estimator_update_state_infrared();
//		ir_roll_neutral = 0.00f;
//		ir_pitch_neutral = 0.00f;
	}

   // downlink_onboardcam();


//   float cam_pan = 0;
//   float pan_diff = 0;

//       pan_diff = adc_onboardcama;
//       pan_diff /=512.0f;
//      pan_diff -=1.0f;

//         cam_pan = MAX_PPRZ * (pan_diff / (RadOfDeg(CAM_PAN_MAX - CAM_PAN_NEUTRAL)));

//        cam_pan = TRIM_PPRZ(cam_pan);

//        #ifdef COMMAND_CAM_PAN
//          ap_state->commands[COMMAND_CAM_PAN] = cam_pan;
//        #endif

}

/*
void downlink_onboardcam(uint8_t channel)
{
//if you use this you have to add your mesage to ap_douwnlink and defauld.xml no need to send it here
//  RunOnceEvery(0.02/PERIOD_VISUALTARGET_DefaultChannel_0, DOWNLINK_SEND_VISUALTARGET (DefaultChannel, &adc_onboardcama,&adc_onboardcamb));
	if ( onboardcam_mode != IS_CAMERA )
	{

		DOWNLINK_SEND_VISUALTARGET (channel, &adc_onboardcama,&adc_onboardcamb);
		DOWNLINK_SEND_IR_SENSORS(channel, &ir_ir1, &ir_ir2, &ir_pitch, &ir_roll, &ir_top);
	}
        else
	{
          DOWNLINK_SEND_VISUALTARGET (channel, &adc_onboardcama, &adc_onboardcamb);
	}
}
*/
void start_onboardcam(void)
{
  onboardcam_status = 1;
  LED_ON(5)
  LED_ON(1)
}

void stop_onboardcam(void)
{
  onboardcam_status = 0;
 LED_OFF(5)
 LED_OFF(1)
}











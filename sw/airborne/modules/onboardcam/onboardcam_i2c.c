#include BOARD_CONFIG

#include "onboardcam_i2c.h"
#include "mcu_periph/adc.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "subsystems/sensors/infrared_adc.c"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"
#include "generated/periodic.h"
#include "estimator.h"

#include "atmega48.h"

#define IS_CAMERA 1


// Status
uint8_t onboardcam_status;
uint8_t onboardcam_mode;
uint8_t blackfin_mode = 0;

// Unused but needed by settings
uint16_t adc_onboardcama = 1;
uint16_t adc_onboardcamb = 2;

// Init
void init_onboardcam(void)
{
  infrared_adc_init();
  onboardcam_mode = !IS_CAMERA;
  atmega48_init();
}


/**

// multi byte resolution
static inline void scale_pprz_to_blackfin(float var, uint8_t* buff)
{
  buff[0] = (var + 100)
}

static inline int scale_blackfin_to_pprz(uint8_t* buff)
{
  float
}

*/

/**********************************************************************
 AFSPRAKEN MET BLACKFIN: interface_sky_segmentation.c
 *********************************************************************/

const int MAX_ROLL_ANGLE = 60;
const int MAX_PITCH_ANGLE = 40;
const int MAX_I2C_BYTE = 254;

typedef struct
{
  int pitch;
  int roll;
} paparazzi_state_struct;


static inline uint8_t scale_to_range(int x, int min, int max, int range)
{
	if (x < min)
		x = min;
	else if (x > max)
		x = max;

	x -= min;
	x *= range;
	x /= (max - min);
	return (uint8_t) x;
}


static inline int unscale_from_range(uint8_t x, int range, int min, int max)
{
	int v = x;

	v *= (max - min);
	v /= range;
	v += min;

	return v;
}


/**********************************************************************
 *********************************************************************/

enum AlgorithmeType { BF_HORIZON = 0, BF_OBSTACLES = 1 };

void periodic_onboardcam(void)
{
	// Send and Receive BlackFun Data
	if (blackfin_mode == 0)
	{
  	to_atmega48[0] = BF_HORIZON;    // Task: (Algorithm) Horizon=0, Obstacles=1
	  to_atmega48[1] = 0;             // 0=max-min segmentation, 1=decision tree
	  to_atmega48[2] = 4;             // Horizon: aantal samples: 4 = 1000 samples9 =14000
	                                  // Obstacles: Threshold: 0 = meer grond detecteren, 9 = meer lucht
	}
	else if (blackfin_mode == 1)
	{
  	to_atmega48[0] = BF_HORIZON;    // Task: (Algorithm) Horizon=0, Obstacles=1
	  to_atmega48[1] = 1;             // 0=max-min segmentation, 1=decision tree
	  to_atmega48[2] = 4;             // Horizon: aantal samples: 4 = 1000 samples9 =14000
	                                  // Obstacles: Threshold: 0 = meer grond detecteren, 9 = meer lucht
	}
	else if (blackfin_mode == 2)
	{
  	to_atmega48[0] = BF_OBSTACLES;    // Task: (Algorithm) Horizon=0, Obstacles=1
	  to_atmega48[1] = 0;             // 0=max-min segmentation, 1=decision tree
	  to_atmega48[2] = 4;             // Horizon: aantal samples: 4 = 1000 samples9 =14000
	                                  // Obstacles: Threshold: 0 = meer grond detecteren, 9 = meer lucht
	}
	else
	{
  	to_atmega48[0] = BF_OBSTACLES;    // Task: (Algorithm) Horizon=0, Obstacles=1
	  to_atmega48[1] = 1;             // 0=max-min segmentation, 1=decision tree
	  to_atmega48[2] = 4;             // Horizon: aantal samples: 4 = 1000 samples9 =14000
	                                  // Obstacles: Threshold: 0 = meer grond detecteren, 9 = meer lucht
	}
	to_atmega48[3] = scale_to_range(DegOfRad(estimator_theta), -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE, MAX_I2C_BYTE);
	to_atmega48[4] = scale_to_range(DegOfRad(estimator_phi), -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE, MAX_I2C_BYTE);

	atmega48_periodic();

	// Infrared Attitude
	if ( onboardcam_mode == IS_CAMERA )
	{
		infrared_adc_update(); //for mesages
		if (from_atmega48[2] < 50)
		{
			estimator_phi  = RadOfDeg(unscale_from_range(from_atmega48[1], MAX_I2C_BYTE, -MAX_ROLL_ANGLE, MAX_ROLL_ANGLE));
			estimator_theta  = RadOfDeg(unscale_from_range(from_atmega48[0], MAX_I2C_BYTE, -MAX_PITCH_ANGLE, MAX_PITCH_ANGLE));
		}
		else
		{
			estimator_phi = 0;
			estimator_theta = 0;
		}
	}
	// Infrared Attitude
	else
	{
		infrared_adc_update();
		estimator_update_state_infrared();
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











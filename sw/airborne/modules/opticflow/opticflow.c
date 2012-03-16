#include "opticflow.h"
#include "mcu_periph/adc.h"
#include "generated/airframe.h"
#include "generated/flight_plan.h"
#include "subsystems/nav.h"

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "estimator.h"

#include BOARD_CONFIG

#ifndef ADC_CHANNEL_OPTICFLOW
#error "Optic Flow needs ADC_CHANNEL_OPTICFLOW"
#endif


static struct adc_buf buf_opticflow;
uint16_t adc_opticflow;

uint16_t adc_opticflow_catastrophic;
uint16_t adc_opticflow_low;

float opticflow_climb_rate;
float opticflow_descend_rate;

uint8_t opticflow_status;
float max_opticflow_phi;
float max_opticflow_theta;

float opticflow_ref_alt;

uint8_t wp_opticflow;
uint8_t wp_border;

void init_opticflow(void)
{
  opticflow_status = 0;
  adc_opticflow = 0;
  wp_opticflow = 0;
  wp_border = 20 ;
  max_opticflow_phi = 10;
  max_opticflow_theta = 10;

  adc_opticflow_catastrophic = OPTICFLOW_THRESHOLD_CATASTROPHIC;
  adc_opticflow_low = OPTICFLOW_THRESHOLD_LOW;

  opticflow_climb_rate = OPTICFLOW_CLIMB_RATE;
  opticflow_descend_rate = OPTICFLOW_DESCEND_RATE;

  opticflow_ref_alt = 0.0f;

#ifndef SITL
  adc_buf_channel(ADC_CHANNEL_OPTICFLOW, &buf_opticflow, ADC_CHANNEL_OPTICFLOW_NB_SAMPLES);
#endif

}

bool_t opticflow_setup(uint8_t wp_flow_start, uint8_t wp_flow)
{
  opticflow_ref_alt = WaypointAlt(wp_flow_start);
  wp_opticflow = wp_flow;
  return FALSE; // no error
}

void periodic_opticflow(void)
{
if ((estimator_phi > RadOfDeg(max_opticflow_phi)) | (estimator_theta > RadOfDeg(max_opticflow_theta)) | (estimator_phi < RadOfDeg(-max_opticflow_phi)) | (estimator_theta < RadOfDeg(-max_opticflow_theta)))
 {
	opticflow_status = 0;
 }
  else
  { if(fabs(estimator_z-WaypointAlt(wp_opticflow)) > wp_border)
	{
		opticflow_status = 0;
	}
	else
	{
		opticflow_status = 1;
	}
  }

#ifndef SITL
  adc_opticflow += ((buf_opticflow.sum / buf_opticflow.av_nb_sample) - adc_opticflow) / 2;
#endif
  if (opticflow_status > 0)
  {
    if (adc_opticflow > adc_opticflow_catastrophic)
    {
      opticflow_ref_alt += opticflow_climb_rate / 60.0f;
    }
    else if (adc_opticflow > adc_opticflow_low)
    {
      //opticflow_ref_alt += opticflow_climb_rate / 600.0f;
    }
    else
    {
      opticflow_ref_alt += opticflow_descend_rate / 60.0f;
    }
    // Do not change altitude of HOME waypoint when you did not setup opticflow yet
    if (wp_opticflow > 0)
    {
      WaypointAlt(wp_opticflow) = opticflow_ref_alt;
    }
  }

  RunOnceEvery(5,DOWNLINK_SEND_OPTICFLOW(DefaultChannel, DefaultDevice, &adc_opticflow, &opticflow_ref_alt));
}

void start_opticflow(void)
{
}

void stop_opticflow(void)
{
}







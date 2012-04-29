#include "modules/ATMOS/killOnGroundDetect.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "messages.h"

bool_t kill_on_ground_detect_enabled;

void atmov_killOnGroundDetect_init(void) {
  kill_on_ground_detect_enabled = TRUE;
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //pullup
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void atmov_killOnGroundDetect_periodic(void) {
  bool_t pip = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
  DOWNLINK_SEND_BARO_RAW(DefaultChannel,DefaultDevice,0,&pip);
  if (!pip) {
    //low, so ground detected
    if (kill_on_ground_detect_enabled) {
      autopilot_set_mode(AP_MODE_KILL);
      /*autopilot_detect_ground = TRUE;
      kill_throttle = TRUE;
      autopilot_motors_on = FALSE;*/
      autopilot_set_motors_on(FALSE);
      //autopilot_arming_set(0);
    }
  }
}

void ArmGroundDetectSwitch(void) {
  kill_on_ground_detect_enabled = TRUE;
}

void DisarmGroundDetectSwitch(void) {
  kill_on_ground_detect_enabled = FALSE;
}

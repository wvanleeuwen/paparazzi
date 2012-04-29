#include "modules/ATMOS/hoverPropsOff.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/actuators/supervision.h"

bool_t killHoverProps;

void transveh_hover_props_off_init(void) {
  supervision.override_by_module=TRUE;
  uint8_t i;
  for (i=0; i<SUPERVISION_NB_MOTOR; i++) {
    supervision.override_value[i] = SUPERVISION_STOP_MOTOR;
  }

}

void transveh_hover_props_off_set(bool_t stop) {
  killHoverProps = stop;
}

void HoverPropsOff(void) {
  transveh_hover_props_off_set(1);
}
void HoverPropsOn(void) {
  transveh_hover_props_off_set(0);
}

void transveh_hover_props_off_periodic(void) {
  //supervision.override_by_module = TRUE;
  #if TRANSVEH_HOVER_PROPS_OFF_NB1
  supervision.override_enabled[0] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB2
  supervision.override_enabled[1] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB3
  supervision.override_enabled[2] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB4
  supervision.override_enabled[3] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB5
  supervision.override_enabled[4] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB6
  supervision.override_enabled[5] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB7
  supervision.override_enabled[6] = killHoverProps;
  #endif
  #if TRANSVEH_HOVER_PROPS_OFF_NB8
  supervision.override_enabled[7] = killHoverProps;
  #endif
}

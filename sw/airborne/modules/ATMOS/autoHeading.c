#include "modules/ATMOV/autoHeading.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h";


int32_t pitch_boundary,roll_boundary;

void atmov_autoheading_init(void) {
  pitch_boundary = ANGLE_BFP_OF_REAL(AUTOHEADING_PITCH_BOUNDARY);
  roll_boundary = ANGLE_BFP_OF_REAL(AUTOHEADING_roll_BOUNDARY);
}

void atmov_autoheading_periodic(void) {
    if ((stab_att_sp_euler.theta > -pitch_boundary
         || stab_att_sp_euler.theta < pitch_boundary) &&
         (stab_att_sp_euler.phi < -roll_boundary
         || stab_att_sp_euler.phi > roll_boundary))  {
        if (stab_att_sp_euler.phi < stab_att_sp_euler.theta) {
            stab_att_sp_euler.psi -= AUTOHEADING_YAW_INCREMENT;
        }
        else {
            stab_att_sp_euler.psi += AUTOHEADING_YAW_INCREMENT;
        }
    }
}

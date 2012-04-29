#include "modules/transitioning_vehicles/multiGain.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "math/pprz_algebra_int.h"


uint8_t gainSetId;

struct Int32AttitudeGains stabilization_multiGains[MULTIGAIN_NUM_GAIN_SETS];


void transveh_multigain_init(void) {
  struct Int32AttitudeGains stabilization_gainsA = {
    {STABILIZATION_ATTITUDE_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
  };
  struct Int32AttitudeGains stabilization_gainsB = {
    {STABILIZATION_ATTITUDE_B_PHI_PGAIN, STABILIZATION_ATTITUDE_B_THETA_PGAIN, STABILIZATION_ATTITUDE_B_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DGAIN, STABILIZATION_ATTITUDE_B_THETA_DGAIN, STABILIZATION_ATTITUDE_B_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DDGAIN, STABILIZATION_ATTITUDE_B_THETA_DDGAIN, STABILIZATION_ATTITUDE_B_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_IGAIN, STABILIZATION_ATTITUDE_B_THETA_IGAIN, STABILIZATION_ATTITUDE_B_PSI_IGAIN }
  };
}

void transveh_multigain_periodic(void) {

}

void transveh_multigain_setGainSet(struct Int32AttitudeGains gainSet) {
  stabilization_gains = gainSet;
}

void SetGainSetA(void) {
  transveh_multigain_setGainSet(stabilization_gainsA);
}

void SetGainSetB(void) {
  transveh_multigain_setGainSet(stabilization_gainsB);
}


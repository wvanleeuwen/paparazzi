#include "modules/ATMOS/multiGain.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "math/pprz_algebra_int.h"


struct Int32AttitudeGains stabilization_gainsA = {
    {STABILIZATION_ATTITUDE_A_PHI_PGAIN, STABILIZATION_ATTITUDE_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_DGAIN, STABILIZATION_ATTITUDE_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_DDGAIN, STABILIZATION_ATTITUDE_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_IGAIN, STABILIZATION_ATTITUDE_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN }
  };
  struct Int32AttitudeGains stabilization_gainsB = {
    {STABILIZATION_ATTITUDE_B_PHI_PGAIN, STABILIZATION_ATTITUDE_B_THETA_PGAIN, STABILIZATION_ATTITUDE_B_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DGAIN, STABILIZATION_ATTITUDE_B_THETA_DGAIN, STABILIZATION_ATTITUDE_B_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DDGAIN, STABILIZATION_ATTITUDE_B_THETA_DDGAIN, STABILIZATION_ATTITUDE_B_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_IGAIN, STABILIZATION_ATTITUDE_B_THETA_IGAIN, STABILIZATION_ATTITUDE_B_PSI_IGAIN }
  };
  struct Int32AttitudeGains stabilization_gainsC = {
    {STABILIZATION_ATTITUDE_C_PHI_PGAIN, STABILIZATION_ATTITUDE_C_THETA_PGAIN, STABILIZATION_ATTITUDE_C_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_DGAIN, STABILIZATION_ATTITUDE_C_THETA_DGAIN, STABILIZATION_ATTITUDE_C_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_DDGAIN, STABILIZATION_ATTITUDE_C_THETA_DDGAIN, STABILIZATION_ATTITUDE_C_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_IGAIN, STABILIZATION_ATTITUDE_C_THETA_IGAIN, STABILIZATION_ATTITUDE_C_PSI_IGAIN }
  };

uint8_t activeGainSet;

void transveh_multigain_init(void) {
  activeGainSet = 1;
  //transveh_multigain_setGainSet(&stabilization_gains,&stabilization_gainsA);
}

void transveh_multigain_periodic(void) {

}

//void transveh_multigain_setGainSet(struct Int32Vect3 target, struct Int32Vect3 * gainSet) {
void transveh_multigain_setGainSet(struct Int32AttitudeGains * gainSet) {
  //struct Int32AttitudeGains* stabilization_gains_ptr;
  //stabilization_gains_ptr = &stabilization_gains;
  //*stabilization_gains_ptr = *gainSet;
  //&stabilization_gains.d = &gainSet->d;
  //target->d = &gainSet->d;
  //target->d = &gainSetd;
  //target = gainSet;
  stabilization_gains.p = gainSet->p;
  stabilization_gains.i = gainSet->i;
  stabilization_gains.d = gainSet->d;
  stabilization_gains.dd = gainSet->dd;
}

void SetGainSetA(void) {
  transveh_multigain_setGainSet(&stabilization_gainsA);
  //stabilization_gains.d = &(stabilization_gainsA.d);
}

void SetGainSetB(void) {
  transveh_multigain_setGainSet(&stabilization_gainsB);
  //transveh_multigain_setGainSet(&stabilization_gains,&stabilization_gainsB);
}

void SetGainSetC(void) {
  transveh_multigain_setGainSet(&stabilization_gainsC);
  //transveh_multigain_setGainSet(&stabilization_gainsC);
}

void multiGain_SetGainSetHandler(uint8_t v) {
  activeGainSet = v;
  if (activeGainSet == 3) {
    SetGainSetC();
  }
  else if (activeGainSet == 2) {
    SetGainSetB();
  }
  else {
    SetGainSetA();
  }
}


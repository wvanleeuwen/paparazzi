#include "modules/ATMOS/multiGain.h"
#include "generated/airframe.h"
#include "firmwares/rotorcraft/autopilot.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "math/pprz_algebra_int.h"


struct Int32AttitudeGains stabilization_gainsA = {
    {STABILIZATION_ATTITUDE_A_PHI_PGAIN, STABILIZATION_ATTITUDE_A_THETA_PGAIN, STABILIZATION_ATTITUDE_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_DGAIN, STABILIZATION_ATTITUDE_A_THETA_DGAIN, STABILIZATION_ATTITUDE_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_DDGAIN, STABILIZATION_ATTITUDE_A_THETA_DDGAIN, STABILIZATION_ATTITUDE_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_IGAIN, STABILIZATION_ATTITUDE_A_THETA_IGAIN, STABILIZATION_ATTITUDE_PSI_IGAIN },
    {STABILIZATION_ATTITUDE_A_PHI_ACTGAIN, STABILIZATION_ATTITUDE_A_THETA_ACTGAIN, STABILIZATION_ATTITUDE_A_PSI_ACTGAIN }

  };
  struct Int32AttitudeGains stabilization_gainsB = {
    {STABILIZATION_ATTITUDE_B_PHI_PGAIN, STABILIZATION_ATTITUDE_B_THETA_PGAIN, STABILIZATION_ATTITUDE_B_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DGAIN, STABILIZATION_ATTITUDE_B_THETA_DGAIN, STABILIZATION_ATTITUDE_B_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_DDGAIN, STABILIZATION_ATTITUDE_B_THETA_DDGAIN, STABILIZATION_ATTITUDE_B_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_IGAIN, STABILIZATION_ATTITUDE_B_THETA_IGAIN, STABILIZATION_ATTITUDE_B_PSI_IGAIN },
    {STABILIZATION_ATTITUDE_B_PHI_ACTGAIN, STABILIZATION_ATTITUDE_B_THETA_ACTGAIN, STABILIZATION_ATTITUDE_B_PSI_ACTGAIN }
  };
  struct Int32AttitudeGains stabilization_gainsC = {
    {STABILIZATION_ATTITUDE_C_PHI_PGAIN, STABILIZATION_ATTITUDE_C_THETA_PGAIN, STABILIZATION_ATTITUDE_C_PSI_PGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_DGAIN, STABILIZATION_ATTITUDE_C_THETA_DGAIN, STABILIZATION_ATTITUDE_C_PSI_DGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_DDGAIN, STABILIZATION_ATTITUDE_C_THETA_DDGAIN, STABILIZATION_ATTITUDE_C_PSI_DDGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_IGAIN, STABILIZATION_ATTITUDE_C_THETA_IGAIN, STABILIZATION_ATTITUDE_C_PSI_IGAIN },
    {STABILIZATION_ATTITUDE_C_PHI_ACTGAIN, STABILIZATION_ATTITUDE_C_THETA_ACTGAIN, STABILIZATION_ATTITUDE_C_PSI_ACTGAIN }
  };

uint8_t activeGainSet;

void transveh_multigain_init(void) {
  activeGainSet = 1;
  SetGainSetA();
}

void transveh_multigain_periodic(void) {

}

struct Int32AttitudeGains stabilization_blended;

void SetGainPercentage(uint32_t percent)  // percent in INT32_PERCENTAGE_FRAC
{
 struct Int32AttitudeGains* ga, gb, gblend;
  int ratio;

  gblend = &stabilization_blended;

  if (percent < ((1<<INT32_PERCENTAGE_FRAC)/2))
  {
    ga = &stabilization_gainsA;
    ratio = percent * 2;
  }
  else
  {
    ga = &stabilization_gainsC;
    ratio = ((1<<INT32_PERCENTAGE_FRAC) - percent) * 2;
  }
  gb = &stabilization_gainsB;

  int64_t g1, g2, gbl; 

  for (int i=0; i < (sizeof(struct Int32AttitudeGains)/sizeof(int32_t)); i++)
  {
    g1 = *(((int32_t*) ga) + i);
    g1 *= (1<<INT32_PERCENTAGE_FRAC) - ratio;
    g2 = *(((int32_t*) gb) + i);
    g2 *= ratio;

    gbl = (g1 + g2) >> INT32_PERCENTAGE_FRACs;
    
    *(((int32_t*) gblend) + i) = (int32_t) gbl;
  }

  stabilization_gains = gblend;
  
}

void transveh_multigain_setGainSet(struct Int32AttitudeGains * gainSet) {
  stabilization_gains = gainSet;
}

void SetGainSetA(void) {
  transveh_multigain_setGainSet(&stabilization_gainsA);
}

void SetGainSetB(void) {
  transveh_multigain_setGainSet(&stabilization_gainsB);
}

void SetGainSetC(void) {
  transveh_multigain_setGainSet(&stabilization_gainsC);
}

void multiGain_SetGainSetHandler(int8_t v) {
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



#include "boards/ardrone/simulink.h"

#include "firmwares/rotorcraft/autopilot.h"

#include "subsystems/gps.h"
#include "subsystems/imu.h"
#include "subsystems/ins.h"
#include "subsystems/ahrs.h"
#include "subsystems/ahrs/ahrs_aligner.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"

#include "paparazzi1.h"

struct InsArdrone2 ins_impl;
struct Ins ins;
struct Ahrs ahrs;
struct GpsState gps;
struct AhrsAligner ahrs_aligner;


void ahrs_update_accel(void) {}
void ahrs_aligner_run(void) {}
void ins_init(void) {}
void ahrs_init(void) {}
void ahrs_align(void) {}
void ahrs_aligner_init(void) {}
void ahrs_propagate(void) {}
void ins_propagate(void) {}
void stabilization_attitude_read_rc_setpoint_eulers(struct Int32Eulers *sp, bool_t in_flight) {}
void stabilization_attitude_reset_care_free_heading(void) {}

void stabilization_attitude_init(void)
{
  paparazzi1_initialize();
}
void stabilization_attitude_read_rc(bool_t in_flight) {}
void stabilization_attitude_enter(void) {}
void stabilization_attitude_set_failsafe_setpoint(void) {}
void stabilization_attitude_set_rpy_setpoint_i(struct Int32Eulers *rpy) {}
void stabilization_attitude_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading) {}
void stabilization_attitude_run(bool_t in_flight)
{
  uint32_t baro = 10;
  uint32_t sonar = 0;
  uint32_t commands[4];
  int32_t g[3], a[3];

  g[0] = imu.gyro.p;
  g[1] = imu.gyro.q;
  g[2] = imu.gyro.r;

  a[0] = imu.accel.x;
  a[1] = imu.accel.y;
  a[2] = imu.accel.z;

  paparazzi1_step(g, a, &baro, &sonar, commands);

  stabilization_cmd[COMMAND_THRUST] = commands[2];
  stabilization_cmd[COMMAND_ROLL]   = commands[0];
  stabilization_cmd[COMMAND_PITCH]  = commands[1];
  stabilization_cmd[COMMAND_YAW]    = commands[3];
}

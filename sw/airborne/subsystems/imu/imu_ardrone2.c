/*
 * Copyright (C) 2012 TU Delft Quatrotor Group 1
 */

#include "subsystems/imu.h"
//FIXME: set correct location and file name
#include "navboard.h"

//FIXME: implement the Parrot navigation board
struct ParrotNavBoard navboard;

void imu_impl_init(void) {
  imu_available = FALSE;
}

void imu_periodic(void) {
  //checks if the navboard has a new dataset ready
//FIXME: fix navboard.size
  if (navboard.size == 60){
	  QUAT_ASSIGN(imu.gyro_unscaled, 1, navboard.vx, navboard.vy, navboard.vz);
	  VECT3_ASSIGN(imu.accel_unscaled, navboard.ax, navboard.ay, navboard.az);
	  VECT3_ASSIGN(imu.mag_unscaled, navboard.mz, -navboard.mx, navboard.my);
	  imu_available = TRUE;
  }

}


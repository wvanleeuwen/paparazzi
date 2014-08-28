/*
 * Copyright (C) OpenUAS
 *
 * This file is part of paparazzi

 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef TICKET_H
#define TICKET_H

//#define TICKET_SHOOT 0x01

extern void ticket_init;
extern void ticket_takephotonow;
//extern void ticket_get_device_capability;
//extern void ticket_set_device_physical_orientation;
//extern void ticket_periodic( void );

//#ifdef TEST_TICKET
extern float test_ticket_estimator_x;
extern float test_ticket_estimator_y;
extern float test_ticket_estimator_z;
extern float test_ticket_estimator_phi;
extern float test_ticket_estimator_theta;
extern float test_ticket_estimator_hspeed_dir;
#endif // TEST_TICKET

#endif


/*
 * Copyright (C) Roland
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/follow_me/follow_me.h"
 * @author Roland
 * follows based on stereo
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H
extern float ref_pitch;
extern float ref_roll;
extern void stereocam_forward_velocity_init(void);
extern void stereocam_forward_velocity_periodic(void);

#endif


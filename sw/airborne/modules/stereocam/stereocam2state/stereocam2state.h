/*
 * Copyright (C) Kimberly McGuire
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/stereocam/stereocam2state/stereocam2state.h"
 * @author Kimberly McGuire
 * This module sends the data retrieved from an external stereocamera modules, to the state filter of the drone. This is done so that the guidance modules can use that information for couadcopter
 */

#ifndef STEREOCAM2STATE_H
#define STEREOCAM2STATE_H

#include <std.h>
#include "modules/stereocam/stereocam.h"
#include "math/pprz_algebra_int.h"

struct Egomotion {
  struct Int32Vect2 div;
  struct Int32Vect2 flow;
  struct Int32Vect3 vel;
};

extern struct Egomotion stereo_motion;

extern void stereo_to_state_init(void);
extern void stereo_to_state_periodic(void);

#endif


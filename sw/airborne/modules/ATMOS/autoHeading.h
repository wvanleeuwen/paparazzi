#ifndef ATMOV_AUTOHEADING_H
#define ATMOV_AUTOHEADING_H

#include "std.h"

extern void atmov_autoHeading_init(void);
extern void atmov_autoHeading_periodic(void);

#if !AUTOHEADING_PITCH_BOUNDARY
#define AUTOHEADING_PITCH_BOUNDARY RadOfDeg(10.)
#endif

#if !AUTOHEADING_ROLL_BOUNDARY
#define AUTOHEADING_ROLL_BOUNDARY RadOfDeg(30.)
#endif

#if !AUTOHEADING_YAW_INCREMENT
#define AUTOHEADING_YAW_INCREMENT ANGLE_BFP_OF_REAL(0.04)
#endif

#endif  /* ATMOV_AUTOHEADING_H */

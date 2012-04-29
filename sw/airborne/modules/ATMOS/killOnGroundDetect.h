#ifndef ATMOV_KOGD_H
#define ATMOV_KOGD_H

#include "std.h"

extern bool_t kill_on_ground_detect_enabled;

extern void atmov_killOnGroundDetect_init(void);
extern void atmov_killOnGroundDetect_periodic(void);
extern void ArmGroundDetectSwitch(void);
extern void DisarmGroundDetectSwitch(void);

#endif  /* ATMOV_KOGD_H */

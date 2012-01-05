/** \file navModeChange.h
 *  \brief functions to be used in the flight plan for in-nav-flight transition
 */

#ifndef NAVMODECHANGE_H
#define NAVMODECHANGE_H

#define TRANSITION_LIMIT_2D 50
#define TRANSITION_LIMIT_3D 50


#define NAV_ACTION_SURVEY 	0
#define NAV_ACTION_EIGHT	1
#define NAV_ACTION_WAYPOINT 2
#define NAV_ACTION_ROUTE	3
#define NAV_ACTION_CIRCLE	4
#define NAV_ACTION_UNKNOWN	5

/*#include "estimator.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"*/
#ifndef GUIDANCE_H_H
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#endif

#ifndef __TOYTRONICS_SETPOINT_H__
#include "firmwares/rotorcraft/toytronics/toytronics_setpoint.h"
#endif

extern int saveToChangeMode(void);
extern int setForward(void);
extern int setHoverForward(void);
extern int autoSetMode2d(void);
extern int autoSetMode3d(void);
#endif

/** \file navModeChange.c
 *  \brief functions to be used in the flight plan for in-nav-flight transition
 */

#ifndef NAVMODECHANGE_C
#define NAVMODECHANGE_C

#include "quadshot_nav_mode_change.h"
//#ifdef USE_TOYTRONICS

int saveToChangeMode(void) {
	return (autopilot_mode == AP_MODE_TUDELFT_QUADSHOT_NAV
			&& autopilot_detect_ground == false
			&& kill_throttle == false);
}

int setHoverForward(void) {
	if (!saveToChangeMode())
		return 0;

	toytronics_mode_enter(GUIDANCE_H_MODE_TOYTRONICS_HOVER_FORWARD);
	return 1;
}
int setForward(void) {
	if (!saveToChangeMode())
		return 0;

	toytronics_mode_enter(GUIDANCE_H_MODE_TOYTRONICS_FORWARD);
	return 1;
}

int determine_nav_action(void) {
	int navAction;
	if (nav_survey_active) {
		navAction = NAV_ACTION_SURVEY;
	}
	else if (eight_status) {
		navAction = NAV_ACTION_EIGHT;
	}
	else {
		switch(horizontal_mode) {
			case HORIZONTAL_MODE_WAYPOINT:
				navAction = NAV_ACTION_WAYPOINT;
				break;
			case HORIZONTAL_MODE_ROUTE:
				navAction = NAV_ACTION_ROUTE;
				break;
			case HORIZONTAL_MODE_CIRCLE:
				navAction = NAV_ACTION_CIRCLE;
				break;
			default:
				navAction = NAV_ACTION_UNKNOWN;
				break;
		}
	}
}

int autoSetMode(void) {
	switch (determine_nav_action) {
		case NAV_ACTION_SURVEY:
			break;
		case NAV_ACTION_EIGHT:
			break;
		case NAV_ACTION_WAYPOINT:
			break;
		case NAV_ACTION_ROUTE:
			break;
		case NAV_ACTION_CIRCLE:
			break;
		case NAV_ACTION_UNKNOWN:
			break;
		default:
			break;
	}
	RunOnceEvery(4,{ auto_set_mode_periodic_task; });
}

void auto_set_mode_periodic_task(void) {

}


/* deprecated
int autoSetMode2d(void) {
	if (compute_dist2_to_wp > TRANSITION_LIMIT_2D*TRANSITION_LIMIT_2D)
		return setForward();
	else
		return setHoverForward();
}
int autoSetMode3d(void) {
	if (compute_dist3_to_wp > TRANSITION_LIMIT_3D*TRANSITION_LIMIT_3D*TRANSITION_LIMIT_3D)
		return setForward();
	else
		return setHoverForward();
}
*/
float compute_dist2_to_wp(int wp) {
	float pw_x = waypoints[wp].x - estimator_x;
	float pw_y = waypoints[wp].y - estimator_y;
	return pw_x*pw_x + pw_y*pw_y;
}

float compute_dist3_to_wp(int wp) {
	float pw_x = waypoints[wp].x - estimator_x;
	float pw_y = waypoints[wp].y - estimator_y;
	float pw_z = waypoints[wp].z - estimator_z;
	return pw_x*pw_x + pw_y*pw_y + pw_z*pw_z;
}

float compute_dist2_between_wps(int wp1,int wp2) {
	float pw_x = abs(waypoints[wp1].x - waypoints[wp2].x);
	float pw_y = abs(waypoints[wp1].y - waypoints[wp2].y);
	return pw_x*pw_x + pw_y*pw_y;
}

float compute_dist3_between_wps(int wp1,int wp2) {
	float pw_x = abs(waypoints[wp1].x - waypoints[wp2].x);
	float pw_y = abs(waypoints[wp1].y - waypoints[wp2].y);
	float pw_y = abs(waypoints[wp1].z - waypoints[wp2].z);
	return pw_x*pw_x + pw_y*pw_y + pw_z*pw_z;
}

#endif
//#endif

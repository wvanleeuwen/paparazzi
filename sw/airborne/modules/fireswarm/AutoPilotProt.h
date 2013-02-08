/**
 * @brief
 * @file AutoPilotProt.h
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from
 * thread pools and TCP/IP components to control architectures and learning algorithms.
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright © 2012 Bart van Vliet <bart@almende.com>
 *
 * @author        Bart van Vliet
 * @date          Nov 13, 2012
 * @project
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef AUTOPILOTPROT_H_
#define AUTOPILOTPROT_H_

#include <stdint.h>

// ------------------------------------------------
// !! Assuming sizeof(float) == 4 !!
// ------------------------------------------------

// Maximum number of waypoints to be set
#define AP_PROT_WAYPOINTS_MAX 5

// Number of directions to give the waypoint bounds
#define AP_PROT_DIRECTIONS 16

// Length in bytes of a message that can be sent over the xbee
#define AP_PROT_XBEE_MSG_LENGTH 8

// Magic number for the header
#define AP_PROT_HEADER 65262 // Binary: 11111110 11101110, hex: FEEE

// Serial speed (75 110 300 1200 2400 4800 9600 19200 38400 57600 115200)
#define AP_PROT_SERIAL_SPEED 57600

// Checksum is sent after header and data.
// Checksum is 1 byte (uint8_t) and sum of header and data.


typedef uint16_t	AutoPilotMsgHeaderType;		// Type used for the magic number
typedef float		AutoPilotMsgPosType;		// In m <-- MERCATOR, origin far way! -- y=north, x=east, z=height
typedef float		AutoPilotMsgDistanceType;	// In m
typedef float		AutoPilotMsgSpeedType;		// In m/s
typedef uint32_t	AutoPilotMsgTimeType;		// milli seconds?
typedef float		AutoPilotMsgRotationType;	// In rad, 0=east


#pragma pack(1)

enum EAutoPilotMsgType
{
	AP_PROT_SET_MODE=0,		// Gumstix wants auto pilot to set mode, see EAutoPilotMode
//	AP_PROT_SET_FIELD,		// Set origin of coordinate system and borders of the field
	AP_PROT_SET_WAYPOINTS,	// Gumstix wants auto pilot to set new waypoints (overwrites old ones)
	AP_PROT_SET_LAND,		// Gumstix wants auto pilot to set new landing data
	AP_PROT_REQ_SENSORDATA,	// Gumstix requests sensor data
	AP_PROT_SENSORDATA,		// Sensor data from auto pilot
	AP_PROT_REQ_WP_STATUS,	// Gumstix requests status of waypoints
	AP_PROT_WP_STATUS,		// Status of waypoints from auto pilot
	AP_PROT_REQ_WP_BOUNDS,	// Gumstix requests bounds of waypoints
	AP_PROT_WP_BOUNDS,		// Bounds of waypoints from auto pilot
	AP_PROT_REQ_XBEE_MSG,	// Gumstix wants to send a msg over xbee
	AP_PROT_XBEE_MSG,		// Auto pilot received an xbee  msg for the gumstix
};

enum EAutoPilotMode
{
	AP_PROT_MODE_WP=0, 	// Follow given waypoints
	AP_PROT_MODE_LAND, 	// Perform a landing
	AP_PROT_MODE_HOME,	// Go back to base station (without landing)
	AP_PROT_MODE_STAY 	// Stay at current position (fly circles)
};

// Auto pilot state bitmask (1 is ok, 0 is error)
#define AP_PROT_STATE_AP_OUTER_LOOP		1
#define AP_PROT_STATE_AP_INNER_LOOP		2
#define	AP_PROT_STATE_AP_TAKEOVER		4	// 1 when accepting waypoints of gumstix (couldn't think of a better name, suggestions welcome)

// Server state bitmask (1 is ok, 0 is error)
#define AP_PROT_STATE_SERVO_PROP		1
#define AP_PROT_STATE_SERVO_WING_LEFT	2
#define AP_PROT_STATE_SERVO_WING_RIGHT	4
#define AP_PROT_STATE_SERVO_TAIL		8		// No Tail Servo

// Sensor state bitmask (1 is ok, 0 is error)
#define AP_PROT_STATE_SENSOR_COMPASS	1
#define AP_PROT_STATE_SENSOR_ACCELERO	2
#define AP_PROT_STATE_SENSOR_GPS		4
#define AP_PROT_STATE_SENSOR_WIND		8
#define AP_PROT_STATE_SENSOR_PRESSURE	16

enum EAutoPilotFlyState
{
	AP_PROT_FLY_STATE_IDLE=0,
	AP_PROT_FLY_STATE_TAKEOFF,
	AP_PROT_FLY_STATE_FLYING,
	AP_PROT_FLY_STATE_LANDING
};

enum EAutoPilotWpType
{
	AP_PROT_WP_LINE=0,
	AP_PROT_WP_ARC,
	AP_PROT_WP_CIRCLE
};

////////////////////////////////////////////////////
// Data Types

typedef struct AutoPilotMsgPositionStruct
{
	AutoPilotMsgPosType X; // In m
	AutoPilotMsgPosType Y; // In m
	AutoPilotMsgPosType Z; // In m
} AutoPilotMsgPosition;

// Unused
//struct AutoPilotMsgPositionGPS
//{
//	float GpsLat;	// In radians?
//	float GpsLong;	// In radians?
//	float GpsZ;		// In m above sea level?
//};


typedef struct AutoPilotMsgWpLineStruct
{
	AutoPilotMsgPosition		From;
	AutoPilotMsgPosition		To;
	// 2*12 = 24 B
} AutoPilotMsgWpLine;

typedef struct AutoPilotMsgWpArcStruct
{
	AutoPilotMsgPosition		Center;
	AutoPilotMsgDistanceType	Radius;
	AutoPilotMsgRotationType	AngleStart;		// Range: [0, 2pi]
	AutoPilotMsgRotationType	AngleArc;		// Range: (-2pi, 2pi), negative is clockwise
	// 12+4+4+4 = 24 B
} AutoPilotMsgWpArc;

typedef struct AutoPilotMsgWpCircleStruct
{
	AutoPilotMsgPosition		Center;
	AutoPilotMsgDistanceType	Radius;
	// 12+4 = 16 B
	uint8_t						Padding[8];
	// 16+8 = 24 B
} AutoPilotMsgWpCircle;

typedef struct AutoPilotMsgWpStruct
{
	uint32_t					Id;
	uint8_t						WpType;			// See EAutoPilotWpType
	AutoPilotMsgSpeedType		GroundSpeed;
	AutoPilotMsgSpeedType		VerticalSpeed;
	union
	{
		AutoPilotMsgWpLine		Line;
		AutoPilotMsgWpArc		Arc;
		AutoPilotMsgWpCircle	Circle;
	};
} AutoPilotMsgWp;

typedef struct AutoPilotMsgWayPointsStruct
{
	uint8_t						NumWayPoints;
	AutoPilotMsgWp				WayPoints[AP_PROT_WAYPOINTS_MAX];
} AutoPilotMsgWayPoints;


struct AutoPilotMsgMode
{
	uint8_t						Mode;	// See EAutoPilotMode
};

/////////////////////////////////////////////////////////
// Actual Messages

// Unused
//struct AutoPilotMsgField
//{
//	AutoPilotMsgPositionGPS		Origin;	// The point x=0, y=0
//	AutoPilotMsgPositionGPS		XBound; // The point x=max, y=0
//	AutoPilotMsgPositionGPS		YBound; // The point x=0, y=max
//	AutoPilotMsgPositionGPS		Home;	// Location of the ground station
//};


typedef struct AutoPilotMsgLandingStruct
{
	AutoPilotMsgPosition		LandPoint; // Z unused?
	AutoPilotMsgRotationType	LandHeading;
	bool						LandLeftTurn;
} AutoPilotMsgLanding;


typedef struct AutoPilotMsgSensorDataStruct
{
	uint8_t						FlyState;		// See EAutoPilotFlyState
	uint8_t						GPSState;		// 0 is none, 255 is best
	AutoPilotMsgTimeType		BatteryLeft;
	// No info about servo's
	uint8_t						ServoState;		// Bitmask, see defines AP_PROT_STATE_AP_*
	uint8_t						AutoPilotState;	// Bitmask, see defines AP_PROT_STATE_SERVO_*
	uint8_t						SensorState;	// Bitmask, see defines AP_PROT_STATE_SENSOR_*

	AutoPilotMsgPosition		Position;
	AutoPilotMsgSpeedType		GroundSpeed;
	AutoPilotMsgSpeedType		VerticalSpeed;
	AutoPilotMsgRotationType	Heading;
	AutoPilotMsgRotationType	Yaw;
	AutoPilotMsgRotationType	Pitch;
	AutoPilotMsgRotationType	Roll;
	AutoPilotMsgRotationType	WindHeading;
	AutoPilotMsgSpeedType		WindSpeed;
} AutoPilotMsgSensorData;


struct AutoPilotMsgWpStatus
{
	uint8_t						NumWaypoints;
	uint32_t					ID[AP_PROT_WAYPOINTS_MAX];
	AutoPilotMsgTimeType		ETA[AP_PROT_WAYPOINTS_MAX]; // ETA of 0 means no ETA available
};


// Bounds in several directions, starting at angle=0
struct AutoPilotMsgWpBounds
{
	AutoPilotMsgDistanceType	MinRadius[AP_PROT_DIRECTIONS];
	AutoPilotMsgSpeedType		MinSpeed[AP_PROT_DIRECTIONS];
	AutoPilotMsgSpeedType		MaxSpeed[AP_PROT_DIRECTIONS];
};


struct AutoPilotMsgXBeeMsgReq
{
	uint8_t						Buffer[AP_PROT_XBEE_MSG_LENGTH];
};


struct AutoPilotMsgXBeeMsg
{
	uint8_t						Buffer[AP_PROT_XBEE_MSG_LENGTH];
};


typedef struct AutoPilotMsgHeaderStruct
{
	uint16_t 					Header;		// Set this to some special number
	uint8_t 					MsgType; 	// EAutoPilotMsgType
	AutoPilotMsgTimeType		TimeStamp;
	uint8_t 					DataSize; 	// Length of the data following in bytes (can be 0)
} AutoPilotMsgHeader;

#pragma pack()

#endif /* AUTOPILOTPROT_H_ */

/* 
* CNavigation.h
*
* Created: 22/10/2015 16:05:08
* Author: Phil2
*/


#ifndef __CNAVIGATION_H__
#define __CNAVIGATION_H__

#include "Includes.h"

typedef struct
{
	int32_t      lat;
	int32_t      lon;
} GPS_COORDINATES;



typedef struct
{
	uint8_t		CrosstrackGain;		// Crosstrack gain *100 (1 - 0.01 , 100 - 1)
	uint16_t	WayPointRadius;				// way point radius, if we within this radius, then we considered that the wp is reached in cm
}NavigationParams_t;





typedef struct
{
	unsigned char WaypointNo;
	int32_t	lat;
	int32_t	lon;
}NewWaypoint_t;

typedef struct
{
	uint8_t  Action;		// Action to follow
	GPS_COORDINATES Coordinates;    //GPS coordinate of the way point
	int16_t	StartBearing;
	int32_t StartDistance;
	uint8_t Flag;

} NextWaypoint_t;

#define RUDDER_LOWPASS_F_FACTOR		2 //  LPF of 16

#define NAVIGATION_NORMAL	0
#define NAVIGATION_DEBUG	1


typedef enum {RC_CONTROL,AUTOPILOT_CONTROL} ControlType_t;

#define LOST_SIGNAL_RESET 1
#define LOST_SIGNAL_CHECK 0


//#define _X 1
//#define _Y 0
#define TO_WAYPOINT	true
#define TO_HOME	false

#define NO_LOCK			0
#define READINGS_FAIL	1
#define NAV_MPU_FAIL	2
#define AUTO_PILOT_FAIL	3
#define READ_OK			4

#define LATITUDE	0
#define LONGITUDE	1
#define GPS_LAG 0.5f

#define FRONT_DISTANCE_ALARM_LIMIT  60
#define SPIN_DISTANCE				45

// Convert deg*100 to radians
#define RADX100                    0.000174532925

#define WP_RADIUS					50



#define WP_FLAG_END         0xA5   //Flags that this is the last step
#define MISSION_FLAG_HOME        0x01   //Returned WP is the home position
#define MISSION_FLAG_HOLD        0x02   //Returned WP is the hold position
#define MISSION_FLAG_DO_LAND     0x20   //Land when reached desired point (used in RTH)

typedef enum
{
	NAV_MANOEUVER_SPEED,
	NAV_FULL_TURN_SPEED,
	NAV_NAVIGATION_SPEED,
	NAV_FAILSAFE_SPEED,
	NAV_REVERSE_SPEED,
	NAV_STOP
}NavigationSpeed_t;





typedef enum{SYSTEM_WORKING,NO_MISSION_FILE,WAYPOINT_PAST_SAFTEY_LIMIT,GPS_FAIL,WAYPOINT_READ_ERROR,MISSION_FINISHED,GPS_FIX_LOST,NAV_DISABLED_UNARMED,DOCKING}eNavigationError_t;

typedef enum 
{
	NONE,
	WAYPOINT,			//Set waypoint
	HOLD_UNLIM,			//Position hold unlimited
	HOLD_TIME,			//Hold for a predetermined time
	RTH,			    //Return to HOME
	SET_POI,            //Set POINT of interest (not implemented jet)
	JUMP,               //jump to the given WP and (number of times)
	SET_HEAD,           //fixes Boats heading 
	DOCK
}WaypointAction_t;


typedef enum
{
	PATH_FAR_RIGHT	= 0,
	PATH_MID_RIGHT	= 1,
	PATH_NEAR_RIGHT = 2,
	PATH_NEAR_LEFT  = 3,
	PATH_MID_LEFT	= 4,
	PATH_FAR_LEFT	= 5,
	PATH_REVERSE	= 6,
	PATH_CENTER		= 7
}SafePath_t;

typedef enum{eControlByRc,eStartNavigation,eOnRouteToNextWapoint,eReturnHome,eNoGpsLock,ePositionHold,eStartPanic,ePanic}NavigationState_t;


typedef struct
{
    float AngleDegree;      // target angle in degrees (0=north, 90=east)
    float Speed;			// speed in meters/second
    uint8_t RelativeAngle;	// 0 = absolute angle, 1 = relative angle to present angle
}NavCommand_t;


class CLeadFilter
{
public:
	CLeadFilter() : _last_velocity(0) {}
	double 	get_position(double pos, int16_t vel)
	{
		vel = (_last_velocity + vel) / 2;
		pos += vel;
		pos += (vel - _last_velocity);
		_last_velocity = vel;
		return pos;
	}
private:
	int16_t		_last_velocity;

};

typedef enum{WP_CORRUPT,WP_RTH,WP_END,WP_OK}WpErr_t;
  
class CNavigation
{
//variables
public:
	int m_RudderPosition;
//	NavInfo_t NavParameters;
	NavCommand_t NavCommand;
	unsigned char m_NextWaypointNo;
	double sqq;
	float rad_dist;
//	int16_t x_actual_speed;
//	int16_t y_actual_speed;

	uint32_t	 nav_loopTimer;							// used to track the elapsed time between GPS reads
	bool m_Debug;
	bool m_SlowDebugNavigation;
	
	// nav error for GUI  =====================================================
	unsigned char m_NavigationError;
	// 0 Navigation system is working
	// 1 Mission File Not Loaded
	// 2 Next waypoint distance is more than the safety limit aborting mission
	// 3GPS reception is compromised - pausing mission, BOAT IS ADRIFT
	// 4 Error while reading next waypoint from memory, aborting mission.
	// 5 Mission Finished.
	// 6 GPS fix lost, mission aborted
	// 7 Boat is disarmed, navigation engine disabled.
	// 8 Docking is in progress.
	//=========================================================================

	int32_t  GPS_WP_latitude,GPS_WP_longitude;			//Currently used WP
	uint8_t  GPS_WP_flags;

	static int32_t last_longitude;
	static int32_t last_latitude;
	double GPS_lead_latitude, GPS_lead_longitude;
	float  dTnav;										// Delta Time in milliseconds for navigation computations, updated with every good GPS read
	SafePath_t m_SafeCourse;
	double WayDist;
	double distance;
	int32_t m_GPSHoldPosition[2];
	NextWaypoint_t m_NextWaypoint;
	bool IsFirstWaypointLoaded;
	NavigationState_t m_NavigationState = eControlByRc;
	NavigationState_t m_LastNavigationState = eControlByRc;
	NavigationSpeed_t m_NavigationSpeedType;
	uint16_t m_NavigationSpeedActual;
	Direction_t m_DirectionOfTravel;
	uint32_t m_LostLockTimer = 0;

protected:
private:
	float m_DesiredHeading;
	float m_Steering;
//functions
public:
	CNavigation();
	~CNavigation();
//	bool Stop();
	bool SendWaypoint(unsigned char Waypoint,double Latitude,double Longitude);
	bool UpdateNavPidControlers(float P,float I, float D,unsigned char Imax );
	void Init(bool Debug);
	bool IsAtHomePosition();
	bool CheckForReachedWP();
	void SetDebugNavigation(bool OnOff);
	void SetDebugSlowNavigation(bool OnOff);
	bool SetPresentPositionWaypoint(unsigned char WaypointNo);
	void GetCrosstrackError(void);
	bool CheckNavigationRunning();
	bool UpdateRudder();
	int16_t GetHeadingError90(int16_t Target, int16_t ActualHeading);
	bool LostSignalReturnToHome();
	bool StartForceReturnToHome();
	bool StartLostSignalReturnHome();
//	void SetDebugReturnedtoHome(bool Start);
	void Update(uint32_t Time);
	bool AutoAvailable();
	void NavigationUpdate(uint32_t Time);
	void PositionHold();
	bool StartNavigation();
	void SetNavigationState(NavigationState_t State);
	bool UpdateBearings();
	void UpdateNextWaypointBearings();
	void UpdatePresentCoords();
	WpErr_t SetNextWaypoint(unsigned char WaypointNo);
	void ResetAutoPilot();
	bool StartReturnToHome();
	void SetNavigationSpeed(NavigationSpeed_t Speed);
	float CalcSteeringToHeading(float DesiredHeading);
	void SpinToHeading(float Heading);
	void UpdateSteeringPid(float DesiredHeading,uint32_t Time);
	float TurnToHeading(NavCommand_t Command);
	void TurnToHeading(float Heading);
	float TurnToHeading2(NavCommand_t Command);
	void TestSteering(uint32_t Time);
	void TestSteering();
	void TestFunctions();
protected:
private:
	CNavigation( const CNavigation &c );
	CNavigation& operator=( const CNavigation &c );
}; //CNavigation

#endif //__CNAVIGATION_H__

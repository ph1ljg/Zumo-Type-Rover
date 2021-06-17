/* 
* CNavigation.cpp
*
* Created: 22/10/2015 16:05:08
* Author: Phil2
*/

#include "Includes.h"

bool RetDebug;
CPid_PID SteeringPid;
float NDebug1;
float NDebug2;
float NDebug3;

float NPidError;


#include "CNavigationFunctions.h"
 CNavigationFunctions NavigationFunctions;
CNavigation::CNavigation()
{
	SetNavigationSpeed(NAV_FAILSAFE_SPEED);
} 


CNavigation::~CNavigation()
{
} 

bool CNavigation::CheckNavigationRunning()
{
	return(false);		
	
}


void CNavigation::Init(bool Debug)
{
	m_RudderPosition = 0;
	m_Debug = Debug;
	ResetAutoPilot();
	float FiltT = 10;
	float FiltE = 10;
	float FiltD = 10;
	float Imax = 30;
	float Dt	= 120;
	if (Config.m_CourseWaypoints[0].nLatitide == 0 )				// just for safety since home position must be set
	{
		Config.m_CourseWaypoints[0].nLatitide = NavigationFunctions.m_Bearings.nPresentLatitude;
		Config.m_CourseWaypoints[0].nLongitude = NavigationFunctions.m_Bearings.nPresentLongitude;
	}
	m_DirectionOfTravel = DIRECTION_FORWARD;
	SteeringPid(Config.m_MainConfig.PidValues[STEERING_RATE_PID].PVal, Config.m_MainConfig.PidValues[STEERING_RATE_PID].IVal, Config.m_MainConfig.PidValues[STEERING_RATE_PID].DVal,0,Imax, FiltT, FiltE, FiltD,Dt);
	//   void operator()(float p_val, float i_val, float d_val,float initial_ff, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt);

	SteeringPid.ResetAll();

//	m_Debug = true;;
}


void CNavigation::Update(uint32_t dt)
{
//	WpErr_t  RetVal;
	float TestHeading = 20.0;
	if(m_Debug)
		Gps.m_GpsStatus = GPS_OK_FIX_3D;
	
	if(Gps.IsGpsUsable())
	{
		UpdatePresentCoords();				// load GPS data

		if(Config.m_RunningFlags.HOME_LOCATION_SET)
			NavigationFunctions.LoadHomeWaypoint();
		NavigationFunctions.UpdateBearings(m_NextWaypoint);
		NavigationFunctions.UpdateHomeBearings();
	}
	
//	if(Ahrs.m_Status & AHRS_COMPASS_OK)
//		UpdateSteeringPid(TestHeading, dt);

	return;
	if(!m_NavigationState == eControlByRc)
		NavigationUpdate(dt);	

}


bool CNavigation::AutoAvailable()
{
	if(!Gps.IsGpsUsable())
		return(false);
	if(!Config.m_RunningFlags.HOME_LOCATION_SET)
		return(false);
	return(true);			
}

void CNavigation::NavigationUpdate(uint32_t Time)
{
	WpErr_t RetVal;	
	if(Gps.GetStatus() == GPS_NO_FIX ||Gps.GetStatus() == GPS_FAILED || Config.m_RunningFlags.HEAD_LIDAR_FAIL)
	{
		Zumo.SetArmedState(false);
		m_NavigationState = eControlByRc;
		return;
	}
	
	
	if( m_NavigationState == eOnRouteToNextWapoint)
	{
		if(NavigationFunctions.m_Bearings.NextWpDistanceCm <Config.m_NavigationConfig.WaypointRadius)									// load next waypoint
		{
			RetVal = SetNextWaypoint(++m_NextWaypointNo);		// set next waypoint
			switch(RetVal)
			{
			case WP_CORRUPT:
				Config.m_RunningFlags.WP_ERROR = true;
				m_NavigationError = WAYPOINT_READ_ERROR;
				SetNavigationSpeed(NAV_FAILSAFE_SPEED);
				m_NavigationState = eControlByRc;
				break;
			case WP_RTH:
				Config.m_RunningFlags.NAV_RETURN_TO_HOME = true;
				m_NavigationState = eReturnHome;
				break;
			case WP_END:
				m_NavigationError = DOCKING;
				Config.m_RunningFlags.AT_HOME_COORDINATES = true;
				SetNavigationSpeed(NAV_FAILSAFE_SPEED);
				m_NavigationState = eControlByRc;
				break;
			case WP_OK:
				break;
			}
		}
	}
	else if( m_NavigationState == eReturnHome)
	{
		if(NavigationFunctions.m_Bearings.NextWpDistanceCm <Config.m_NavigationConfig.WaypointRadius)
		{
			Zumo.SetArmedState(false);
			m_NavigationState = eControlByRc;
		}
	}
	else if( m_NavigationState == ePositionHold)
	{
		PositionHold();
	}	
	UpdateSteeringPid(10, Time);

}


void CNavigation::PositionHold()
{
	
}

 
bool CNavigation::StartNavigation()
{
	if(!Config.m_RunningFlags.HOME_LOCATION_SET)
	{
		return(false);
	}
	
	if(!Config.m_RunningFlags.MISSION_DATA_LOADED )
		return(false);
	
	ResetAutoPilot();
		
	Config.m_RealTimeGuiFlags &= ~DEBUG_START_NAV;
	m_NavigationSpeedType = NAV_NAVIGATION_SPEED;
	m_DirectionOfTravel = DIRECTION_FORWARD;
	SetNavigationSpeed(m_NavigationSpeedType);
		
	if(SetNextWaypoint(m_NextWaypointNo) == WP_OK)
	{
		m_NavigationState = eOnRouteToNextWapoint;
		Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = true;
		return(true);

	}
	return(false);
}



void CNavigation::SetNavigationState(NavigationState_t State)
{
	if(State == eStartPanic)
	{ 
		if(m_NavigationState != ePanic)
			m_NavigationState = eStartPanic;
		return;	
	}
	m_NavigationState = State;
}
 
 
 bool CNavigation::UpdateBearings()
 {
	UpdatePresentCoords();				// load GPS data
	NavigationFunctions.UpdateBearings(m_NextWaypoint);
	NavigationFunctions.UpdateHomeBearings();
	return(true);
 }
 
 


void CNavigation::UpdateNextWaypointBearings()
{
	CMyMath Math;
	NavigationFunctions.m_Bearings.NextWpDistanceCm = NavigationFunctions.DistanceCm(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon);
	NavigationFunctions.m_Bearings.NextWpBearing = NavigationFunctions.GetBearing(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon);
	NavigationFunctions.GetCrosstrackError(m_NextWaypoint);
	NavigationFunctions.m_Bearings.RelativeBearing = NavigationFunctions.GetRelativeBearing(NavigationFunctions.m_Bearings.NextWpBearing,Math.Round(Ahrs.GetHeading()));
}



void CNavigation::UpdatePresentCoords()
{
	if(m_Debug )
	{
		uint16_t Speed = 5;
		if(m_SlowDebugNavigation)
			Speed = 2;
		NavigationFunctions.DebugMoveAlongCourse(NavigationFunctions.m_Bearings,Ahrs.GetHeading(),Speed);
	}
	else
	{
		NavigationFunctions.m_Bearings.nPresentLatitude = Gps.m_Coords.Latitude;
		NavigationFunctions.m_Bearings.nPresentLongitude = Gps.m_Coords.Longitude;
	}
}

bool CNavigation::IsAtHomePosition()
{
	return(NavigationFunctions.m_Bearings.DistanceToHome <Config.m_NavigationConfig.WaypointRadius);
}

bool CNavigation::CheckForReachedWP()
{
	return((NavigationFunctions.m_Bearings.NextWpDistanceCm) <Config.m_NavigationConfig.WaypointRadius);
}






void CNavigation::SetDebugNavigation(bool OnOff)
{
	m_Debug = OnOff;
}

void CNavigation::SetDebugSlowNavigation(bool OnOff)
{
	m_SlowDebugNavigation = OnOff;
}

// set next Waypoint to aim to
WpErr_t CNavigation::SetNextWaypoint(unsigned char WaypointNo)
{
	CMyMath Math;
	WpErr_t RetVal = WP_CORRUPT;

	if(((Config.m_CourseWaypoints[WaypointNo].Action & RTH) == RTH)|| ((Config.m_CourseWaypoints[WaypointNo].Flag & WP_FLAG_END) == WP_FLAG_END))
	{
		if(Config.m_CourseWaypoints[WaypointNo].nLatitide != 0 && Config.m_CourseWaypoints[WaypointNo].nLongitude != 0)
		{
			m_NextWaypoint.Coordinates.lat = Config.m_CourseWaypoints[0].nLatitide;
			m_NextWaypoint.Coordinates.lon = Config.m_CourseWaypoints[0].nLongitude;
			m_NextWaypoint.StartDistance = NavigationFunctions.DistanceCm(m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon,NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude);
			m_NextWaypoint.StartBearing = NavigationFunctions.GetBearing(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon);
			UpdateNextWaypointBearings();
			RetVal = WP_RTH;
		}
		else
		{
			Alarms.SetWaypointAlm(WaypointNo);
		}
			
	}
	else if(Config.m_CourseWaypoints[WaypointNo].nLatitide != 0 && Config.m_CourseWaypoints[WaypointNo].nLongitude != 0)
	{
		m_NextWaypoint.Action = Config.m_CourseWaypoints[WaypointNo].Action;
		m_NextWaypoint.Flag = Config.m_CourseWaypoints[WaypointNo].Flag;

		m_NextWaypoint.Coordinates.lat = Config.m_CourseWaypoints[WaypointNo].nLatitide;
		m_NextWaypoint.Coordinates.lon = Config.m_CourseWaypoints[WaypointNo].nLongitude;
		m_NextWaypoint.StartDistance = NavigationFunctions.DistanceCm(m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon,NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude);
		m_NextWaypoint.StartBearing = NavigationFunctions.GetBearing(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,m_NextWaypoint.Coordinates.lat,m_NextWaypoint.Coordinates.lon);
		UpdateNextWaypointBearings();

		RetVal = WP_OK;
	}
	else
		Alarms.SetWaypointAlm(WaypointNo);
	
	return(RetVal);
}




void CNavigation::ResetAutoPilot()
{
	Config.m_RunningFlags.WP_ERROR = false;
	Config.m_RunningFlags.RETURN_HOME = false;
	Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = false;
	Config.m_RunningFlags.NAV_RETURN_TO_HOME = false;
//	AttitudeCtl.CalibrateLevel();
	m_NavigationError = NAV_DISABLED_UNARMED;
	m_NextWaypointNo = 1;
	IsFirstWaypointLoaded = false;
	m_NavigationState = eControlByRc;
}


bool CNavigation::StartReturnToHome()
{
	WpErr_t Error;
	Config.m_RunningFlags.NAV_RETURN_TO_HOME = true;
	m_NextWaypointNo = 0;

	if(!Config.m_RunningFlags.HOME_LOCATION_SET)
		return(false);
		
	if((!Gps.IsGpsUsable()) && !m_Debug)
		return(false);

	Error = SetNextWaypoint(0);			
	if(Error != WP_OK)
		return(false);

	m_NavigationState = eReturnHome;
	m_NavigationError = SYSTEM_WORKING;
	Config.m_RealTimeGuiFlags &= ~DEBUG_START_NAV;
	Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = true;
	SetNavigationSpeed(NAV_NAVIGATION_SPEED);
	return(true);
	
}




bool CNavigation::StartForceReturnToHome()
{
	if(Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT)
	{
		if(!Config.m_RunningFlags.RETURN_HOME)
		{
			m_NextWaypointNo = 0;
			SetNextWaypoint(0);
			return(true);
		}
	}
	else
		if(StartReturnToHome())
			return(true);
	return(false);
}




void CNavigation::SetNavigationSpeed(NavigationSpeed_t Speed)
{
	switch(Speed)
	{
	case NAV_NAVIGATION_SPEED:
		m_NavigationSpeedActual = Config.m_NavigationConfig.CruiseSpeed;
		break;
	case NAV_MANOEUVER_SPEED:
		m_NavigationSpeedActual = Config.m_NavigationConfig.CruiseThrottle;
		break;
	case NAV_FULL_TURN_SPEED:
		m_NavigationSpeedActual = Config.m_NavigationConfig.MaxTurnRate;
		break;
	case NAV_REVERSE_SPEED:
		m_NavigationSpeedActual = Config.m_NavigationConfig.ReverseSpeed;
		break;
	case NAV_FAILSAFE_SPEED:
		m_NavigationSpeedActual = Config.m_MainConfig.FailsafeThrottleValue;
		break;
	case NAV_STOP:
		m_NavigationSpeedActual = MOTOR_STOP_VALUE;
		break;
	default:
		m_NavigationSpeedActual = MOTOR_STOP_VALUE;
		break;
	}
}










//================================================================================================================================

//========================================== Rover ===============================================================================






// calculate steering output to drive towards desired heading
float CNavigation::CalcSteeringToHeading(float DesiredHeading)
{
	
	
	DesiredHeading = 360;
	float NewHeading = 0;//DesiredHeading+(AttitudeCtl.GeHeadingError(DesiredHeading)*RAD_TO_DEG); 
	DebugDisplay.Printf("New heading %f      Present Heading %f\n",NewHeading,Ahrs.GetHeading());
	return(NewHeading);
}





void CNavigation::SpinToHeading(float Heading)
{
	float Error = 0; //AttitudeCtl.GeHeadingError(Heading);
	
	if(Error >-0.0436332 && Error  < 0.0436332) // +-1.5 deg
	{
		DebugDisplay.Printf("Error %f      STOP    Heading %f\n",Error,Ahrs.GetHeading());
		Motors.AllMotorsStop();
	}
	else
 	{	if(is_positive(Error) )
		 {
//			MotorControl.SpinToAngle(200,DIR_SPIN_RIGHT,RAD_TO_DEG*Error);
			DebugDisplay.Printf("Error %f      Spin Right  Heading %f\n",Error,Ahrs.GetHeading());

		 }
		else
		{	
			DebugDisplay.Printf("Error %f      Spin Left    Heading %f\n",Error,Ahrs.GetHeading());
//			MotorControl.SpinToAngle(200,DIR_SPIN_LEFT,RAD_TO_DEG*Error);
		}
	}

}


void CNavigation::UpdateSteeringPid(float DesiredHeading,uint32_t Time)
{
	
//	float ErrVal;
	NPidError = NavigationFunctions.GetRelativeBearing(DesiredHeading,Ahrs.GetHeading());
	m_NavigationSpeedActual = 600;
//	ErrVal = Error;
	if(NPidError < 2.5 && NPidError > -2.5)	
	{				// Dead  Zone
		NPidError = 0;
		m_NavigationSpeedActual = 0;
	}
NDebug1 = NPidError;
	m_Steering =  SteeringPid.update_all(0, NPidError,false);
	
//	MotorControl.CalcSteering180(m_NavigationSpeedActual,m_Steering);

}

// void CNavigation::TurnToHeading(uint16_t Speed, float Heading)
// {
// 	m_DesiredHeading = Heading;
// 	MotorControl.TurnToDegree(600,m_Steering);
// }


// do_set_yaw_speed - turn to a specified heading and achieve a given speed
float CNavigation::TurnToHeading2(NavCommand_t Command)
{
	float DesiredHeading;

	// get final angle, 1 = Relative, 0 = Absolute
	if (Command.RelativeAngle > 0)  // relative angle
		DesiredHeading = wrap_180_cd(Ahrs.GetHeading() + Command.RelativeAngle * 100.0f);
	else		// absolute angle
		DesiredHeading = Command.AngleDegree * 100.0f;
	return(DesiredHeading);
}

//==================================================================================================================================

void CNavigation::TestSteering()
{
	

		CalcSteeringToHeading(10);
//		Core.delay(200);
	
// 		static bool Start = false;
// 	if(!Start)
// 	{
// 		m_NavigationSpeedActual = 800;
// 		m_DesiredHeading = 188;
// 		Start = true;
// 	}
// 	SpinToHeading(180);
// 	//UpdateSteeringPid();
}





//====================================================
// test func 52.429165,-1.946265 to 52.429358, -1.946273
// Distance:			0.02147 km (to 4 SF*)
// Initial bearing:	358°?33??08?
// Final bearing:		358°?33??08?
// Midpoint:			52°?25??45??N, 001°?56??47??W
//====================================================
void CNavigation::TestFunctions()
{
	float Distance;
	int32_t Bearing;
	CMyMath Math;
	NextWaypoint_t NextWaypoint;
	NavigationFunctions.m_Bearings.nPresentLatitude = -1946273;
	NavigationFunctions.m_Bearings.nPresentLongitude = 52429358;
	NextWaypoint.Coordinates.lat = 52429165;
	NextWaypoint.Coordinates.lon = -1946265;
	while(1)
	{
		Distance = NavigationFunctions.DistanceCm(NextWaypoint.Coordinates.lat,NextWaypoint.Coordinates.lon,NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude);
		Bearing = NavigationFunctions.GetBearing(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,52429165,-1946265);
		DebugDisplay.Printf(20,20,40,"Distance %f        Bearing %ld",  Math.Round(Distance/100),Bearing );
		Core.delay(500);
		NavigationFunctions.m_Bearings.nPresentLatitude = -19840407;
		NavigationFunctions.m_Bearings.nPresentLongitude = 524422680;
		NextWaypoint.Coordinates.lat = 524415758;
		NextWaypoint.Coordinates.lon = -19841774;
		
		Distance = NavigationFunctions.DistanceCm(NextWaypoint.Coordinates.lat,NextWaypoint.Coordinates.lon,NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude);
		Bearing = NavigationFunctions.GetBearing(NavigationFunctions.m_Bearings.nPresentLatitude,NavigationFunctions.m_Bearings.nPresentLongitude,524415758,-19841774);
		DebugDisplay.Printf(20,20,40,"Distance %f        Bearing %ld",  Math.Round(Distance/100),Bearing );
		Core.delay(500);
	}

	
}
	
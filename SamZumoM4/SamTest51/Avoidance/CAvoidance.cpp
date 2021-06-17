/* 
* CObsticalAvoidance.cpp
*
* Created: 04/06/2019 19:00:56
* Author: phil
*/

#include "Includes.h"

CProximity Proximity;

volatile uint8_t SCount  =0;

// default constructor
CAvoidance::CAvoidance()
{
	_accel_max = 3.0;	//Maximum acceleration with which obstacles will be avoided with. Set zero to disable acceleration limits
	_margin = 2.0f;
	
} //CObsticalAvoidance

// default destructor
CAvoidance::~CAvoidance()
{
} //~CObstacalAvoidance



void CAvoidance::Init()
{
	//TestSpin();
}

// Called by task manager every 60ms
void CAvoidance::Update()
{	
	
	if(Config.m_RunningFlags.IN_REVERSE)
	{
		CheckRearRanger();
	}
	else
	{
		if(HeadControl.GetSafeCourse(Sector))
			m_AvoidActive = false;
		else
			m_AvoidActive = true;	
		
	}
}


bool CAvoidance::GetCourse(float	 &Steer)
{
	 Steer = Sector.Angle*100;
	 return(m_AvoidActive);
}

// called from calc throttle
void CAvoidance::AdjustSpeed( float &Speed, float dt)
{
	return; //**********************************************************************************************************
	  float DesiredRate;
	  if(Sector.Angle == 0)
		return;							// no turn in progress
		
	  float Heading = Ahrs.GetHeading();
	  // convert heading and speed into velocity vector
	  Vector2f vel_xy;
	  vel_xy.x = cosf(Heading) * Speed * 100.0f;
	  vel_xy.y = sinf(Heading) * Speed * 100.0f;
	 
	Heading =  wrap_360(Heading+ (Sector.Angle));
	 
	  DesiredRate = AttitudeControl.get_turn_rate_from_heading(radians(Heading), 0); 
		Speed = min(DesiredRate,Speed);
}

// Adjust desired horizontal speed so that the vehicle stops before the fence or object
// accel (maximum acceleration/deceleration) is in m/s/s
// speed is in m/s
// kP should be zero for linear response, non-zero for non-linear response
// void CObstacleAvoidance::Adjust (kP,float accel_cmss,Vector2f &desired_vel_cms)
// {
// 	const float desired_speed = desired_vel_cms.length();
// 	const Vector2f stopping_point = position_xy + desired_vel_cms*(get_stopping_distance(kP, accel_cmss, desired_speed)/desired_speed);
//	const float stopping_point_dist_from_home = stopping_point.length();
//}





//  Computes the speed such that the stopping distance  of the vehicle will be exactly the input distance.
float CAvoidance::get_max_speed(float kP, float accel_cmss, float distance_cm, float dt)
{
	if (is_zero(kP))
	return safe_sqrt(2.0f * distance_cm * accel_cmss);
	else
	return  sqrt_controller(distance_cm, kP, accel_cmss, dt) ;
}


// Computes distance required to stop, given current speed.
float CAvoidance::get_stopping_distance(float kP, float accel_cmss, float speed_cms) const
{
	// avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
	if (accel_cmss <= 0.0f || is_zero(speed_cms))
	return 0.0f;
	
	if (kP <= 0.0f)			// handle linear deceleration
	return 0.5f * sq(speed_cms) / accel_cmss;

	// calculate distance within which we can stop accel_cmss/kP is the point at which velocity switches from linear to sqrt
	if (speed_cms < accel_cmss/kP)
		return speed_cms/kP;
	else
		return accel_cmss/(2.0f*kP*kP) + (speed_cms*speed_cms)/(2.0f*accel_cmss); // accel_cmss/(2.0f*kP*kP) is the distance at which we switch from linear to sqrt response
}





// Proportional controller with piecewise sqrt sections to constrain second derivative
float CAvoidance::sqrt_controller(float error, float p, float second_ord_lim, float dt)
{
	CMyMath Math;
	float correction_rate;
	if (is_negative(second_ord_lim) || is_zero(second_ord_lim)) 
		correction_rate = error * p;		// second order limit is zero or negative.

	else if (is_zero(p)) 
	{
		if (is_positive(error))				// P term is zero but we have a second order limit.
			correction_rate = safe_sqrt(2.0f * second_ord_lim * (error));

		else if (is_negative(error)) 
			correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error));
		else 
			correction_rate = 0.0f;
	} 
	else 
	{
		float linear_dist = second_ord_lim / sq(p);		// Both the P and second order limit have been defined.
		if (error > linear_dist) 
			correction_rate = safe_sqrt(2.0f * second_ord_lim * (error - (linear_dist / 2.0f)));

		else if (error < -linear_dist) 
			correction_rate = -safe_sqrt(2.0f * second_ord_lim * (-error - (linear_dist / 2.0f)));
		
		else 
			correction_rate = error * p;
	}
	if (!is_zero(dt)) 
	{
		// this ensures we do not get small oscillations by over shooting the error correction in the last time step.
		return Math.constrain_float(correction_rate, -fabsf(error) / dt, fabsf(error) / dt);
	} 
	else 
		return correction_rate;
}


	
void CAvoidance::CheckFrontRanger()
{
	static enum{Normal,FrontObstacle}FrontAvoidanceState = Normal;
	
	switch(FrontAvoidanceState)
	{
		case Normal:
			if(Config.m_RunningFlags.FRONT_DISTANCE_ALARM)
				FrontAvoidanceState = FrontObstacle;
			break;
		case FrontObstacle:
			if(HandleNavFrontObstruction())
				FrontAvoidanceState = Normal;
			else
				Config.m_RunningFlags.PANIC_MODE_TRIGGERED = true;	
			break;
	}	
}


void CAvoidance::CheckRearRanger()
{
		static enum{Normal,RearObstacle}RearAvoidanceState = Normal;

	switch(RearAvoidanceState)
	{
		case Normal:
// 			if(!SternModule.CheckForRearObstruction())
// 			{
// 				Rudder.SetRudderposition(AmidShips);
// 				Navigation.SetNavigationSpeed(MANOEUVER_SPEED,MOTOR_REVERSE);
// 				SternAvoidanceState = RearObstacle;
// 			}
			break;	
		case RearObstacle:
			if(HandleRearObstruction())
				RearAvoidanceState = Normal;
			break;					
	}
}


bool CAvoidance::SailSafeCourse()
{
	static enum{StartScan,WaitForScan,SetCourse}eSafeState = StartScan;
	switch(eSafeState)
	{
	case StartScan:
		Navigation.SetNavigationSpeed(NAV_MANOEUVER_SPEED);
//		Radar.SetState(eStartFrontScan);
		eSafeState = WaitForScan;
		return(false);
		break;
	case WaitForScan:
// 		if(!Radar.CheckForCompletedScan())
// 			return(false);
// 		if(Radar.m_SafeLidarCourse.Valid)
// 		{
// // 			if(Radar.m_SafeLidarCourse.Direction == NO_COURSE)
// // 				Config.m_RunningFlags.PANIC_MODE = true;
// // 			else	
// // 				Rudder.SteerToPath(Radar.m_SafeLidarCourse.Direction);
// // 			eSafeState = StartScan;
// 			Radar.SetState(eGetFrontRange);
// 		}
 		break;
	case SetCourse:
		break;	 
	}
	return(true);
}


bool CAvoidance::HandleNavFrontObstruction()
{
	static enum{Start,StartScan, WaitForScan,SetCourse,Avoid}eObsState;
//	static uint8_t ScanFailCount =0;
	Navigation.SetNavigationSpeed(NAV_STOP);
	switch(eObsState)
	{
		case Start:
//			ScanFailCount =0;
			eObsState = StartScan;
			break;
		case StartScan:
			Navigation.SetNavigationSpeed(NAV_MANOEUVER_SPEED);
//			Radar.SetState(eStartFrontScan);
			eObsState = WaitForScan;
				return(false);
			break;
		case WaitForScan:
//			if(!Radar.CheckForCompletedScan())
// 			{
// 				if(ScanFailCount++ > 50)
// 					Config.m_RunningFlags.PANIC_MODE = true;
// 				return(false);
// 			}
// 			if(Radar.m_SafeLidarCourse.Valid)
// 			{
// 				if(Radar.m_SafeLidarCourse.Direction == NO_COURSE)
// 					Config.m_RunningFlags.PANIC_MODE = true;
// 				else	
// 					Rudder.SteerToPath(Radar.m_SafeLidarCourse.Direction);
// 				Radar.SetState(eCheckProperCourseRange);
// 				eObsState =Avoid;
//			}
			break;
		case Avoid:
// 			if(!Radar.CheckForCompletedScan())
// 			{
// 				if(ScanFailCount++ > 50)
// 					Config.m_RunningFlags.PANIC_MODE = true;
// 				return(false);
// 			}
// 
// 			if(Radar.m_ProperCourseValue <Config.m_MainConfig.SafeFrontDistance)
// 			{
// 					Config.m_RunningFlags.FRONT_DISTANCE_ALARM = false;
// 					return(true);
// 			}
// 			else if(Radar.m_RadarValues.LidarRange <Config.m_MainConfig.SafeFrontDistance)
// 				eObsState = StartScan;
// 			else
// 				Radar.SetState(eCheckProperCourseRange);
			break;
		case SetCourse:
			break;	
	}

	return(true);
}



// every 60 ms
void CAvoidance::CheckRCFrontObstruction()
{
	static enum{eCheckForwardRange,eCheckForClearCourse,eGetSafePath}ForwardRangeCheck = eCheckForwardRange;
	static uint8_t FailCount = 0;
//	eRangeResult_t RetVal;
	switch(ForwardRangeCheck)
	{
		case eCheckForwardRange:
			if(Config.m_RunningFlags.FRONT_DISTANCE_ALARM)
			{
				Config.m_RunningFlags.AVOID_CONTROL_OVERIDE = true;
				ForwardRangeCheck = eGetSafePath;
			}
			break;
		case eGetSafePath:
			if(SailSafeCourse())
			{
				Config.m_RunningFlags.AVOID_CONTROL_OVERIDE = false;
				ForwardRangeCheck = eCheckForClearCourse;
			}
			else
				if(++FailCount >20)
					Config.m_RunningFlags.PANIC_MODE_TRIGGERED = true;
			break;
		case eCheckForClearCourse:
			if(!Config.m_RunningFlags.FRONT_DISTANCE_ALARM )
			{
	//			Rudder.SteerToPath(CENTER_PATH);
				ForwardRangeCheck = eCheckForwardRange;
			}	
	}
}



bool CAvoidance::GetSpinRoom()
{
// 	if(!GetRanges())
// 	return(false);
// 	if(m_MinimumRange > SPIN_DISTANCE)
// 	return(true);
	return(false);
}


bool CAvoidance::HandleRearObstruction()
{
//	UpdateRudder(CENTER_PATH);
	Navigation.SetNavigationSpeed(NAV_MANOEUVER_SPEED);
 return(true);
}


void CAvoidance::TestSpin()
{
	Config.m_FunctionFlags.PassThroughFlag = true;
//	MotorControl.SpinToAngle(1000,DIR_SPIN_RIGHT,1);
	while(1)
	{
		Core.delay(1000);
	}
}



/* 
* CAttitudeControl.cpp
*
* Created: 11/03/2021 18:06:32
* Author: philg
*/
#include "Includes.h"

extern CGps Gps;
extern CImu Imu;
extern CAhrs Ahrs;

 extern CCmps12 Cmps12;
CPid_P _steer_angle_p(ATT_CONTROL_STEER_ANG_P);
// default constructor
CAttitudeControl::CAttitudeControl()
{
} //CAttitudeControl

// default destructor
CAttitudeControl::~CAttitudeControl()
{
} //~CAttitudeControl

void CAttitudeControl::Init()
{
	ThrottleSpeedPid(Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].PVal, Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].IVal, Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].DVal,
				 0.0f, ATT_CONTROL_THR_SPEED_IMAX, 0.0f, ATT_CONTROL_THR_SPEED_FILT, 0.0f, ATT_CONTROL_DT);

	SteeringRatePid(Config.m_MainConfig.PidValues[STEERING_RATE_PID].PVal, Config.m_MainConfig.PidValues[STEERING_RATE_PID].IVal, Config.m_MainConfig.PidValues[STEERING_RATE_PID].DVal,
			 ATTCONTROL_STEER_RATE_FF, ATTCONTROL_STEER_RATE_IMAX, 0.0f, ATTCONTROL_STEER_RATE_FILT, 0.0f, ATTCONTROL_DT),

	_stop_speed = ATTCONTROL_STOP_SPEED_DEFAULT;
	ThrottleDecelMax = 0;	// Speed control and deceleration maximum in m/s/s.  0 to use ATC_ACCEL_MAX for deceleration
	_throttle_accel_max = ATTCONTROL_THR_ACCEL_MAX;
	_steer_accel_max = ATTCONTROL_STEER_ACCEL_MAX;// 30 rpm
}

// get acceleration limited desired speed
float CAttitudeControl::GetDesiredSpeedAccelLimited(float DesiredSpeed, float dt) 
{
	CMyMath Math;
	
	if (!SpeedControlActive() || !is_positive(	Config.m_MainConfig.MaxThrottleAcceleration))            // return input value if no recent calls to speed controller apply no limiting when ATC_ACCEL_MAX is set to zero
		return DesiredSpeed;

	dt = Math.constrain_float(dt, 0.0f, 1.0f);                          // sanity check dt
	
	float PreviousSpeed = LastDesiredSpeed;                                    // use previous desired speed as basis for accel limiting
	LastDesiredSpeed = DesiredSpeed;
	if (!SpeedControlActive())                                              // if no recent calls to speed controller limit based on current speed
		PreviousSpeed = GetForwardSpeed();

	
	float SpeedChangeMax;                                                  // acceleration limit desired speed
	if (fabsf(DesiredSpeed) < fabsf(LastDesiredSpeed) && is_positive(ThrottleDecelMax))
		SpeedChangeMax = ThrottleDecelMax * dt;
	else
		SpeedChangeMax = _throttle_accel_max * dt;
	return Math.constrain_float(DesiredSpeed, PreviousSpeed - SpeedChangeMax, PreviousSpeed + SpeedChangeMax);
}



// return a throttle output from -1 to +1 given a desired speed in m/s (use negative speeds to travel backwards) 
// motor_limit should be true if motors have hit their upper or lower limits
// cruise speed should be in m/s, cruise throttle should be a number from -1 to +1
bool CAttitudeControl::get_throttle_out_speed(float desired_speed, float cruise_speed, float cruise_throttle, float dt,float &Throttle)
{
	CMyMath Math;
	// sanity check dt
	dt = Math.constrain_float(dt, 0.0f, 1.0f);

	// get speed forward
	float speed =  GetForwardSpeed();;

	// if not called recently, reset input filter and desired speed to actual speed (used for accel limiting)
	if (!SpeedControlActive())
	{
		ThrottleSpeedPid.ResetFilter();
		ThrottleSpeedPid.ResetI();
		DesiredSpeed = speed;
	}
	m_SpeedLastMs = Core.millis();
	
	DesiredSpeed = GetDesiredSpeedAccelLimited(desired_speed, dt);     // acceleration limit desired speed
	
	ThrottleSpeedPid.set_dt(dt);                                     // set PID's dt
	
	float throttle_base = 0.0f;                                         // calculate base throttle (protect against divide by zero)
	if (is_positive(cruise_speed) && is_positive(cruise_throttle))
	throttle_base = desired_speed * (cruise_throttle / cruise_speed);

	// calculate final output

//	DebugDisplay.Printf("Des %f    Speed %f\n",desired_speed, speed);
	float throttle_out = ThrottleSpeedPid.update_all(desired_speed, speed, (ThrottleLowLimit || ThrottleHighLimit));
	throttle_out += ThrottleSpeedPid.GetFF();
	throttle_out += throttle_base;

	// clear local limit flags used to stop i-term build-up as we stop reversed outputs going to motors
	ThrottleLowLimit = false;
	ThrottleHighLimit = false;
//	DebugDisplay.Printf("Des %f    Sp %f   out %f\n",desired_speed,speed,throttle_out);

	// protect against reverse output being sent to the motors unless braking has been enabled
	if (!BrakeEnable)
	{
		// if both desired speed and actual speed are positive, do not allow negative values
		if ((desired_speed >= 0.0f) && (throttle_out <= 0.0f))
		{
			throttle_out = 0.0f;
			ThrottleLowLimit = true;
		}
		else if ((desired_speed <= 0.0f) && (throttle_out >= 0.0f))
		{
			throttle_out = 0.0f;
			ThrottleHighLimit = true;
		}
	}

	// final output throttle in range -1 to 1
	Throttle = throttle_out;  
	return(true) ;
}

// return a throttle output from -1 to +1 to perform a controlled stop.  returns true once the vehicle has stopped
float CAttitudeControl::get_throttle_out_stop( float cruise_speed, float cruise_throttle, float dt, bool& stopped)
{
	// get current system time
	const uint32_t now = Core.millis();

	// if we were stopped in the last 300ms, assume we are still stopped
	bool _stopped = (_stop_last_ms != 0) && (now - _stop_last_ms) < 300;

	// get deceleration limited speed
	float desired_speed_limited = GetDesiredSpeedAccelLimited(0.0f, dt);

	// get speed forward
	float speed;
	speed = GetForwardSpeed();

	// if desired speed is zero and vehicle drops below _stop_speed consider it stopped
	if (is_zero(desired_speed_limited) && fabsf(speed) <= fabsf(_stop_speed))
		_stopped = true;

	// set stopped status for caller
	stopped = _stopped;

	// if stopped return zero
	if (stopped)
	{
		// update last time we thought we were stopped
		_stop_last_ms = now;
		// set last time speed controller was run so accelerations are limited
		m_SpeedLastMs = now;
		return 0.0f;
	}

	// clear stopped system time
	_stop_last_ms = 0;
	float Thottle;
	// run speed controller to bring vehicle to stop
	 get_throttle_out_speed(desired_speed_limited, cruise_speed, cruise_throttle, dt,Thottle);
	return Thottle;
}




// check if speed controller active
bool CAttitudeControl::SpeedControlActive() const
{
	// active if there have been recent calls to speed controller
	if ((m_SpeedLastMs == 0) || (Core.millis() - m_SpeedLastMs) > ATTCONTROL_TIMEOUT_MS)
	return false;

	return true;
}



// get forward speed in m/s (earth-frame horizontal velocity but only along vehicle x-axis).  returns true on success
float  CAttitudeControl::GetForwardSpeed() const
{
	int32_t Head;
	float GpsSpeed =0;
	float Speed;
	bool GotGpsSpeed = false; 
	Speed = Imu.GetForwardSpeed();
//	DebugDisplay.Printf(" Speed %f\n",Speed);

	// use less accurate GPS, assuming entire length is along forward/back axis of vehicle
	if (Config.m_RunningFlags.GPS_3D_LOCK)
	{
		Head = wrap_180_cd(Ahrs.GetHeadingCD() - Gps.GroundCourseCD());
		if (abs(Head) <= 9000)
			GpsSpeed = Gps.m_GpsReadings.GroundSpeed;
		else
			GpsSpeed = -Gps.m_GpsReadings.GroundSpeed;
		GotGpsSpeed = true;	
	}
	if(GotGpsSpeed)
		Speed = max(GpsSpeed,Speed);

	return(Speed);
//	return(0.4);
}

float CAhrs::GetRoll()
{
	return(m_AhrsValues.Eular.y);
}

float CAhrs::GetPitch()
{
	return(m_AhrsValues.Eular.x);
}

float CAhrs::GetYawR()
{
	return(DEGREES_TO_RADIANS(m_AhrsValues.Eular.z));
}


// return a steering servo output from -1 to +1 given a heading in radians set rate_max_rads to a non-zero number to apply a limit on the desired turn rate
// return value is normally in range -1.0 to +1.0 but can be higher or lower
float CAttitudeControl::get_steering_out_heading(float heading_rad, float rate_max_rads, bool Limit, float dt)
{
	// calculate the desired turn rate (in radians) from the angle error (also in radians)
	float desired_rate = get_turn_rate_from_heading(heading_rad, rate_max_rads);

	return GetSteeringOutOfRate(desired_rate, Limit, dt);
}

// return a desired turn-rate given a desired heading in radians
float CAttitudeControl::get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const
{
	CMyMath Math;
	const float yaw_error = wrap_PI(heading_rad - Ahrs.GetYawR());

	float desired_rate = _steer_angle_p.Get_p(yaw_error);       // Calculate the desired turn rate (in radians) from the angle error (also in radians)
	
	if (is_positive(rate_max_rads))                             // limit desired_rate if a custom pivot turn rate is selected, otherwise use ATC_STR_RAT_MAX
		desired_rate = Math.constrain_float(desired_rate, -rate_max_rads, rate_max_rads);

	return desired_rate;
}

// return a steering servo output from - 1.0 to + 1.0 given a desired lateral acceleration rate in m / s / s.
// positive lateral acceleration is to the right.
float CAttitudeControl::get_steering_out_lat_accel(float desired_accel, bool Limit, float dt)
{
	//_steer_lat_accel_last_ms = Core.millis();
	_desired_lat_accel = desired_accel;

	// get speed forward
	float speed  = GetForwardSpeed();

	const float desired_rate = get_turn_rate_from_lat_accel(desired_accel, speed);

	return GetSteeringOutOfRate(desired_rate, Limit, dt);
}


// calculate the turn rate in rad/sec given a lateral acceleration (in m/s/s) and speed (in m/s)
float CAttitudeControl::get_turn_rate_from_lat_accel(float lat_accel, float speed) const
{
	// enforce minimum speed to stop oscillations when first starting to move
	if (fabsf(speed) < ATT_CONTROL_STEER_SPEED_MIN)
	{
		if (is_negative(speed))
			speed = -ATT_CONTROL_STEER_SPEED_MIN;
		else
			speed = ATT_CONTROL_STEER_SPEED_MIN;
	}
	return lat_accel / speed;
}




// return a steering servo output from -1 to +1 given a  desired yaw rate in radians/sec. Positive yaw is to the right.
float CAttitudeControl::GetSteeringOutOfRate(float desired_rate, bool Limit, float dt)
{
	uint32_t Period;
	CMyMath Math;
	
	if(desired_rate ==0)
		return(0);
	// sanity check dt
		dt = Math.constrain_float(dt, 0.0f, 1.0f);

	// if not called recently, reset input filter and desired turn rate to actual turn rate (used for accel limiting)
	const uint32_t now = Core.millis();

	if ((_steer_turn_last_ms == 0) || (now -_steer_turn_last_ms > ATTCONTROL_TIMEOUT_MS))
	{
		SteeringRatePid.ResetFilter();
		SteeringRatePid.ResetI();
		_desired_turn_rate = Ahrs.get_yaw_rate_earth();
	}
	_steer_turn_last_ms = now;

	// acceleration limit desired turn rate
	if(is_positive(_steer_accel_max))
	{
		const float change_max = radians(_steer_accel_max) * dt;
		desired_rate = Math.constrain_float(desired_rate, _desired_turn_rate - change_max, _desired_turn_rate + change_max);
	}
//	GuiFunctions.SetPidOutputValues(desired_rate,GUI_PID_OUTPUT_ERROR ); 
	_desired_turn_rate = desired_rate;

	// rate limit desired turn rate
	if (is_positive(_steer_rate_max))
	{
		const float steer_rate_max_rad = radians(_steer_rate_max);
		_desired_turn_rate = Math.constrain_float(_desired_turn_rate, -steer_rate_max_rad, steer_rate_max_rad);
	}

    // Get  the steering rate  (rad/sec)use  earth frame to allow for rover leaning over in hard corners
    const float MeasuredRate = ( Ahrs.get_yaw_rate_earth());

	// set PID's dt
	SteeringRatePid.set_dt(dt);
	
	float Output = SteeringRatePid.update_all(_desired_turn_rate,MeasuredRate , Limit);
	 Output += SteeringRatePid.GetFF(); 
	// constrain and return final output
	Output = Math.constrain_float(Output,-1,1);
	return(Output);
}








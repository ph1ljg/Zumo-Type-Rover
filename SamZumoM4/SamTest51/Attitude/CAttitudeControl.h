/* 
* CAttitudeControl.h
*
* Created: 11/03/2021 18:06:32
* Author: philg
*/


#ifndef __CATTITUDECONTROL_H__
#define __CATTITUDECONTROL_H__
#define ATTCONTROL_TIMEOUT_MS        200
#define ATT_CONTROL_STEER_ANG_P       2.50f
#define ATT_CONTROL_DT                0.02f
#define ATT_CONTROL_TIMEOUT_MS        200
#define ATT_CONTROL_STEER_SPEED_MIN   1.0f		// minimum speed in m/s
#define ATTCONTROL_STEER_ACCEL_MAX   180.0f

class CAttitudeControl
{
	//variables
public:
	bool BrakeEnable = false;
	float ThrottleDecelMax ;	// Speed control and deceleration maximum in m/s/s.  0 to use ATC_ACCEL_MAX for deceleration
	float _throttle_accel_max ;
	float DesiredSpeed = 0;       // desired speed in m/s
	bool  ThrottleLowLimit = false;   // throttle output was limited from going too low (used to reduce i-term buildup)
	bool  ThrottleHighLimit = false;  // throttle output was limited from going too high (used to reduce i-term buildup)
	CPid_PID   SteeringRatePid;       // steering rate controller
	CPid_PID ThrottleSpeedPid;
	float   LastDesiredSpeed;        // last recorded desired speed
protected:
private:
	uint32_t m_SpeedLastMs;        // system time of last call to get_throttle_out_speed
	float ThrottleAccelMax ;

	// steering control
//	uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
	uint32_t _steer_turn_last_ms;		// system time of last call to steering rate controller
	float    _desired_lat_accel;		// desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
	float    _desired_turn_rate;		// desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller
	CPid_P     _steer_angle_p;        // steering angle controller
	uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
	float _stop_speed;
	float _steer_accel_max;
	float _steer_rate_max;       // steering rate control maximum rate in deg/s

	//functions
public:
	CAttitudeControl();
	~CAttitudeControl();
	void Init();
	float GetDesiredSpeedAccelLimited(float desired_speed, float dt) ;
	bool get_throttle_out_speed(float desired_speed, float cruise_speed, float cruise_throttle, float dt,float &Throttle);
	bool SpeedControlActive() const;
	float GetForwardSpeed() const;
	float get_steering_out_heading(float heading_rad, float rate_max_rads, bool Limit, float dt);
	float get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const;
	float get_steering_out_lat_accel(float desired_accel, bool Limit, float dt);
	float get_turn_rate_from_lat_accel(float lat_accel, float speed) const;
	float GetSteeringOutOfRate(float desired_rate, bool Limit, float dt);
	float get_throttle_out_stop(float cruise_speed, float cruise_throttle, float dt, bool& stopped);
protected:
private:
	CAttitudeControl(const CAttitudeControl& c);
	CAttitudeControl& operator=(const CAttitudeControl& c);

}; //CAttitudeControl

#endif //__CATTITUDECONTROL_H__

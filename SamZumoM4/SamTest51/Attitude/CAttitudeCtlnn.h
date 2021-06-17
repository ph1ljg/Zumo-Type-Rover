/* 
* CImuCompass.h
*
* Created: 02/04/2020 17:51:28
* Author: Admin
*/


#ifndef __CIMUCOMPASS_H__
#define __CIMUCOMPASS_H__

#include "cmymath.h"

#define MAG_CALIBRATION_MASK (3<<0)
#define ACC_CALIBRATION_MASK (3<<2)
#define GIR_CALIBRATION_MASK (3<<4)

#define G_Dt                            0.02f

#define ATT_CONTROL_STEER_ANG_P       2.50f
#define STEER_RATE_FF     0.20f	// FF Gain which produces an output value that is proportional to the demanded input
#define STEER_RATE_P      0.20f
#define STEER_RATE_I      0.20f
#define STEER_RATE_IMAX   1.00f
#define STEER_RATE_D      0.00f
#define STEER_RATE_FILT   10.00f
#define STEER_RATE_DT     0.20f


#define ATT_CONTROL_STEER_RATE_MAX    360.0f
#define ATTCONTROL_STEER_ACCEL_MAX   180.0f
#define ATT_CONTROL_THR_SPEED_P       0.20f
#define ATT_CONTROL_THR_SPEED_I       0.20f
#define ATT_CONTROL_THR_SPEED_IMAX    1.00f
#define ATT_CONTROL_THR_SPEED_D       0.00f
#define ATT_CONTROL_THR_SPEED_FILT    10.00f
#define AR_ATTCONTROL_PITCH_THR_P       1.80f
#define AR_ATTCONTROL_PITCH_THR_I       1.50f
#define AR_ATTCONTROL_PITCH_THR_D       0.03f
#define AR_ATTCONTROL_PITCH_THR_IMAX    1.0f
#define AR_ATTCONTROL_PITCH_THR_FILT    10.0f
#define AR_ATTCONTROL_BAL_SPEED_FF      1.0f
#define ATT_CONTROL_TIMEOUT_MS        200


// minimum speed in m/s
#define ATTITUDE_STEER_SPEED_MIN   1.0f





class CAttitudeCtl
{
//variables
public:
	float m_YawP = 1;
	   
    // steering control
    uint32_t _steer_lat_accel_last_ms;  // system time of last call to lateral acceleration controller (i.e. get_steering_out_lat_accel)
    uint32_t _steer_turn_last_ms;   // system time of last call to steering rate controller
    float    _desired_lat_accel;    // desired lateral acceleration (in m/s/s) from latest call to get_steering_out_lat_accel (for reporting purposes)
    float    _desired_turn_rate;    // desired turn rate (in radians/sec) either from external caller or from lateral acceleration controller

    // throttle control
    uint32_t _speed_last_ms;        // system time of last call to get_throttle_out_speed
    float    _desired_speed;        // last recorded desired speed
    float _desired_yaw_cd;
	uint32_t _stop_last_ms;         // system time the vehicle was at a complete stop
    bool     _throttle_limit_low;   // throttle output was limited from going too low (used to reduce i-term buildup)
    bool     _throttle_limit_high;  // throttle output was limited from going too high (used to reduce i-term buildup)
	float _steer_accel_max;
	float _steer_rate_max;
protected:
private:
	int16_t m_PitchOffset;
	int16_t m_RollOffset;
//functions
public:
	CAttitudeCtl();
	~CAttitudeCtl();
	void Init();
	void Update();
	void calc_steering_to_heading(float desired_heading_cd, float rate_max_degs = 0.0f);
	void CalibrateLevel();
	void do_nav_set_yaw_speed(float angle,bool Relative);
	void do_nav_set_yaw_speed(float angle,bool Relative,float speed);
	void UpDateCompass();
	void CheckCalibration();
	void SaveCompassCalibration();
	float GetTurnRateFromLatAccel(float LatAccel, float Speed) const;
	float get_steering_out_heading(float heading_rad, float rate_max_rads, bool motor_limit_left, bool motor_limit_right, float dt);
	float get_turn_rate_from_heading(float heading_rad, float rate_max_rads) const;
	float get_steering_out_rate(float desired_rate, bool motor_limit_left, bool motor_limit_right, float dt);
	float GeHeadingError(float Heading);
	float GetTurnRateFromHeading(float heading_rad, float rate_max_rads) const;
	void GetLateralAcceleration();
	bool GetForwardSpeed(float &speed) const;
	void TestSpeed();
protected:
private:
	CAttitudeCtl( const CAttitudeCtl &c );
	CAttitudeCtl& operator=( const CAttitudeCtl &c );
}; //CImuCompass

#endif //__CIMUCOMPASS_H__

/* 
* CSteering.h
*
* Created: 29/10/2020 10:09:21
* Author: philg
*/


#ifndef __CSTEERING_H__
#define __CSTEERING_H__


class CSteering
{
//variables
public:
	float m_SteerDebug;
	bool m_SteerLimit;			// we have reached the steering controller's right most limit or most limit
	float _speed_scale_base;  // speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
protected:
private:
	uint16_t m_ServoChannel;
	float _desired_lat_accel;   // desired lateral acceleration calculated from pilot steering input
	bool    _scale_steering; // true if we should scale steering by speed or angle
	float m_Steer_dt =0;

//functions
public:
	CSteering();
	~CSteering();
	void Update(uint32_t TimeUs);
	float calc_steering_from_lateral_acceleration(float lat_accel);
	void set_steering(float steering_value,float Throttle);
	void Output(float ground_speed, float steering);
	void GetRadioCDesiredSteeringAndSpeed(float& steering_out, float& speed_out);
	float CalcSpeedMax(float cruise_speed, float cruise_throttle) const;
	void UpdateSkidRcControl(uint32_t dt);
	void OutputSkidSteering(int16_t steering, uint16_t throttle, float dt);
//	void SetLimitsFromInput(uint8_t Motor,uint8_t armed, float steering, float throttle);
	void UpdateRcMecanControl();
	void CalculateMecanSteering(int MotorSpeed,float Angle,float SpeedOfChange );
	void Test();
	void TurnTest();
protected:
    float SteeringThrottleMix; // Steering vs Throttle prioritisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid in Skid Steering 

private:
	CSteering( const CSteering &c );
	CSteering& operator=( const CSteering &c );
	void get_pilot_input(float& steering_out, float& throttle_out);							// The navigation code will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns

}; //CSteering

#endif //__CSTEERING_H__

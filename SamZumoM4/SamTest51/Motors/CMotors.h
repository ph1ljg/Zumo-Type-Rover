/*
* CmotorControl.h
*
* Created: 20/05/2020 12:34:51
* Author: philg
*/
/*
Red: Motor power + positive (change can control motor forward and reverse)
Black: Code power - negative (positive and negative can not be connected incorrectly 3.3-5V)
Yellow: Signal feedback (11 signals from the motor)
Green: Signal feedback (11 signals from the motor)
Blue: Encoder power + positive (positive and negative can not be connected incorrectly 3.3-5V)
White: Motor power - negative (change can control motor forward and reverse)

*/

#ifndef __CMOTORCONTROL_H__
#define __CMOTORCONTROL_H__


// wheel rate control defaults
#define AP_WHEEL_RATE_MAX_DEFAULT   12.0f  // maximum wheel rotation rate in rad/sec (about 115rpm, 687deg/sec)
#define WHEEL_RATE_CONTROL_FF    8.00f
#define WHEEL_RATE_CONTROL_P     2.00f
#define WHEEL_RATE_CONTROL_I     2.00f
#define WHEEL_RATE_CONTROL_IMAX  1.00f
#define WHEEL_RATE_CONTROL_D     0.01f
#define WHEEL_RATE_CONTROL_FILT  0.00f
#define WHEEL_RATE_CONTROL_DT    0.02f
#define AP_WHEEL_RATE_CONTROL_TIMEOUT_MS 200


#define STEERING_STANDARD	1
#define STEERING_SKID		2
#define STEERING_MECAN		3


#define WHEELSPAN 180
#define MINCHECK 1250
#define MAXCHECK 1850

#define MOTOR_STOP_VALUE 0
#define MOTORS_STOPPED	0
#define MOTORS_NORMAL	1

#define MOTOR_RAW_DRIVE_PWM_MIN 0 //TThe minimum value the motor output can be..
#define MOTOR_RAW_DRIVE_PWM_MAX 5000  //The maximum value the motor output can be.
#define MOTOR_RAW_DRIVE_PWM_MID MOTOR_RAW_DRIVE_PWM_MAX/2

#define MOTOR_RC_MIN_PWM 1000 //
#define MOTOR_RC_MAX_PWM 2000 //
#define MOTOR_RC_MID_PWM MOTOR_RC_MAX_PWM/2


#ifdef FOUR_WHEEL_ZUMO
	#define FRONT_RIGHT_FORWARD	false
	#define FRONT_RIGHT_REVERSE	true
	#define FRONT_LEFT_FORWARD	true
	#define FRONT_LEFT_REVERSE	    false
	#define REAR_RIGHT_FORWARD	false
	#define REAR_RIGHT_REVERSE		true
	#define REAR_LEFT_FORWARD		true
	#define REAR_LEFT_REVERSE		false
#endif
#ifdef TWO_WHEEL_ZUMO
	#define FRONT_RIGHT_FORWARD	false
	#define FRONT_RIGHT_REVERSE	true
	#define FRONT_LEFT_FORWARD	true
	#define FRONT_LEFT_REVERSE	    false
	#define REAR_RIGHT_FORWARD	true
	#define REAR_RIGHT_REVERSE		false
	#define REAR_LEFT_FORWARD		false
	#define REAR_LEFT_REVERSE		true
#endif
#ifdef  LARGE_ROVER
	#define FRONT_RIGHT_FORWARD	false
	#define FRONT_RIGHT_REVERSE	true
	#define FRONT_LEFT_FORWARD	true
	#define FRONT_LEFT_REVERSE	    false
	#define REAR_RIGHT_FORWARD	false
	#define REAR_RIGHT_REVERSE		true
	#define REAR_LEFT_FORWARD		true
	#define REAR_LEFT_REVERSE		false
#endif	

#define SPEED_MPH			1
#define SPEED_RAD_SEC		2
#define SPEED_PWM			3

typedef enum {DIRECTION_FORWARD,DIRECTION_REVERSE,DIR_RIGHT,DIR_LEFT,DIR_STOP} Direction_t;

typedef enum {MOTOR_FORWARD,MOTOR_REVERSE,MOTOR_STOP} MotorDirection_t;
typedef enum {FRONT_RIGHT_MOTOR,FRONT_LEFT_MOTOR,REAR_RIGHT_MOTOR,REAR_LEFT_MOTOR,ALL_MOTORS,LEFT_MOTORS,RIGHT_MOTORS} eMotors_t;


#define MOTOR_UPDATE_TIMEOUT_MS        200

#define LAST_DIR_FRONT_RIGHT	(1<<0)
#define LAST_DIR_FRONT_LEFT		(1<<1)
#define LAST_DIR_REAR_RIGHT		(1<<2)
#define LAST_DIR_REAR_LEFT		(1<<3)

typedef struct
{
	float SpeedRs;
	bool Enabled;
	int16_t PwmValue;
	MotorDirection_t Direction;
	bool LastTurnValue;
}Motor_t;


typedef struct
{
	Motor_t FrontRightMotor;
	Motor_t FrontLeftMotor;
	Motor_t RearRightMotor;
	Motor_t RearLeftMotor;
	Direction_t TravelDirection;
	uint8_t LastDirectionState;
}Motors_t;



typedef   struct Motors_limit
    {
	    uint8_t steer_left      : 1; // we have reached the steering controller's left most limit
	    uint8_t steer_right     : 1; // we have reached the steering controller's right most limit
	    uint8_t throttle_lower  : 1; // we have reached throttle's lower limit
	    uint8_t throttle_upper  : 1; // we have reached throttle's upper limit
	    bool    lower;  // reached this instance's lower limit on last iteration
	    bool    upper;  // reached this instance's upper limit on last iteration
    } MotorLimit_t;






class CMotors
{
	//variables
public:
	Motors_t m_Motors;
	float m_MotorDebug;
	uint16_t m_NavigationThrottle;
	uint8_t m_FrontRightServoNo;
	uint8_t m_FrontLeftServoNo;
	uint8_t m_RearRightServoNo;
	uint8_t m_RearLeftServoNo;
	bool m_IsMaxSteerRight;
	bool m_IsMaxSteerLeft;
	bool m_IsThrottleMax;
	bool m_IsThrottleMin;
    // structure for holding motor limit flags
	float       _throttle_in;               // last throttle input from set_throttle caller
	float speed_cruise;
	int8_t throttle_cruise ;
	MotorLimit_t   m_Limit;
	uint8_t m_SteeringType;
	float _speed_scale_base;
	bool    _scale_steering = true; // true if we should scale steering by speed or angle
	float		m_Motor_dt;
protected:
	uint16_t	m_Rc_Throttle;
	uint16_t	m_MotorThrottleValue;
	int16_t		m_MotorSteerValue;
    CPid_PID    m_RatePid0 = CPid_PID(WHEEL_RATE_CONTROL_P, WHEEL_RATE_CONTROL_I, WHEEL_RATE_CONTROL_D, WHEEL_RATE_CONTROL_FF, WHEEL_RATE_CONTROL_IMAX, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_DT);
    CPid_PID    m_RatePid1 = CPid_PID(WHEEL_RATE_CONTROL_P, WHEEL_RATE_CONTROL_I, WHEEL_RATE_CONTROL_D, WHEEL_RATE_CONTROL_FF, WHEEL_RATE_CONTROL_IMAX, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_FILT, WHEEL_RATE_CONTROL_DT);
	float _steering_throttle_mix;
private:
	uint32_t	m_SpeedLastMs;
	bool		m_IsUpdatePending;
    uint32_t    m_LastUpdateMs;    // system time of last call to get_rate_controlled_throttle
	//functions
public:
	CMotors();
	~CMotors();
	void Init();
	void OutputThrottle(eMotors_t Motor, float throttle);
	void UpdateFromSteering(float Speed);
	bool CalcThrottle(float target_speed,float Dt,float &Throttle);
	float GetRateControlledThrottle(eMotors_t Motor,float throttle, float dt);
	float RateControlledThrottle(eMotors_t Motor, float desired_rate_pct, float dt);
	void SetThrottle(float throttle_in);
	float GetScaledThrottle(float throttle) const;
	void Update();
	void SetMotorPwm(eMotors_t Motor,float Value,bool Passthrough);
	void DebugSetMotorPwm(uint8_t Motor,uint16_t Value);
	void PassthroughUpdate(UpdateMotors_t UpdateMotors);
	void AllMotorsStop();
	void set_limits_from_input( float steering, float throttle);
	void output_regular(bool armed, float ground_speed, float steering, float throttle);
	void OutputStandardSteering(float Steering,float Throttle);
	void SetMotorSpeed(uint8_t Motor,uint16_t Speed,uint8_t SpeedType,bool IsUpdate = false);
	void CheckMotorSpeed();
	void SetMotorDirection(eMotors_t Motor, MotorDirection_t Direction );
	void SetFailsafeSpeed();
	void SetTravelDirection(Direction_t Direction);
	void WriteMotors(uint16_t Speed,uint16_t Steering);
	void CalcSteering180(uint16_t speedMMPS,float Heading);
	void SetMotors5(float Steer, uint16_t Speed);
	void WriteMotors2(uint16_t Throt,uint16_t Steer);
	void WriteMotors3(uint16_t Throt,uint16_t Steer);
	bool IsMotorsStopped();
	void UpdatePassThrough();
	bool CheckSpeedControlUpdate() ;
	void CheckOutputs();
	uint16_t GetDesiredSpeedAccelLimited(uint16_t DesiredSpeed);
	void CheckSafeSpeed();
	void SpinToAngle(uint16_t Speed,Direction_t Direction,float Angle);
	uint16_t SetSpeedMph(float Mph);
	uint16_t SetPwmRadPerSec(float RadPerSec);
	void TestMotors();
	void CheckPitchAndRoll();
	void SetUpdate(bool State);
	void TestMotorDirection();
	void Output( float steering, float throttle,float GroundSpeed);
protected:
private:
	CMotors( const CMotors &c );
	CMotors& operator=( const CMotors &c );
	void SetMotorSkidSteer(int Speed, int Steer);
	void output_skid_steering( float steering, float throttle);
}; //CmotorControl

#endif //__CMOTORCONTROL_H__

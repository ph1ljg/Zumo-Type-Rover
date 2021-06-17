/* 
* CSteering.cpp
*
* Created: 29/10/2020 10:09:21
* Author: philg
*/

//1m/s= 2.236936mph


#include "Includes.h"

float throttle_scaled;
volatile float saturation_value;

extern Cpwm FrontRightMotorPwm;
extern Cpwm FrontLeftMotorPwm;
extern Cpwm RearRightMotorPwm;
extern Cpwm RearLeftMotorPwm;



// default constructor
CSteering::CSteering()
{
	SteeringThrottleMix = 0.5;  //Steering vs Throttle prioritisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid in Skid Steering
	SteeringThrottleMix = 0.5;  //Steering vs Throttle prioritisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid in Skid Steering 
	_speed_scale_base = 1.0f;  // speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
} //CSteering

// default destructor
CSteering::~CSteering()
{
} //~CSteering


void CSteering::Update(uint32_t TimeUs)
{
	float SteerOut;
	static uint32_t LastTime =0;
	float DesiredSteering, desired_speed;
	float speed;
	float Steer;
	bool Limit = false;
	CMyMath Math;

	m_Steer_dt = (float)(TimeUs -LastTime)/1000000.0;   // loop time secs

	speed = AttitudeControl.GetForwardSpeed();		// get speed forward   from wheel encoders

	GetRadioCDesiredSteeringAndSpeed(DesiredSteering, desired_speed);
//   	if( Config.m_RunningFlags.AVOIDANCE_ACTIVE)
// 	{
// 		if(Avoidance.GetCourse(Steer))
// 			DesiredSteering = Steer;
// 	}

  
    if(DesiredSteering >=4500 || DesiredSteering <=-4500)
		Limit = true;
	//Ahrs.get_yaw_rate_earth();  // rotation rate
    if (is_zero(desired_speed))	// determine if pilot is requesting pivot turn
	{
		// pivot turning using turn rate controller convert pilot steering input to desired turn rate in radians/sec
        const float target_turn_rate = (DesiredSteering / 4500.0f) * radians(Config.m_NavigationConfig.MaxTurnRate);
        _desired_lat_accel = 0.0f;

		// run steering turn rate controller and throttle controller
//	   GuiFunctions.SetPidOutputValues( target_turn_rate,GUI_PID_OUTPUT_INPUT); 

        const float steering_out = AttitudeControl.GetSteeringOutOfRate(target_turn_rate, Limit,m_Steer_dt); // returns -1 to 1
        set_steering(steering_out * 4500.0f,desired_speed);
    } 
	else 
	{
 		// In auto control lateral acceleration directly. For regular steering vehicles  use the maximum lateral acceleration at full steering lock for this speed: 
		float max_g_force = speed * speed / Max(Config.m_NavigationConfig.TurnRadius, 0.1f);	//V^2/R where R is the radius of turn.
		max_g_force = Math.constrain_float(max_g_force, 0.1f, Config.m_NavigationConfig.MaxTurnRate * GRAVITY_MSS);

		_desired_lat_accel = max_g_force * (DesiredSteering / 4500.0f);	// convert pilot steering input to desired lateral acceleration
	//	DebugDisplay.Printf("out %f \n",_desired_lat_accel);

		if (Config.m_RunningFlags.IN_REVERSE)		// reverse target lateral acceleration if backing up
			_desired_lat_accel = -_desired_lat_accel;

		// run lateral acceleration to steering controller
		SteerOut = calc_steering_from_lateral_acceleration(_desired_lat_accel);
        float Throttle;
		if( Motors.CalcThrottle(desired_speed,m_Steer_dt,Throttle))	// get throttle from speed m/s
			set_steering(SteerOut,Throttle);
		//DebugDisplay.Printf("out %f Thr %f  Des %f\n",SteerOut,Throttle,desired_speed);
	}

	
}


//	calculate steering output given lateral_acceleration
float CSteering::calc_steering_from_lateral_acceleration(float lat_accel)
{
	CMyMath Math;
	// constrain to max G force
	lat_accel = Math.constrain_float(lat_accel, -Config.m_NavigationConfig.MaxTurnRate * GRAVITY_MSS, Config.m_NavigationConfig.MaxTurnRate * GRAVITY_MSS);
	const float steering_out = AttitudeControl.get_steering_out_lat_accel(lat_accel, m_SteerLimit, m_Steer_dt);
//	DebugDisplay.Printf("out %f accel %f  \n",steering_out,lat_accel);

	return(steering_out * 4500.0f);
}

// steering -4500 to 4500 throttle -100 to 100
void CSteering::set_steering(float steering_value,float Throttle)
{
	CMyMath  Math;
	float  Sector;

//	RadioControl.stick_mixing(STEER_CHANNEL,(int16_t)3000);
	Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT = true;
	Config.m_RunningFlags.STICK_MIXING = false;
	if (Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT && Config.m_RunningFlags.STICK_MIXING)
	{
		steering_value = RadioControl.stick_mixing(STEER_CHANNEL,(int16_t)steering_value);
	}
	steering_value = Math.constrain_float(steering_value, -4500.0f, 4500.0f);
	float GndSpeed = Ahrs.GetHeadingVelocity();
	//Config.m_RunningFlags.IN_REVERSE;

	// scale steering down as speed increase above MOT_SPD_SCA_BASE (1 m/s default)
	if (is_positive(_speed_scale_base) && (fabsf(GndSpeed) > _speed_scale_base))
		steering_value *= (_speed_scale_base / fabsf(GndSpeed));

	else
	{
		Motors.m_Limit.steer_left = true;	// regular steering at low speed so set limits to stop I-term build-up in controllers
		Motors.m_Limit.steer_right = true;
	}
	
	if (is_negative(GndSpeed))		// reverse steering direction when backing up
		steering_value *= -1.0f;

//	GuiFunctions.SetPidOutputValues( steering_value/100,GUI_PID_OUTPUT_OUTPUT);

	Motors.Output( steering_value,Throttle,GndSpeed);
}





// Get pilot steering and return steering_out and speed_out (in m/s)
void CSteering::GetRadioCDesiredSteeringAndSpeed(float& steering_out, float& speed_out)
{
	float desired_throttle;
	RadioControl.GetRadioInput(steering_out, desired_throttle);
	speed_out = desired_throttle * 0.01f * CalcSpeedMax(Config.m_NavigationConfig.CruiseSpeed, Config.m_NavigationConfig.CruiseThrottle * 0.01f);

	float speed_out_limited = AttitudeControl.GetDesiredSpeedAccelLimited(speed_out, m_Steer_dt);		// check for special case of input and output throttle being in opposite directions

		if ((is_negative(speed_out) != is_negative(speed_out_limited)))
			steering_out *= -1;

	speed_out = speed_out_limited;
}








// estimate maximum vehicle speed (in m/s)  cruise_speed is in m/s, cruise_throttle should be in the range -1 to +1
float CSteering::CalcSpeedMax(float cruise_speed, float cruise_throttle) const
{
	CMyMath Math;
	float speed_max;

	// sanity checks
	if (cruise_throttle > 1.0f || cruise_throttle < 0.05f)
	speed_max = cruise_speed;

	else if (is_positive(Config.m_NavigationConfig.MaximumSpeed))
		speed_max = Config.m_NavigationConfig.MaximumSpeed;
	else
		speed_max = (1.0f / cruise_throttle) * cruise_speed;	// project vehicle's maximum speed

	return Math.constrain_float(speed_max, 0.0f, 0.8f); // constrain to 0.8m/s (2.880001555km/h) (1.78955mph)and return
	//return Math.constrain_float(speed_max, 0.0f, 0.96f); // constrain to 0.8m/s (3.456km/h) (2.147459mph)and return
}







void CSteering::UpdateSkidRcControl(uint32_t Milli)
{
	CMyMath Math;
	float dt = Milli / 1000.0f;
	double Degrees;
	uint16_t Throttle;
	int16_t Steering;
	Steering = RadioControl.m_RadioDataIn[STEER_CHANNEL_X] ;

// 	if(Steering <1520 && Steering >1480)
// 		Steering =  1500;

	Steering -= 1500;
	
	Throttle = Math.Map(RadioControl.m_RadioDataIn[THROTTLE_CHANNEL],MOTOR_RC_MIN_PWM,MOTOR_RC_MAX_PWM,0,100);
	Steering = Math.Map(Steering,-500,500,-4500,4500);
	
	OutputSkidSteering( Steering,  Throttle, dt);
}


// output to skid steering channels
// Steering -4500 - 4500 Throttle  0-100
void CSteering::OutputSkidSteering(int16_t steering, uint16_t throttle, float dt)
{
	CMyMath Math;
	Motors.set_limits_from_input( steering, throttle);		// Clear and set limits based on input
	Motors.set_limits_from_input( steering, throttle);		// Clear and set limits based on input

	steering = Math.constrain_float(steering, -4500.0f, 4500.0f);				// Constrain steering


	// skid steering mixer
	float steering_scaled = (float)steering / 4500.0f; // steering scaled -1 to +1
	 throttle_scaled = (float)throttle / 100.0f;  // throttle scaled -1 to +1

	// apply constraints
	steering_scaled = Math.constrain_float(steering_scaled, -1.0f, 1.0f);
	throttle_scaled = Math.constrain_float(throttle_scaled, -1.0f, 1.0f);

	const float saturation_value = fabsf(steering_scaled) + fabsf(throttle_scaled);		// check for saturation and scale back throttle and steering proportionally
	
	if (saturation_value > 1.0f) 
	{
		const float str_thr_mix = Math.constrain_float(SteeringThrottleMix, 0.0f, 1.0f);
		const float fair_scaler = 1.0f / saturation_value;
		if (str_thr_mix >= 0.5f) 
		{
			
			steering_scaled *= Math.LinearInterpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);					// prioritise steering over throttle
			throttle_scaled = (1.0f - fabsf(steering_scaled)) * (is_negative(throttle_scaled) ? -1.0f : 1.0f);
		} 
		else 
		{
			
			throttle_scaled *= Math.LinearInterpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);			// prioritise throttle over steering
			steering_scaled = (1.0f - fabsf(throttle_scaled)) * (is_negative(steering_scaled) ? -1.0f : 1.0f);
		}
	}

	// add in throttle and steering
	const float motor_left = throttle_scaled + steering_scaled;
	const float motor_right = throttle_scaled - steering_scaled;

	// send pwm value to each motor
	Motors.OutputThrottle(LEFT_MOTORS, 100.0f * motor_left);
	Motors.OutputThrottle(RIGHT_MOTORS, 100.0f * motor_right);
}













void CSteering::UpdateRcMecanControl()
{
	CMyMath Math;
	float Xvalue;
	float Yvalue;
	float Degrees;
	double Angle;
	uint16_t SteerChannelX ;
	uint16_t SteerChannelY ;

	uint16_t MotorThrottleValue;

	SteerChannelX = RadioControl.m_RadioDataIn[STEER_CHANNEL_X] ;
	SteerChannelY = RadioControl.m_RadioDataIn[STEER_CHANNEL_Y] ;

	if(SteerChannelX <1510 && SteerChannelX >1490)
		SteerChannelX =  1500;

	if(SteerChannelY <1510 && SteerChannelY >1490)
		SteerChannelY =  1500;

	Yvalue = SteerChannelY-1500;
	Xvalue = SteerChannelX- 1500;
	
	Angle = (atan2f( Xvalue,Yvalue));
	
	if(Angle <0.0)
	Angle = (2*PI) +Angle;

	Degrees = Angle* RAD_TO_DEG;
	MotorThrottleValue = Math.Map(RadioControl.m_RadioDataIn[THROTTLE_CHANNEL],MOTOR_RC_MIN_PWM,MOTOR_RC_MAX_PWM,0,100);
	
	//	DebugDisplay.Printf("Angle %0.3f\n",Degrees);

	CalculateMecanSteering(MotorThrottleValue,Degrees,0); // ******************** rename incorrect spell Calculatet
//	Motors.CheckSafeSpeed(); ************************
//	Motors.Update(); ****************************
}

//==============================================================================================
// Mecanum Wheel Control
// V1=Vty+Vtx-w(a+b)
// V2=Vty-Vtx-w(a+b)
// V3=Vty+Vtx+w(a+b)
// V4=Vty-Vtx+w(a+b)
// Motor speed  = 0-100
//==============================================================================================
void CSteering::CalculateMecanSteering(int MotorSpeed,float Angle,float SpeedOfChange )
{
	CMyMath Math;
	float Speed ;
	
	#define ROTATION_45 PI/4
	//	float Speed = (float)MotorSpeed;
	Speed = (float)MotorSpeed;

	
//	float Radian = Angle *DEG_TO_RAD;
	float MotorMuliplier[4];
	float FrontRight;
	float FrontLeft;
	float RearRight;
	float RearLeft;
	float LastHighValue = 0;


	Angle = Angle +90;
	if(Angle >360)
	Angle = 360 -Angle;
	float Radian2 = Angle *DEG_TO_RAD;
	
	Speed /= 100;
	
	MotorMuliplier[FRONT_LEFT_MOTOR] = (Speed*sin(Radian2)-Speed*cos(Radian2)+SpeedOfChange); // Calc force Vectors
	MotorMuliplier[FRONT_RIGHT_MOTOR] = (Speed*sin(Radian2)+Speed*cos(Radian2)-SpeedOfChange);
	MotorMuliplier[REAR_LEFT_MOTOR] = (Speed*sin(Radian2)+Speed*cos(Radian2)+SpeedOfChange);
	MotorMuliplier[REAR_RIGHT_MOTOR] = (Speed*sin(Radian2)-Speed*cos(Radian2)-SpeedOfChange);

	for(int i =0;i<4;i++)
	{
		if(MotorMuliplier[i] > LastHighValue )
		LastHighValue = MotorMuliplier[i];
	}
	
	if(LastHighValue < -1 || LastHighValue> 1)
	{
		for(int i =0;i<4;i++)
		MotorMuliplier[i] /= LastHighValue;
	}
	
	FrontLeft = MOTOR_RAW_DRIVE_PWM_MAX *MotorMuliplier[FRONT_LEFT_MOTOR];
	FrontRight = MOTOR_RAW_DRIVE_PWM_MAX *MotorMuliplier[FRONT_RIGHT_MOTOR];
	RearLeft = MOTOR_RAW_DRIVE_PWM_MAX *MotorMuliplier[REAR_LEFT_MOTOR];
	RearRight = MOTOR_RAW_DRIVE_PWM_MAX*MotorMuliplier[REAR_RIGHT_MOTOR];

	if(is_negative(FrontRight))
		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,false);
	else
		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,true);
	
	if(is_negative(FrontLeft))
		PCF8574.SetOutput(FRONT_LEFT_MOTOR,true);
	else
		PCF8574.SetOutput(FRONT_LEFT_MOTOR,false);

	if(is_negative(RearRight))
		PCF8574.SetOutput(REAR_RIGHT_MOTOR,false);
	else
		PCF8574.SetOutput(REAR_RIGHT_MOTOR,true);

	if(is_negative(RearLeft))
		PCF8574.SetOutput(REAR_LEFT_MOTOR,true);
	else
	PCF8574.SetOutput(REAR_LEFT_MOTOR,false);
	
	Motors.m_Motors.FrontRightMotor.PwmValue = round(abs(FrontRight));
	Motors.m_Motors.FrontLeftMotor.PwmValue = round(abs(FrontLeft));
	Motors.m_Motors.RearRightMotor.PwmValue = round(abs(RearRight));
	Motors.m_Motors.RearLeftMotor.PwmValue = round(abs(RearLeft));
	Motors.CheckMotorSpeed();

	FrontRightMotorPwm.SetValue(Motors. m_Motors.FrontRightMotor.PwmValue); //**********************
	FrontLeftMotorPwm.SetValue(Motors.m_Motors.FrontLeftMotor.PwmValue);		//***********************
	RearRightMotorPwm.SetValue(Motors.m_Motors.FrontRightMotor.PwmValue);	//***********************
	RearLeftMotorPwm.SetValue(Motors.m_Motors.RearLeftMotor.PwmValue);	//*************************

//	Motors.Update(); ************************************
}




void CSteering::Test()
{
	uint16_t speed = 100;
	
	//Spin(40,DIR_SPIN_RIGHT);
	CalculateMecanSteering(speed,0,0.0);		// Right
	speed = 50;
	CalculateMecanSteering(speed,0,0.0);		// Right
	
	speed = 75;
	CalculateMecanSteering(speed,0,0.0);		// Right
	speed = 70;
	CalculateMecanSteering(speed,0,0);		// Right
	
	
	CalculateMecanSteering(speed,90,0); // forward
	CalculateMecanSteering(speed,180,0); // Back off
	CalculateMecanSteering(speed,270,0);		// Left
	CalculateMecanSteering(speed,360,0); // Upper left
	CalculateMecanSteering(speed,60,0);	// Lower Left
	CalculateMecanSteering(speed,20,0);  // Lower Right
	CalculateMecanSteering(speed,280,0);	// Lower right
	//	setCarMove(0,0,omega);			// rotate
	//	CalculatetSteering(0,0,speedMMPS/(sqrt(pow(WHEELSPAN/2,2)*2))); // rotate left
	//	CalculatetSteering(0,0,-speedMMPS/(sqrt(pow(WHEELSPAN/2,2)*2)));	// rotate right
}

void CSteering::TurnTest()
{
	uint16_t speed = 100;
	CalculateMecanSteering(speed,-45,0 );
	while(1)
	{
		Core.delay(300);
	}
	// 	CalculatetMecanSteering(speed,0,0);
	// 	//Spin(40,DIR_SPIN_RIGHT);
	// 	SkidSteerSetCourse(0,speed,0);		// Right
	// 	Core.delay(500);
	// 	speed = 50;
	// 	SkidSteerSetCourse(0,speed,0);		// Right
	// 	Core.delay(500);
	// 	speed = 75;
	// 	SkidSteerSetCourse(0,speed,0);		// Right
	// 	Core.delay(500);
	// 	speed = 70;
	// 	SkidSteerSetCourse(0,speed,0);		// Right
	// 	Core.delay(500);
	//
	//
	// 	SkidSteerSetCourse(10,speed,0); // forward
	// 	SkidSteerSetCourse(15,speed,0); // Back off
	// 	SkidSteerSetCourse(30,speed,0);		// Left
	// 	SkidSteerSetCourse(-10,speed,0); // Upper left
	// 	SkidSteerSetCourse(-20,speed,0);	// Lower Left
	// 	SkidSteerSetCourse(-30,speed,0);  // Lower Right
}


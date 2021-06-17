/*
* CmotorControl.cpp
*
* Created: 20/05/2020 12:34:51
* Author: philg
*/

#include "Includes.h"

// max speed = 0.096865773485 m/s
// max speed = 9.6865773485 cm/s
// real max speed = 0.847 m/s


bool MotorDebug = false;
//bool MotorDebug = true;


extern CAvoidance Avoidance;

Cpwm FrontRightMotorPwm;
Cpwm FrontLeftMotorPwm;
Cpwm RearRightMotorPwm;
Cpwm RearLeftMotorPwm;
CPCF8574 PCF8574;


float DebugAngle;
float Xvalue;
float Yvalue;
float Degrees;
double Angle;
uint16_t SteerChannelX ;
uint16_t SteerChannelY ;
double Radian2;

float FrontRight;
float FrontLeft;
float RearRight;
float RearLeft;
	
	
//	float Radian = Angle *DEG_TO_RAD;
float MotorMuliplier[4];

CWheelEncoder WheelEncoder;

// default constructor
CMotors::CMotors()
{

	m_Motors.FrontRightMotor.LastTurnValue	= true;
	m_Motors.FrontLeftMotor.LastTurnValue	= true;
	m_Motors.RearRightMotor.LastTurnValue	= true;
	m_Motors.RearLeftMotor.LastTurnValue	= true;

	m_Motors.FrontRightMotor.Direction	= MOTOR_FORWARD;
	m_Motors.FrontLeftMotor.Direction	= MOTOR_FORWARD;
	m_Motors.RearRightMotor.Direction	= MOTOR_FORWARD;
	m_Motors.RearLeftMotor.Direction	= MOTOR_FORWARD;
	m_SpeedLastMs = 0;
	m_IsUpdatePending = true;
	m_IsMaxSteerRight = false;
	m_IsMaxSteerLeft = false;
	m_IsThrottleMax = false;
	m_IsThrottleMin = false;
	Config.m_RunningFlags.RATE_CONTROLED_THROTTLE = false;
	speed_cruise = CRUISE_SPEED;
	throttle_cruise = 50;	// Steering vs Throttle prioritisation.  Higher numbers prioritise steering, lower numbers prioritise throttle.  Only valid in Skid Steering 
	_steering_throttle_mix = 0.5;
	m_SteeringType = STEERING_SKID;
	_speed_scale_base =1;	// speed above which steering is scaled down when using regular steering/throttle vehicles.  zero to disable speed scaling
} //MotorControl

// default destructor
CMotors::~CMotors()
{
} //~CMotorControl



void CMotors::Init()
{
	PCF8574.PCF8574OutData(0);
	FrontRightMotorPwm.Init(PORTA,16,0,TCC1_CH0);			// D5 
	FrontLeftMotorPwm.Init(PORTA,14,0,TCC1_CH2);			// D4 
	RearLeftMotorPwm.Init(PORTA,20,0,TCC0_CH0);			// D10 Rear Left motor 
	RearRightMotorPwm.Init(PORTA,21,0,TCC0_CH1);			// D11 Front right Motor  

	SetTravelDirection(DIRECTION_FORWARD);
	AllMotorsStop();
	WheelEncoder.Init();
}



// output throttle value to main throttle channel, left throttle or right throttle.  throttle should be scaled from -100 to 100
void CMotors::OutputThrottle(eMotors_t Motor, float throttle)
{
	CMyMath Math;
	static uint32_t LastTime =0;
	m_Motor_dt = (Core.millis()  - LastTime)/1000.0;
	
	if(throttle <0)
	{
		SetMotorDirection(Motor,MOTOR_REVERSE);		
	}
	else
		SetMotorDirection(Motor,MOTOR_FORWARD);		
	
	throttle = GetScaledThrottle(throttle);	// constrain and scale output
	throttle = GetRateControlledThrottle(Motor, throttle, m_Motor_dt);		// apply rate control

	SetMotorPwm(Motor,throttle);
	LastTime = Core.millis();
	
}

void CMotors::UpdateFromSteering(float Speed)
{
//	float throttle_out = 100.0f * AttitudeControl.get_throttle_out_speed(Speed, m_Limit.throttle_lower, m_Limit.throttle_upper, speed_cruise, throttle_cruise * 0.01f, m_Motor_dt);
//	OutputThrottle(throttle_out,m_Motor_dt);
}



bool CMotors::CalcThrottle(float target_speed,float Dt,float &Throttle)
{
	float TargetSpeed = AttitudeControl.GetDesiredSpeedAccelLimited(target_speed, Dt);	// get acceleration limited target speed

	
	if (Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT ||Config.m_RunningFlags.AVOIDANCE_ACTIVE ) 
		Avoidance.AdjustSpeed( TargetSpeed, Dt);


	// call throttle controller and convert output to -100 to +100 range
	float throttle_out = 0.0f;

	// call speed or stop controller
	//m_MotorDebug = TargetSpeed;
	if (is_zero(TargetSpeed)) 
	{
		bool stopped;
		throttle_out = 100.0f * AttitudeControl.get_throttle_out_stop( Config.m_NavigationConfig.CruiseSpeed,	Config.m_NavigationConfig.CruiseThrottle * 0.01f, Dt,stopped);
	} 
	else
	{ 
		 if(AttitudeControl.get_throttle_out_speed(TargetSpeed,Config.m_NavigationConfig.CruiseSpeed, Config.m_NavigationConfig.CruiseThrottle * 0.01f, Dt,throttle_out))
		{	
			 Throttle = throttle_out*100;
			return(true);	 
		}
		else
			return(false);
	}
	return(true);
}







// use rate controller to achieve desired throttle
float CMotors::GetRateControlledThrottle(eMotors_t Motor,float throttle, float dt)
{
	// require non-zero dt
	if (!is_positive(dt)) 
		return throttle;

	// attempt to rate control left throttle
	if (Config.m_RunningFlags.RATE_CONTROLED_THROTTLE && Motor == LEFT_MOTORS) 
		return (RateControlledThrottle(LEFT_MOTORS, throttle, dt));

	// rate control right throttle
	if (Config.m_RunningFlags.RATE_CONTROLED_THROTTLE && Motor == RIGHT_MOTORS)
		return (RateControlledThrottle(RIGHT_MOTORS, throttle, dt));

	// return throttle unchanged
	return throttle;
}


// get throttle output in the range -100 to +100 given a desired rate expressed as a percentage of the rate_max (-100 to +100)
float CMotors::RateControlledThrottle(eMotors_t Motor, float desired_rate_pct, float dt)
{
	    // determine which PID instance to use
	    CPid_PID& rate_pid = (Motor ==RIGHT_MOTORS ) ? m_RatePid0 : m_RatePid1;

	// set PID's dt
	rate_pid.set_dt(dt);

	// check for timeout
	uint32_t now =Core.millis();
	if (now - m_LastUpdateMs > AP_WHEEL_RATE_CONTROL_TIMEOUT_MS) 
	{
		rate_pid.ResetFilter();
		rate_pid.ResetI();
		m_Limit.lower = false;
		m_Limit.upper = false;
	}
	m_LastUpdateMs = now;

	// convert desired rate as a percentage to radians/sec
	float desired_rate = desired_rate_pct / 100.0f * 6;

	// get actual rate from wheel encoder
	float actual_rate;
	if(Motor == RIGHT_MOTORS)
		actual_rate = WheelEncoder.GetRate(RIGHT_WHEEL_SENSOR);
	else
		actual_rate = WheelEncoder.GetRate(LEFT_WHEEL_SENSOR);

	// constrain and set limit flags
	float output = rate_pid.update_all(desired_rate, actual_rate, (m_Limit.lower || m_Limit.upper));
	output += rate_pid.GetFF();

	// set limits for next iteration
	m_Limit.upper = output >= 100.0f;
	m_Limit.lower = output <= -100.0f;

	return output;
}


// range 0 ~ 1
void CMotors::SetThrottle(float throttle_in)
{
	_throttle_in = throttle_in;
};



//=============================================================================================================================
// scale a throttle using the _throttle_min and _thrust_curve_expo parameters.  throttle should be in the range -100 to +100
//=============================================================================================================================
float CMotors::GetScaledThrottle(float throttle) const
{
	CMyMath Math;
	// exit immediately if throttle is zero
	if (is_zero(throttle)) 
	{
		return throttle;
	}

	// scale using throttle_min
	if (Config.m_MainConfig.MinThrottleValue > 0) 
	{
		if (is_negative(throttle)) 
		{
			throttle = -Config.m_MainConfig.MinThrottleValue + (throttle * ((100.0f - Config.m_MainConfig.MinThrottleValue) / 100.0f));
		} 
		else 
		{
			throttle = Config.m_MainConfig.MinThrottleValue + (throttle * ((100.0f - Config.m_MainConfig.MinThrottleValue) / 100.0f));
		}
	}
	float Expo = (float)Config.m_MainConfig.RcExpo/100.0;
	// skip further scaling if thrust curve disabled or invalid
	if (is_zero(Expo) || (Expo > 1.0f) || (Expo < -1.0f)) 
		return throttle;

	// calculate scaler
	const float sign = (throttle < 0.0f) ? -1.0f : 1.0f;
	const float throttle_pct = Math.constrain_float(throttle, -100.0f, 100.0f) / 100.0f;
	return( 100.0f * sign * ((Expo - 1.0f) + safe_sqrt((1.0f - Expo) * (1.0f - Expo) + 4.0f * Expo * fabsf(throttle_pct))) / (2.0f * Expo));
}


// void CMotors::SetMotorSkidSteer(int Speed, int Steer)
// {
// 	CMyMath Math;
// 	uint16_t RightMotorsPwm ;
// 	uint16_t LeftMotorsPwm ;
// 	int SpeedTargetRight;  //target speed
// 	int SpeedTargetLeft;  //target speed
// 	uint8_t Motor = 0;
// 	if (Speed == 0 && Steer !=0)
// 	{
// 		SpeedTargetLeft = Steer;
// 		SpeedTargetRight = -Steer;
// 	}
// 	else
// 	{
// 		if(Config.m_RunningFlags.IN_REVERSE)
// 		Speed *= -1;
// 		SpeedTargetLeft = Speed * ((-255 - Steer) / -255.0);
// 		SpeedTargetRight = Speed * ((255 - Steer) / 255.0);
// 		
// 		if (Speed > 0 && SpeedTargetLeft > Speed)
// 		SpeedTargetLeft = Speed;
// 		if (Speed > 0 && SpeedTargetRight > Speed)
// 		SpeedTargetRight = Speed;
// 
// 		if (Speed < 0 && SpeedTargetLeft < Speed)
// 		SpeedTargetLeft = Speed;
// 		if (Speed < 0 && SpeedTargetRight < Speed)
// 		SpeedTargetRight = Speed;
// 
// 	}
// 	if(SpeedTargetRight <0)
// 	{
// 		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,false);
// 		PCF8574.SetOutput(REAR_RIGHT_MOTOR,false);
// 		Motors.m_Motors.FrontRightMotor.Direction = MOTOR_REVERSE;
// 		Motors.m_Motors.RearRightMotor.Direction = MOTOR_REVERSE;
// 	}
// 	else
// 	{
// 		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,true);
// 		PCF8574.SetOutput(REAR_RIGHT_MOTOR,true);
// 		Motors.m_Motors.FrontRightMotor.Direction = MOTOR_FORWARD;
// 		Motors.m_Motors.RearRightMotor.Direction = MOTOR_FORWARD;
// 	}
// 	
// 	if(SpeedTargetLeft <0)
// 	{
// 		PCF8574.SetOutput(FRONT_LEFT_MOTOR,true);
// 		PCF8574.SetOutput(REAR_LEFT_MOTOR,true);
// 		Motors.m_Motors.FrontLeftMotor.Direction = MOTOR_REVERSE;
// 		Motors.m_Motors.RearLeftMotor.Direction = MOTOR_REVERSE;
// 	}
// 	else
// 	{
// 		PCF8574.SetOutput(FRONT_LEFT_MOTOR,false);
// 		PCF8574.SetOutput(REAR_LEFT_MOTOR,false);
// 		Motors.m_Motors.FrontLeftMotor.Direction = MOTOR_FORWARD;
// 		Motors.m_Motors.RearLeftMotor.Direction = MOTOR_FORWARD;
// 	}
// 	SpeedTargetLeft = abs(SpeedTargetLeft);
// 	SpeedTargetRight = abs(SpeedTargetRight);
// 
// 	RightMotorsPwm = Math.Map(SpeedTargetRight,0,255,100,5000);
// 	LeftMotorsPwm = Math.Map(SpeedTargetLeft,0,255,100,5000);
// 	Motor |= ((1<<FRONT_RIGHT_MOTOR)| (1<<REAR_RIGHT_MOTOR));
// 	Motors.SetMotorSpeed(Motor,(float) RightMotorsPwm,SPEED_PWM,false);
// 	Motor = 0;
// 	Motor |= ((1<<FRONT_LEFT_MOTOR)| (1<<REAR_LEFT_MOTOR));
// 	Motors.SetMotorSpeed(Motor, LeftMotorsPwm,SPEED_PWM);
// }


void CMotors::Output( float steering, float throttle,float GroundSpeed)
{
	switch(m_SteeringType)
	{
	case STEERING_SKID:
			output_skid_steering(steering,throttle);
			break;
	case STEERING_MECAN:
			break;
	case STEERING_STANDARD:
			break;
	}
}


// output to skid steering channels
void CMotors::output_skid_steering( float steering, float throttle)
{
	CMyMath Math;
//		DebugDisplay.Printf(12,10,6," Steer in %f  "    ,steering);

	set_limits_from_input( steering, throttle);		// clear and set limits based on input
	

	steering = Math.constrain_float(steering, -4500.0f, 4500.0f);		// constrain steering

	// handle simpler disarmed case
	if (!Config.m_RunningFlags.ARMED) 
	{
		if(!Config.m_FunctionFlags.PassThroughFlag)
			SetMotorPwm(ALL_MOTORS,0);
		return;
	}

	// skid steering mixer
	float steering_scaled = steering / 4500.0f; // steering scaled -1 to +1
	float throttle_scaled = throttle / 100.0f;  // throttle scaled -1 to +1

//		DebugDisplay.Printf(14,10,6," Steer %f Thottle %f "    ,steering_scaled);


	// apply constraints
	steering_scaled = Math.constrain_float(steering_scaled, -1.0f, 1.0f);
	throttle_scaled = Math.constrain_float(throttle_scaled, -1.0f, 1.0f);

	// check for saturation and scale back throttle and steering proportionally
	const float saturation_value = fabsf(steering_scaled) + fabsf(throttle_scaled);
	if (saturation_value > 1.0f) 
	{
		const float str_thr_mix = Math.constrain_float(_steering_throttle_mix, 0.0f, 1.0f);
		const float fair_scaler = 1.0f / saturation_value;
		if (str_thr_mix >= 0.5f) 
		{
			// prioritise steering over throttle
			steering_scaled *= Math.LinearInterpolate(fair_scaler, 1.0f, str_thr_mix, 0.5f, 1.0f);
			throttle_scaled = (1.0f - fabsf(steering_scaled)) * (is_negative(throttle_scaled) ? -1.0f : 1.0f);
		} 
		else 
		{
			// prioritise throttle over steering
			throttle_scaled *= Math.LinearInterpolate(fair_scaler, 1.0f, 0.5f - str_thr_mix, 0.0f, 0.5f);
			steering_scaled = (1.0f - fabsf(throttle_scaled)) * (is_negative(steering_scaled) ? -1.0f : 1.0f);
		}
	}

	// add in throttle and steering
	const float motor_left = throttle_scaled + steering_scaled;
	const float motor_right = throttle_scaled - steering_scaled;

	// send pwm value to each motor
	OutputThrottle(LEFT_MOTORS, 100.0f * motor_left);
	OutputThrottle(RIGHT_MOTORS, 100.0f * motor_right);
}

// set limits based on steering and throttle input
void CMotors::set_limits_from_input( float steering, float throttle)
{
	bool armed = Config.m_RunningFlags.ARMED;
	// set limits based on inputs
	m_Limit.steer_left = !armed || (steering <= -4500.0f);
	m_Limit.steer_right = !armed || (steering >= 4500.0f);
	m_Limit.throttle_lower = !armed || (throttle <= -Config.m_MainConfig.MaxThrottleValue);
	m_Limit.throttle_upper = !armed || (throttle >= Config.m_MainConfig.MaxThrottleValue);
}




// output to regular steering and throttle channels
void CMotors::output_regular(bool armed, float ground_speed, float steering, float throttle)
{
    CMyMath Math;
	// output to throttle channels
    if (armed) 
	{
        if (_scale_steering) 
		{
            // scale steering down as speed increase above MOT_SPD_SCA_BASE (1 m/s default)
            if (is_positive(_speed_scale_base) && (fabsf(ground_speed) > _speed_scale_base)) 
			{
                steering *= (_speed_scale_base / fabsf(ground_speed));
            } 
			else 
			{
                // regular steering rover at low speed so set limits to stop I-term build-up in controllers
                    m_Limit.steer_left = true;
                    m_Limit.steer_right = true;
            }
            // reverse steering direction when backing up
            if (is_negative(ground_speed)) 
			{
                steering *= -1.0f;
            }
        } 
		else 
		{
            // reverse steering direction when backing up
            if (is_negative(throttle)) 
			{
                steering *= -1.0f;
            }
        }
		// constrain steering
		steering = Math.constrain_float(steering, -4500.0f, 4500.0f);
       OutputStandardSteering(steering,throttle);
    } 
	else 
	{
          AllMotorsStop();
//	   SetMotorSpeed(ALL_MOTORS,0,SPEED_PWM,true); ****************************
	   OutputStandardSteering(steering,0);		// always allow steering to move
    }

    // clear and set limits based on input do this here  speed scaling may have reduced steering request
    set_limits_from_input( steering, throttle);
}

void CMotors::OutputStandardSteering(float Steering,float Throttle)
{
	OutputThrottle(ALL_MOTORS, Throttle);
	//Servo::Write(Steering)
}



void CMotors::CheckMotorSpeed()
{
	CMyMath Math;
	m_Motors.FrontRightMotor.PwmValue	 += Config.m_MainConfig.MotorTrims[0];
	m_Motors.FrontLeftMotor.PwmValue	+= Config.m_MainConfig.MotorTrims[1];
	m_Motors.RearRightMotor.PwmValue	+= Config.m_MainConfig.MotorTrims[2];
	m_Motors.RearLeftMotor.PwmValue		+= Config.m_MainConfig.MotorTrims[3];

	m_Motors.FrontRightMotor.PwmValue = Math.Constrain(m_Motors.FrontRightMotor.PwmValue, Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
	m_Motors.FrontLeftMotor.PwmValue = Math.Constrain(m_Motors.FrontLeftMotor.PwmValue,  Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
	m_Motors.RearRightMotor.PwmValue = Math.Constrain(m_Motors.RearRightMotor.PwmValue, Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
	m_Motors.RearLeftMotor.PwmValue = Math.Constrain(m_Motors.RearLeftMotor.PwmValue,  Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
	
}



void CMotors::SetMotorDirection(eMotors_t Motor, MotorDirection_t Direction )
{
	switch(Motor)
	{
	case FRONT_RIGHT_MOTOR:
		if(Direction == MOTOR_FORWARD)
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_FORWARD);
		else	
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_REVERSE);
		m_Motors.FrontRightMotor.Direction = Direction;
		break;	
	case FRONT_LEFT_MOTOR:
		if(Direction == MOTOR_FORWARD)
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_FORWARD);
		else	
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_REVERSE);
		m_Motors.FrontLeftMotor.Direction = Direction;
		break;	
	case REAR_RIGHT_MOTOR:
		if(Direction == MOTOR_FORWARD)
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_FORWARD);
		else	
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_REVERSE);
		m_Motors.RearRightMotor.Direction = Direction;
		break;	
	case REAR_LEFT_MOTOR:
		if(Direction == MOTOR_FORWARD)
			PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_FORWARD);
		else
			PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_REVERSE);
		m_Motors.RearLeftMotor.Direction = Direction;
		break;	
	case RIGHT_MOTORS:
		if(Direction == MOTOR_FORWARD)
		{
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_FORWARD);
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_FORWARD);
		}
		else
		{
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_REVERSE);
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_REVERSE);
		}
		m_Motors.RearRightMotor.Direction = Direction;
		m_Motors.FrontRightMotor.Direction = Direction;
		break;	
	case LEFT_MOTORS:
		if(Direction == MOTOR_FORWARD)
		{
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_FORWARD);
			PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_FORWARD);
		}
		else
		{
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_REVERSE);
			PCF8574.SetOutput(REAR_LEFT_MOTOR,FRONT_RIGHT_REVERSE);
		}
		m_Motors.FrontLeftMotor.Direction = Direction;
		m_Motors.RearLeftMotor.Direction = Direction;
		break;	
	case ALL_MOTORS:
		if(Direction == MOTOR_FORWARD)
		{
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_FORWARD);
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,FRONT_RIGHT_FORWARD);
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,REAR_LEFT_FORWARD);
			PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_FORWARD);
		}
		else
		{
			PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_REVERSE);
			PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_REVERSE);
			PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_REVERSE);
			PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_REVERSE);
		}
			
		m_Motors.FrontRightMotor.Direction = Direction;
		m_Motors.RearRightMotor.Direction = Direction;
		m_Motors.FrontLeftMotor.Direction = Direction;
		m_Motors.RearLeftMotor.Direction = Direction;
		break;
	}
}


// rear motors are wired inversely to front To much trouble to do in hardware so done here
void CMotors::SetTravelDirection(Direction_t Direction )
{
	m_Motors.TravelDirection = Direction;
	switch(Direction)
	{
	case DIRECTION_FORWARD:
		SetMotorDirection(ALL_MOTORS,MOTOR_FORWARD );
		break;
	case DIRECTION_REVERSE:
		SetMotorDirection(ALL_MOTORS,MOTOR_REVERSE );
		break;
	case DIR_RIGHT:
		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_REVERSE);					// Right front reverse
		PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_FORWARD);					// Left front forward
		PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_FORWARD);					// Right Rear forward
		PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_REVERSE);					// Left Rear reverse
		break;
	case DIR_LEFT:
		PCF8574.SetOutput(FRONT_RIGHT_MOTOR,FRONT_RIGHT_REVERSE);					// Right front reverse
		PCF8574.SetOutput(FRONT_LEFT_MOTOR,FRONT_LEFT_FORWARD);					// Left front forward
		PCF8574.SetOutput(REAR_RIGHT_MOTOR,REAR_RIGHT_FORWARD);					// Right Rear forward
		PCF8574.SetOutput(REAR_LEFT_MOTOR,REAR_LEFT_REVERSE);					// Left Rear reverse
		break;
	case DIR_STOP:
		default:
		AllMotorsStop();
		break;
	}
}




//=================================================================================
// Set Motor PWM 0-5000 input -100 - 100 -value reverse
//=================================================================================
void CMotors::SetMotorPwm(eMotors_t Motor,float Value)
{
	CMyMath Math;
	uint16_t Pwm;
	if(is_negative(Value))
		SetMotorDirection(Motor,MOTOR_REVERSE);
	else		
		SetMotorDirection(Motor,MOTOR_FORWARD);
	
	Value = abs(Value);

	Pwm = Math.Round(Value *50);

	switch(Motor)
	{
	case FRONT_RIGHT_MOTOR:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
		m_Motors.FrontRightMotor.PwmValue = Pwm;
		break;
	case  FRONT_LEFT_MOTOR:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].max);
		m_Motors.FrontLeftMotor.PwmValue = Pwm;
		break;
	case REAR_RIGHT_MOTOR:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].max);
		m_Motors.RearRightMotor.PwmValue = Pwm;
		break;
	case REAR_LEFT_MOTOR:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].max);
		m_Motors.RearLeftMotor.PwmValue = Pwm;
		break;
	case ALL_MOTORS:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].max);
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].max);
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].max);
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
		m_Motors.FrontRightMotor.PwmValue = Pwm;
		m_Motors.FrontLeftMotor.PwmValue = Pwm;
		m_Motors.RearRightMotor.PwmValue = Pwm;
		m_Motors.RearLeftMotor.PwmValue = Pwm;
		break;
	case LEFT_MOTORS:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_L_MOTOR].max);
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_L_MOTOR].max);
		m_Motors.FrontLeftMotor.PwmValue = Pwm;
		m_Motors.RearLeftMotor.PwmValue = Pwm;
		break;
	case RIGHT_MOTORS:
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_R_R_MOTOR].max);
		Pwm = Math.Constrain(Pwm,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].min,Config.m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max);
		m_Motors.FrontRightMotor.PwmValue = Pwm;
		m_Motors.RearRightMotor.PwmValue = Pwm;
		break;
	
	}
	CheckMotorSpeed();
	if(!MotorDebug && Config.m_FunctionFlags.FunctionFlag_1)
	{
		FrontRightMotorPwm.SetValue( m_Motors.FrontRightMotor.PwmValue);      // set PWM driver with pwm value
		FrontLeftMotorPwm.SetValue(m_Motors.FrontLeftMotor.PwmValue);
		RearRightMotorPwm.SetValue(m_Motors.RearRightMotor.PwmValue);
		RearLeftMotorPwm.SetValue(m_Motors.RearLeftMotor.PwmValue);
	}
	else
	{	
//		FrontLeftMotorPwm.SetValue(m_Motors.FrontLeftMotor.PwmValue);
//		FrontRightMotorPwm.SetValue(m_Motors.FrontRightMotor.PwmValue); // PSB 20/05/21
//		RearLeftMotorPwm.SetValue(m_Motors.RearRightMotor.PwmValue);   // PSB 22/05/21
//		RearRightMotorPwm.SetValue(m_Motors.RearLeftMotor.PwmValue);    // PSB 22/05/21
	}
}

//=================================================================================
// Set Motor PWM 0-5000
//=================================================================================
void CMotors::DebugSetMotorPwm(uint8_t Motor,uint16_t Value)
{
	switch(Motor)
	{
	case FRONT_RIGHT_MOTOR:
		m_Motors.FrontRightMotor.PwmValue = Value;
		break;
	case  FRONT_LEFT_MOTOR:
		m_Motors.FrontLeftMotor.PwmValue = Value;
		break;
	case REAR_RIGHT_MOTOR:
		m_Motors.FrontRightMotor.PwmValue = Value;
		break;
	case REAR_LEFT_MOTOR:
		m_Motors.RearLeftMotor.PwmValue = Value;
		break;
	case ALL_MOTORS:
		m_Motors.FrontRightMotor.PwmValue = Value;
		m_Motors.FrontLeftMotor.PwmValue = Value;
		m_Motors.RearRightMotor.PwmValue = Value;
		m_Motors.RearLeftMotor.PwmValue = Value;
		break;
	case LEFT_MOTORS:
		m_Motors.RearRightMotor.PwmValue = Value;
		m_Motors.RearLeftMotor.PwmValue = Value;
		break;
	case RIGHT_MOTORS:
		m_Motors.FrontRightMotor.PwmValue = Value;
		m_Motors.FrontLeftMotor.PwmValue = Value;
		break;
	
	}
}




void CMotors::AllMotorsStop()
{
	FrontRightMotorPwm.SetValue( 0);
	FrontLeftMotorPwm.SetValue(0);
	RearRightMotorPwm.SetValue(0);
	RearLeftMotorPwm.SetValue(0);
	m_SpeedLastMs = Core.millis();
	m_IsUpdatePending = false;

}


void CMotors::SpinToAngle(uint16_t Speed,Direction_t Direction,float Angle)
{
// 	uint8_t WheelTicks =Angle*1.85;    // set tick count for angle ticks = (Pi/180 *w)*106 turn radius = 502
// 	
// 	SetTravelDirection(Direction);
// 	m_Motors.FrontRightMotor.PwmValue = round(abs(Speed));
// 	m_Motors.FrontLeftMotor.PwmValue = round(abs(Speed));
// 	m_Motors.RearRightMotor.PwmValue = round(abs(Speed));
// 	m_Motors.RearLeftMotor.PwmValue = round(abs(Speed));
// 	m_IsUpdatePending = true;
// 	WheelEncoder.ResetWheelSensor(BOTH_WHEELS,WheelTicks);
// 	Update();
// 	while(!WheelEncoder.m_LeftWheelSensor.Trigger)
// 	{
// 		Core.delayMicroseconds(50);
// 	}
// 	AllMotorsStop();
}


uint16_t CMotors::SetSpeedMph(float Mph)
{
	uint16_t Pwm;
	Pwm = (2803.4*Mph)+197.41;
	return(Pwm);
}

uint16_t CMotors::SetPwmRadPerSec(float RadPerSec)
{
	uint16_t Pwm;
	Pwm = (709.3*RadPerSec)+198.61;
	return(Pwm);
}












bool CMotors::IsMotorsStopped()
{
	if(m_Motors.TravelDirection == DIR_STOP)
		return(true);
	return(false);
}






void CMotors::CheckSafeSpeed()
{
//	uint16_t SafeSpeed;
//	int16_t TurnAngle =0;
	CMyMath Math;
	
	if( !Config.m_RunningFlags.ARMED)
	{
		m_MotorThrottleValue = THROTTLE_SAFE;	// boat not armed so motor off
		return;
	}
	
	
	if( m_MotorThrottleValue <STOP_UPPER_LIMIT) // mid position
	{
		m_MotorThrottleValue = THROTTLE_SAFE;
		return;
	}

	if(m_Motors.TravelDirection == DIRECTION_FORWARD)
	{
		CheckPitchAndRoll();
		
		if(Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT)
			m_MotorThrottleValue = min(m_MotorThrottleValue,Config.m_NavigationConfig.CruiseSpeed);
	}
}



//===================================================================
// used bu GUI
//===================================================================
void CMotors::PassthroughUpdate(UpdateMotors_t UpdateMotors)
{
	int8_t Direction  =1;
	if(!Config.m_RunningFlags.ARMED && Config.m_FunctionFlags.PassThroughFlag)
	{
		if(UpdateMotors.Direction >0)			Direction = -1;
		
		SetMotorPwm(FRONT_RIGHT_MOTOR,UpdateMotors.FrontRightMotor*Direction);
		SetMotorPwm(FRONT_LEFT_MOTOR,UpdateMotors.FrontLeftMotor*Direction);
		SetMotorPwm(REAR_RIGHT_MOTOR,UpdateMotors.RearRightMotor*Direction);
		SetMotorPwm(REAR_LEFT_MOTOR,UpdateMotors.RearLeftMotor*Direction);
	}
}




void CMotors::SetFailsafeSpeed()
{
	//SetMotorSpeed(ALL_MOTORS,Config.m_MainConfig.FailsafeThrottleValue);
}





void CMotors::SetUpdate(bool State)
{
	m_IsUpdatePending = State;
}





void CMotors::TestMotorDirection()
{
	TestMotors();
	while (1)
	{
		OutputThrottle(RIGHT_MOTORS, 15);
		OutputThrottle(LEFT_MOTORS, 15);

		SetMotorDirection(FRONT_RIGHT_MOTOR,MOTOR_FORWARD);
		SetMotorDirection(FRONT_RIGHT_MOTOR,MOTOR_REVERSE);
		
		SetMotorDirection(FRONT_LEFT_MOTOR,MOTOR_FORWARD);
		SetMotorDirection(FRONT_LEFT_MOTOR,MOTOR_REVERSE);
		
		SetMotorDirection(REAR_RIGHT_MOTOR,MOTOR_FORWARD);
		SetMotorDirection(REAR_RIGHT_MOTOR,MOTOR_REVERSE);
		
		SetMotorDirection(REAR_LEFT_MOTOR,MOTOR_FORWARD);
		SetMotorDirection(REAR_LEFT_MOTOR,MOTOR_REVERSE);
		
		SetMotorDirection(RIGHT_MOTORS,MOTOR_FORWARD);
		SetMotorDirection(RIGHT_MOTORS,MOTOR_REVERSE);
		
		SetMotorDirection(LEFT_MOTORS,MOTOR_FORWARD);
		SetMotorDirection(LEFT_MOTORS,MOTOR_REVERSE);
		
		SetMotorDirection(ALL_MOTORS,MOTOR_FORWARD);
		SetMotorDirection(ALL_MOTORS,MOTOR_REVERSE);

		OutputThrottle(RIGHT_MOTORS, 0);
		OutputThrottle(LEFT_MOTORS, 0);

	}

}


void CMotors::CheckPitchAndRoll()
{
//	uint8_t RollAngle = abs(Ahrs.m_AhrsValues.Roll);
//	uint8_t PitchAngle = abs(Ahrs.m_AhrsValues.Pitch);
// 	if(RollAngle > Config.m_NavigationConfig.MaxNavRoll || PitchAngle > Config.m_NavigationConfig.MaxPitchAngle)
// 	{
// 		m_MotorThrottleValue = min(m_MotorThrottleValue,Config.m_NavigationConfig.TurnRadius);
// 	}
}








void CMotors::TestMotors()
{
	 Config.m_FunctionFlags.PassThroughFlag = true;
	uint16_t Pwm = 15;
	while(1)
	{
		OutputThrottle(RIGHT_MOTORS, Pwm);
		OutputThrottle(RIGHT_MOTORS, Pwm*-1);
		
		OutputThrottle(LEFT_MOTORS, Pwm);
		OutputThrottle(LEFT_MOTORS, Pwm*-1);
		
		OutputThrottle(RIGHT_MOTORS, 0);
		OutputThrottle(LEFT_MOTORS, 0);
	}
}



void CMotors::UpdatePassThrough()
{
	if(Config.m_FunctionFlags.MotorReverseFlag) // for set by GUI
	SetTravelDirection(DIRECTION_REVERSE) ;
	else
	SetTravelDirection(DIRECTION_FORWARD) ;
}



// check if speed controller active
bool CMotors::CheckSpeedControlUpdate() 
{
	// True if there have been recent calls to speed controller
	if ((m_SpeedLastMs == 0) || (Core.millis() - m_SpeedLastMs) > MOTOR_UPDATE_TIMEOUT_MS)
		return(false);
	m_SpeedLastMs = 0;
	return( true);
}
void CMotors::CheckOutputs()
{
		PinPeripheral.SetPinMode(PORTA,16, OUTPUT);		// 48	Front Right Motor DIR
	PinPeripheral.SetPinMode(PORTB,14, OUTPUT);		// 49	Front Left Motor DIR
	PinPeripheral.SetPinMode(PORTA,20, OUTPUT);		// 46	Rear Left Motor DIR
	PinPeripheral.SetPinMode(PORTA,21, OUTPUT);		// 47	Rear Right Motor DIR
	PinPeripheral.SetPin(PORTA, 16, HIGH );
	PinPeripheral.SetPin(PORTA, 16, LOW );
	
	PinPeripheral.SetPin(PORTA, 16, LOW );
	PinPeripheral.SetPin(PORTA, 16, HIGH );
	PinPeripheral.SetPin(PORTA, 16, LOW );

	PinPeripheral.SetPin(PORTA, 14, LOW );
	PinPeripheral.SetPin(PORTA, 14, HIGH );
	PinPeripheral.SetPin(PORTA, 14, LOW );

	PinPeripheral.SetPin(PORTA, 20, LOW );
	PinPeripheral.SetPin(PORTA, 20, HIGH );
	PinPeripheral.SetPin(PORTA, 20, LOW );

	PinPeripheral.SetPin(PORTA, 21, LOW );
	PinPeripheral.SetPin(PORTA, 21, HIGH );
	PinPeripheral.SetPin(PORTA, 21, LOW );

 }
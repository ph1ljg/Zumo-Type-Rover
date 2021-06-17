/* 
* CWheelEncoder.cpp
*
* Created: 29/10/2020 12:18:56
* Author: philg
*/


#include "Includes.h"

volatile float WhDebug;


// default constructor
CWheelEncoder::CWheelEncoder()
{
} //CWheelEncoder

// default destructor
CWheelEncoder::~CWheelEncoder()
{
} //~CWheelEncoder



void CWheelEncoder::Init()
{
	SetInterupts.SetInterrupt(PORTA,4,EXTERNAL_INT_4,EXTERNAL_INT_4,FALLING); //left Wheel encoder
	SetInterupts.SetInterrupt(PORTA,18,EXTERNAL_INT_2,EXTERNAL_INT_2,FALLING); // right wheel encoder
	ResetWheelSensor(BOTH_WHEEL_SENSORS);
// 	CopyIrqState();
// 	Motors.SetMotorPwm(FRONT_LEFT_MOTOR,30);
// 	while(1)
// 	{
// 		Core.delay(10);
// 	}
//	CalibrateRightWheelSensor();
}


void  CWheelEncoder::UpDate()
{
	if(Config.m_RunningFlags.ENABLE_WHEEL_SENSORS)
		CopyIrqState();
}



//=========================================================
// Used by interrupt
void  CWheelEncoder::IncrementRightWheel()
{
	m_RightWheelSensor.DistanceCount++;
	m_RightWheelSensor.TotalCount++;
}

void  CWheelEncoder::IncrementLeftWheel()
{
	m_LeftWheelSensor.DistanceCount++;
	m_LeftWheelSensor.TotalCount++;
//	CalibrateRightWheelSensor();
}
//==========================================================

void CWheelEncoder::CopyIrqState()
{
	static uint32_t LastTime =0;
//	uint32_t Now = Core.millis();

	
	SetInterupts.SetEnable(EXTERNAL_INT_4,false);		//Disable right interrupt
	SetInterupts.SetEnable(EXTERNAL_INT_2,false);		//Disable Left interrupt

	m_SampleTime = (Core.millis() - LastTime);
	LastTime = Core.millis();
	
	m_RightDistanceCount	= m_RightWheelSensor.DistanceCount;
	m_LeftDistanceCount		= m_LeftWheelSensor.DistanceCount;
	m_RightTotalCount = m_RightWheelSensor.TotalCount;
	m_LeftTotalCount = m_LeftWheelSensor.TotalCount;
	m_RightWheelSensor.DistanceCount = 0;
	m_LeftWheelSensor.DistanceCount = 0;
	SetInterupts.SetEnable(EXTERNAL_INT_2,true);		//Enable right interrupt
	SetInterupts.SetEnable(EXTERNAL_INT_4,true);		//Enable Left interrupt
	
	float DeltaRight = M_2PI * (m_RightDistanceCount / (float)COUNTS_PER_REVOLUTION);
	float DeltaLeft   = M_2PI * (m_LeftDistanceCount / (float)COUNTS_PER_REVOLUTION);
	
	m_RightDistance = DeltaRight*Config.m_NavigationConfig.WheelRadius;  
	m_LeftDistance =  DeltaLeft*Config.m_NavigationConfig.WheelRadius;
	
	m_RightWheelRate =  DeltaRight / (m_SampleTime * 1e-3f);  	// calculate delta_angle (in radians) per second
	m_LeftWheelRate =  DeltaLeft / (m_SampleTime * 1e-3f);

	m_RightWheelMeters_Sec =  ((m_RightDistance/1000) / (m_SampleTime * 1e-3f)); // to meters/s
	m_LeftWheelMeters_Sec =  ((m_LeftDistance /1000)/ (m_SampleTime * 1e-3f)); // to meters/s

	m_MPH_Left = m_LeftWheelMeters_Sec*2.237;
	m_MPH_Right = m_RightWheelMeters_Sec*2.237;

}

void CWheelEncoder::ResetWheelSensor(Wheelsensor_t Wheel)
{
	if(Wheel == RIGHT_WHEEL_SENSOR || Wheel == BOTH_WHEEL_SENSORS)
	{
		SetInterupts.SetEnable(EXTERNAL_INT_4,false);		//Disable Right interrupt
		m_RightWheelSensor.TotalCount = 0;
		m_RightWheelSensor.DistanceCount = 0;
		SetInterupts.SetEnable(EXTERNAL_INT_4,true);		//Enable Right interrupt
}

	if(Wheel == LEFT_WHEEL_SENSOR || Wheel == BOTH_WHEEL_SENSORS)
	{
		SetInterupts.SetEnable(EXTERNAL_INT_2,false);		//Disable Left interrupt
		m_LeftWheelSensor.TotalCount = 0;
		m_LeftWheelSensor.DistanceCount = 0;
		SetInterupts.SetEnable(EXTERNAL_INT_2,true);		//Enable Left interrupt
	}
}

// Get total delta angle (in radians) measured by the wheel encoder
// float CWheelEncoder::GetDeltaAngle(Wheelsensor_t Sensor) const
// {
// 	if(Sensor == RIGHT_WHEEL_SENSOR)
// 		return M_2PI * (float)m_RightWheelSensor.DistanceCount / COUNTS_PER_REVOLUTION;
// 	else
// 		return M_2PI * (float)m_LeftWheelSensor.DistanceCount / COUNTS_PER_REVOLUTION;
// }
// 
// Get the total distance traveled in meters
// float CWheelEncoder::GetRps(Wheelsensor_t Sensor) const
// {
// 	
// 	return GetDeltaAngle(Sensor) * WHEEL_RADIUS;
// }



// Get the total distance traveled in meters
float CWheelEncoder::GetDistance(Wheelsensor_t Sensor) const
{
	if(Sensor == RIGHT_WHEEL_SENSOR)
		return m_RightDistance;
	else if(Sensor == LEFT_WHEEL_SENSOR)
		return(m_LeftDistance);
	else
		return(max(m_LeftDistance,m_RightDistance));	
}

float CWheelEncoder::GetMetersSec(Wheelsensor_t Sensor) 
{
	float Speed;


	if(Sensor == RIGHT_WHEEL_SENSOR )
		Speed = m_RightWheelMeters_Sec;

	else if(Sensor == LEFT_WHEEL_SENSOR  )
		Speed = m_LeftWheelMeters_Sec;
	
	else 
		Speed = max(m_RightWheelMeters_Sec,m_LeftWheelMeters_Sec);
	m_NewData = false;
	return(Speed);
}


float CWheelEncoder::GetMph(Wheelsensor_t Sensor) 
{
	float Mph;
	Mph= GetMetersSec(Sensor);
	return(Mph/0.44704);	
}


// get the instantaneous rate in radians/second
float CWheelEncoder::GetRate(Wheelsensor_t Wheel) const
{
		float Velocity;
		if(Wheel == RIGHT_WHEEL_SENSOR )
			Velocity = m_RightWheelRate;

		else if(Wheel == LEFT_WHEEL_SENSOR  )
			Velocity = m_LeftWheelRate;
		else
			Velocity = max(m_RightWheelRate,m_LeftWheelRate);
		return(Velocity);
}



// get the system time (in milliseconds) of the last update
// uint32_t CWheelEncoder::GetLastSample_ms(Wheelsensor_t Wheel) const
// {
// 	if(Wheel == RIGHT_WHEEL_SENSOR || Wheel == BOTH_WHEEL_SENSORS)
// 		return( m_RightWheelSensor.SampleTime);
// 	
// 	return( m_LeftWheelSensor.SampleTime);
// }



void CWheelEncoder::CalibrateRightWheelSensor()
{
	Motors.SetMotorPwm(FRONT_LEFT_MOTOR,20);
	Motors.SetMotorPwm(FRONT_RIGHT_MOTOR,20);
	uint32_t Starttime = 0;
	uint32_t DisplayTime = 0;
	
	
	while(1)
	{
		if((Core.millis() -Starttime) > 500)      // was 20
		{
			CopyIrqState();
// 			if(m_LeftWheelSensor.DistanceCount >=COUNTS_PER_REVOLUTION)
// 			{
// 										
// 			}
			Starttime = Core.millis();
		}
		
		if((Core.millis() -DisplayTime) > 500)   //if((Core.millis() -DisplayTime) > 2000)
		{
			DebugDisplay.DisplayWheelSensors();
			ResetWheelSensor(LEFT_WHEEL_SENSOR);
			ResetWheelSensor(RIGHT_WHEEL_SENSOR);  // PSB 20/05/21
			DisplayTime = Core.millis();
		}
	}
}


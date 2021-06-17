/* 
* CSensors.cpp
*
* Created: 15/06/2016 12:23:53
* Author: phil
*/
#include "Includes.h"

extern CAnalog Analog;
extern CStatusControl LedControl;
CVL53L1X VL53L1X;

CSetInterupts SetInterupts;
uint16_t WcCount =0;

//unsigned long	Priority;


float Distance;  //inches 
bool fred2 = false;
uint16_t Count = 0;
bool End = false;

CSensors::CSensors()
{
	m_SensorValues.MahUsed =0;
} //CSensors


CSensors::~CSensors()
{
	m_BatteryCount = 2;
} //~CSensors


void CSensors::Init()
{
	Analog.Init();
	InitLidar();
//	CalibrateRightWheelSensor();
//	CalibrateLeftWheelSensor();
	
}


void  CSensors::UpDate()
{
	GetBatteryVolts();
	UpdateLidarValues();	
}




void  CSensors::GetMotorCurrent()
{ 
	
	return;
	CMyMath Math;
	uint8_t i = 0;
	uint16_t TmpVal =0;
	static uint32_t RunningTotal =0;
	for(i=0;i<5;i++)
	{
		TmpVal += (uint16_t)  (Analog.AnalogReadVolts(ADC1,PORTC,3)*100);//-0.177;
	}
	
	RunningTotal = (TmpVal/5);
	if(RunningTotal <= 14)
	{
		m_SensorValues.Current = 0;
		return;	
	}
		
	RunningTotal -=14;
//	RunningTotal *=39;
	m_SensorValues.Current = (RunningTotal/100.0);
	m_SensorValues.Current *=39;

	if(m_SensorValues.Current <1.6)
		printf(" ");
}




void  CSensors::GetBatteryVolts()
{
	CMyMath Math;
	static uint8_t FailSafeCriticalCount = 0;
//	static uint8_t FailSafeWarningCount = 0;
	float TmpVal;
//	float Scale = (float)(Config.m_MainConfig.Batt_1_Scale/100.00); ***********************************************************************
	float Scale = 2.55f;
	TmpVal = (float)Analog.AnalogReadVolts(ADC0,PORTB,9);
	TmpVal *= Scale;
	m_SensorValues.VoltsBattery_1 = (uint16_t)(Math.Round(TmpVal*10));
	
	if(m_SensorValues.VoltsBattery_1 <Config.m_MainConfig.Battery_1_Critical)
	{
		if( !Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL)
		{
			if(++FailSafeCriticalCount > 4)
			{
				Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL	= true;
				Config.m_RunningFlags.BATTERY_FAILSAFE			= true;
				Config.m_RunningFlags.BATTERY_1_WARNING			= false;
				Buzzer.Tone();
			}
		}
		return;
	}
	else
		Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL	= false;
	
}




void  CSensors::InitLidar()
{
	if(!VL53L1X.Init(VL53L1X.DistanceMode::Medium))
	{
		Config.m_RunningFlags.RIGHT_LIDAR_FAIL = true;
	}

}





void CSensors::UpdateLidarValues()
{
	if(VL53L1X.Update())
	{
		m_SensorValues.RearLidar = VL53L1X.m_DistanceReading;

		if(m_SensorValues.RearLidar < REAR_DISTANCE_ALARM_VALUE )
			Config.m_RunningFlags.REAR_DISTANCE_ALARM = true;
		else
			Config.m_RunningFlags.REAR_DISTANCE_ALARM = false;
		Config.m_RunningFlags.REAR_LIDAR_FAIL = false;
	}
	else
		Config.m_RunningFlags.REAR_LIDAR_FAIL = true;
}

	





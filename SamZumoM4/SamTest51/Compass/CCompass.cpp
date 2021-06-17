/* 
* CCompass.cpp
*
* Created: 11/03/2021 18:41:08
* Author: philg
*/

#include "Includes.h"
#include "CCmps12.h"
//CCmps12 Cmps12;

// default constructor
CCompass::CCompass()
{
} //CCompass

// default destructor
CCompass::~CCompass()
{
} //~CCompass


// Default init method
//
void CCompass::init()
{
	Core.delay(100);
	UpDate();

	init_done = true;
}



bool CCompass::UpDate(void)
{
	CompassState.Healthy = false;
	if(Update())                    // cmp12 update
	{
		
		uint32_t time = Core.millis();
		bool any_healthy = false;
		CompassState.Healthy = (time - CompassState._last_update_ms < 500);
	}

	UpdateCalState();
	return( healthy());
}

bool CCompass::SaveCalibration()
{
	return(StoreCalibration());
}



bool CCompass::UpdateCalState()
{
	return(CCmps12::UpdateCalState());
}

float CCompass::GetHeadingDegrees(void)
{
	return((float)m_CmpsData.HeadingDeg/10.0);
}

uint16_t CCompass::GetHeading()
{
	return(m_CmpsData.HeadingDeg);
}
float CCompass::GetPitch()
{
	return(m_CmpsData.Pitch);
}
float CCompass::GetRoll()
{
	return(m_CmpsData.Roll);
}

uint16_t CCompass::GetTemperature()
{
	return(m_CmpsData.Temperature);
}

float CCompass::GetTurnRate()
{
	return(m_AngularVelocity);
}

float CCompass::UpdateTurnRate()
{
	static uint32_t LastTime =0;
	static float LastHeading =GetHeadingDegrees();
	float NowHeading = GetHeadingDegrees();
	uint32_t NowTime = Core.millis(); 
	float TempVar;
	uint32_t Time;
/*	uint32_t NowTime = Core.millis();*/
	if(LastHeading != NowHeading)
	{
		uint32_t NowTime = Core.millis();  // PSB 25/05/21 moved for slightly more accurate time measurement
	//	if((NowHeading+10) < LastHeading)||(NowHeading-10) > LastHeading)
		TempVar = abs(NowHeading- LastHeading);
		TempVar = wrap_360(TempVar);		
		Time = NowTime - LastTime;
		TempVar = radians(TempVar);
		m_AngularVelocity = (TempVar/Time)*1000;   // PSB 25/05/21 time is in mS so *1000 = seconds. Was  m_AngularVelocity = TempVar/Time*1000;  // radian Per sec
		LastHeading= NowHeading;
		LastTime = NowTime;                     // PSB 30/05/21
		return(TempVar);
	}
	
	return(0);
}




bool CCompass::GetGyroCal()
{
	return(m_CmpsData.GyroInCal);
}

bool CCompass::GetAccelCal()
{
	return(m_CmpsData.AccelInCal);
}

bool CCompass::GetMagCal()
{
	return(m_CmpsData.MagInCal);
}

bool CCompass::GetSysCal()
{
	return(m_CmpsData.SystemInCal);
}

/* 
* CGpsModule.cpp
*
* Created: 24/04/2017 12:05:12
* Author: phil
*/
#include "Includes.h"


uint8_t GpsDebug =0;
uint8_t GpsDebug1 =0;
CUblox Ublox;

extern CCore Core;
// default constructor

CGps::CGps(CUart * Uart)
{
	m_Uart = Uart;
	GPS_filter_index	= 0;
	m_Debug				= false;
	m_Navfilter			= GPS_ENGINE_PEDESTRIAN;
	m_MinElevation		=  -100;
	m_GnssMode			= 103;					// GPS+GLONASS+SBAS
	m_SbasMode			= 1;					// set to on
	m_CanSaveConfig		= 0;					// No save config as most modules do not support this
	m_LastConfigTime	= 0;
	m_ConfigSaved		= false;
	m_GpsStatus		= GPS_NO_FIX;
} //CGpsModule

// default destructor
CGps::~CGps()
{
} //~CGps



void CGps::Init()
{
	m_Uart->Init(9600);
	m_Uart->ClearRxBuffer();
	m_Uart->ClearTxBuffer();
	Core.delay(1000);

	if(Ublox.Init(m_Uart))
	{
		m_Uart->ClearRxBuffer();
		m_Uart->ClearTxBuffer();
		Config.m_RunningFlags.GPS_ONLINE = true;
	}
}



void CGps::Update()
{
	static bool IsFirst = true;
	
	if(IsFirst)
	{
		IsFirst = false;
		m_Uart->ClearRxBuffer();
		m_Uart->ClearTxBuffer();
		return;
	}
//	return;
	int32_t	PositionReadings[2];
//	static uint32_t LastTime =0;
	unsigned char Data;
	uint8_t axis;
	uint16_t fraction3[2];
//	static unsigned char LockCount =0;
	static unsigned char FailCount =0;
	bool GoodPacket = false;
	
	if (m_Update)
		m_Update = false;		// flashes Pkt led on GUI
	else
		m_Update = true;

	if(Ublox.m_UnconfiguredMessages)
		DoConfig();
	else
	{
		if(!m_ConfigSaved)
			SaveConfig();									// save config if possible (some units are Rom based)
	}
//	return;
	while (m_Uart->Read(&Data))
	{
		if (Ublox.GetNewFrame(Data))
		{
			// Now have a valid GGA frame and we have lat and lon in GPS_read_lat and GPS_read_lon, apply moving average filter
			// this is a little bit tricky since the 1e7/deg precision easily overflow a long, so we apply the filter to the fractions
 			// only, and strip the full degrees part. This means that the filter must be disabled if when very close to a degree line

			if (Config.m_NavigationConfig.NavigationFlags & NAV_FLAG_GPS_FILTER)       //is filtering switched on ?
			{
				PositionReadings[0] = m_GpsPositionReadings.Latitude;
				PositionReadings[1] = m_GpsPositionReadings.Longitude;
				GPS_filter_index = (GPS_filter_index + 1) % GPS_FILTER_VECTOR_LENGTH;
						
				for (axis = 0; axis< 2; axis++)
				{
					GPS_degree[axis] = PositionReadings[axis] / 10000000;  // get the degree to assure the sum fits to the int32_t
							
					// How close  to a degree line ? its the first three digits from the fractions of degree
					//Check if  close to a degree line, if yes, disable averaging,
					fraction3[axis] = (PositionReadings[axis]- GPS_degree[axis]*10000000) / 10000;
							
					GPS_filter_sum[axis] -= GPS_filter[axis][GPS_filter_index];
					GPS_filter[axis][GPS_filter_index] = PositionReadings[axis] - (GPS_degree[axis]*10000000);
					GPS_filter_sum[axis] += GPS_filter[axis][GPS_filter_index];
					GPS_filtered[axis] = GPS_filter_sum[axis] / GPS_FILTER_VECTOR_LENGTH + (GPS_degree[axis]*10000000);
				}
				if ( fraction3[LAT]>1 && fraction3[LAT]<999 )
					m_Coords.Latitude = GPS_filtered[LAT];
				else
					m_Coords.Latitude = m_GpsPositionReadings.Latitude;
							
				if ( fraction3[LON]>1 && fraction3[LON]<999 )
					m_Coords.Longitude = GPS_filtered[LON];
				else
					m_Coords.Longitude = m_GpsPositionReadings.Longitude;
			}
			else
			{ // ignore filtering since it switched off in the nav_flags
				m_Coords.Latitude = m_GpsPositionReadings.Latitude;
				m_Coords.Longitude = m_GpsPositionReadings.Longitude;
			}
			
			if (m_GpsReadings.Update) 
				m_GpsReadings.Update = false;		// flashes Pkt led on GUI
			else 
				m_GpsReadings.Update = true;
			GoodPacket = true;
			FailCount = 0;
			break;
			
		} // new frame
				
	} //while
	if(!GoodPacket)
	{
		if(++FailCount > 100)
		{
			FailCount =100;
			m_GpsStatus = GPS_FAILED;
		}
		return;
	}

	CheckGpsLock();
}


void CGps::CheckGpsLock()
{
	static uint32_t LastTime = 0;
//	GPS_Status_t RetVal = GPS_TEMP_LOS;
	bool ErrorFail = false;
	bool ErrorLock = false;
	uint32_t NowGpsTime =0;
	static unsigned long LastGpsTime = 0xffffffff;
//	static unsigned char Count =0;
	NowGpsTime = (unsigned long) m_GpsTimeValues.hour*3600 +m_GpsTimeValues.minute*60+m_GpsTimeValues.seconds;
	if(NowGpsTime > 86405 || (LastGpsTime - NowGpsTime) ==0) // corruption check should not be > 24 hours 0r 0
	{
		ErrorFail = true;
	}
	LastGpsTime = NowGpsTime;


	if(m_GpsReadings.GpsFixType == GPS_OK_FIX_2D)
		m_GpsStatus = GPS_OK_FIX_2D;
	else if(m_GpsReadings.GpsFixType == GPS_OK_FIX_3D)
		m_GpsStatus = GPS_OK_FIX_3D;
	else if(m_GpsReadings.GpsFixType == GPS_OK_FIX_DGPS)
		m_GpsStatus = GPS_OK_FIX_DGPS;	
	else if(m_GpsReadings.GpsFixType == GPS_OK_FIX_3D_RTK_FLOAT)
		m_GpsStatus = GPS_OK_FIX_3D_RTK_FLOAT;
	else if(m_GpsReadings.GpsFixType == GPS_OK_FIX_3D_RTK_FIXED)
		m_GpsStatus = GPS_OK_FIX_3D_RTK_FIXED;
	else
	   ErrorLock = true;

	if(ErrorFail || ErrorLock)
	{
		if((Core.millis() - LastTime) >4000)
		{
			if(ErrorFail )
				m_GpsStatus = GPS_FAILED;
			else
				m_GpsStatus = GPS_NO_FIX;
			return;		
		}
		else
			return;
	}   
	LastTime = Core.millis();
}


GPS_Status_t CGps::GetStatus()
{
	return(m_GpsStatus);
}


bool CGps::IsGpsUsable()
{
	switch(m_GpsStatus)
	{
	case GPS_FAILED:
	case GPS_NO_FIX:
	case GPS_OK_FIX_TIME:
	case GPS_TEMP_LOS:
		return(false);
		break;	
	case GPS_OK_FIX_2D:
	case GPS_OK_FIX_3D:
	case GPS_OK_FIX_DGPS:
	case GPS_OK_FIX_3D_RTK_FLOAT:
	case GPS_OK_FIX_3D_RTK_FIXED:
		return(true);
		break;
	}
	return(false);
}


uint32_t CGps::GroundSpeed()
{
	return(m_GpsReadings.GroundSpeed);
}

int32_t CGps::GroundCourseCD()
{
	return(Gps.m_GpsReadings.GroundCourse*100);
}

int32_t CGps::GroundCourse()
{
	return(Gps.m_GpsReadings.GroundCourse);
}



void CGps::DoConfig()
{
//	  static uint8_t NextConfigMask =0;
//	  uint8_t data;
//	  int16_t numc;
	  static uint32_t DelayTime = 0;
//	  bool parsed = false;
	  uint32_t millis_now = Core.millis();

	  // walk through the gps configuration at 1 message per second
	  if (millis_now - m_LastConfigTime >= DelayTime) 
	  {
		  m_LastConfigTime = millis_now;
		  if(!Ublox.m_UnconfiguredMessages)
			return;
		  
		  Ublox.RequestNextConfig();
		  if ( Ublox.m_UnconfiguredMessages)		// send the updates faster until fully configured
		  { 
			  if (Ublox.m_NextMessage < STEP_PORT) // blast the initial settings out
				  DelayTime = 0;
			  else 
				  DelayTime = 750;
		  } 
		  else 
			  DelayTime = 2000;
	  }
}

bool CGps::SaveConfig()
{
	if(!m_CanSaveConfig)
	{
		m_ConfigSaved  = true;
		return(true);
	}
	 uint32_t millis_now = Core.millis();
	if(!Ublox.m_UnconfiguredMessages  && !Ublox.m_CfgSaved &&	  Ublox.m_NumCfgSaveTries < 5 && (millis_now - Ublox.m_LastCfgSentTime) > 5000 ) 
	{
		//save the configuration sent until now
		if (m_CanSaveConfig == 1 ||  (m_CanSaveConfig == 2 && Ublox.m_CfgNeedsSave)) 
		{
			Ublox.SaveCfg();
		}
		m_ConfigSaved = true;
		return(true);
	}
	return(false);
}



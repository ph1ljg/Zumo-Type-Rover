/*
* CConfig.cpp
*
* Created: 04/06/2016 10:35:30
* Author: phil
*/
#include "Includes.h"

unsigned char ChkSum;
unsigned int DebugInt;

#define 	RC_THROTTLE 2

#define SBUS_CONFIG

CQSpiFlash QSpiFlash;


// default constructor
CConfig::CConfig()
{
	InitRunningFlags();
	m_RealTimeGuiFlags	= 0;
	ClearWaypoints();
	m_LastWaypoint		= 0;

} //CConfig

// default destructor
CConfig::~CConfig()
{
} //~CConfig




bool CConfig::Init()
{
	QSpiFlash.Init();
	return(ReadAllConfigs());
}


void CConfig::InitRunningFlags()
{
	memset(&m_RunningFlags,0,sizeof(m_RunningFlags));
	Config.m_RunningFlags.LOST_RC_SIGNAL	= true;
	m_RunningFlags.STICK_MIXING = true;

}



bool CConfig::ReadAllConfigs()
{
	bool RetVal = true;
	if(!ReadNavigationConfig())
	{
		m_RunningFlags.RD_NAVIGATION_CFG_FAIL = true;
		LoadDefaultNavigationConfig();
		RetVal = false;
	}
	if(!ReadMainConfig())
	{
		m_RunningFlags.RD_MAIN_CONFIG_FAIL = true;
		LoadDefaultConfig(true);
		RetVal = false;
	}
	if(!ReadRcConfig())
	{
		m_RunningFlags.RD_RC_CONFIG_FAIL = true;
		LoadDefaultRcConfig(true);
		RetVal = false;
	}

	if(!ReadFunctionFlagsConfig())
	{
		m_RunningFlags.RD_FUNCTION_CONFIG_FAIL = true;
		LoadDefaultFunctionFlags();
		RetVal = false;
	}

	if(!ReadCourseFile())
	{
		m_RunningFlags.LOAD_WAYPOINTS_FAIL = true;
	}
	if(RetVal)
	m_RunningFlags.ALL_CONFIGS__LOADED = true;
	return(RetVal);
}


void CConfig::WriteAllConfigs()
{
	WriteNavigationConfig();
	WriteMainConfig();
}

bool CConfig::ReadNavigationConfig()
{
	ChkSum = QSpiFlash.readBuffer(NAVIGATION_ADDRESS,(uint8_t*)&m_NavigationConfig, sizeof(NavigationConfig_t));
	ChkSum = CalculateConfigChecksum((unsigned char*)&m_NavigationConfig, sizeof(m_NavigationConfig)-1);
	if(ChkSum != m_NavigationConfig.checksum)
	return(false);
	return(true);
}


bool  CConfig::WriteNavigationConfig()
{
	m_NavigationConfig.checksum = CalculateConfigChecksum((unsigned char*)&m_NavigationConfig, sizeof(m_NavigationConfig)-1);
	QSpiFlash.eraseSector (1);
	QSpiFlash.writeBuffer (NAVIGATION_ADDRESS,(uint8_t*) &m_NavigationConfig, sizeof(NavigationConfig_t));
	Core.delay(100);
	return(ReadNavigationConfig());
	
}

bool CConfig::ReadMainConfig()
{
	QSpiFlash.readBuffer(MAIN_CONFIG_ADDRESS,(uint8_t*)&m_MainConfig, sizeof(MainConfig_t));
	ChkSum = CalculateConfigChecksum((unsigned char*)&m_MainConfig, sizeof(MainConfig_t)-1);
	if( ChkSum != m_MainConfig.checksum)
	return false;
	return true;    // setting is OK
}


bool CConfig::WriteMainConfig()
{
	m_MainConfig.checksum = CalculateConfigChecksum((unsigned char*)&m_MainConfig, sizeof(MainConfig_t)-1);
	QSpiFlash.eraseSector (0);
	QSpiFlash.writeBuffer (MAIN_CONFIG_ADDRESS,(uint8_t*) &m_MainConfig, sizeof(MainConfig_t));
	return(ReadMainConfig());
}


bool CConfig::ReadRcConfig()
{
	uint8_t i;
	CMyMath Math;
	QSpiFlash.readBuffer(RC_CONFIG_ADDRESS,(uint8_t*)&m_RcChannels, sizeof(RcChannels_t));

	ChkSum = CalculateConfigChecksum((unsigned char*)&m_RcChannels, sizeof(RcChannels_t)-1);
	if( ChkSum != m_RcChannels.checksum)
	{
		m_RunningFlags.RD_RC_CONFIG_FAIL = true;
		return false;
	}
	for(i=0;i<16;i++)
	{
		m_RcChannels.ChannnelsMin[i] = Math.Constrain(m_RcChannels.ChannnelsMin[i],230,1810);
	}
	return true;    // setting is OK
}



bool CConfig::WriteRcConfig()
{
	CMyMath Math;
	uint8_t i;
	for(i=0;i<16;i++)
	{
		m_RcChannels.ChannnelsMin[i] = Math.Constrain(m_RcChannels.ChannnelsMin[i],230,1810);
	}
	m_RcChannels.checksum = CalculateConfigChecksum((unsigned uint8_t*)&m_RcChannels, sizeof(RcChannels_t)-1);
	QSpiFlash.eraseSector (2);
	QSpiFlash.writeBuffer(RC_CONFIG_ADDRESS,(uint8_t*) &m_RcChannels, sizeof(RcChannels_t));
	return(ReadRcConfig());
}



bool CConfig::ReadFunctionFlagsConfig()
{
	QSpiFlash.readBuffer(FUNCTION_ADDRESS,(uint8_t*)&m_FunctionFlags, sizeof(FunctionFlags_t));

	ChkSum = CalculateConfigChecksum((unsigned char*)&m_FunctionFlags, sizeof(FunctionFlags_t)-1);
	if( ChkSum != m_FunctionFlags.checksum)
	return false;
	return true;    // setting is OK
}


bool CConfig::WriteFunctionFlagsConfig()
{
	CMyMath Math;
	m_FunctionFlags.checksum = CalculateConfigChecksum((unsigned uint8_t*)&m_FunctionFlags, sizeof(FunctionFlags_t)-1);
	QSpiFlash.eraseSector (3);
	QSpiFlash.writeBuffer(FUNCTION_ADDRESS,(uint8_t*) &m_FunctionFlags, sizeof(FunctionFlags_t));
	return(ReadFunctionFlagsConfig());
}








bool CConfig::LoadDefaultRcConfig(bool WriteToFlash)
{
	uint8_t i;
	#ifdef SBUS_CONFIG
	for(i=0;i<18;i++)
	{
		m_RcChannels.ChannnelsMax[i] = 1800;
		m_RcChannels.ChannnelsMin[i] = 1000;
		m_RcChannels.FailsafeValues[i] = 1500;
		m_RcChannels.DeadZone[i] = 10;
		m_RcChannels.Trim[i] = 1500;

	}
	m_RcChannels.FailsafeValues[RC_THROTTLE] = 1000;
	#else
	for(i=0;i<6;i++)
	{
		m_RcChannels.ChannnelsMax[i] = 2000;
		m_RcChannels.ChannnelsMin[i] = 1000;
		m_RcChannels.FailsafeValues[i] = 1500;
	}

	#endif
	m_RcChannels.FailsafeValues[RC_THROTTLE] = 1000;
	
	for(i=4;i<15;i++)
	{
		m_RcChannels.FailsafeValues[i] = 1000;
	}

	if(WriteToFlash)
	return(WriteRcConfig());
	return(true);
}




bool CConfig::LoadDefaultNavigationConfig()
{
	int i;
	for(i=0;i<3;i++)
	{
		m_NavigationConfig.AcelerometerOffsets[i]	= 0;
		m_NavigationConfig.GiroOffsets[i]			= 0;
		m_NavigationConfig.MagnetometerOffsets[i]	= 0;
	}
	m_NavigationConfig.DataFileNo					= 0;
	m_NavigationConfig.CourseFileNo				= 0;
	m_NavigationConfig.MagDeclination			= (int16_t)(MAG_DECLINATION * 10);
	m_NavigationConfig.WaypointRadius			= WAYPOINT_RADIUS;
	m_NavigationConfig.CruiseThrottle				= MANOEUVER_SPEED_VALUE;
	m_NavigationConfig.CrosstrackGain			= CROSSTRACK_GAIN;
	m_NavigationConfig.MaxNavRoll					= NAV_ROLL_MAX;
	m_NavigationConfig.MaxNavPitch				= NAV_PITCH_MAX;
	m_NavigationConfig.CruiseSpeed				= CRUISE_SPEED;
	m_NavigationConfig.SteerAccelMax			= STEER_ACCEL_MAX;
	m_NavigationConfig.MaxTurnRate				= MAX_TURN_RATE;
	m_NavigationConfig.WheelRadius				= WHEEL_RADIUS;	
	m_NavigationConfig.TurnRadius					= MAX_TURN_RADIUS;
	m_NavigationConfig.ReverseSpeed				= REVERSE_SPEED_VALUE;
	m_NavigationConfig.CruiseThrottle				= MANOEUVER_SPEED_VALUE;
	m_NavigationConfig.FenceDistance				= FENCE_DISTANCE;
	m_NavigationConfig.SafeWpDistance			= SAFE_WAYPOINT_DISTANCE;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_GPS_FILTER;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_GPS_LEAD_FILTER;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_ENABLE_RC_LOSS_RTH;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_ENABLE_RUDDER_RATE;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_ENABLE_WIND_SPEED;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_ENABLE_SONAR;
	m_NavigationConfig.NavigationFlags			&= ~NAV_FLAG_ENABLE_SLOW_NAV;
	m_NavigationConfig.ReadWaypointsFromSDcard	= false;
	m_NavigationConfig.ClearLog						= false;
	m_NavigationConfig.MaxNumberOfWayPoints		= 12;
	m_NavigationConfig.MaxPitchAngle				= 20;	// in degrees
	m_NavigationConfig.MaxRollAngle				= 40;	// in degrees
	m_NavigationConfig.MaximumSpeed		= 30;// 0 to 100 %
	m_NavigationConfig.CruiseThrottle				= CRUISE_THROTTLE;  // in % 0-100
	m_NavigationConfig.MaximumSpeed			= MAX_SPEED;  
	

	return(WriteNavigationConfig());

}



bool CConfig::LoadDefaultConfig(bool WriteToFlash)
{
	unsigned char i;
	m_MainConfig.MinThrottleValue				= MINTHROTTLE;
	m_MainConfig.FailsafeThrottleValue		= MIDTHROTTLE;
	m_MainConfig.MaxThrottleValue				= MAXTHROTTLE;
	m_MainConfig.Bat_1_LevelWarning		= BATTERY_1_WARNING_V;
	m_MainConfig.Bat_2_LevelWarning		= BATTERY_2_WARNING_V;
	m_MainConfig.Battery_1_Critical			= BATTERY_1_CRITICAL;
	m_MainConfig.Battery_2_Critical			= BATTERY_2_CRITICAL;
	m_MainConfig.Batt_1_Scale					= BATTERY_1_SCALE;
	m_MainConfig.Batt_2_Scale					= BATTERY_2_SCALE;
	for(i=0;i<NO_OF_PIDS;i++)
	{
		m_MainConfig.PidValues[i].PVal				= 1;
		m_MainConfig.PidValues[i].IVal				= 0.2f;
		m_MainConfig.PidValues[i].DVal				= 0.5f;
	}
	
	m_MainConfig.PidValues[STEERING_RATE_PID].PVal = ATTCONTROL_STEER_RATE_P;
	m_MainConfig.PidValues[STEERING_RATE_PID].IVal = ATTCONTROL_STEER_RATE_I;
	m_MainConfig.PidValues[STEERING_RATE_PID].DVal = ATTCONTROL_STEER_RATE_D;
	
	m_MainConfig.PidValues[THROTTLE_SPEED_PID].PVal = ATT_CONTROL_THR_SPEED_P;
	m_MainConfig.PidValues[THROTTLE_SPEED_PID].IVal = ATT_CONTROL_THR_SPEED_I;
	m_MainConfig.PidValues[THROTTLE_SPEED_PID].DVal = ATT_CONTROL_THR_SPEED_D;


	m_MainConfig.RcRate						= 0;
	m_MainConfig.RcExpo						= 0;
	for(i=0;i<NO_OF_STICK_FUNCTIONS;i++)
	m_MainConfig.SticksSettings[i]			= 0;
	
	for(i=0;i<NO_OF_AUX_FUNCTIONS;i++)
	m_MainConfig.AuxSwitchSettings[i]		= 0;
	
	m_MainConfig.SwitchSettings.Switch_1		= 0;
	m_MainConfig.SwitchSettings.Switch_2		= 0;
	m_MainConfig.SwitchSettings.Switch_3		= 0;
	m_MainConfig.SwitchSettings.Switch_4		= 0;

	m_MainConfig.ServoConf[SERVO_F_R_MOTOR].min		= F_R_MOTOR_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_F_R_MOTOR].max		= F_R_MOTOR_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_F_R_MOTOR].middle	= F_R_MOTOR_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_F_R_MOTOR].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_F_R_MOTOR].Rate	= 0;

	m_MainConfig.ServoConf[SERVO_F_L_MOTOR].min		= F_L_MOTOR_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_F_L_MOTOR].max		= F_L_MOTOR_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_F_L_MOTOR].middle	= F_L_MOTOR_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_F_L_MOTOR].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_F_L_MOTOR].Rate	= 0;

	m_MainConfig.ServoConf[SERVO_R_R_MOTOR].min		= R_R_MOTOR_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_R_R_MOTOR].max		= R_R_MOTOR_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_R_R_MOTOR].middle	= R_R_MOTOR_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_R_R_MOTOR].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_R_R_MOTOR].Rate	= 0;

	m_MainConfig.ServoConf[SERVO_R_L_MOTOR].min		= R_L_MOTOR_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_R_L_MOTOR].max		= R_L_MOTOR_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_R_L_MOTOR].middle	= R_L_MOTOR_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_R_L_MOTOR].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_R_L_MOTOR].Rate	= 0;


	m_MainConfig.ServoConf[SERVO_HEAD_VERT].min		= HEAD_VERT_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_HEAD_VERT].max		= HEAD_VERT_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_HEAD_VERT].middle	= HEAD_VERT_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_HEAD_VERT].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_HEAD_VERT].Rate	= 0;

	m_MainConfig.ServoConf[SERVO_HEAD_HORZ].min		= HEAD_HORZ_SERVO_LOWER_LIMIT;
	m_MainConfig.ServoConf[SERVO_HEAD_HORZ].max		= HEAD_HORZ_SERVO_UPPER_LIMIT;
	m_MainConfig.ServoConf[SERVO_HEAD_HORZ].middle	= HEAD_HORZ_SERVO_MID;
	m_MainConfig.ServoConf[SERVO_HEAD_HORZ].Reverse = 0;
	m_MainConfig.ServoConf[SERVO_HEAD_HORZ].Rate	= 0;

	for(i=0;i<8;i++)
	{
		m_MainConfig.RadioTrims[i].min		= HEAD_HORZ_SERVO_LOWER_LIMIT;
		m_MainConfig.RadioTrims[i].max		= HEAD_HORZ_SERVO_UPPER_LIMIT;
		m_MainConfig.RadioTrims[i].middle	= HEAD_HORZ_SERVO_MID;
		m_MainConfig.RadioTrims[i].Reverse = 0;
	}
	
	
	m_MainConfig.SafeFrontDistance				= 1000;
	m_MainConfig.LastFileNumber					= 0;
	m_MainConfig.WindSpeedScale				= 50;

	m_MainConfig.AuxSwitchSettings[0]		= 7;
	m_MainConfig.AuxSwitchSettings[1]		= 448;
	m_MainConfig.AuxSwitchSettings[2]		= 2560;
	m_MainConfig.AuxSwitchSettings[3]		= 28672;
	m_MainConfig.AuxSwitchSettings[4]		= 0;
	m_MainConfig.AuxSwitchSettings[5]		= 40;
	m_MainConfig.AuxSwitchSettings[6]		= 163840;
	for(i=0;i<4;i++)
	m_MainConfig.MotorTrims[i]					= 0;

	m_MainConfig.SafeFrontDistance				= 200; //mm
	m_MainConfig.SafeRearDistance				= 200;
	m_MainConfig.SafeSideDistance				= 200;
	m_MainConfig.MaxThrottleAcceleration				= ATTCONTROL_THR_ACCEL_MAX;
	m_MainConfig.MaxMotorPwm					= 5000;
//	m_MainConfig.WheelRadius					= 0.030f;
	if(WriteToFlash)
	{
		return(WriteMainConfig());
	}
	return(true);
}


bool CConfig::LoadDefaultFunctionFlags()
{
	m_FunctionFlags.MotorEnable = false;
	m_FunctionFlags.SoundEnable = false;
	m_FunctionFlags.FunctionFlag_3 = false;
	m_FunctionFlags.FunctionFlag_4 = false;
	m_FunctionFlags.FunctionFlag_5 = false;
	m_FunctionFlags.FunctionFlag_6 = false;
	m_FunctionFlags.PassThroughFlag = false;
	m_FunctionFlags.MotorReverseFlag= false;
	return(WriteFunctionFlagsConfig());
}

unsigned char CConfig::CalculateConfigChecksum(unsigned char *pBlock , unsigned int Size)
{
	unsigned char Sum = 0x55;	// checksum init
	unsigned char Count =0;
	while(Size--)
	{
		Sum += (unsigned char)*pBlock++;		// calculate checksum (without checksum byte)
		Count++;
	}
	return Sum;
}




bool CConfig::WriteCourseWp()
{
	unsigned char ChkSum;
	uint16_t Offset  =(sizeof(m_NavigationConfig)+1)+(sizeof(MainConfig_t)+1)+(sizeof(RcChannels_t)+1)+(sizeof(FunctionFlags_t)+1);
	unsigned char i;
	for(i =1;i<12;i++)
	{
		ChkSum = CalculateConfigChecksum((unsigned char*)&m_CourseWaypoints[i], sizeof(Waypoint_t)-1);
		m_CourseWaypoints[i].ChkSum = ChkSum;
		QSpiFlash.writeBuffer(WAYPOINT_ADDRESS,(uint8_t*) &m_CourseWaypoints[i], sizeof(Waypoint_t));
		Offset= Offset+(i*sizeof(Waypoint_t));
	}
	if(ReadCourseFile())
	{
		return(true);
	}
	return(false);
}


bool CConfig::ReadCourseFile()
{
	unsigned char ChkSum;
	uint16_t Offset  =(sizeof(m_NavigationConfig)+1)+(sizeof(MainConfig_t)+1)+(sizeof(RcChannels_t)+1)+(sizeof(FunctionFlags_t)+1);
	unsigned char i;
	m_CourseWaypoints[11].Flag = WP_FLAG_END;

	for(i =1;i<MAX_WAYPOINTS;i++)
	{
		if(QSpiFlash.readBuffer(WAYPOINT_ADDRESS,(uint8_t*)&m_CourseWaypoints[i], sizeof(Waypoint_t)) != sizeof(Waypoint_t))
		{
			m_RunningFlags.MISSION_DATA_LOADED = false;
			return(false);
		}
		Offset= Offset+(i*sizeof(Waypoint_t));
		ChkSum = CalculateConfigChecksum((unsigned char*)&m_CourseWaypoints[i], sizeof(Waypoint_t)-1);
		if( ChkSum != m_CourseWaypoints[i].ChkSum)
		{
			m_RunningFlags.MISSION_DATA_LOADED = false;
			return false;
		}
		if( m_CourseWaypoints[i].nLatitide == 0)
		m_CourseWaypoints[i].Flag = WP_FLAG_END;
	}
	m_RunningFlags.MISSION_DATA_LOADED = true;
	//	m_RunningFlags.WP_TABLE_UPDATED = true;
	return true;    // setting is OK
}


void CConfig::ClearWaypoints()
{
	unsigned char i;
	for(i=1;i<MAX_WAYPOINTS;i++)
	{
		m_CourseWaypoints[i].Action		= 0;
		m_CourseWaypoints[i].nLatitide	= 0;
		m_CourseWaypoints[i].nLongitude	= 0;
		m_CourseWaypoints[i].Number		= 0;	// Waypoint number
		m_CourseWaypoints[i].Action		= 0;	// Action to follow
		m_CourseWaypoints[i].Parameter1	= 0;	// Parameter for the wp action
		m_CourseWaypoints[i].Parameter2	= 0;	// Parameter for the wp action
		m_CourseWaypoints[i].Parameter3	= 0;	// Parameter for the wp action
		m_CourseWaypoints[i].Flag		= 0;	// flags the last wp
		m_CourseWaypoints[i].altitude	= 0;	// Altitude in cm (AGL) not used for boat but needed for gui

	}
}


bool CConfig::ReadWaypoints()
{
	unsigned char i;
	// Waypoint 0 is home position
	if(ReadCourseFile())
	{
		for(i=1;i<9;i++)
		{
			if((m_CourseWaypoints[i].nLatitide == 0)||(m_CourseWaypoints[i].nLongitude == 0))
			break;
		}
		if(i>2)
		{
			m_LastWaypoint = i-1;
			m_RunningFlags.MISSION_DATA_LOADED = true;
			return true;    // setting is OK
		}
		else
		{
			m_RunningFlags.MISSION_DATA_LOADED = false;
			return(false);
		}

	}
	m_RunningFlags.MISSION_DATA_LOADED = false;
	return(false);
}


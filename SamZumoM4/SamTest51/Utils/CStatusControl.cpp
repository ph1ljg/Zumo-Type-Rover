/* 
* CLedControl.cpp
*
* Created: 27/04/2017 12:47:07
* Author: phil
*/
#include "includes.h"
#include "CLedDriver.h"
CLedDriver LedDriver;



// default constructor
CStatusControl::CStatusControl()
{
	m_Brightness = RGB_LED_LOW;
} //CLedControl

// default destructor
CStatusControl::~CStatusControl()
{
} //~CLedControl

void CStatusControl::Init()
{
	LedDriver.Init();
}


uint8_t CStatusControl::GetBrightness(void) const
{
	uint8_t Brightness;

	switch (m_Brightness) 
	{
	case RGB_LED_OFF:
		Brightness = 0;
		break;
	case RGB_LED_LOW:
		Brightness = 40;
		break;
	case RGB_LED_MEDIUM:
		Brightness = 150;
		break;
	case RGB_LED_HIGH:
		Brightness = 250;
		break;
	default:	
		Brightness = 250;
		break;
	}
	return(Brightness);
}


void CStatusControl::Update()
{
	UpdateAlarms();
	UpdateLedStatus();
	UpdateLed();
}



void CStatusControl::UpdateAlarms()
{
	//static uint8_t I2CBus_0FailCount =0;
	//static uint8_t I2CBus_1FailCount =0;
	
	//============================================= Config ==========================================
	

	if(Config.m_RunningFlags.RD_MAIN_CONFIG_FAIL)
		Alarms.AddAlarm(ALM_READ_MAIN_CONFIG_FAIL,ALM_READ_MAIN_CONFIG_FAIL_PRIORITY);
	
	

	if(Config.m_RunningFlags.RD_NAVIGATION_CFG_FAIL)
	Alarms.AddAlarm(ALM_READ_NAVIGATION_CONFIG_FAIL,ALM_READ_NAVIGATION_CONFIG_FAIL_PRIORITY);
	
	
	if(Config.m_RunningFlags.RD_RC_CONFIG_FAIL )
		Alarms.AddAlarm(ALM_READ_NAVIGATION_CONFIG_FAIL,ALM_READ_NAVIGATION_CONFIG_FAIL_PRIORITY);

	
	
	
	if(	Config.m_RunningFlags.RD_FUNCTION_CONFIG_FAIL )
		Alarms.AddAlarm(ALM_READ_FUNCTION_CONFIG_FAIL,ALM_READ_FUNCTION_CONFIG_FAIL_PRIORITY);
	
	//============================================= Battery ==========================================
	

	if(Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL)
	{
		Alarms.AddAlarm(ALM_BATTERY_1_CRITICAL,ALM_BATTERY_CRITICAL_PRIORITY);
		Alarms.ClearAlarm(ALM_BATTERY_1_WARN);
	}
	else
	{
		Alarms.ClearAlarm(ALM_BATTERY_1_CRITICAL);
		if(Config.m_RunningFlags.BATTERY_1_WARNING  )
			Alarms.AddAlarm(ALM_BATTERY_1_WARN,ALM_BATTERY_1_WARN_PRIORITY);
		else
			Alarms.ClearAlarm(ALM_BATTERY_1_WARN);
	}
	
	if(Config.m_RunningFlags.MAIN_INIT_COMPLETED)
	{
		//========================================== IMU module ==================================

		if(Config.m_RunningFlags.IMU_FAIL)
		{
			Alarms.ClearAlarm(ALM_ACC_NOT_CALIBRATED);
			Alarms.ClearAlarm(ALM_GIRO_NOT_CALIBRATED);
			Alarms.ClearAlarm(ALM_MAG_NOT_CALIBRATED);
			Alarms.AddAlarm(ALM_IMU_FAIL,ALM_IMU_FAILED_READ_PRIORITY);
		}
		else
		{
			Alarms.ClearAlarm(ALM_IMU_FAIL);
			if(!Config.m_RunningFlags.ACC_CALIBRATED )
				Alarms.AddAlarm(ALM_ACC_NOT_CALIBRATED,7);
			else
				Alarms.ClearAlarm(ALM_ACC_NOT_CALIBRATED);

			if(!Config.m_RunningFlags.GYRO_CALIBRATED )
				Alarms.AddAlarm(ALM_GIRO_NOT_CALIBRATED,7);
			else
				Alarms.ClearAlarm(ALM_GIRO_NOT_CALIBRATED);
			
			if(!Config.m_RunningFlags.MAG_CALIBRATED )
				Alarms.AddAlarm(ALM_MAG_NOT_CALIBRATED,7);
			else
				Alarms.ClearAlarm(ALM_MAG_NOT_CALIBRATED);

		}
		
		//=========================== Lidars ======================================
		
		if(!Config.m_RunningFlags.HEAD_LIDAR_FAIL)
			Alarms.AddAlarm(ALM_HEAD_LIDAR_FAILED,ALM_HEAD_LIDAR_FAILED_PRIORITY);
		else
		Alarms.ClearAlarm(ALM_HEAD_LIDAR_FAILED);

		//==================== GPS ===========================================
		if( !Gps.IsGpsUsable())
			Alarms.AddAlarm(ALM_GPS_FAIL,ALM_GPS_FAIL_PRIORITY);
		else
			Alarms.ClearAlarm(ALM_GPS_FAIL);

		
		//===================================== FPV ================================================
		// 		if(!Config.m_RunningFlags.VTX_PRESENT)
		// 			AddAlarm(ALM_VTX_SETTINGS,7);
		// 		else
		// 			ClearAlarm(ALM_VTX_SETTINGS);
		//
		// 		if(!Config.m_RunningFlags.CAMERA_PRESENT)
		// 			AddAlarm(ALM_CAMERA_FAIL,5);
		// 		else
		// 			ClearAlarm(ALM_CAMERA_FAIL);
	}
	//======================================= FPV End ==============================================

	if(!Config.m_RunningFlags.ALL_CONFIGS__LOADED )
		Alarms.AddAlarm(ALM_READ_MAIN_CONFIG_FAIL,7);
	
	
	// 	if(!Config.m_RunningFlags.SD_CARD_PRESENT)
	// 		Alarms.AddAlarm(ALM_SD_CARD_FAIL,7);
	// 	else
	// 		ClearAlarm(ALM_SD_CARD_FAIL);
	
	//======================================== I2c Buses ==============================================
	if(Config.m_RunningFlags.I2C_INTERFACE_FAIL)
		Alarms.AddAlarm(ALM_I2C_FAIL_BUS_0,ALARM_PRIORITY_9);
	else
		Alarms.ClearAlarm(ALM_I2C_FAIL_BUS_0);


	//======================================== I2c Bus end ===========================================
	
	
	//========================================= Navigation ==============================================
	if(Config.m_RunningFlags.NAVIGATION_FAIL)
		Alarms.AddAlarm(ALM_NAVIGATION_UPDATE_FAILED,ALM_NAVIGATION_UPDATE_FAILED_PRIORITY);
	else
		Alarms.ClearAlarm(ALM_NAVIGATION_UPDATE_FAILED);
	
	if(!Config.m_RunningFlags.MISSION_DATA_LOADED)
		Alarms.AddAlarm(ALM_NO_MISSION_FILE_LOADED,7);
	else
		Alarms.ClearAlarm(ALM_NO_MISSION_FILE_LOADED);
	
	if( !Config.m_RunningFlags.HOME_LOCATION_SET)
	{
		if(Navigation.m_NavigationState == eControlByRc)
			Alarms.AddAlarm(ALM_HOME_WAYPOINT_NOT_SET,3);
		else
			Alarms.AddAlarm(ALM_HOME_WAYPOINT_NOT_SET,10);
	}
	else
		Alarms.ClearAlarm(ALM_HOME_WAYPOINT_NOT_SET);
	//======================================== Radio =======================================================
	if(Config.m_RunningFlags.LOST_RC_SIGNAL)
		Alarms.AddAlarm(ALM_RC_DATA_LOSS,ALM_RC_DATA_LOSS_PRIORITY);
	else
		Alarms.ClearAlarm(ALM_RC_DATA_LOSS);

}



void CStatusControl::UpdateLedStatus()
{
	Alarm_t Alarm  =   Alarms.GetTopAlarm();
	
	switch(Alarm)
	{
	case ALM_BATTERY_1_WARN:
		CurrentColourSequence = SequenceBatteryWarning;
		 //Buzzer.Tone();
	break;
	case ALM_BATTERY_2_WARN:
		CurrentColourSequence = SequenceBatteryWarning;
		//Buzzer.Tone();
	break;
	case ALM_BATTERY_1_CRITICAL:
		//Buzzer.Tone();
		CurrentColourSequence = SequenceBatteryFailPanic;
	break;
	case ALM_BATTERY_2_CRITICAL:
		//Buzzer.Tone();
		CurrentColourSequence = SequenceBatteryFailPanic;
	break;
	case ALM_NAVIGATION_UPDATE_FAILED:
	case ALM_SD_CARD_FAIL_INIT:
	case ALM_SD_CARD_FAIL:
	case ALM_VTX_SETTINGS:
	case ALM_ACC_NOT_CALIBRATED:
	case ALM_GIRO_NOT_CALIBRATED:
	case ALM_MAG_NOT_CALIBRATED:
	case ALM_RUNNING_NO_GPS_LOCK:
	case ALM_BAD_EEPROM_READ:
	case ALM_EEPROM_WRITE:
	case ALM_INIT_FAILED:
	case ALM_OPEN_SD_LOGFILE_FAULT:
	case ALM_OPEN_FLASH_LOGFILE_FAULT:
	case ALM_DELETE_SD_LOGFILE_FAULT:
	case ALM_READ_SD_LOGFILE_FAULT:
	case ALM_READ_FLASH_LOGFILE_FAULT:
	case ALM_IMU_FAIL:
	case ALM_HEAD_LIDAR_FAILED:
	case ALM_HOME_WAYPOINT_NOT_SET:
	case ALM_FAIL_WAYPOINT_1_ERROR:
	case ALM_FAIL_WAYPOINT_2_ERROR:
	case ALM_FAIL_WAYPOINT_3_ERROR:
	case ALM_FAIL_WAYPOINT_4_ERROR:
	case ALM_FAIL_WAYPOINT_5_ERROR:
	case ALM_FAIL_WAYPOINT_6_ERROR:
	case ALM_FAIL_WAYPOINT_7_ERROR:
	case ALM_FAIL_WAYPOINT_8_ERROR:
	case ALM_FAIL_WAYPOINT_9_ERROR:
	case ALM_FAIL_WAYPOINT_10_ERROR:
	case ALM_FAIL_WAYPOINT_11_ERROR:
	case ALM_FAIL_WAYPOINT_12_ERROR:
	case ALM_I2C_FAIL_BUS_1:
	case ALM_I2C_FAIL_BUS_0:
	case ALM_GPS_FAIL:
	case ALM_NO_MISSION_FILE_LOADED:
	case ALM_RC_DATA_LOSS:
	case ALM_WIND_SENS_FAIL:
	case ALM_CAMERA_FAIL:
	case ALM_FRONT_LIDAR_FAIL:
	case ALM_REAR_LIDAR_FAIL:
	case ALM_RIGHT_LIDAR_FAIL:
	case ALM_LEFT_LIDAR_FAIL:
	case ALM_READ_NAVIGATION_CONFIG_FAIL:
	case ALM_WRITE_NAVIGATION_CONFIG_FAIL:
	case ALM_READ_MAIN_CONFIG_FAIL:
	case ALM_WRITE_MAIN_CONFIG_FAIL:
		CurrentColourSequence = GeneralAlarm;
		break;
	case ALM_NOT_DEFINED:
	default:
		UpdateLedSequence();
		break;

	}


}

void CStatusControl::UpdateStartup()
{
	if(!Compass.GetGyroCal())
		CurrentColourSequence = SequenceGyroCal;
	else 	if(!Compass.GetAccelCal())
		CurrentColourSequence = SequenceGyroCal;
	else 	if(!Compass.GetAccelCal())
		CurrentColourSequence = SequenceGyroCal;
	else
	{
		if(!Config.m_RunningFlags.MAIN_INIT_COMPLETED)
			CurrentColourSequence = SequenceInitialising;
	}
}

void CStatusControl::SetLedSequence(uint32_t Sequence)
{
	CurrentColourSequence = Sequence;
	UpdateLed();
}

void CStatusControl::UpdateLedSequence()
{
	
	
	if(Config.m_RunningFlags.ARMED )
	{
		CurrentColourSequence  = SequenceArmedNoGps;
		if(Gps.IsGpsUsable())
			CurrentColourSequence  = SequenceArmedGps;
	}
	else
	{
		if(Gps.IsGpsUsable())
			CurrentColourSequence  = SequenceUnarmedGps;
		else
			CurrentColourSequence  = SequenceUnarmedNoGps;
	}

	UpdateStartup();

}


void CStatusControl::SetAllLedsOff()
{
	CurrentColourSequence = SequenceAllOff;
	UpdateLed();
	
}





void CStatusControl::UpdateLed()
{
	static uint8_t Step =0;
	uint8_t Red,Green,Blue; 
	uint8_t Brightness = GetBrightness();
	const uint8_t Colour  = (CurrentColourSequence >> (Step*3)) & 7;
 	if(Step <10)
 		Step++;
 	else
 		Step =0;
	
	Red = (Colour & RED) ? Brightness : 0;
	Green = (Colour & GREEN) ? Brightness : 0;
	Blue = (Colour & BLUE) ? Brightness : 0;
	LedDriver.SetRGB( Red, Green, Blue);
}


void CStatusControl::TestLeds()
{
}




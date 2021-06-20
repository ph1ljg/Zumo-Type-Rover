/* 
* CLedControl.cpp
*
* Created: 27/04/2017 12:47:07
* Author: phil
*/
#include "includes.h"
#include "CPlayer.h"





// 	#define STATUS_CONTROL_NEG_FEEDBACK						0	{ "MFT200L4<<<B#A#2", false },
// 	#define STATUS_CONTROL_NEU_FEEDBACK						2	{ "MFT200L4<B#", false },
// 	#define STATUS_CONTROL_POS_FEEDBACK					4	{ "MFT200L4<A#B#", false },
// 	#define STATUS_CONTROL_READY_OR_FINISHED			7	{ "MFT200L4<G#6A#6B#4", false },
// 	#define STATUS_CONTROL_ATTENTION_NEEDED			8	{ "MFT100L4>A#A#A#A#", false },
// 	#define STATUS_CONTROL_ARMING_WARNING				9	{ "MNT75L1O2G", false },
// 	#define STATUS_CONTROL_WP_COMPLETE						10	{ "MFT200L8G>C3", false },
// 	#define STATUS_CONTROL_SIREN										11	{ "MBT200L2A-G-A-G-A-G-", true },
// 	#define STATUS_CONTROL_VEHICLE_LOST			12	{ "MBT200>A#1", true },
// 	#define STATUS_CONTROL_BATTERY_ALERT		13	{ "MBNT255>A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8A#8", true },
// 	#define STATUS_CONTROL_COMPASS_CALIBRATING_CTS 14		{ "MBNT255<C16P2", true },
// 	#define STATUS_CONTROL_WAITING_FOR_THROW					15	{ "MBNT90L4O2A#O3DFN0N0N0", true},
// 	#define STATUS_CONTROL_TUNING_START 23		{ "MFT100L20>C#D#", false},
// 	#define STATUS_CONTROL_TUNING_SAVE 24		{ "MFT100L10DBDB>", false},
// 	#define STATUS_CONTROL_TUNING_ERROR 25		{ "MFT100L10>BBBBBBBB", false},
// 	#define STATUS_CONTROL_LEAK_DETECTED 26		{ "MBT255L8>A+AA-", true},
// 	#define STATUS_CONTROL_QUIET_SHUTDOWN 27	{ "MFMST200L32O3ceP32cdP32ceP32c<c>c<cccP8L32>c>c<P32<c<c", false },
// 	#define STATUS_CONTROL_QUIET_NOT_READY_OR_NOT_FINISHED 28		{ "MFT200L4<B#4A#6G#6", false },




// Tunes follow the syntax of the Microsoft GWBasic/QBasic PLAY   statement, with some exceptions and extensions.  See https://firmware.ardupilot.org/Tools/ToneTester/


const CStatusControl::Tone CStatusControl::Tones[]
{
	 { "MFT200L4<B#4A#6G#6", true },										// STATUS_CONTROL_MAIN_CONFIG_FAIL				
	{ "MFT200L4<B#4A#6G#6", false },											//  STATUS_CONTROL_NAVIGATION_CONFIG_FAIL
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_READ_FUNCTION_CONFIG_FAIL
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_BATTERY_1_CRITICAL	
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_BATTERY_1_WARN				
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_ACC_NOT_CALIBRATED				
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_IMU_FAIL								
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_LIDAR_FAILED:							
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_GPS_FAIL									
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_VTX_SETTINGS							
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_CAMERA_FAIL									
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_I2C_FAIL_BUS_0								
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_I2C_FAIL_BUS_1								
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_NAVIGATION_UPDATE_FAILED			
	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_NO_MISSION_FILE_LOADED				
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_HOME_WAYPOINT_NOT_SET				
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_WAYPOINT_1_ERROR						
 	{ "MFT200L4<B#4A#6G#6", false },											// STATUS_CONTROL_RC_DATA_LOSS								
 	{ "MNBGG", false },																	// STATUS_CONTROL_NO_SDCARD									
	{ "MFT240L8O4aO5dcO4aO5dcO4aO5dcL16dcdcdcdc", false },	// STATUS_CONTROL_STARTUP			
 	{ "MFT100L8>B", false},															// STATUS_CONTROL_LOUD_1											
	{ "MFT100L8>BB", false},															// STATUS_CONTROL_LOUD_2											
	{ "MFT100L8>BBB", false},														// STATUS_CONTROL_LOUD_3													
	{ "MFT100L8>BBBB", false},														// STATUS_CONTROL_LOUD_4												
	{ "MFT100L8>BBBBB", false},													// STATUS_CONTROL_LOUD_5											
	{ "MFT100L8>BBBBBB", false},													// STATUS_CONTROL_LOUD_6											
 	{ "MFT100L8>BBBBBBB", false},												// STATUS_CONTROL_LOUD_7											
	{ "MFT200L4<<<B#A#2", false },												// STATUS_CONTROL_NEG_FEEDBACK		
	{ "MFT200L4<B#", false },														// STATUS_CONTROL_NEU_FEEDBACK			
	{ "MFT200L4<A#B#", false }													// STATUS_CONTROL_POS_FEEDBACK				

};


CPlayer Player;

bool soundDebug = true;
// default constructor
CStatusControl::CStatusControl()
{
	m_RepeatTonePlaying = -1;
} //CLedControl

// default destructor
CStatusControl::~CStatusControl()
{
} //~CLedControl

void CStatusControl::Init()
{
    
}

// update -   Should be called at 50Hz
void CStatusControl::Update()
{
	UpdateAlarms();
	UpdateSound();
}

void CStatusControl::Startup()
{
	 	if(Compass.GetGyroCal() &&Compass.GetAccelCal()&&Compass.GetAccelCal())
			PlayTone(STATUS_CONTROL_STARTUP);
		else
			PlayTone(STATUS_CONTROL_STARTUP);	
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




void CStatusControl::UpdateSound()
{
	 Alarm_t Alarm  =   Alarms.GetTopAlarm();
	 uint8_t   ToneIndex;
	CheckForRepeatTone();
	
	if(Alarm != ALM_NOT_DEFINED )
	{
		switch(Alarm)
		{
		//============================================= Config ==========================================
		case 	ALM_SD_CARD_FAIL:
	//		ToneIndex = STATUS_CONTROL_NO_SDCARD; 
			break;
		case 	ALM_READ_MAIN_CONFIG_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_READ_NAVIGATION_CONFIG_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_READ_FUNCTION_CONFIG_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;

		//============================================= Battery ==========================================
	
		case 	ALM_BATTERY_1_CRITICAL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;

		case 	ALM_BATTERY_1_WARN:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;

		//========================================== IMU module ==================================

		case 	ALM_ACC_NOT_CALIBRATED:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_GIRO_NOT_CALIBRATED:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_MAG_NOT_CALIBRATED:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_IMU_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		//=========================== Lidars ======================================
		case 	ALM_HEAD_LIDAR_FAILED:
		case ALM_FRONT_LIDAR_FAIL:
 		case ALM_REAR_LIDAR_FAIL:
 		case ALM_RIGHT_LIDAR_FAIL:
 		case ALM_LEFT_LIDAR_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		//==================== GPS ===========================================
		case 	ALM_GPS_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		//===================================== FPV ================================================
		case 	ALM_VTX_SETTINGS:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_CAMERA_FAIL:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		//======================================== I2c Buses ==============================================
		case 	ALM_I2C_FAIL_BUS_0:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_I2C_FAIL_BUS_1:
		//	ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
	
		//========================================= Navigation ==============================================
		case 	ALM_NAVIGATION_UPDATE_FAILED:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_NO_MISSION_FILE_LOADED:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		case 	ALM_HOME_WAYPOINT_NOT_SET:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
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
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		//======================================== Radio =======================================================
		case 	ALM_RC_DATA_LOSS:
	//		ToneIndex = STATUS_CONTROL_LOUD_1;
			break;
		default:
			break;	
		}
		PlayTone(ToneIndex);
	}

	Player.Update();
}


//  play one of the pre-defined tunes
void CStatusControl::PlayTone(const uint8_t ToneIndex)
{
	if(Player.m_IsPlaying || !Config.m_FunctionFlags.SoundEnable || m_TonePlaying == ToneIndex && m_RepeatTonePlaying != -1)
		return;
	uint32_t NowMs = Core.millis();
	const Tone &ToneRequested = Tones[ToneIndex];

	if (ToneRequested.Repeat)
	{
		m_RepeatTonePlaying = ToneIndex;
	}

	m_TonePlaying = ToneIndex;
	m_ToneBeginningMs = NowMs;

	PlayTune(ToneRequested.ToneSequenceStr);
}


void CStatusControl::PlayTune(const char *Str)
{

	Player.Stop();
	strncpy(ToneSquenceBuffer, Str, STATUS_CONTROL_TONE_BUF_SIZE);
	ToneSquenceBuffer[STATUS_CONTROL_TONE_BUF_SIZE-1] = 0;
	Player.Play(ToneSquenceBuffer);
}

void CStatusControl::StopContTone()
{
	if (m_RepeatTonePlaying == m_TonePlaying)
	{
		PlayTune("");
		m_TonePlaying = -1;
	}
	m_RepeatTonePlaying = -1;
}

void CStatusControl::CheckForRepeatTone()
{
	uint32_t tnow_ms = Core.millis();
	// if we are supposed to be playing a continuous tone, and it was interrupted, and the interrupting tone has timed out, resume the continuous tone

	if (m_RepeatTonePlaying != -1  && tnow_ms-m_ToneBeginningMs > STATUS_CONTROL_REPEAT_TONE_REPEAT_TIME_MS)
	{
		PlayTune(Tones[m_TonePlaying].ToneSequenceStr);
//		PlayTone(m_RepeatTonePlaying);
		m_ToneBeginningMs = Core.millis();
	}
}



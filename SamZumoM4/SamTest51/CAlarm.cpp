/*
* Alarm.cpp
*
* Created: 20/10/2015 10:49:17
* Author: Phil2
*/

#include "Includes.h"


CAlarm::CAlarm()
{
	ClearAllAlarms();
	AlarmInProgress= 0;
	SoundPatternNo =1;
	CheckPriority = false;
	m_AlarmCount = 0;
	m_StatusCount =0;
}

// default destructor
CAlarm::~CAlarm()
{
} //~Alarm


void CAlarm::InitAlarms()
{
	
//	LedControl.UpdateAlarmLed();

}



void CAlarm::ClearAllDirectionAlarms()
{
	
}







void CAlarm::PlaySound(void)
{
	
	//	alarmPatternComposer(SoundPatternNo);

}



void CAlarm::AddAlarm(Alarm_t Alarm,unsigned char Priority)
{
	unsigned char i;

		
	for(i=0;i<16;i++)			// only allow 1 alarm of each type
	{
		if(AlarmList[i].Alarm == Alarm)
		return;
	}
	if(AlarmList[15].Priority < Priority)
	{
		AlarmList[15].Priority = Priority;
		AlarmList[15].Alarm = Alarm;
		SetPiority();
		
		
	}
}

uint16_t CAlarm::GetAlarm(uint16_t AlarmNo)
{
	if(AlarmNo >15)
	 AlarmNo = 15;
	return(AlarmList[AlarmNo].Alarm);
}

bool CAlarm::IsAlarmActive()
{
	if(AlarmList[0].Alarm	== ALM_NOT_DEFINED)
		return(false);
	return(true);	
}


uint8_t CAlarm::GetAlarmCount()
{
	unsigned char i;
	for(i=0;i<16;i++)
	{
		if(AlarmList[i].Alarm	== ALM_NOT_DEFINED)
			return(i);
	}
	return(0);
}


void CAlarm::ClearAlarm(Alarm_t Alarm)
{
	unsigned char i;

	for(i=0;i<16;i++)
	{
		if(AlarmList[i].Alarm == Alarm)
		{
			AlarmList[i].Priority = ALARM_NO_PRIORITY;
			AlarmList[i].Alarm = ALM_NOT_DEFINED;
			SetPiority();
			break;
			
		}
	}
}


void CAlarm::Update()
{
		
}








void CAlarm::ClearAllAlarms()
{
	unsigned char i;
	for(i=0;i<16;i++)
	{
		AlarmList[i].Priority	= ALARM_NO_PRIORITY;
		AlarmList[i].Alarm		= ALM_NOT_DEFINED;
	}
	
}

// sort 0 = highest priority
void CAlarm::SetPiority()
{
	unsigned char i;
	unsigned char  Priority1,Priority2,x;
	Alarm_t Alarm;

	for(i=0;i<16;i++)
	{
		for(x=0;x<15-i;x++)
		{
			Priority1 = AlarmList[x].Priority;
			Priority2 = AlarmList[x+1].Priority;
			if(Priority1 < Priority2)
			{
				// swap
				Alarm	= AlarmList[x+1].Alarm;
				
				AlarmList[x+1].Alarm		= AlarmList[x].Alarm;
				AlarmList[x+1].Priority		= AlarmList[x].Priority;
				
				AlarmList[x].Alarm			= Alarm;
				AlarmList[x].Priority		= Priority2;
			}
		}
		CheckPriority = true;
	}

}





 Alarm_t   CAlarm::GetTopAlarm()
{
	return(AlarmList[0].Alarm );
	
}

bool  CAlarm::GetAlarmState(Alarm_t Alarm)
{
	unsigned char i;

	for(i=0;i<16;i++)
	{
		if(AlarmList[i].Alarm == Alarm)
		return(true);
	}
	return(false);
	
}


void CAlarm::SetWaypointAlm(uint8_t WaypointNo)
{
	Alarm_t Alm = ALM_NOT_DEFINED;
	switch(WaypointNo)
	{
		case 1:
			Alm = ALM_FAIL_WAYPOINT_1_ERROR;		
			break;
		case 2:
			Alm = ALM_FAIL_WAYPOINT_2_ERROR;
			break;
		case 3:
			Alm = ALM_FAIL_WAYPOINT_3_ERROR;
			break;
		case 4:
			Alm = ALM_FAIL_WAYPOINT_4_ERROR;
			break;
		case 5:
			Alm = ALM_FAIL_WAYPOINT_5_ERROR;
			break;
		case 6:
			Alm = ALM_FAIL_WAYPOINT_6_ERROR;
			break;
		case 7:
			Alm = ALM_FAIL_WAYPOINT_7_ERROR;
			break;
		case 8:
			Alm = ALM_FAIL_WAYPOINT_8_ERROR;
			break;
		case 9:
			Alm = ALM_FAIL_WAYPOINT_9_ERROR;
			break;
		case 10:
			Alm = ALM_FAIL_WAYPOINT_10_ERROR;
			break;
		case 11:
			Alm = ALM_FAIL_WAYPOINT_11_ERROR;
			break;
		case 12:
			Alm = ALM_FAIL_WAYPOINT_12_ERROR;
			break;
	}
	AddAlarm(Alm,9);
	
}

void CAlarm::ClearWaypointAlms()
{
	ClearAlarm(ALM_FAIL_WAYPOINT_1_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_2_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_3_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_4_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_5_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_6_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_7_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_8_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_9_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_10_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_11_ERROR);
	ClearAlarm(ALM_FAIL_WAYPOINT_12_ERROR);
}


void CAlarm::SoundMalfunction()
{
	
}


void  CAlarm::GetAlarmText(unsigned char AlarmNo,char *AlarmText)
{

		switch(AlarmList[AlarmNo].Alarm)
		{
			case ALM_NAVIGATION_UPDATE_FAILED:
				strcpy(AlarmText,"Nav update fail");
				break;					
			case ALM_SD_CARD_FAIL_INIT:
			case ALM_SD_CARD_FAIL:
				strcpy(AlarmText,"SD card Fail");
				break;
			case ALM_BATTERY_1_WARN:
				strcpy(AlarmText,"Batt Warning 1");
				break;
			case ALM_BATTERY_2_WARN:
				strcpy(AlarmText,"Batt Warning 2");
				break;
			case ALM_BATTERY_1_CRITICAL:
				strcpy(AlarmText,"Batt 1 Critical");
				break;
			case ALM_BATTERY_2_CRITICAL:
				strcpy(AlarmText,"Batt 2 Critical");
				break;
			case ALM_VTX_SETTINGS:
				strcpy(AlarmText,"Vtx Set Fail");
				break;
			 case ALM_ACC_NOT_CALIBRATED:
				strcpy(AlarmText,"ACC Not Cal");
				break;
			case ALM_GIRO_NOT_CALIBRATED:
				strcpy(AlarmText,"Giro Not Cal");
				break;
			case ALM_MAG_NOT_CALIBRATED:
				strcpy(AlarmText,"Mag Not Cal");
				break;
			case ALM_RUNNING_NO_GPS_LOCK:
				strcpy(AlarmText,"GPS No Lock");
				break;
			case ALM_BAD_EEPROM_READ:
				strcpy(AlarmText,"EEprom Fail");
				break;
			case ALM_EEPROM_WRITE:
				strcpy(AlarmText,"EEprom Fail");
				break;
			case ALM_INIT_FAILED:
				strcpy(AlarmText,"Setup Failed");
				break;
			case ALM_OPEN_SD_LOGFILE_FAULT:
				strcpy(AlarmText,"Open SD Log");
				break;
			case ALM_OPEN_FLASH_LOGFILE_FAULT:
				strcpy(AlarmText,"Open Flash log");
				break;
			case ALM_DELETE_SD_LOGFILE_FAULT:
				strcpy(AlarmText,"SD Delete Fail");
				break;
				
			case ALM_READ_SD_LOGFILE_FAULT:
				strcpy(AlarmText,"Read Log failed");
				break;	
			case ALM_READ_FLASH_LOGFILE_FAULT:
				strcpy(AlarmText,"Read F Log fail");
				break;
			case ALM_IMU_FAIL:
				strcpy(AlarmText,"Compass Failed");
				break;
			case ALM_HEAD_LIDAR_FAILED:
				strcpy(AlarmText,"Dist Sens Fail");
				break;
			case ALM_HOME_WAYPOINT_NOT_SET:
				strcpy(AlarmText,"No Home Waypoint");
				break;
			case ALM_FAIL_WAYPOINT_1_ERROR:
				strcpy(AlarmText,"Waypoint 1 Error");
				break;
			case ALM_FAIL_WAYPOINT_2_ERROR:
				strcpy(AlarmText,"Waypoint 2 Error");
				break;
			case ALM_FAIL_WAYPOINT_3_ERROR:
				strcpy(AlarmText,"Waypoint 3 Error");
				break;
			case ALM_FAIL_WAYPOINT_4_ERROR:
				strcpy(AlarmText,"Waypoint 4 Error");
				break;
			case ALM_FAIL_WAYPOINT_5_ERROR:
				strcpy(AlarmText,"Waypoint 5 Error");
				break;
			case ALM_FAIL_WAYPOINT_6_ERROR:
				strcpy(AlarmText,"Waypoint 6 Error");
				break;
			case ALM_FAIL_WAYPOINT_7_ERROR:
				strcpy(AlarmText,"Waypoint 7 Error");
				break;
			case ALM_FAIL_WAYPOINT_8_ERROR:
				strcpy(AlarmText,"Waypoint 8 Error");
				break;
			case ALM_FAIL_WAYPOINT_9_ERROR:
				strcpy(AlarmText,"Waypoint 9 Error");
				break;
			case ALM_FAIL_WAYPOINT_10_ERROR:
				strcpy(AlarmText,"WP 10 Error");
				break;
			case ALM_FAIL_WAYPOINT_11_ERROR:
				strcpy(AlarmText,"WP 11 Error");
				break;
			case ALM_FAIL_WAYPOINT_12_ERROR:
				strcpy(AlarmText,"WP 12 Error");
				break;
			case ALM_I2C_FAIL_BUS_1:
				strcpy(AlarmText,"I2c Bus 0 Fail");
				break;
			case ALM_I2C_FAIL_BUS_0:
				strcpy(AlarmText,"I2c Bus 1 Fail");
				break;
			case ALM_GPS_FAIL:
				strcpy(AlarmText,"GPS Read Fail");
				break;
			case ALM_NO_MISSION_FILE_LOADED:
				strcpy(AlarmText,"No Mission File");
				break;
			case ALM_RC_DATA_LOSS:
				strcpy(AlarmText,"Rc Signal Fail");
				break;
			case ALM_WIND_SENS_FAIL:
				strcpy(AlarmText,"Wind Sens Fail");
				break;
			case ALM_CAMERA_FAIL:
				strcpy(AlarmText,"Camera Fail");
				break;
			case ALM_FRONT_LIDAR_FAIL:	
				strcpy(AlarmText,"Front Lidar fail");
				break;
			case ALM_REAR_LIDAR_FAIL:	
				strcpy(AlarmText,"Rear Lidar fail");
				break;
			case ALM_RIGHT_LIDAR_FAIL:
				strcpy(AlarmText,"Right Lidar fail");
				break;
			case ALM_LEFT_LIDAR_FAIL:
				strcpy(AlarmText,"Left Lidar fail");
				break;
			case ALM_READ_NAVIGATION_CONFIG_FAIL:
			case ALM_WRITE_NAVIGATION_CONFIG_FAIL:
				strcpy(AlarmText,"Nav Config fail");
				break;
			case ALM_READ_MAIN_CONFIG_FAIL:
			case ALM_WRITE_MAIN_CONFIG_FAIL: 
				strcpy(AlarmText,"Main Config fail");
				break;
			case ALM_NOT_DEFINED:
				strcpy(AlarmText,"Undefined");
				break;
			default:
				sprintf(AlarmText,"Unknown Msg No %u",AlarmNo);
				break;

		}


}
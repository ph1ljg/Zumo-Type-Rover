/* 
* CDebugDisplay.cpp
*
* Created: 11/06/2020 12:42:50
* Author: philg
*/

#include "Includes.h"

static const char *GpsConfigFail[] =
{
	"navigation rate",
	"posllh rate",
	"status rate",
	"solution rate",
	"velned rate",
	"dop rate",
	"Chip version",
	"navigation settings",
	"GNSS settings",
	"SBAS settings",
	"TIME UTC",
	"SW Version"
};

// default constructor
// CDebugDisplay::CDebugDisplay(CSoftwareSerial *Serial)
// {
// 	m_Serial = Serial;
// 	Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY = false;
// } //CDebugDisplay


// default constructor
CDebugDisplay::CDebugDisplay(CUart *Serial)
{
	m_Serial = Serial;
	Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY = false;
} //CDebugDisplay

// default destructor
CDebugDisplay::~CDebugDisplay()
{
} //~CDebugDisplay



void CDebugDisplay::Init()
{
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		return;
	WriteChar(27);	// ESC command
	WriteBytes((uint8_t*)"[2J",3);
	WriteChar(27);	// ESC command
	WriteBytes((uint8_t*)"[H",2);
	SetCursorState(false);
}

void CDebugDisplay::InitScreen()
{
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		return;
	WriteChar(27);	// ESC command
	WriteBytes((uint8_t*)"[2J",3);
	WriteChar(27);	// ESC command
	WriteBytes((uint8_t*)"[H",2);
	SetCursorState(false);
}



void CDebugDisplay::DisplayLiveData()
{
	static uint8_t Displaymodule =0;
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		return;

	m_PrintfActive = true;
//		DisplayWheelSensors();
//		DisplayMotors();
	return;

//	DisplayLidars();
	switch(Displaymodule++)
	{
	case 0:
// 		if(!Ublox.m_UnconfiguredMessages)
// 			DisplayGps();
// 		else
// 			DisplayConfigFail();
// 		Displaymodule = 0;
		break;
	case 1:
		Displaymodule = 0;
		DisplayWheelSensors();
		break;
	case 2:
		DisplayImu();
		Displaymodule = 0;
		break;
	default:
		Displaymodule = 0;
	}
	m_PrintfActive = false;
}


void CDebugDisplay::SetDisplayActive(DebugDisplayActive_t Type)
{
	InitScreen();
	switch(Type)
	{
	case DISPLAY_INACTIVE:
		TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,false);
		m_PrintfActive = false;
		break;
	case UPDATE_ACTIVE:
		TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,true);
		m_PrintfActive = false;
		break;
	case PRINTF_ACTIVE:
		TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,false);
		m_PrintfActive = true;
		break;
	}
}

void CDebugDisplay::DisplayGps()
{
	unsigned char Row = DEBUG_GPS_ROW;
	unsigned char Col = DEBUG_GPS_COL;
	static bool GpsText = true;
	//	unsigned char i;
	if(GpsText == true )
	{
		Printf(Row,Col+4,7,"GPS Data");
	}
	Row++;
	
	if(GpsText)
	{
		Printf(Row++,Col,20,"GPS Time");
		Printf(Row++,Col,8,"Has Fix");
		Printf(Row++,Col,19,"No of Sats");
		Printf(Row++,Col,25,"Lat:");
		Printf(Row++,Col,25,"Long");
		Printf(Row++,Col,18,"Altitude");
		Printf(Row++,Col,18,"Ground Speed");
		Printf(Row++,Col,18,"Course");
		Printf(Row++,Col,18,"HDOP");
		GpsText = false;
	}
	Row = DEBUG_GPS_ROW+1;
	Col +=14;
	Printf(Row++,Col,9,"%d:%d:%d",Gps.m_GpsTimeValues.hour,Gps.m_GpsTimeValues.minute,Gps.m_GpsTimeValues.seconds);

	if(Gps.IsGpsUsable() )
		Printf(Row++,Col,6,"YES");
	else
		Printf(Row++,Col,6,"No");
	
	Printf(Row++,Col,6,"%02d",Gps.m_GpsReadings.NumSats);
	Printf(Row++,Col,9,"%f",Gps.m_GpsPositionReadings.Latitude );
	Printf(Row++,Col,9,"%f",Gps.m_GpsPositionReadings.Longitude);
	Printf(Row++,Col,6,"%u",Gps.m_GpsReadings.Altitude);
	Printf(Row++,Col,6,"%3.3f",(float)Gps.m_GpsReadings.GroundSpeed*0.0194384 );
	Printf(Row++,Col,7,"%3.2f",(float)Gps.m_GpsReadings.GroundCourse/10 );
	Printf(Row++,Col,6,"%3.2f",(double)Gps.m_GpsReadings.Hdop/100 );
	
}


void CDebugDisplay::DisplayConfigFail(void)
{
	uint8_t Row = DEBUG_GPS_ROW;
	uint8_t Col = DEBUG_GPS_COL;
	static bool Start = true;
	uint8_t i;
	if(Start)
	{
		for(i=0;i<ARRAY_SIZE(GpsConfigFail);i++)
			DebugDisplay.Printf(Row+i,Col,1,"%s No Config", GpsConfigFail[i]);
		Start = false;	
		
	}
	for(i=0;i<ARRAY_SIZE(GpsConfigFail);i++)
	{
		if ((Ublox.m_UnconfiguredMessages & (1 << i)) == 0)
			DebugDisplay.Printf(Row+i,Col+30,1,"OK");
	}
}

void CDebugDisplay::GpsVersion()
{
	uint8_t Row = DEBUG_GPS_ROW+14;
	uint8_t Col = DEBUG_GPS_COL;

	DebugDisplay.Printf(Row++,Col,0,"Sw Version %s",Ublox.GpsRecieveBuffer.mon_ver.swVersion);
	DebugDisplay.Printf(Row++,Col,0,"Hw Version %s",Ublox.GpsRecieveBuffer.mon_ver.hwVersion);
	DebugDisplay.Printf(Row++,Col,0,"Rom Version %s",Ublox.GpsRecieveBuffer.mon_ver.RomVersion);
	//	DebugDisplay.Printf(Row++,Col,0,"Rom Version %s",Ublox.GpsRecieveBuffer.mon_ver.extension);
	Core.delay(2000);
}

void CDebugDisplay::DisplayDistance()
{
// 	static bool DisplayText =true;
// 	uint8_t Row = DEBUG_DISTANCE_SENSOR_ROW;
// 	uint8_t Col = DEBUG_DISTANCE_SENSOR_COL;
// 	if(DisplayText)
// 	{
// 		Printf(Row,Col+12,1,"Lidar");
// 		Printf(Row++,Col+20,1,"Lidar Strength");
// 		Printf(Row++,Col,1,"Far Right");
// 		Printf(Row++,Col,1,"Mid Right");
// 		Printf(Row++,Col,1,"Near Right");
// 		Printf(Row++,Col,1,"Center Range");
// 		Printf(Row++,Col,1,"Near Left");
// 		Printf(Row++,Col,1,"Mid Left");
// 		Printf(Row++,Col,1,"Far Left");
// 		Printf(Row++,Col,1,"Safe Course");
// 		DisplayText = false;
// 	}
// 	
// 	Row = DEBUG_DISTANCE_SENSOR_ROW+1;
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[0]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[1]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[2]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[3]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[4]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[5]);
//  	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[6]);
// 
// 	Row = DEBUG_DISTANCE_SENSOR_ROW+1;
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[0]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[1]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[2]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[3]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[4]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[5]);
// 	Printf(Row++,Col+25,6,"%d",HeadControl.m_ForwardRanges.LidarStrength[6]);
// 
// 	//	Printf(++Row,Col+13,6,"%d",Ranger.m_Distance.SonarRange);
// 	//	Printf(Row++,Col+20,6,"%d",Ranger.m_Distance.LidarDistance);
// 
// 	switch(Navigation.m_SafeCourse)
// 	{
// 			case PATH_FAR_LEFT:
// 		Printf(Row,Col+13,11,"Far Left");
// 		break;
// 	case PATH_MID_LEFT:
// 		Printf(Row,Col+13,11,"Mid Left");
// 		break;
// 	case PATH_NEAR_LEFT:
// 		Printf(Row,Col+13,11,"Near Left");
// 			break;
// 	case PATH_CENTER:
// 		Printf(Row,Col+13,11,"Center");
// 		break;
// 	case PATH_NEAR_RIGHT:
// 		Printf(Row,Col+13,11,"Near Right");
// 		break;
// 	case PATH_MID_RIGHT:
// 		Printf(Row,Col+13,11,"Mid Right");
// 		break;
// 	case PATH_FAR_RIGHT:
// 		Printf(Row,Col+13,11,"Far Right");
// 		break;
// 	case PATH_REVERSE:
// 		Printf(Row,Col+13,11,"Reverse");
// 		break;
// 		
// 	}

}


void CDebugDisplay::DisplayLidars()
{
	static bool DisplayText =true;
	uint8_t Row = DEBUG_DISTANCE_SENSOR_ROW;
	uint8_t Col = DEBUG_DISTANCE_SENSOR_COL;
	if(DisplayText)
	{
		Printf(Row,Col+12,1,"Lidar");
		Printf(Row++,Col,1,"Head");
		Printf(Row++,Col,1,"Front");
		Printf(Row++,Col,1,"Right");
		Printf(Row++,Col,1,"Rear");
		Printf(Row++,Col,1,"Left");
		DisplayText = false;
	}
	
	Row = DEBUG_DISTANCE_SENSOR_ROW;
//	Printf(Row++,Col+20,6,"%d",HeadControl.m_ForwardRanges.LidarRange[HEAD_CENTER]);
	Printf(Row++,Col+20,6,"%d",Sensors.m_SensorValues.RightLidar);
	Printf(Row++,Col+20,6,"%d",Sensors.m_SensorValues.LeftLidar);
	Printf(Row++,Col+20,6,"%d",Sensors.m_SensorValues.RearLidar);
	Printf(Row,Col+20,6,"%d",Sensors.m_SensorValues.FrontLidar);
}


void CDebugDisplay::DisplayWheelSensorsCal()
{
	float Ms =	 WheelEncoder.GetMetersSec(RIGHT_WHEEL_SENSOR);
	Printf(" Mph %0.3f  RperS %0.3f    Motor Pwm %d\n",WheelEncoder.GetMph(RIGHT_WHEEL_SENSOR),Ms,Motors.m_Motors.FrontLeftMotor.PwmValue);
		
}

void CDebugDisplay::DisplayWheelSensors()
{
	static bool DisplayText =true;
	static uint16_t TotalTime;
	static float  MaxDeg, MaxRad;
	float Data;
	float Data2;
	uint8_t Row = DEBUG_WHEEL_SENSORS_ROW;
	uint8_t Col = DEBUG_WHEEL_SENSORS_COL;
	
	if(DisplayText)
	{
		Printf(Row,Col+13,1,"Right");
		Printf(Row++,Col+22,1,"Left");

		Printf(Row++,Col,1,"Mph");
		Printf(Row++,Col,1,"m/s");
		Printf(Row++,Col,1,"Rad/s");
		Printf(Row++,Col,1,"Wheel Count");
		Printf(Row++,Col,1,"Distance M");
		Printf(++Row,Col,1,"Sample Time");
		Printf(++Row,Col,1,"Total Time");
		Printf(++Row,Col,1,"Heading");
		Printf(++Row,Col,1,"Rate Rads/S");
		Printf(++Row,Col,1,"Rate Deg/S");
		Printf(++Row,Col,1,"Battery Volts");
		DisplayText = false;
	}
	
	Row = DEBUG_DISTANCE_SENSOR_ROW+1;
	Data = WheelEncoder.m_MPH_Right;
	Printf(Row,Col+13,6,"%3.3f",Data);
	Data = WheelEncoder.m_MPH_Left;
	Printf(Row++,Col+22,1,"%3.3f",Data);
	
	Data = WheelEncoder.GetMetersSec(RIGHT_WHEEL_SENSOR);
	Printf(Row,Col+13,6,"%3.3f ",Data);
	Data = WheelEncoder.GetMetersSec(LEFT_WHEEL_SENSOR);
	Printf(Row++,Col+22,1,"%3.3f",Data);

	Data = WheelEncoder.GetRate(RIGHT_WHEEL_SENSOR);
	Printf(Row,Col+13,6,"%3.3f ",Data);
	Data = WheelEncoder.GetRate(LEFT_WHEEL_SENSOR);
	Printf(Row++,Col+22,1,"%3.3f",Data);

	Data2 = WheelEncoder.m_RightDistanceCount;
	Printf(Row,Col+13,1,"%3.0f",Data2);
	Data2 = WheelEncoder.m_LeftDistanceCount;
	Printf(Row++,Col+22,1,"%3.0f",Data2);
	
	Data2 = WheelEncoder.GetDistance(RIGHT_WHEEL_SENSOR);
	Printf(Row,Col+13,1,"%3.3f",(double)Data2);
	Data2 = WheelEncoder.m_LeftDistance;
	Data2 = WheelEncoder.GetDistance(LEFT_WHEEL_SENSOR);
	Printf(Row++,Col+22,1,"%3.3f",(double)Data2);
	
	Printf(++Row,Col+13,6,"%d ",WheelEncoder.m_SampleTime);

	Printf(++Row,Col+13,6,"%d ",TotalTime++);
	Data2 = Compass.GetHeadingDegrees();
	Printf(++Row,Col+13,6,"%3.2f ",(double)Data2);
	
	Data2 = Compass.GetTurnRate();
	//Printf(++Row,Col+13,6,"%f ",Data2);
	if(Data2 > MaxRad)
	{
		Printf(++Row,Col+13+10,6,"%f",(double)Data2);
		MaxRad=Data2;
	}
		else
		Printf(++Row,Col+13,6,"%f",(double)Data2);
		
	Data2 = Data2*57.3;
	if(Data2 > MaxDeg)
	{
		Printf(++Row,Col+13+10,6,"%f ",(double)Data2);
		MaxDeg =Data2;
	}	
		else
		Printf(++Row,Col+13,6,"%f ",(double)Data2);
		
	Sensors.UpDate();		
	Data2 = Sensors.m_SensorValues.VoltsBattery_1;
	Printf(++Row,Col+14,6,"%3.2f ",(double)Data2/10);
	if(Data2 < 69)
	   Buzzer.Tone();

}

void CDebugDisplay::DisplayMotors()
{
	static bool DisplayText =true;
	float Data;
	uint8_t Row = DEBUG_MOTOR_ROW;
	uint8_t Col = DEBUG_MOTOR_ROW;
	
	if(DisplayText)
	{
		Printf(Row++,Col,1,"Front Right");
		Printf(Row++,Col,1,"Front Left");
		Printf(Row++,Col,1,"Rear Right");
		Printf(Row++,Col,1,"Rear Left");
		DisplayText = false;
	}
	Col += 14;
	Row = DEBUG_MOTOR_ROW;
//	Printf(Row++,Col,5,"%d",Motors.m_Motors.FrontRightMotor.PwmValue);
//	Printf(Row++,Col,5,"%d",Motors.m_Motors.FrontLeftMotor.PwmValue);
//	Printf(Row++,Col,5,"%d",Motors.m_Motors.RearRightMotor.PwmValue);
//	Printf(Row++,Col,5,"%d",Motors.m_Motors.RearLeftMotor.PwmValue);

}
	


void CDebugDisplay::DisplayImu()
{
	static bool DisplayHeadings = true;
	unsigned char Row = DEBUG_IMU_ROW;
	unsigned char Col = DEBUG_IMU_COL;
	if(DisplayHeadings)
	{
		
		Printf(Row++,Col+4,4,"IMU");
		Printf(Row,Col,4,"ROLL");
		Printf(Row++,Col+19,4,"deg");
		Printf(Row,Col,5,"PITCH");
		Printf(Row++,Col+19,3,"deg");
		Printf(Row,Col,12,"Heading Comp");
		Printf(Row++,Col+19,3,"deg");
		Printf(Row,Col,15,"Heading Flat");
		Printf(Row++,Col+19,3,"deg");
		DisplayHeadings = false;
	}
	Row = DEBUG_IMU_ROW+1;
	Col +=14;
	Printf(Row++,Col,4,"%d",Ahrs.GetRollD());
	Printf(Row++,Col,4,"%d",Ahrs.GetPitchD());
	Printf(Row++,Col,4,"%+3.1f",Ahrs.GetHeading());
}




void CDebugDisplay::Printf(unsigned char Row,unsigned char Col, unsigned char ClearLen,const char *fmt,  ... )
{
	uint8_t Len;
	char Buffer[DEBUG_MAX_LINE_LEN];
	va_list ap;
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY )
	{
		if(Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY)
			Printf(fmt);
		return;
	}

	va_start(ap, fmt);
	Len = vsprintf(Buffer, fmt, ap);
	va_end(ap);
	//	ATOMIC_SECTION_ENTER;
	ClearLine(Row,Col,ClearLen);
	m_Serial->WriteTxBuffer((uint8_t*) Buffer,Len,true);
// 	for(uint8_t i=0;i<Len;i++)
// 	{
// 		if (Buffer[i] == '\n')
// 			WriteChar('\r');
// 		WriteChar(Buffer[i]);
// 	}
	//	ATOMIC_SECTION_LEAVE;
}

void CDebugDisplay::Printf(const char *fmt,  ... )
{
	uint8_t Len;
	char Buffer[DEBUG_MAX_LINE_LEN];
	va_list ap;
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY  && !Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY)
		return;

	va_start(ap, fmt);
	Len = vsprintf(Buffer, fmt, ap);
	va_end(ap);
	//	ATOMIC_SECTION_ENTER;
	for(uint8_t i=0;i<Len;i++)
	{
		if (Buffer[i] == '\n')
		WriteChar('\r');
		WriteChar(Buffer[i]);
	}
	//	ATOMIC_SECTION_LEAVE;
}





void CDebugDisplay::ClearLine(unsigned char Row,unsigned char Col,unsigned char Length)
{
	unsigned char i;
	if(Length >70)
	Length = 70;
	SetDisplayRowCol(Row,Col);
	
	for(i=0;i<Length;i++)
	WriteChar(' ');
	SetDisplayRowCol(Row,Col);
}


void CDebugDisplay::SetDisplayRowCol(unsigned char Row,unsigned char Col)
{

	uint8_t TmpVal;
	unsigned char Hundreds;
	WriteChar(27);
	WriteChar('[');
	TmpVal = (Row/10)+'0';
	WriteChar(TmpVal);
	TmpVal = (Row%10)+'0';
	WriteChar(TmpVal);
	WriteChar(';');
	Hundreds = (Col/100);
	WriteChar(Hundreds+'0');
	Hundreds *= 100;
	TmpVal = Col - Hundreds;
	TmpVal = (TmpVal/10)+'0';
	WriteChar(TmpVal);
	TmpVal = (Col%10)+'0';
	WriteChar(TmpVal);
	WriteChar('f');

}


void CDebugDisplay::SetCursorState(bool OnOff)
{
	WriteChar( 27);	// ESC command
	if(OnOff)
		WriteBytes((uint8_t*) "[?25h",5);
	else
		WriteBytes((uint8_t*)"[?25l",5);
	
}


void CDebugDisplay::WriteChar(uint8_t Char)
{
	if(!Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		m_Serial->WriteChar(Char); 
}

void CDebugDisplay::WriteBytes(uint8_t *Bytes,uint8_t Size)
{
	if(!Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		m_Serial->WriteTxBuffer(Bytes,Size,true);

}

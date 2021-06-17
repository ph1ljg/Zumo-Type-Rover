/* 
* CGuiFunctions.cpp
*
* Created: 01/08/2017 14:04:58
* Author: phil
*/
#include "includes.h"

double TestDebug;

volatile uint16_t Fred23;
const char RcStickNames[]  = // names for dynamic generation of config GUI
"ARM;"
"UNARM;"
;
const char RcStickNamesNo = 2;


const char IntAuxSwitchNames[]  = // names for dynamic generation of config GUI
"Set OSD Page;"
"Auto Pilot;"
"Fwd / Rev;"
"Mechan  Mode;"
"Rth;"
"Control Head X;"
"Control Head Y;"
;
const char IntAuxSwitchNamesNo = 8;


const char GuiTaskNames[]  = // names for dynamic generation of config GUI
"IMU;"
"RADIO;"
"LED FLASH;"
"GUI;"
"SENSORS;"
"AHRS;"
"NAVIGATION;"
"BEARINGS;"
"AVOIDANCE;"
"FRSKY TEL;"
"OSD;"
"GPS;"
"ATITUDE;"
"Sensors;"
;

const char GuiAlarmNames[]  = // names for dynamic generation of config GUI
"BATTERY 1;"
"RC LOSS;"
"LOAD CONFIG;"
"IMU;"
"I2C;"
"RANGER;"
"MOTOR;"
"GPS FAIL;"
"NAVI CONFIG;"
"NAVI UPDATE;"
"COURSE FILE;"
"CALIBRATION;"
;

const char GuiSensorNames[]  = // names for dynamic generation of config GUI
"COMPASS;"
"Head;"
"R SONAR;"
"GPS;"
"VTX;"
"CAMERA;"
;

const char GuiFunctionFlagNames[]  = // names for dynamic generation of config GUI
"Motors On;"
"Store Mag Cal;"
"Erase Mag Cal;"
;

const char GuiServoNames[]  = // names for dynamic generation of config GUI
"Front L Mot PWM;"
"Front R Mot PWM;"
"Rear  R Mot PWM;"
"Rear  L Mot PWM;"
"Head Vert;"
"Head Horz;"
;




const char GuiPidNames[] = // names for dynamic generation of config GUI
"Steer Rate;"
"Steer Angle;"
"Nav;"
"Throttle;"
;
unsigned char NoOfPids = NO_OF_PIDS;

unsigned char NoOfServoes = 3;



// default constructor
// CGuiFunctions::CGuiFunctions(CUart* Uart)
// {
// 	intPowerTrigger1 = 0;
// 	capability = 0;
// 	m_Uart = Uart;
// 	m_CalibrateSticks = CALIBRATE_STICKS_NONE;
// 
// } //CGuiFunctions


// default constructor
CGuiFunctions::CGuiFunctions(CUart* Uart)
//CGuiFunctions::CGuiFunctions(CSoftwareSerial* Uart)
{
	intPowerTrigger1 = 0;
	capability = 0;
	m_Uart = Uart;
	m_CalibrateSticks = CALIBRATE_STICKS_NONE;

} //CGuiFunctions



// default destructor
CGuiFunctions::~CGuiFunctions()
{
	

} //~CGuiFunctions



void CGuiFunctions::TestTransmit(void)
{
	unsigned char i;
	for(i=0;i<20;i++)
	{
		m_Uart->WriteChar('A');
	}
}






void CGuiFunctions::EvaluateCommand()
{
	CMyMath Math;
	ServoConfig_t servoConf[NO_OF_SERVOS];
	RadioTrims_t RadioTrim[8];
	GuiNavigationConfig_t NavigationConfig;
	Limits_t Limits;
	GpsMission_t GpsMission;	GpsMission.NAV_state =0;
	GpsMission.NAV_error =0;
	uint8_t NavState =0;
	uint8_t Ans = 1;
	status_t status;
	GuiWaypoint_struct_t GuiWaypoint_struct;
	uint32_t AlarmReg =0;

	switch(m_RecievedCommand)
	{
	case GUIP_IDENT:															//out message     Type + Version + protocol version + capability variable		Ident_t Ident;		Ident.Version     = VERSION;		Ident.t     = 0;		Ident.GuiVersion = GUIP_VERSION;		Ident.NavigationVersion   = NAVIGATION_VERSION; 		SendOutputFrame((unsigned char*)&Ident,8);		TaskManager.EnableTask(TASK_UPDATE_DEBUG_DISPLAY,false);			// turn off debug as both cannot run at the same time		break;	case GUIP_STATUS:														//out message    Cycle time & errors_count & sensor present & box activation & current setting number		status.cycleTime = 0;//  AutoBoat.cycleTime;		status.I2CErrorsCount	= I2c.m_I2cErrorsTotal;		status.sensor = Zumo.m_DevicesOnLine;		status.sensor	|= SEN_UBLOX_M8; 					//out message    		status.flag				= 0;		status.flag				|= Config.m_RunningFlags.ARMED;
		SendOutputFrame((unsigned char*)&status,11);		break;	case GUIP_LIMITS: 		Limits.MaximumPitchAngle		= Config.m_NavigationConfig.MaxPitchAngle;	// in degrees
 		Limits.MaximumRollAngle		= Config.m_NavigationConfig.MaxRollAngle;;	// in degrees
 		Limits.MinFrontDistance			= Config.m_MainConfig.SafeFrontDistance;
 		Limits.MinRearDistance			= Config.m_MainConfig.SafeRearDistance;
 		Limits.MinSideDistance			= Config.m_MainConfig.SafeSideDistance;
		Limits.MaxThrottleAcceleration			= round(Config.m_MainConfig.MaxThrottleAcceleration*100);	// m/s
		Limits.MaximumSpeed			= round(Config.m_NavigationConfig.MaximumSpeed*100);	
		SendOutputFrame((unsigned char*)&Limits,sizeof(Limits_t));		break;	case GUIP_SET_LIMITS: 		RecieveData((unsigned char*)&Limits,sizeof(Limits_t));
		Config.m_NavigationConfig.MaxPitchAngle		= Limits.MaximumPitchAngle;		// in degrees
 		Config.m_NavigationConfig.MaxRollAngle		= Limits.MaximumRollAngle;		// in degrees
		Config.m_MainConfig.SafeFrontDistance			= Limits.MinFrontDistance;	// to Cm
		Config.m_MainConfig.SafeRearDistance			= Limits.MinRearDistance;	// to CM
		Config.m_MainConfig.SafeSideDistance			= Limits.MinSideDistance;	// to CM
		Config.m_MainConfig.MaxThrottleAcceleration			= (float)Limits.MaxThrottleAcceleration/100; // m/s	
		Config.m_NavigationConfig.MaximumSpeed	= (float)Limits.MaximumSpeed/100;		// cm/s				break; 		case GUIP_MOTOR_TRIM:		MotorTrim_t mTrims;		mTrims.FrontRightMotorTrim = Config.m_MainConfig.MotorTrims[0];		mTrims.FrontLeftMotorTrim = Config.m_MainConfig.MotorTrims[1];		mTrims.RearRightMotorTrim = Config.m_MainConfig.MotorTrims[2];		mTrims.RearLeftMotorTrim = Config.m_MainConfig.MotorTrims[3];		SendOutputFrame((unsigned char*)&mTrims,sizeof(MotorTrim_t));		break;	case GUIP_SET_MOTOR_TRIM:		MotorTrim_t Trims; 		RecieveData((unsigned char*)&Trims,sizeof(MotorTrim_t));
		Config.m_MainConfig.MotorTrims[0]	= Trims.FrontRightMotorTrim;	
		Config.m_MainConfig.MotorTrims[1]	= Trims.FrontLeftMotorTrim;	
		Config.m_MainConfig.MotorTrims[2]	= Trims.RearRightMotorTrim;	
		Config.m_MainConfig.MotorTrims[3]	= Trims.RearLeftMotorTrim;	
		break;		case GUIP_PID_NAMES:		WriteNamesToOutputFrame(GuiPidNames);		 break;	case GUIP_SET_PID:		RecieveData((unsigned char*)&m_PidValues,sizeof(GuiPidValues_t));		//in message         Set  P I D coefficients (9 are used currently)		for(uint i=0;i<NO_OF_PIDS;i++)		{
			Config.m_MainConfig.PidValues[i].PVal = (float)m_PidValues.Pid[i].P/10.0;
			Config.m_MainConfig.PidValues[i].IVal = (float)m_PidValues.Pid[i].I/100.0;
			Config.m_MainConfig.PidValues[i].DVal = (float)m_PidValues.Pid[i].D/10.0;
		}
		Acknowledge();		AttitudeControl.SteeringRatePid.SetValues(Config.m_MainConfig.PidValues[STEERING_RATE_PID].PVal,Config.m_MainConfig.PidValues[STEERING_RATE_PID].IVal,Config.m_MainConfig.PidValues[STEERING_RATE_PID].DVal);		AttitudeControl.ThrottleSpeedPid.SetValues(Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].PVal,Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].IVal,Config.m_MainConfig.PidValues[THROTTLE_SPEED_PID].DVal);		//SteeringAnglePid.Set_p(Config.m_MainConfig.PidValues[STEER_ANGLE_PID_NO].PVal);		break;	case GUIP_PID:		for(uint i=0;i<NO_OF_PIDS;i++)		{			m_PidValues.Pid[i].P = (uint16_t)(Config.m_MainConfig.PidValues[i].PVal*10);			m_PidValues.Pid[i].I = (uint16_t)(Config.m_MainConfig.PidValues[i].IVal*100);			m_PidValues.Pid[i].D = (uint16_t)(Config.m_MainConfig.PidValues[i].DVal*10);		}		SendOutputFrame((unsigned char*)&m_PidValues,sizeof(GuiPidValues_t));		break;	case GUIP_SERVO_NAMES:
		WriteNamesToOutputFrame(GuiServoNames);		break;
    case GUIP_RADIO_TRIMS:
		for(uint8_t i =0;i<8;i++)
		{
			RadioTrim[i].min = Config.m_MainConfig.RadioTrims[i].min;
			RadioTrim[i].max = Config.m_MainConfig.RadioTrims[i].max;
			RadioTrim[i].middle = Config.m_MainConfig.RadioTrims[i].middle;
			RadioTrim[i].Reverse = Config.m_MainConfig.RadioTrims[i].Reverse;
		}
		SendOutputFrame((unsigned char*)&RadioTrim[0],sizeof(RadioTrims_t)*8); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
		break;
	case GUIP_SET_RADIO_TRIMS:		RecieveData((unsigned char*)&RadioTrim[0],sizeof(RadioTrims_t)*8);		for(uint8_t i =0;i<8;i++)
		{
			Config.m_MainConfig.RadioTrims[i].min			= RadioTrim[i].min;
			Config.m_MainConfig.RadioTrims[i].max			= RadioTrim[i].max;
			Config.m_MainConfig.RadioTrims[i].middle		= RadioTrim[i].middle;
			Config.m_MainConfig.RadioTrims[i].Reverse	= RadioTrim[i].Reverse;
		}		Acknowledge();		break;	case GUIP_SERVO_CONF:
		for(uint8_t i =0;i<NO_OF_SERVOS;i++)
		{
			servoConf[i].min = Config.m_MainConfig.ServoConf[i].min;
			servoConf[i].max = Config.m_MainConfig.ServoConf[i].max;
			servoConf[i].middle = Config.m_MainConfig.ServoConf[i].middle;
			servoConf[i].Rate = Config.m_MainConfig.ServoConf[i].Rate;
			servoConf[i].Reverse = Config.m_MainConfig.ServoConf[i].Reverse;
		}
		SendOutputFrame((unsigned char*)&servoConf[0],sizeof(ServoConfig_t)*NO_OF_SERVOS); // struct servo_conf_ is 7 bytes length: min:2 / max:2 / middle:2 / rate:1    ----     8 servo =>  8x7 = 56
 		break;
	case GUIP_SET_SERVO_CONF:		RecieveData((unsigned char*)&servoConf[0],sizeof(ServoConfig_t)*NO_OF_SERVOS);		for(uint8_t i =0;i<NO_OF_SERVOS;i++)
		{
			Config.m_MainConfig.ServoConf[i].min		= servoConf[i].min; 
			Config.m_MainConfig.ServoConf[i].max		= servoConf[i].max;
			Config.m_MainConfig.ServoConf[i].middle		= servoConf[i].middle;
			Config.m_MainConfig.ServoConf[i].Rate		= servoConf[i].Rate;
			Config.m_MainConfig.ServoConf[i].Reverse	= servoConf[i].Reverse;
		}		Acknowledge();		break; 	case GUIP_SET_MISC:														//in message          power meter trig + 8 free for future use		 RecieveData((unsigned char*)&misc,sizeof(misc_t));		 Acknowledge();		 intPowerTrigger1											= misc.PowerTrigger;		 Config.m_MainConfig.MinThrottleValue			= misc.MinThrottleValue ;		 Config.m_MainConfig.MaxThrottleValue			= misc.MaxThrottleValue;		 Config.m_MainConfig.FailsafeThrottleValue		= misc.FailsafeThrottleValue;		 Config.m_NavigationConfig.MagDeclination		= misc.MagDeclination;		 Config.m_MainConfig.Bat_1_LevelWarning		= misc.Bat_1_LevelWarning;		 Config.m_MainConfig.Bat_2_LevelWarning		= misc.Bat_2_LevelWarning;		 Config.m_MainConfig.Battery_1_Critical			= misc.Battery_1_Critical;		 Config.m_MainConfig.Battery_2_Critical			= misc.Battery_2_Critical;		 Config.m_MainConfig.Batt_1_Scale					= misc.Batt_1_Scale;		 Config.m_MainConfig.Batt_2_Scale					= misc.Batt_2_Scale;		 Config.m_MainConfig.MaxMotorPwm				= misc.MaxMotorPwm;		 break;	case GUIP_MISC:															//out message         power meter trig(not used), Minimum Throttle, Max Throttle,Min Command		 misc.PowerTrigger			= intPowerTrigger1;		 misc.MinThrottleValue		= Config.m_MainConfig.MinThrottleValue;		 misc.MaxThrottleValue		= Config.m_MainConfig.MaxThrottleValue;		 misc.FailsafeThrottleValue	= Config.m_MainConfig.FailsafeThrottleValue;		 misc.MagDeclination		= Config.m_NavigationConfig.MagDeclination;		 misc.Batt_1_Scale				= Config.m_MainConfig.Batt_1_Scale;		 misc.Batt_2_Scale				= Config.m_MainConfig.Batt_2_Scale;		 misc.Bat_1_LevelWarning		= Config.m_MainConfig.Bat_1_LevelWarning;		 misc.Bat_2_LevelWarning		= Config.m_MainConfig.Bat_2_LevelWarning;		 misc.Battery_1_Critical		= Config.m_MainConfig.Battery_1_Critical;		 misc.Battery_2_Critical		= Config.m_MainConfig.Battery_2_Critical;		 misc.MaxMotorPwm			= Config.m_MainConfig.MaxMotorPwm;		 SendOutputFrame((unsigned char*)&misc,sizeof(misc_t));		 break;	case GUIP_SET_HEAD:		 RecieveData((unsigned char*)&magHold,2);		 Acknowledge();		 break;	case GUIP_MOTOR:		 {			 CMyMath Math;			 GuiMotors_t GuiMotors;			 GuiMotors.MotorSpeed_1	= Math.Map(Motors.m_Motors.FrontRightMotor.PwmValue,MOTOR_RAW_DRIVE_PWM_MIN,MOTOR_RAW_DRIVE_PWM_MAX,0,100);			 GuiMotors.MotorSpeed_2 = Math.Map(Motors.m_Motors.FrontLeftMotor.PwmValue,MOTOR_RAW_DRIVE_PWM_MIN,MOTOR_RAW_DRIVE_PWM_MAX,0,100);			 GuiMotors.MotorSpeed_3	= Math.Map(Motors.m_Motors.RearRightMotor.PwmValue,MOTOR_RAW_DRIVE_PWM_MIN,MOTOR_RAW_DRIVE_PWM_MAX,0,100);			 GuiMotors.MotorSpeed_4 = Math.Map(Motors.m_Motors.RearLeftMotor.PwmValue,MOTOR_RAW_DRIVE_PWM_MIN,MOTOR_RAW_DRIVE_PWM_MAX,0,100);			 GuiMotors.MotorDirection_1 = Motors.m_Motors.FrontRightMotor.Direction;			 GuiMotors.MotorDirection_2 = Motors.m_Motors.FrontLeftMotor.Direction;			 GuiMotors.MotorDirection_3 = Motors.m_Motors.RearLeftMotor.Direction;			 GuiMotors.MotorDirection_4 = Motors.m_Motors.RearRightMotor.Direction;//			 GuiMotors.RobotSpeed = (uint8_t)(WheelEncoder.GetMph()*10);			 SendOutputFrame((unsigned char*)&GuiMotors,sizeof(GuiMotors_t));					//out message       up to  4 motors		 }		 break;	case GUIP_SET_MOTOR_SPEED:		UpdateMotors_t UpdateMotors;		RecieveData((unsigned char*)&UpdateMotors,9);		Acknowledge();		Motors.PassthroughUpdate(UpdateMotors);		break; 	case GUIP_RAW_GPS:
		 RawGpsValues_t RawGpsValues;
 		RawGpsValues.GuiGpsFixType			= Gps.m_GpsReadings.GpsFixType;
 		RawGpsValues.NumSats			= Gps.m_GpsReadings.NumSats;
 		if(Navigation.m_Debug)
 		{
	 		RawGpsValues.PresentLatitude	= NavigationFunctions.m_Bearings.nPresentLatitude;
	 		RawGpsValues.PresentLongitude	= NavigationFunctions.m_Bearings.nPresentLongitude;
 		}
 		else
 		{
			if(Gps.m_GpsReadings.GpsFixType == GPS_OK_FIX_3D || Gps.m_GpsReadings.GpsFixType == GPS_OK_FIX_DGPS)
	 		{
				RawGpsValues.PresentLatitude	= Gps.m_GpsPositionReadings.nLatitude ;
		 		RawGpsValues.PresentLongitude	= Gps.m_GpsPositionReadings.nLongitude;
	 		}
	 		else
	 		{
		 		RawGpsValues.PresentLatitude = 524413275;
		 		RawGpsValues.PresentLongitude =  -19842523;
	 		}
	 		
 		}
 		RawGpsValues.Altitude			= Gps.m_GpsReadings.Altitude;
 		RawGpsValues.GroundSpeed		= Gps.m_GpsReadings.GroundSpeed;
 		SendOutputFrame((unsigned char*)&RawGpsValues,14);
 		break;
	 case GUIP_COMP_GPS:
 		struct
 		{
	 		uint16_t DistanceToHome;
	 		int16_t BoatToHomeBearing;
	 		uint8_t Update;
			uint8_t AtHomePosition; 
 		} MspBearings;
 		MspBearings.DistanceToHome		= (uint16_t)NavigationFunctions.m_Bearings.DistanceToHome*10;
 		MspBearings.BoatToHomeBearing	= NavigationFunctions.m_Bearings.BoatToHomeBearing;
 		MspBearings.Update				= Gps.m_Update;								// controls GUI Bridge gps pkt led to show update
 		MspBearings.AtHomePosition		= NavigationFunctions.m_AtHomePosition;
		 SendOutputFrame((unsigned char*)&MspBearings,5);
 		break;
	case GUIP_SET_NAV_CONFIG:
 		RecieveData((unsigned char*)&NavigationConfig,sizeof(GuiNavigationConfig_t)); 		Acknowledge();
 		Config.m_NavigationConfig.NavigationFlags			= NavigationConfig.Flags;											// bit 1 Filtering 2 Lead filter 3 rudder pid  4 enable Lidar 5 Enable sonar 6?  7 slow nav
 		Config.m_NavigationConfig.WaypointRadius			= NavigationConfig.WpRadius;									// in cm
 		Config.m_NavigationConfig.CruiseSpeed					= NavigationConfig.CruiseSpeed/100;						// in cm/s
 		Config.m_NavigationConfig.SteerAccelMax				= (float)(NavigationConfig.SteerAccelMax)/100;		// in rads/sec
 		Config.m_NavigationConfig.MaxTurnRate				= (float)(NavigationConfig.MaxTurnRate)/100;			// in rads/sec
 		Config.m_NavigationConfig.WheelRadius				= (float)(NavigationConfig.WheelRadius)/10;				// in mm
 		Config.m_NavigationConfig.CruiseThrottle				= NavigationConfig.CruiseThrottle;								// in % 0-100
 		Config.m_NavigationConfig.TurnRadius					= (float) NavigationConfig.TurnRadius/10;					// in meters
 		Config.m_NavigationConfig.ReverseSpeed				= NavigationConfig.ReverseSpeed;								// in cm/s
 		Config.m_NavigationConfig.MaxNavRoll					= NavigationConfig.MaxNavRoll;									// degree * 100; (3000 default)
 		Config.m_NavigationConfig.MaxNavPitch					= NavigationConfig.MaxNavPitch;								// degree * 100; (3000 default)
 		Config.m_NavigationConfig.CrosstrackGain				= NavigationConfig.CrosstrackGain;							// * 100 (0-2.56)
 		Config.m_NavigationConfig.FenceDistance				= NavigationConfig.FenceRadius;								// fence control in meters
 		Config.m_NavigationConfig.MaxNumberOfWayPoints		= NavigationConfig.MaxNumberOfWayPoints;
		 Config.m_NavigationConfig.CruiseSpeed	/=10; // to m/s
		 break;
 	case GUIP_NAV_CONFIG:
 		
		 NavigationConfig.Flags 					= Config.m_NavigationConfig.NavigationFlags;
 		NavigationConfig.WpRadius				= Config.m_NavigationConfig.WaypointRadius;								// in cm
 		NavigationConfig.MaxNavRoll				= Config.m_NavigationConfig.MaxNavRoll;										// in cm
 		NavigationConfig.MaxNavPitch			= Config.m_NavigationConfig.MaxNavPitch;
 		NavigationConfig.CruiseSpeed			= Config.m_NavigationConfig.CruiseSpeed*100;								// Cm/s for gui
 		NavigationConfig.SteerAccelMax			= round(Config.m_NavigationConfig.SteerAccelMax*100);
 		NavigationConfig.MaxTurnRate			= round(Config.m_NavigationConfig.MaxTurnRate*100);
 		NavigationConfig.WheelRadius			= round(Config.m_NavigationConfig.WheelRadius*10);
 		NavigationConfig.CruiseThrottle		= Config.m_NavigationConfig.CruiseThrottle;										// in cm/s
 		NavigationConfig.TurnRadius				= round(Config.m_NavigationConfig.TurnRadius*10);						//meters
 		NavigationConfig.ReverseSpeed			= Config.m_NavigationConfig.ReverseSpeed;									// in cm/s
 		NavigationConfig.CrosstrackGain		= Config.m_NavigationConfig.CrosstrackGain;									// * 100 (0-2.56)
 		NavigationConfig.FenceRadius			= Config.m_NavigationConfig.FenceDistance;									// fence control in meters
 		NavigationConfig.MaxNumberOfWayPoints	= Config.m_NavigationConfig.MaxNumberOfWayPoints;
 		NavigationConfig.CruiseSpeed			*= 10;

		SendOutputFrame((unsigned char*)&NavigationConfig,sizeof(GuiNavigationConfig_t));
 		break;
 	case GUIP_NAV_STATUS: 
 		GpsMission.GPS_Hdop			= Gps.m_GpsReadings.Hdop; // Gps Hdop 		GpsMission.NAV_state		= Navigation.m_NavigationState;										// NAV_STATE_NONE;  /// State of the nav engine 		GpsMission.action			= 0;										// Action to follow 		GpsMission.number			= Navigation.m_NextWaypointNo;				// Waypoint number 		GpsMission.NAV_error		= Navigation.m_NavigationError;										// NAV_ERROR_NONE; 		GpsMission.target_bearing	= NavigationFunctions.m_Bearings.NextWpBearing;		// This is the angle from the boat to the "next_WP" location in degrees * 100 		SendOutputFrame((unsigned char*)&GpsMission,sizeof(GpsMission)); 		break;
 	case GUIP_WP:
 		uint8_t wp_no;
 		RecieveData((unsigned char*)&wp_no,1);
 		GuiWaypoint_struct.number = wp_no;
 		
 		if (wp_no == 0)				//send HOME coordinates
 		{
	 		GuiWaypoint_struct.GpsCordsLat = Config.m_CourseWaypoints[wp_no].nLatitide;		// waypoint 0 is home waypoint
	 		GuiWaypoint_struct.GpsCordsLon = Config.m_CourseWaypoints[wp_no].nLongitude;
 		}
 		if (wp_no == 255)			//send  pos hold coordinates
 		{
	 		GuiWaypoint_struct.GpsCordsLat = Navigation.m_GPSHoldPosition[0];
	 		GuiWaypoint_struct.GpsCordsLon = Navigation.m_GPSHoldPosition[1];
	 		GuiWaypoint_struct.flag = MISSION_FLAG_HOLD;
 		}
 		if ((wp_no>0) && (wp_no<255))
 		{
	 		if(!Config.m_RunningFlags.MISSION_DATA_LOADED)
	 		{
		 		GuiWaypoint_struct.GpsCordsLat = 0;
		 		GuiWaypoint_struct.GpsCordsLon = 0;
		 		GuiWaypoint_struct.flag = WP_FLAG_LOAD_ERROR;
	 		}
	 		
	 		if (NavState == NAV_STATE_NONE)
	 		{
		 		GuiWaypoint_struct.GpsCordsLat	= Config.m_CourseWaypoints[wp_no].nLatitide;
		 		GuiWaypoint_struct.GpsCordsLon	= Config.m_CourseWaypoints[wp_no].nLongitude;
		 		GuiWaypoint_struct.flag			= Config.m_CourseWaypoints[wp_no].Flag;
	 		}
	 		else
	 		{
		 		GuiWaypoint_struct.GpsCordsLat	= Config.m_CourseWaypoints[0].nLatitide;
		 		GuiWaypoint_struct.GpsCordsLon	= Config.m_CourseWaypoints[0].nLongitude;
		 		GuiWaypoint_struct.flag			= MISSION_FLAG_NAV_IN_PROG;
	 		}
 		}

 		GuiWaypoint_struct.action		= Config.m_CourseWaypoints[wp_no].Action;
 		GuiWaypoint_struct.altitude		= Config.m_CourseWaypoints[wp_no].altitude;
 		GuiWaypoint_struct.parameter1	= Config.m_CourseWaypoints[wp_no].Parameter1;
 		GuiWaypoint_struct.parameter2	= Config.m_CourseWaypoints[wp_no].Parameter2;
 //		GuiWaypoint_struct.parameter3	= NavigationProcessor.m_CourseWaypoints[wp_no].Parameter3;
 		SendOutputFrame((unsigned char*)&GuiWaypoint_struct,sizeof(GuiWaypoint_struct_t));
 		break;
	case GUIP_SET_WP:															// In Message get WPs
 		RecieveData((unsigned char*)&GuiWaypoint_struct,sizeof(GuiWaypoint_struct));
 		Acknowledge();
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].Number = GuiWaypoint_struct.number;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].Action = GuiWaypoint_struct.action;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].nLatitide = GuiWaypoint_struct.GpsCordsLat;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].nLongitude = GuiWaypoint_struct.GpsCordsLon;
 		
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].altitude = GuiWaypoint_struct.altitude;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].Parameter1 = GuiWaypoint_struct.parameter1;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].Parameter2 = GuiWaypoint_struct.parameter2;
// 		NavigationProcessor.m_CourseWaypoints[GuiWaypoint_struct.number].Parameter3 = GuiWaypoint_struct.parameter3;
 		Config.m_CourseWaypoints[GuiWaypoint_struct.number].Flag = GuiWaypoint_struct.flag;
 		if(GuiWaypoint_struct.flag ==WP_FLAG_END)
 		{
	 		Config.WriteCourseWp();
	 		Config.m_RunningFlags.MISSION_DATA_LOADED = true;
 		}
 		break;
 		
 	case GUIP_LOG_FILE:
//  		uint8_t RecordNum;
// 		LoggerRecord_t Record;
//  		RecieveData((unsigned char*)&RecordNum,1);
//  		if(RecordNum == 1)
//  		{
// 	 		DataLogger.ReadLogRecord(true,RecordNum,Record);
// 	 		
//  		}
//  		else
//  			DataLogger.ReadLogRecord(false,RecordNum,Record);
// 		SendOutputFrame((unsigned char*)&Record,sizeof(LoggerRecord_t));
// 		printf("%d\n",RecordNum);
//		 		 break;
	case GUIP_DELETE_LOG_FILE:
//		 DataLogger.DeleteLogfile();
		 Acknowledge();
		break;	 
	case GUIP_RADAR:
		UpdateRadar_t UpdateRadar;
		UpdateRadar.gHEAD_40			= HeadControl.m_Sectors[HEAD_40].Distance;
		UpdateRadar.gHEAD_30			= HeadControl.m_Sectors[HEAD_30].Distance;
		UpdateRadar.gHEAD_20			= HeadControl.m_Sectors[HEAD_20].Distance;
		UpdateRadar.gHEAD_10			= HeadControl.m_Sectors[HEAD_10].Distance;
		UpdateRadar.gHEAD_CENTER = HeadControl.m_Sectors[HEAD_CENTER].Distance;
		UpdateRadar.gHEAD_NEG_10 = HeadControl.m_Sectors[HEAD_NEG_10].Distance;
		UpdateRadar.gHEAD_NEG_20 = HeadControl.m_Sectors[HEAD_NEG_20].Distance;
		UpdateRadar.gHEAD_NEG_30 = HeadControl.m_Sectors[HEAD_NEG_30].Distance;
		UpdateRadar.gHEAD_NEG_40 = HeadControl.m_Sectors[HEAD_NEG_40].Distance;
		SendOutputFrame((unsigned char*)&UpdateRadar,sizeof(UpdateRadar_t));					//out message
		break;
// 	case GUIP_RADAR:
// 		UpdateRadar_t UpdateRadar;
// 		UpdateRadar.gHEAD_40			= 300;
// 		UpdateRadar.gHEAD_30			= 300;
// 		UpdateRadar.gHEAD_20			= 300;
// 		UpdateRadar.gHEAD_10			=100 ;
// 		UpdateRadar.gHEAD_CENTER =300;
// 		UpdateRadar.gHEAD_NEG_10 =270;
// 		UpdateRadar.gHEAD_NEG_20 = 300;
// 		UpdateRadar.gHEAD_NEG_30 = 300;
// 		UpdateRadar.gHEAD_NEG_40 = 300;
// 		SendOutputFrame((unsigned char*)&UpdateRadar,sizeof(UpdateRadar_t));					//out message
// 		break;
	case GUIP_DISTANCE:	 		 UpdateDistance_t UpdateDistance;		 UpdateDistance.Front= Avoidance.Sector.Distance;		 UpdateDistance.Rear = Avoidance.m_RearDistance;		 UpdateDistance.Right= Sensors.m_SensorValues.RightLidar;		 UpdateDistance.Left = Sensors.m_SensorValues.LeftLidar;		 SendOutputFrame((unsigned char*)&UpdateDistance,sizeof(UpdateDistance_t));					//out message     		 break;  	 case GUIP_WHEEL_COUNTERS:
		 UpdateWheelSensors_t WheelSensors;
		 WheelSensors.RightWheel = WheelEncoder.m_RightTotalCount;
		 WheelSensors.LeftWheel = WheelEncoder.m_LeftTotalCount;
		 SendOutputFrame((unsigned char*)&WheelSensors,sizeof(UpdateWheelSensors_t));					//out message     		 break;
	case GUIP_RAW_IMU:														//out message   Send the unfiltered Gyro & Accelerometer values to GUI.		GuiImu_t ImuValues;		ImuValues.gyroData[0] = Imu.m_GyroFiltered.x*100;		// send raw values		ImuValues.gyroData[1] = Imu.m_GyroFiltered.y*100;		// send raw values		ImuValues.gyroData[2] = Imu.m_GyroFiltered.z*100;		// send raw values			 		ImuValues.AccData[0]= Imu.m_AccelFiltered.x*100;		ImuValues.AccData[1]= Imu.m_AccelFiltered.y*100;		ImuValues.AccData[2]= Imu.m_AccelFiltered.z*100;			 		ImuValues.MagData[0] = Imu.m_LinAccelFiltered.x*100;		ImuValues.MagData[1] = Imu.m_LinAccelFiltered.y*100;		ImuValues.MagData[2] = Imu.m_LinAccelFiltered.z*100;		SendOutputFrame((unsigned char*)&ImuValues,sizeof(GuiImu_t));		break;	case GUIP_ATTITUDE:		 GuiAttitude_t Att;		 Att.Angle[ROLL] = Ahrs.GetRollD()*10;		 Att.Angle[PITCH] = Ahrs.GetPitchD()*10;		 Att.Heading = (int16_t) Ahrs.GetHeading();		 Att.RelativeBearing = NavigationFunctions.m_Bearings.RelativeBearing;		 SendOutputFrame((unsigned char*)&Att,sizeof(GuiAttitude_t));							//out message       2 angles 1 heading		 break;	case GUIP_ANALOG:		 AnalogTemp_t Analog;		 Analog.Battery_1_Volts = (uint16_t)(Math.Round(Sensors.m_SensorValues.VoltsBattery_1));		 Analog.Battery_2_Volts = 0;		 Analog.intPowerMeterSum = 0;		 Analog.rssi = 0;		 Analog.amperage = 0;		 SendOutputFrame((unsigned char*)&Analog,sizeof(AnalogTemp_t));		 break;	case GUIP_RESET_CONF:													//in message          Reset to defaults		 if(!Config.m_RunningFlags.ARMED)		 Config.LoadDefaultConfig(false);		 Acknowledge();		 break;
	case GUIP_ACC_CALIBRATION:												//in message          Start Accelerometer Calibration		 Ahrs.CalibrateLevel();		 Acknowledge();		 break;	 case GUIP_MAG_CALIBRATION:												//in message          Start Magnetometer Calibration		 Config.m_RunningFlags.CALIBRATE_MAG = true;		 Acknowledge();		 break;	case GUIP_EEPROM_WRITE:													//in message          no param		 Acknowledge();		 Config.WriteAllConfigs();		 break;	case GUIP_TASK_TIMINGS:		uint8_t Task_no;		RecieveData((unsigned char*)&Task_no,1);		TaskTimings_t TaskTimings;		TaskInfo_t  taskInfo;
		TaskManager.GetTaskInfo(Task_no, &taskInfo);
		TaskTimings.TaskId					= Task_no;
		TaskTimings.TaskPriority			= taskInfo.TaskPriority;
		TaskTimings.DesiredPeriod			= taskInfo.DesiredPeriod;
		TaskTimings.ActualPeriod			= taskInfo.ActualPeriod;
		TaskTimings.maxExecutionTime		= taskInfo.maxExecutionTime;
		TaskTimings.averageExecutionTime	= taskInfo.averageExecutionTime;
		TaskTimings.SystemLoad				= taskInfo.SystemLoad;
		TaskTimings.QueSize					= taskInfo.QueSize;
		SendOutputFrame((unsigned char*)&TaskTimings,sizeof(TaskTimings_t));		break;	case GUIP_TASK_NAMES:															//out message        Box Enable		WriteNamesToOutputFrame(GuiTaskNames);		break;	
	case GUIP_ALARM_NAMES:
		WriteNamesToOutputFrame(GuiAlarmNames);		break;	
	case GUIP_ALARMS:
		AlarmReg = GetAlarmStatus();
		SendOutputFrame((unsigned char*)&AlarmReg,4);						//out message
		break;
	case GUIP_SENSOR_NAMES:
		WriteNamesToOutputFrame(GuiSensorNames);		break;	
	case GUIP_CLI:
		break;
	case GUIP_RUNNING_FLAGS:
		GuiRunningFlags_t GuiRunningFlags;
		GetRunningFlags(GuiRunningFlags);
		SendOutputFrame((unsigned char*)&GuiRunningFlags,sizeof(GuiRunningFlags_t));						//out message
		break;
	case GUIP_FUNCTION_FLAGS:
		{
			uint32_t RFlags = 0;
		if(	Config.m_FunctionFlags.FunctionFlag_1)
			RFlags |= 0x1;
		if(	Config.m_FunctionFlags.FunctionFlag_2)
			RFlags |= 0x2;
		if(	Config.m_FunctionFlags.FunctionFlag_3)
			RFlags |= 0x4;
		if(	Config.m_FunctionFlags.FunctionFlag_4)
			RFlags |= 0x8;
		if(	Config.m_FunctionFlags.FunctionFlag_5)
			RFlags |= 0x10;
		if(	Config.m_FunctionFlags.FunctionFlag_6)
			RFlags |= 0x20;
		if(	Config.m_FunctionFlags.PassThroughFlag)
			RFlags |= 0x40;

		if(	Config.m_FunctionFlags.MotorReverseFlag)
			RFlags |= 0x80;

		if(	Config.m_RunningFlags.ARMED)			// this is preset cannot be changed with names
			RFlags |= 32768;
		 SendOutputFrame((unsigned char*)&RFlags,4);	
		}
		break;
	case GUIP_FUNCTION_FLAGS_NAMES:
		WriteNamesToOutputFrame(GuiFunctionFlagNames);		break;

	case GUIP_SET_FUNCTION_FLAGS:
		uint16_t R_Flags;		RecieveData((unsigned char*)&R_Flags,2);		Acknowledge();
		if(R_Flags & 0x01)
		{
			Config.m_FunctionFlags.FunctionFlag_1	= true;
		}
		else
		{
			Config.m_FunctionFlags.FunctionFlag_1	= false;
		}

		if(R_Flags & 0x02)
			Config.m_FunctionFlags.FunctionFlag_2	= true;
		else
			Config.m_FunctionFlags.FunctionFlag_2	= false;

		if(R_Flags & 0x04)
			Config.m_FunctionFlags.FunctionFlag_3	= true;
		else
			Config.m_FunctionFlags.FunctionFlag_3	= false;

		if(R_Flags & 0x08)
			Config.m_FunctionFlags.FunctionFlag_4	= true;
		else
			Config.m_FunctionFlags.FunctionFlag_4	= false;

		if(R_Flags & 0x10)
			Config.m_FunctionFlags.FunctionFlag_5	= true;
		else
			Config.m_FunctionFlags.FunctionFlag_5	= false;

		if(R_Flags & 0x20)
			Config.m_FunctionFlags.FunctionFlag_6	= true;
		else
			Config.m_FunctionFlags.FunctionFlag_6	= false;
		
		if(R_Flags & 0x40)
			Config.m_FunctionFlags.PassThroughFlag		= true;
		else
			Config.m_FunctionFlags.PassThroughFlag		= false;

		if(R_Flags & 0x80)
			Config.m_FunctionFlags.MotorReverseFlag		= true;
		else
			Config.m_FunctionFlags.MotorReverseFlag		= false;



		if(R_Flags & 0x8000)
			Zumo.SetArmedState(true);
		else
			Zumo.SetArmedState(false);

		break;
	case GUI_PID_RT_VALUES: //=====================================================================
	 {
			SendOutputFrame((unsigned char*)&m_PidRtValues,sizeof(PidRuningValues_t));
			break;
	 }
 		case GUIP_RC:																	//out message       8 rc channels and more		 {			uint16_t RadioDataOut[12];
			RadioDataOut[0] = RadioControl.m_RadioDataIn[RC_THROTTLE];									RadioDataOut[1] = RadioControl.m_RadioDataIn[RC_ELEV];									RadioDataOut[2] = RadioControl.m_RadioDataIn[RC_AILE];								RadioDataOut[3] = RadioControl.m_RadioDataIn[RC_RUDDER];																RadioDataOut[4] = RadioControl.m_RadioDataIn[RC_CHANNEL_5];																	RadioDataOut[5] = RadioControl.m_RadioDataIn[RC_CHANNEL_6];																	RadioDataOut[6] = RadioControl.m_RadioDataIn[RC_CHANNEL_7];																	RadioDataOut[7] = RadioControl.m_RadioDataIn[RC_CHANNEL_8];																	RadioDataOut[8] = RadioControl.m_RadioDataIn[RC_CHANNEL_9];																	RadioDataOut[9] = RadioControl.m_RadioDataIn[RC_CHANNEL_10];																	RadioDataOut[10] = RadioControl.m_RadioDataIn[RC_CHANNEL_11];																	RadioDataOut[11] = RadioControl.m_RadioDataIn[RC_CHANNEL_12];														 			SendOutputFrame((unsigned char*)&RadioDataOut,12*2);		 } 			break;	case GUIP_STICK_NAMES: 		WriteNamesToOutputFrame(RcStickNames);		break;	case GUIP_SET_RC_STICKS:														//in message          Internal Flags setup		RecieveData((unsigned char*)&Config.m_MainConfig.SticksSettings[0],2*RcStickNamesNo);		Acknowledge();		break;	case GUIP_RC_STICKS:															//out message        Sticks in use		SendOutputFrame((unsigned char*)&Config.m_MainConfig.SticksSettings[0],2*RcStickNamesNo);		break;	case GUIP_AUX_SWITCH_NAMES: 		WriteNamesToOutputFrame(IntAuxSwitchNames);		break;		 	case GUIP_SET_RC_AUX:														//in message          Internal Flags setup		RecieveData((unsigned char*)&Config.m_MainConfig.AuxSwitchSettings[0],IntAuxSwitchNamesNo*4);		Acknowledge();		break;	case GUIP_RC_AUX:															//out message        Box Enable		SendOutputFrame((unsigned char*)&Config.m_MainConfig.AuxSwitchSettings[0],IntAuxSwitchNamesNo*4);		break;	case GUIP_SET_DEBUG_FLAGS:
		RecieveData((unsigned char*)&Config.m_RealTimeGuiFlags,2);
		Acknowledge();
		Navigation.SetDebugNavigation( (Config.m_RealTimeGuiFlags & SET_NAVIGATION_DEBUG_MODE));
			
		if((Config.m_RealTimeGuiFlags & DEBUG_START_NAV))
		{
			Navigation.SetNavigationState(eStartNavigation);
		}
		
			
		if(Config.m_RealTimeGuiFlags & DEBUG_USE_SLOW_NAV)
			Navigation.SetDebugSlowNavigation(true);
		else
			Navigation.SetDebugSlowNavigation(false);
		break;
	case GUIP_RC_TUNING:
		{
			RcTuning_t RcTuning;
			RcTuning.DynThrPID		= 0;
			RcTuning.RcExpo			= Config.m_MainConfig.RcExpo;
			RcTuning.RcRate			= Config.m_MainConfig.RcRate;
			RcTuning.RollPitchRate	= 0;
			RcTuning.YawRate		= 0;
			SendOutputFrame((unsigned char*)&RcTuning,sizeof(RcTuning_t));
		}
		break;
	case GUIP_SET_RC_TUNING:
		{
 			RcTuning_t RcTuning;
 			RecieveData((unsigned char*)&RcTuning,sizeof(RcTuning));
			Config.m_MainConfig.RcExpo = RcTuning.RcExpo;
			Config.m_MainConfig.RcRate = RcTuning.RcRate;  
		}
		break;
	case GUIP_SET_START:
		Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY = true;
		 Acknowledge();		break;
	case GUIP_SET_STOP:
		Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY = false;
		Acknowledge();
		break;
	case GUIP_CALIBRATE_STICKS:
		m_CalibrateSticks = CALIBRATE_STICKS;
		SendOutputFrame((unsigned char*)&Ans,sizeof(uint8_t));
		break;			
	case GUIP_CALIBRATE_STICKS_SAVE:
		m_CalibrateSticks = CALIBRATE_STICKS_SAVE;
		SendOutputFrame((unsigned char*)&Ans,sizeof(uint8_t));
		break;
	case GUIP_RSSI:
		SendOutputFrame((unsigned char*)&RadioControl.m_Rssi,2);
		break;	
	case GUIP_SWITCH_NAMES:
		Acknowledge();
	case GUIP_SWITCHES: 
		Acknowledge();
	 }

}

void CGuiFunctions::Printf(const char *fmt,  ... )
{
	if(	Config.m_RunningFlags.ENABLE_GUI_CLI_DISPLAY )
		return;
		
	uint8_t Len;
	char Buffer[DEBUG_MAX_LINE_LEN];
	va_list ap;
	if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY )
		return;

	va_start(ap, fmt);
	Len = vsprintf(Buffer, fmt, ap);
	va_end(ap);

	for(uint8_t i=0;i<Len;i++)
	{
		if (Buffer[i] == '\n')
		m_Uart->WriteChar('\r');
		m_Uart->WriteChar(Buffer[i]);
	}
	//	ATOMIC_SECTION_LEAVE;
}


void CGuiFunctions::SetPidOutputValues(float Value, uint8_t Type)
{
	if(Type == GUI_PID_OUTPUT_OUTPUT)
			m_PidRtValues.PidOutput = int16_t(Value*100);
	if(Type == GUI_PID_OUTPUT_ERROR)
		m_PidRtValues.PidError = int16_t (Value*100);
	if(Type == GUI_PID_OUTPUT_INPUT)
		m_PidRtValues.PidInput = int16_t(Value*100);
}


uint32_t CGuiFunctions::GetAlarmStatus()
{
	uint32_t AlarmReg =0;
	uint8_t AlmIndex = 0;
	if(Config.m_RunningFlags.BATTERY_1_WARNING )
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(Config.m_RunningFlags.LOST_RC_SIGNAL)
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(!Config.m_RunningFlags.ALL_CONFIGS__LOADED)
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(Config.m_RunningFlags.IMU_FAIL)
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(Config.m_RunningFlags.I2C_INTERFACE_FAIL )
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(Config.m_RunningFlags.HEAD_LIDAR_FAIL)
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if(Config.m_RunningFlags.MOTOR_FAIL)
		AlarmReg |= (1<<AlmIndex);
	AlmIndex++;	
	if( !Gps.IsGpsUsable())
		AlarmReg |= (1<<AlmIndex);

	AlmIndex++;	
	if( Config.m_RunningFlags.RD_NAVIGATION_CFG_FAIL)
		AlarmReg |= (1<<AlmIndex);

	AlmIndex++;	
	if(Config.m_RunningFlags.NAVIGATION_FAIL)
		AlarmReg |= (1<<AlmIndex);

	AlmIndex++;	
	if(!Config.m_RunningFlags.MISSION_DATA_LOADED)
		AlarmReg |= (1<<AlmIndex);
	
	AlmIndex++;	
	if(!Config.m_RunningFlags.GYRO_CALIBRATED || !Config.m_RunningFlags.MAG_CALIBRATED || !Config.m_RunningFlags.ACC_CALIBRATED )
		AlarmReg |= (1<<AlmIndex);


	return(AlarmReg);
}



void CGuiFunctions::GetRunningFlags(GuiRunningFlags_t &GuiRunningFlags)
{
	 GuiRunningFlags.RFlags1 = 0;
	 GuiRunningFlags.RFlags2 = 0;
	  
	if(Config.m_RunningFlags.MAIN_INIT_COMPLETED)
		GuiRunningFlags.RFlags1 |= (1<<0);
	if(Config.m_RunningFlags.ALL_CONFIGS__LOADED)
		GuiRunningFlags.RFlags1 |= (1<<1);
	if(Config.m_RunningFlags.RSSI_LEVEL)
		GuiRunningFlags.RFlags1 |= (1<<2);
//	if(Config.m_RunningFlags.SD_CARD_PRESENT)
//		GuiRunningFlags.RFlags1 |= (1<<3);
	if(Config.m_RunningFlags.AVOIDANCE_INHIBIT)
		GuiRunningFlags.RFlags1 |= (1<<4);
	if(Config.m_RunningFlags.EEPROM_PRESENT)
		GuiRunningFlags.RFlags1 |= (1<<5);

	//=========	Sensors =================
	if(Config.m_RunningFlags.ACC_CALIBRATED)
		GuiRunningFlags.RFlags1 |= (1<<6);
	if(Config.m_RunningFlags.GYRO_CALIBRATED)
		GuiRunningFlags.RFlags1 |= (1<<7);
	if(Config.m_RunningFlags.MAG_CALIBRATED)
		GuiRunningFlags.RFlags1 |= (1<<8);
	if(Config.m_RunningFlags.CALIBRATE_MAG)
		GuiRunningFlags.RFlags1 |= (1<<9);
	if(Config.m_RunningFlags.HEAD_DISTANCE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<10);
	if(Config.m_RunningFlags.FRONT_DISTANCE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<11);
	if(Config.m_RunningFlags.RIGHT_DISTANCE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<12);
	if(Config.m_RunningFlags.LEFT_DISTANCE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<13);
	if(Config.m_RunningFlags.REAR_DISTANCE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<14);

	if(Config.m_RunningFlags.VTX_PRESENT)
		GuiRunningFlags.RFlags1 |= (1<<15);

			
	//=========	Alarms =================
	if(Config.m_RunningFlags.PANIC_MODE_TRIGGERED)
		GuiRunningFlags.RFlags1 |= (1<<17);
	if(Config.m_RunningFlags.ARMED)	  
		GuiRunningFlags.RFlags1 |= (1<<18);
	if(Config.m_RunningFlags.ANGLE_ALARM)
		GuiRunningFlags.RFlags1 |= (1<<19);
	if(Config.m_RunningFlags.LOST_RC_SIGNAL)
		GuiRunningFlags.RFlags1 |= (1<<20);
	//	  if(Config.m_RunningFlags.RC_FAILSAVE_IN_USE)
	//		 GuiRunningFlags.RFlags1 |= (1<<21);
	if(Config.m_RunningFlags.FAILSAFE_ACTIVE)
		GuiRunningFlags.RFlags1 |= (1<<22);
	if(Config.m_RunningFlags.LOGGING_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<23);
	if(Config.m_RunningFlags.NAVIGATION_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<24);
	if(Config.m_RunningFlags.RD_MAIN_CONFIG_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<25);
	if(Config.m_RunningFlags.RD_NAVIGATION_CFG_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<26);
	if(Config.m_RunningFlags.LOAD_WAYPOINTS_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<27);
	if(Config.m_RunningFlags.RD_RC_CONFIG_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<28);
	if(Config.m_RunningFlags.RD_FUNCTION_CONFIG_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<29);
	if(Config.m_RunningFlags.IMU_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<30);
	if(Config.m_RunningFlags.COMPASS_FAIL)
		GuiRunningFlags.RFlags1 |= (1<<31);
	if(Config.m_RunningFlags.HEAD_LIDAR_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<0);
	if(Config.m_RunningFlags.FRONT_LIDAR_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<1);
	if(Config.m_RunningFlags.RIGHT_LIDAR_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<2);
	if(Config.m_RunningFlags.LEFT_LIDAR_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<3);
	if(Config.m_RunningFlags.REAR_LIDAR_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<4);
	if(Config.m_RunningFlags.I2C_INTERFACE_FAIL)
		GuiRunningFlags.RFlags2 |= (1<<5);
			
	//========= MOTORS ==================
	  if(Config.m_RunningFlags.MOTOR_FAIL)
		 GuiRunningFlags.RFlags2 |= (1<<6);
	  if(!Config.m_RunningFlags.IN_REVERSE)
		 GuiRunningFlags.RFlags2 |= (1<<7);
	  if(!Config.m_RunningFlags.IN_REVERSE)
		 GuiRunningFlags.RFlags2 |= (1<<8);
			
	//========== Battery ===============
	  if(Config.m_RunningFlags.BATTERY_1_WARNING)
		 GuiRunningFlags.RFlags2 |= (1<<9);
	  if(Config.m_RunningFlags.BATTERY_2_WARNING)
		 GuiRunningFlags.RFlags2 |= (1<<10);
	  if(Config.m_RunningFlags.BATTERY_1_ALARM_CRITICAL)
		 GuiRunningFlags.RFlags2 |= (1<<11);
	  if(Config.m_RunningFlags.BATTERY_2_ALARM_CRITICAL)
		 GuiRunningFlags.RFlags2 |= (1<<12);
	  if(Config.m_RunningFlags.BATTERY_FAILSAFE)
		 GuiRunningFlags.RFlags2 |= (1<<13);
			
			
	//========== Displays ===============
	  if(Config.m_RunningFlags.DISABLE_DEBUG_DISPLAY)
		 GuiRunningFlags.RFlags2 |= (1<<14);
			
	//========== Logging ===============
	  if(Config.m_RunningFlags.LOGGING_IN_PROGRESS)
		 GuiRunningFlags.RFlags2 |= (1<<15);
			

	//========== Navigation ============
	  if(Config.m_RunningFlags.HOME_LOCATION_SET)
		 GuiRunningFlags.RFlags2 |= (1<<16);
	  if(Config.m_RunningFlags.AT_HOME_COORDINATES)
		 GuiRunningFlags.RFlags2 |= (1<<17);
	  if(Config.m_RunningFlags.CONTROL_BY_AUTO_PILOT)
		 GuiRunningFlags.RFlags2 |= (1<<18);
	  if(Config.m_RunningFlags.RETURN_HOME)
		 GuiRunningFlags.RFlags2 |= (1<<19);
	  if(Config.m_RunningFlags.AVOID_CONTROL_OVERIDE)
		 GuiRunningFlags.RFlags2 |= (1<<20);
	  if(Config.m_RunningFlags.GPS_ONLINE)
		 GuiRunningFlags.RFlags2 |= (1<<21);
	  if(Config.m_RunningFlags.NAV_HOME_COORD_SET)
		 GuiRunningFlags.RFlags2 |= (1<<22);
	  if(Config.m_RunningFlags.NAV_RETURN_TO_HOME)
		 GuiRunningFlags.RFlags2 |= (1<<23);
	  if(Config.m_RunningFlags.MISSION_DATA_LOADED)
		 GuiRunningFlags.RFlags2 |= (1<<24);
	  if(Config.m_RunningFlags.WP_ERROR)
		 GuiRunningFlags.RFlags2 |= (1<<25);
	  if(Config.m_RunningFlags.MISSION_COMPLETED)
		 GuiRunningFlags.RFlags2 |= (1<<26);
	  if(Config.m_RunningFlags.INIT_DOCKING)
		 GuiRunningFlags.RFlags2 |= (1<<27);
	  if(Config.m_RunningFlags.NAV_PROCESSOR_RUNNING)
		 GuiRunningFlags.RFlags2 |= (1<<28);
	  if(Config.m_RunningFlags.GEO_FENCE_BREACH)
		 GuiRunningFlags.RFlags2 |= (1<<29);
	  if(Config.m_RunningFlags.DEBUG_1)
		  GuiRunningFlags.RFlags2 |= (1<<30);
	  if(Config.m_RunningFlags.DEBUG_2)
		GuiRunningFlags.RFlags2 |= (1<<31);

}

//=======================================================================================================================================================
bool CGuiFunctions::SerialReadByte(unsigned char *Byte)
{
	return(m_Uart->Read( Byte));
}

bool CGuiFunctions::SerialReadInt(int16_t &Data)
{
	unsigned char Byte1,Byte2;
	if(SerialReadByte(&Byte1))
	{
		if(SerialReadByte(&Byte2))
		{
			Data = (int16_t)(Byte2<<8)+Byte1;
			return(true);
		}
	}
	return(false);
}


bool CGuiFunctions::SerialReadLong(int32_t &Data)
{
	int16_t Tmp1,Tmp2;

	if(SerialReadInt(Tmp1))
	{
		if(SerialReadInt(Tmp2))
		{
			Data = (int32_t)(Tmp1<<8)+Tmp2;
			return(true);
		}
	}
	return(false);
}

void CGuiFunctions::WriteByteToOutputFrame(const unsigned char a)
{
	m_Uart->WriteTxBuffer((uint8_t*)&a,1,false);
	m_Checksum ^= a;
}

void CGuiFunctions::WriteSerialFrameHeader(unsigned char error, unsigned char FrameLength)
{
	WriteByteToOutputFrame('$');
	WriteByteToOutputFrame('M');
	WriteByteToOutputFrame(error ? '!' : '>');
	m_Checksum = 0;											// start calculating a new checksum
	WriteByteToOutputFrame(FrameLength);
	WriteByteToOutputFrame(m_RecievedCommand);
//	printf("R%u,",m_RecievedCommand);
}

void CGuiFunctions::Acknowledge()
{
	WriteFrameHeaderNoError(0);
	SendSerialOutputFrame();
}
void CGuiFunctions::WriteFrameHeaderNoError(unsigned char FrameLength)
{
	WriteSerialFrameHeader(0, FrameLength);
}


void inline CGuiFunctions::headSerialError(unsigned char s)
{
	WriteSerialFrameHeader(1, s);
}

void CGuiFunctions::SendSerialOutputFrame()
{
	m_Uart->WriteTxBuffer((uint8_t*)&m_Checksum,1,true);
//	WriteByteToOutputFrame(m_Checksum);						// Add Checksum to end of frame

}

void CGuiFunctions::WriteNamesToOutputFrame(const char * s)
{
	unsigned char Len = strlen(s);
	unsigned char i;
	WriteFrameHeaderNoError(Len);
	for (i = 0; i<Len; i++)
	{
		WriteByteToOutputFrame(*(s+i));
	}
	SendSerialOutputFrame();
}

void CGuiFunctions::SerialCom()
{
	unsigned char InChar;
	static uint32_t LastTime =0;
	static unsigned char offset;
	static unsigned char dataSize;
	static enum _serial_state{ IDLE,HEADER_M,HEADER_ARROW,HEADER_SIZE,HEADER_CMD,GET_DATA} SerialRecieveState = IDLE;	// = IDLE;
	
	if((Core.millis() - LastTime) >8000 && SerialRecieveState != IDLE)
	{
		SerialRecieveState = IDLE;
	}

	if (m_Uart->TxAvailable()  < m_Uart->TxBufferSize() - 80 )
	{
		return;											// ensure there is enough free TX buffer to go further (50 bytes margin)
	}
	while (m_Uart->Read(&InChar))
	{
		LastTime = Core.millis();
	//	printf("%d",InChar);
		switch(SerialRecieveState)
		{
		case IDLE:
			if(InChar=='$')
				SerialRecieveState = HEADER_M;
		break;
		case HEADER_M:
			SerialRecieveState = (InChar=='M') ? HEADER_ARROW : IDLE;
		break;
		case HEADER_ARROW:
			SerialRecieveState = (InChar=='<') ? HEADER_SIZE : IDLE;
		break;
		case HEADER_SIZE:
			if (InChar > INBUF_SIZE)				// now we are expecting the payload size
			{
				SerialRecieveState = IDLE;
				continue;
			}
			dataSize = InChar;
			offset = 0;
			m_Checksum = 0;
			indRX = 0;
			m_Checksum ^= InChar;
			SerialRecieveState = HEADER_CMD;		// the command is to follow
			break;
		case HEADER_CMD:
			m_RecievedCommand = InChar;
			m_Checksum ^= InChar;
			SerialRecieveState = GET_DATA;
			break;
		case GET_DATA:
			if(offset < dataSize)
			{
				m_Checksum ^= InChar;
				ReceiveDataBuffer[offset++] = InChar;
			}
			else
			{

				if (m_Checksum == InChar)			// compare calculated and transferred checksum
				{
					m_ReceivedDataSize = dataSize; 
				//	printf("\n");
					EvaluateCommand();				// we got a valid packet, evaluate it
			 		

				}
				m_ReceivedDataSize = 0; 
				SerialRecieveState = IDLE;
				return;
			}
			break;
		}
		
	}
	
}


void  CGuiFunctions::SendOutputFrame(unsigned char *pOutData,unsigned char Size)
{
	WriteToOutputFrame(pOutData,Size);
	SendSerialOutputFrame();	
}



void  CGuiFunctions::WriteToOutputFrame(unsigned char *pOutData,unsigned char Size)
{
	unsigned char i;
	WriteFrameHeaderNoError(Size);
	for(i=0;i<Size;i++) 
	{
		WriteByteToOutputFrame(*pOutData++);
	}
}

void CGuiFunctions::RecieveData(unsigned char *pInData,unsigned char Size)
{
	unsigned char i =0;
//	WriteFrameHeaderNoError(0);
	if(Size >64)
		return;
	if(Size>m_ReceivedDataSize)
		Size = m_ReceivedDataSize;
	while(Size--)
	{
		*pInData++ = ReceiveDataBuffer[i++];
	}
}



// void CGuiFunctions::Serialize8(uint8_t a) 
// {
// 	m_Uart->WriteTxBuffer_0(a);
// 	m_Checksum ^= a;
// }
// 
// void CGuiFunctions::Serialize16(int16_t a) 
// {
//   Serialize8((a   ) & 0xFF);
//   Serialize8((a>>8) & 0xFF);
// }
// 
// void CGuiFunctions::Serialize32(uint32_t a) 
// {
//   Serialize8((a    ) & 0xFF);
//   Serialize8((a>> 8) & 0xFF);
//   Serialize8((a>>16) & 0xFF);
//   Serialize8((a>>24) & 0xFF);
// }







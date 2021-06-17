/* 
* CGuiFunctions.h
*
* Created: 01/08/2017 14:04:58
* Author: phil
*/


#ifndef __CGUIFUNCTIONS_H__
#define __CGUIFUNCTIONS_H__

#define INBUF_SIZE			200
#define VERSION				250
#define GUIP_VERSION		231
#define  NAVIGATION_VERSION	21     




#define SEN_MPU_6050	1 
#define SEN_HMC_5883L	2
#define SEN_MS_5611		4
#define SEN_UBLOX_M8	8	
#define SEN_SONAR		16


//================================ Adapted MultiWii Serial Protocol to use MultiWii GUI ================================================
#define GUIP_IDENT									100   //out message         
#define GUIP_STATUS								101   //out message         cycletime & errors_count & sensor present & box activation & current setting number
#define GUIP_RAW_IMU							102   //out message         9 DOF
//#define GUIP_SERVO					103   //out message         8 servos   (not used)
#define GUIP_MOTOR								104   //out message         8 motors
#define GUIP_RC										105   //out message         8 rc chan and more
#define GUIP_RAW_GPS							106   //out message         fix, numsat, lat, lon, alt, speed, ground course (not used)
#define GUIP_COMP_GPS							107   //out message         distance home, direction home	(not used)
#define GUIP_ATTITUDE							108   //out message         2 angles 1 heading
#define GUIP_ALTITUDE							109   //out message         altitude
#define GUIP_ANALOG								110   //out message         vbat, power meter sum, rssi if available on RX
#define GUIP_RC_TUNING							111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define GUIP_PID										112   //out message         P I D coeff (9 are used currently)
#define GUI_PID_RT_VALUES					113	  //out message         Error from PID on navigation
#define GUIP_MISC									114   //out message         power meter trig
#define GUIP_AUX_SWITCH_NAMES			115   //out message         Output from PID on navigation
#define GUIP_STICK_NAMES						116   //out message         Internal Flag  names
#define GUIP_SERVO_NAMES					117   //out message			the indicator names
#define GUIP_SWITCH_NAMES					118   //out message		    Rear Switch function names
#define GUIP_WP										119   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold (not used)
#define GUIP_SERVO_CONF						120   //out message         Servo settings 
#define GUIP_NAV_STATUS						121   //out message         Returns navigation status
#define GUIP_NAV_CONFIG						122   //out message         Returns navigation parameters
#define GUIP_RADIO_TRIMS						123   //out message         Radio Trims
#define GUIP_RADAR								124   //out message         Radar values
#define GUIP_LOG_FILE							140   //out message         Upload log file
#define GUIP_RC_AUX								141   //out message         Aux switches setup (number is Dependant of your setup)
#define GUIP_SWITCHES							142   //out message         Rear Switches settings
#define GUIP_TASK_TIMINGS					143	  //out message
#define GUIP_ALARMS								144   //out message
#define GUIP_WHEEL_COUNTERS				145   //out message
#define GUIP_DISTANCE							146   //out message				
#define GUIP_TASK_NAMES						147	  //out message
#define GUIP_ALARM_NAMES					148	  //out message
#define GUIP_SENSOR_NAMES					149   //out message	
#define GUIP_RC_STICKS							150   //out message 
#define GUIP_PID_NAMES							151	  //out message
#define GUIP_FUNCTION_FLAGS_NAMES	152	  //out message
#define GUIP_FUNCTION_FLAGS				153   //out message
#define GUIP_LIMITS								154   //out message
#define GUIP_CALIBRATE_STICKS_OK		155   //out message
#define GUIP_RSSI									156	  //out message	
#define GUIP_MOTOR_TRIM						157	  //out message
#define GUIP_RUNNING_FLAGS					158   //out message
#define GUIP_CLI										159   //out message


#define GUIP_SET_RAW_GPS					201   //in message          fix, numsat, lat, lon, alt, speed
#define GUIP_SET_PID							202   //in message          P I D coeff (9 are used currently)
#define GUIP_SET_RC_STICKS				203   //in message          Sticks setup
#define GUIP_SET_RC_TUNING				204   //in message          rc rate, rc expo, roll pitch rate, yaw rate, dyn throttle PID
#define GUIP_ACC_CALIBRATION			205   //in message          command no param
#define GUIP_MAG_CALIBRATION			206   //in message          command no param
#define GUIP_SET_MISC						207   //in message
#define GUIP_RESET_CONF					208   //in message          no param
#define GUIP_SET_WP							209   //in message          sets a given WP (WP#,lat, lon, alt, flags)
#define GUIP_SET_HEAD						211   //in message          define a new heading hold direction
#define GUIP_SET_SERVO_CONF			212	  //in message
#define GUIP_SET_RADIO_TRIMS			213	  //in message
#define GUIP_SET_RC_AUX					214   //in message 
#define GUIP_SET_NAV_CONFIG			215   //in message          Sets nav config parameters 
#define GUIP_SET_SWITCHES				216   //in message
#define GUIP_SET_MOTOR_SPEED			217
#define GUIP_SET_FUNCTION_FLAGS	218   //in message
#define GUIP_SET_DEBUG_FLAGS			219   //in message
#define GUIP_SET_LIMITS						220   //in message
#define GUIP_SET_START						221	  //in message          
#define GUIP_SET_STOP						222   //in message          
#define GUIP_CALIBRATE_STICKS			223	  //in message 	
#define GUIP_CALIBRATE_STICKS_SAVE	224	  //in message
#define GUIP_DELETE_LOG_FILE			225	  //in message	
#define GUIP_SET_MOTOR_TRIM			226	  //in message	
#define GUIP_GUI_START						227   //in message          no param
#define GUIP_GUI_STOP						228   //in message          no param
#define GUIP_GUI_RUNNING_FLAGS		229   //in message          no param

#define GUIP_EEPROM_WRITE				250   //in message          no param

#define WP_FLAG_OK   0x00				//WP OK
#define WP_FLAG_LOAD_ERROR   0xFE		//No WP Loaded or WP error
#define MISSION_FLAG_NAV_IN_PROG 0xff   //Navigation is in progress, returned wp is home

#define GUI_PID_OUTPUT_OUTPUT	0
#define GUI_PID_OUTPUT_ERROR		1
#define GUI_PID_OUTPUT_INPUT		2

//     WP_Query
// 	    public const int OK = 0;        //ok WP returned
//         public const int Timeout = 1;   //No answer
//         public const int Error = 2;     //Error condition (NAV is active)
//         public const int Boundary = 3;  //Invalid WP number
//         public const int CRC = 3;


enum navstate // not used just for GUI
{
	NAV_STATE_NONE = 0,
	NAV_STATE_RTH_START,
	NAV_STATE_RTH_ENROUTE,
	NAV_STATE_HOLD_INFINIT,
	NAV_STATE_HOLD_TIMED,
	NAV_STATE_WP_ENROUTE,
	NAV_STATE_PROCESS_NEXT,
	NAV_STATE_DO_JUMP,
	NAV_STATE_LAND_START,
	NAV_STATE_LAND_IN_PROGRESS,
	NAV_STATE_LANDED,
	NAV_STATE_LAND_SETTLE,
	NAV_STATE_LAND_START_DESCENT
};




typedef struct  __attribute__ ((packed))
{
	uint16_t GPS_Hdop;			// Hdop
	uint8_t NAV_state;		// NAV_STATE_NONE;  /// State of the nav engine
	uint8_t action;				//Action to follow
	uint8_t number;				//Waypoint number
	uint8_t NAV_error;		// NAV_ERROR_NONE;
	int16_t target_bearing;		// This is the angle from the boat to the "next_WP" location in degrees * 100
}GpsMission_t;


typedef struct __attribute__ ((packed))
{
	uint16_t		PowerTrigger;
	uint16_t		MinThrottleValue;
	uint16_t		MaxThrottleValue;
	uint16_t		FailsafeThrottleValue;
	uint16_t		MaxMotorPwm;
	int16_t			MagDeclination;
	uint16_t		Batt_1_Scale;
	uint16_t		Batt_2_Scale;
	unsigned char	Bat_1_LevelWarning;
	unsigned char	Bat_2_LevelWarning;
	unsigned char	Battery_1_Critical;
	unsigned char	Battery_2_Critical;
} misc_t;

typedef struct __attribute__ ((packed))
{
	uint32_t		RFlags1;
	uint32_t		RFlags2;
} GuiRunningFlags_t;




typedef struct  __attribute__ ((packed))
{
	uint16_t cycleTime,I2CErrorsCount,sensor;
	uint32_t flag;
	unsigned char set;
} status_t;

typedef struct  __attribute__ ((packed))
{
	uint16_t Switch_1;
	uint16_t Switch_2;
	uint16_t Switch_3;
	uint16_t Switch_4;
} Switches_t;

typedef struct __attribute__ ((packed))
{
	unsigned char Version;
	unsigned char t;
	unsigned char GuiVersion;
	uint32_t NavigationVersion;
} Ident_t;

typedef struct __attribute__ ((packed))
{
	unsigned char CurrentUserSetting;
	int16_t accZero[3];
	int16_t magZero[3];
	uint16_t GiroZero[3];
	uint16_t FlashChksum;
	unsigned char checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
} SensorOffsets_t;

typedef struct __attribute__ ((packed))
{
	int16_t  AccData[3];
	int16_t  gyroData[3];
	int16_t  MagData[3];
} GuiImu_t;


typedef struct __attribute__ ((packed))
{
	int16_t Angle[2];            // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
	int16_t Heading;
	int16_t RelativeBearing;
} GuiAttitude_t;




typedef struct __attribute__ ((packed))
{
	int32_t  EstAlt;             // in cm
	int16_t  BoatSpeed;              // variometer in cm/s
} GuiAlt_t;

// typedef struct __attribute__ ((packed))
// {
// 	unsigned char P8;
// 	unsigned char I8;
// 	unsigned char D8;
// }pid_;


typedef struct __attribute__ ((packed))
{
	unsigned char  Battery_1_Volts;               // battery voltage in 0.1V steps
	unsigned char  Battery_2_Volts;               // battery voltage in 0.1V steps
	uint16_t intPowerMeterSum;
	uint16_t rssi;              // range: [0;1023]
	uint16_t amperage;
} AnalogTemp_t;

typedef struct __attribute__ ((packed))
{
  uint8_t  number;     //Waypoint number
  uint8_t  action;     //Action to follow
  int32_t  GpsCordsLat;     //GPS position 
  int32_t  GpsCordsLon;     //GPS position 
  uint32_t altitude;   //Altitude in cm (AGL)
  int16_t  parameter1; //Parameter for the action
  int16_t  parameter2; //Parameter for the action
 // int16_t  parameter3; //Parameter for the action
  uint8_t  flag;       //flags the last wp and other fancy things that are not yet defined
  uint8_t  checksum;   //this must be at the last position
} GuiWaypoint_struct_t;

typedef union
{
	float Float;
	int32_t Int32;	
}Convert_t;

typedef struct __attribute__ ((packed))
{
	unsigned char RcRate;
	unsigned char RcExpo;
	unsigned char RollPitchRate;
	unsigned char YawRate;
	unsigned char DynThrPID;
	
}RcTuning_t;


typedef struct __attribute__ ((packed))
{
	unsigned char RcRate;
	unsigned char RuddeExpo;
	unsigned char rollPitchRate;
	unsigned char yawRate;
	unsigned char MaxPitchAngle;
	unsigned char Unused1;
	unsigned char Unused2;
	
}RcRate_t;

typedef struct __attribute__ ((packed))
{
	int16_t MotorSpeed_1;
	int16_t MotorSpeed_2;
	int16_t MotorSpeed_3;
	int16_t MotorSpeed_4;
	uint8_t MotorDirection_1; 
	uint8_t MotorDirection_2; 
	uint8_t MotorDirection_3; 
	uint8_t MotorDirection_4; 
	uint8_t RobotSpeed;
}GuiMotors_t;



#define POSHOLD_P              .11
#define POSHOLD_I              0.0
#define POSHOLD_IMAX           20        // degrees
#define POSHOLD_RATE_P         2.0
#define POSHOLD_RATE_I         0.08      // Wind control
#define POSHOLD_RATE_D         0.045     // try 2 or 3 for POSHOLD_RATE 1
#define POSHOLD_RATE_IMAX      20        // degrees

#define FILTERING				(1<<0)
#define LEAD_FILTER				(1<<1)
#define DONT_RESET_HOME_ARM		(1<<2)
#define NAV_CONTROLS_HEADING	(1<<3)		// copter faces toward the navigation point, maghold must be enabled for it
#define NAV_TAIL_FIRST			(1<<4)		// true - copter comes in with tail first
#define NAV_RTH_TAKEOFF_HEADING	(1<<5)
#define SLOW_NAV				(1<<6)
#define WAIT_FOR_RTH_ALT		(1<<7)		//Wait to reach RTH alt before start moving to home (0-no, 1-yes)
#define IGNORE_THROTTLE			(1<<0)		//Throttle stick input will be ignored  (only in BARO)
#define TAKEOVER_BARO			(1<<2)



typedef	struct  __attribute__ ((packed))
{
	uint16_t Flags; 
	uint16_t WpRadius;				// in cm
	uint16_t CruiseSpeed;			// in m/s 
	uint16_t SteerAccelMax;			// in device units
	uint16_t MaxTurnRate;			// in device units
	uint16_t WheelRadius;			// in device units
	uint16_t CruiseThrottle;		// in device units
	uint16_t TurnRadius;			// in device units
	uint16_t ReverseSpeed;			// in device units
	uint16_t MaxNavRoll;			// degree * 100; (3000 default)
	uint16_t MaxNavPitch;			// in degrees
	uint16_t CrosstrackGain;		// * 100 (0-2.56)
	uint16_t FenceRadius;			// fence control in meters
	unsigned char MaxNumberOfWayPoints;
} GuiNavigationConfig_t;

typedef	struct  __attribute__ ((packed))
{
	int16_t MaximumPitchAngle;
	int16_t MaximumRollAngle;
	int16_t MinFrontDistance;
	int16_t MinRearDistance;
	int16_t MinSideDistance;
	int16_t MaxThrottleAcceleration;
	uint16_t MaximumSpeed;

}Limits_t;

typedef	struct  __attribute__ ((packed))
{
	int16_t FrontRightMotorTrim;
	int16_t FrontLeftMotorTrim;
	int16_t RearRightMotorTrim;
	uint16_t RearLeftMotorTrim;
}MotorTrim_t;



typedef	struct  __attribute__ ((packed))
{
	uint8_t GuiGpsFixType;
	uint8_t NumSats;
	int32_t PresentLatitude;
	int32_t PresentLongitude;
	int16_t Altitude;
	uint16_t GroundSpeed;

}RawGpsValues_t;


typedef struct __attribute__ ((packed))
{
	uint16_t FrontRightMotor;
	uint16_t FrontLeftMotor;              // range: [0;1023]
	uint16_t RearRightMotor;
	uint16_t RearLeftMotor;              // range: [0;1023]
	uint8_t Direction;
} UpdateMotors_t;

typedef struct __attribute__ ((packed))
{
	uint16_t FrontRightMotor;
	uint16_t FrontLeftMotor;              // range: [0;1023]
	uint16_t RearRightMotor;
	uint16_t RearLeftMotor;              // range: [0;1023]
	bool Direction;
} UpdatTestMotors_t;

typedef struct __attribute__ ((packed))
{
	uint16_t Front;
	uint16_t Rear;              // range: [0;1023]
	uint16_t Right;
	uint16_t Left;              // range: [0;1023]
} UpdateDistance_t;

typedef struct __attribute__ ((packed))
{
	uint16_t gHEAD_40;
	uint16_t gHEAD_30;
	uint16_t gHEAD_20;
	uint16_t gHEAD_10;
	uint16_t gHEAD_CENTER;
	uint16_t gHEAD_NEG_10;
	uint16_t gHEAD_NEG_20;
	uint16_t gHEAD_NEG_30;
	uint16_t gHEAD_NEG_40;
} UpdateRadar_t;
// 


typedef struct __attribute__ ((packed))
{
	uint16_t RightWheel;
	uint16_t LeftWheel;              // range: [0;1023]
} UpdateWheelSensors_t;

typedef struct __attribute__ ((packed))
{
	int16_t PidOutput;
	int16_t PidError; 
	int16_t PidInput;             // range: [0;1023]
} PidRuningValues_t;



typedef struct __attribute__ ((packed))
{
	uint8_t		TaskId;
	uint8_t		TaskPriority;
	uint32_t	DesiredPeriod;
	uint32_t	ActualPeriod;
	uint32_t	maxExecutionTime;
	uint32_t	averageExecutionTime;
	uint16_t	SystemLoad;
	uint16_t	QueSize;
} TaskTimings_t;


typedef struct __attribute__ ((packed))
{
	uint16_t P;
	uint16_t I;
	uint16_t D;
}GuiPid_t;

typedef struct __attribute__ ((packed))
{
	GuiPid_t Pid[NO_OF_PIDS];
}GuiPidValues_t;

class CGuiFunctions
{
//variables
public:
	unsigned char m_Checksum;
	unsigned char indRX;
	unsigned char m_RecievedCommand;
	uint16_t m_ReceivedDataSize;
	uint16_t intPowerTrigger1;
	uint32_t capability;
	misc_t misc;
	unsigned char ReceiveDataBuffer[INBUF_SIZE];
	int16_t  magHold;
//	GuiPidValues_t    NavigationPid;
	GuiPidValues_t m_PidValues;
//	CSoftwareSerial* m_Uart;
	CUart* m_Uart;
	uint8_t m_CalibrateSticks;
	UpdatTestMotors_t m_UpdatTestMotors;
protected:
private:
	PidRuningValues_t m_PidRtValues;
//functions
public:
	CGuiFunctions(CUart* Uart);
//	CGuiFunctions(CSoftwareSerial* Uart);
	~CGuiFunctions();
	void TestTransmit(void);
	void EvaluateCommand();
	void Printf(const char *fmt, ... );
	void SetPidOutputValues(float Value, uint8_t Type);
	bool SerialReadByte(unsigned char *Byte);
	bool SerialReadInt(int16_t &Data);
	bool SerialReadLong(int32_t &Data);
	void WriteByteToOutputFrame(const unsigned char a);
	void WriteIntToOutputFrame(int16_t a);
	void WriteLongToOutputFrame(uint32_t a);
	void WriteSerialFrameHeader(unsigned char error, unsigned char FrameLength);
	void Acknowledge();
	void WriteFrameHeaderNoError(unsigned char FrameLength);
	void inline headSerialError(unsigned char s);
	void SendSerialOutputFrame();
	void WriteNamesToOutputFrame(const char * s);
	void SerialCom();
	void SendOutputFrame(unsigned char *pOutData,unsigned char Size);
	void WriteToOutputFrame(unsigned char *pOutData,unsigned char Size);
	void RecieveData(unsigned char *pInData,unsigned char Size);
	void Serialize8(uint8_t a);
	void Serialize16(int16_t a);
	void Serialize32(uint32_t a);
	uint32_t GetAlarmStatus();
	void GetRunningFlags(GuiRunningFlags_t &GuiRunningFlags);
protected:
private:
	CGuiFunctions( const CGuiFunctions &c );
	CGuiFunctions& operator=( const CGuiFunctions &c );

}; //CGuiFunctions

#endif //__CGUIFUNCTIONS_H__

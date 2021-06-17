	/*
	* CConfig.h
	*
	* Created: 04/06/2016 10:35:30
	* Author: phil
	*/
	#ifndef __CCONFIG_H__
	#define __CCONFIG_H__

	#include "cmymath.h"
	//#include "NavigationStructures.h"
	//#include "CNavigationFunctions.h"
	#include "ci2c.h"




	#define MAIN_CONFIG_ADDRESS	0x00
	#define NAVIGATION_ADDRESS	0x1000
	#define RC_CONFIG_ADDRESS	0x2000
	#define FUNCTION_ADDRESS	0x3000
	#define WAYPOINT_ADDRESS	0x4000



	#define ARM_NO_FUNCTION	0
	#define ARM				1
	#define DIS_ARM			2

	#define ON 1
	#define OFF 0

	#define MAX_WAYPOINTS			12


	#define BATTERY_CAPACITY	5000
	//================================= Set Receiver Type ==============================

	//#define RADIOL_PWM
	#define RADIOL_SBUS
	//#define RADIO_PPM
	//==================================================================================================

	//============================ RC  SETTINGS ============================================================
	#define VTX_CHANNEL							8
	#define CAMERA_CHANNEL					9
	#define RADIO_TRIM_UPPER_LIMIT		2000
	#define RADIO_TRIM_MID						1500
	#define RADIO_TRIM_LOWER_LIMIT     1000
		
	
	//==================================================================================================



	//================================= Navigation Debug flags set by GUI ==============================

	#define SET_NAVIGATION_DEBUG_MODE	0x01
	#define DEBUG_START_NAV				0x02
	#define DEBUG_USE_SLOW_NAV			0x04


	//================================= Navigation flags set by GUI ==============================
	#define NAV_FLAG_GPS_FILTER			0x1     // GPS coordinates are filtered via a 5 element moving average filter
	#define NAV_FLAG_GPS_LEAD_FILTER	0x2     // Forward prediction filter
	#define NAV_FLAG_ENABLE_RC_LOSS_RTH	0x4     // If this bit set Boat will return to home if RC Loss
	#define NAV_FLAG_ENABLE_RUDDER_RATE	0x8    // If this bit set Rudder is controlled by a Rate slope
	#define NAV_FLAG_ENABLE_WIND_SPEED	0x10    // If this bit set Wind speed is measured
	#define NAV_FLAG_ENABLE_SONAR		0x20    // If this bit set Distance is measured by Sonar
	#define NAV_FLAG_ENABLE_SLOW_NAV	0x80




	//=========================== Battery volts ===============================================================================
	#define VBAT_SMOOTH		16  //  len of averaging vector for smoothing the VBAT readings; should be power of 2
	#define BATTERY_1_WARNING_V	69	// volts * 10
	#define BATTERY_2_WARNING_V	69
	#define BATTERY_1_CRITICAL	65 //critical condition: if vbat ever goes below this value, permanent alarm is triggered
	#define BATTERY_2_CRITICAL	65 //critical condition: if vbat ever goes below this value, permanent alarm is triggered
	#define BATTERY_1_SCALE		190 // *100 used to allow for Potential divider On bat sensing eg 7.4 to 3.3 for controller battery
	#define BATTERY_2_SCALE		1025 // *100 used to allow for Potential divider On bat sensing eg 7.4 to 3.3 for motor battery
	//=========================================================================================================================

	#define MAXTHROTTLE 100
	#define MINTHROTTLE 000
	#define MIDTHROTTLE 50
	#define MINCOMMAND 1500

	//==================================STEERING ============================================
	#define ATTCONTROL_STEER_RATE_FF     0.20f
	#define ATTCONTROL_STEER_RATE_P      0.20f
	#define ATTCONTROL_STEER_RATE_I      0.20f
	#define ATTCONTROL_STEER_RATE_IMAX   1.00f
	#define ATTCONTROL_STEER_RATE_D      0.00f
	#define ATTCONTROL_STEER_RATE_FILT   10.00f
	#define ATTCONTROL_STEER_RATE_MAX    360.0f
	#define ATTCONTROL_DT			 0.02f	
	#define ATTCONTROL_THR_ACCEL_MAX		3.0f  // throttle/speed control maximum acceleration/deceleration (in m/s) (_ACCEL_MAX parameter default)
	//=======================================================================================
	
	//==================================Throttle=============================================
	#define ATT_CONTROL_THR_SPEED_P       0.20f
	#define ATT_CONTROL_THR_SPEED_I       0.20f
	#define ATT_CONTROL_THR_SPEED_IMAX    1.00f
	#define ATT_CONTROL_THR_SPEED_D       0.00f
	#define ATT_CONTROL_THR_SPEED_FILT    10.00f
	
	#define MAX_SPEED									0.7     // m/s
	#define THROTTLE_SAFE							0
	#define STOP_UPPER_LIMIT						1
	#define STOP_LOWER_LIMIT						0
	#define MANOEUVER_SPEED_VALUE			40
	#define REVERSE_SPEED_VALUE				25
	#define CRUISE_SPEED								2		 // Navigation cruise speed in m/s
	#define CRUISE_THROTTLE						45		 // Navigation cruise	Throttle 0-100 %
	#define WHEEL_RADIUS							18.2	 // Wheel radius in mm
	#define STEER_ACCEL_MAX						1.0	 // Navigation speed in %
	#define MAX_TURN_RATE							0.6	 // The maximum turning acceleration (in units of gravities)
	#define MAX_TURN_RADIUS						0.2	 // Turn radius of vehicle in meters while at low speeds.  Lower values produce tighter turns in auto
	#define ATTCONTROL_STOP_SPEED_DEFAULT    0.1f		// speed (in m/s) at or below which vehicle is considered stopped (_STOP_SPEED parameter default)
	#define ATT_CONTROL_STEER_RATE_MAX		360.0f
	#define CRUISE_SPEED    2  // in m/s
	//=======================================================================================

	
	//================================== FRONT DISTANCE =====================================
	#define FRONT_DISTANCE_THRESHOLD		180		// distance in mm
	#define AI_SPIN_CLEARANCE_VALUE			90		// distance in mm
	#define RC_CLEARENCE_VALUE				200		// distance in mm
	#define HEAD_MIN_DISTANCE				0		// distance in mm
	#define HEAD_MAX_DISTANCE				350		// distance in mm


	//================================== Lidar DISTANCE =====================================
	#define FRONT_DISTANCE_ALARM_VALUE		180		// distance in mm
	#define REAR_DISTANCE_ALARM_VALUE		180		// distance in mm
	#define SIDE_DISTANCE_ALARM_VALUE		180		// distance in mm


	//================================== REAR DISTANCE =====================================
	#define REAR_DISTANCE_THESHOLD		180		// distance in mm



	//===================== Servos ============================================================================================

	typedef enum{SERVO_F_R_MOTOR,SERVO_F_L_MOTOR,SERVO_R_R_MOTOR,SERVO_R_L_MOTOR,SERVO_HEAD_VERT,SERVO_HEAD_HORZ}Servo_t;
	#define	NO_OF_SERVOS					6
	//========================== Motor Servo ===================		Although not a servo requires same output
	#define F_R_MOTOR_SERVO_UPPER_LIMIT     5000
	#define F_R_MOTOR_SERVO_MID				1500
	#define F_R_MOTOR_SERVO_LOWER_LIMIT     0
	//===========================================================

	//========================== Motor Servo ===================		Although not a servo requires same output
	#define F_L_MOTOR_SERVO_UPPER_LIMIT     5000
	#define F_L_MOTOR_SERVO_MID				1500
	#define F_L_MOTOR_SERVO_LOWER_LIMIT     0
	//===========================================================

	//========================== Motor Servo ===================		Although not a servo requires same output
	#define R_R_MOTOR_SERVO_UPPER_LIMIT     5000
	#define R_R_MOTOR_SERVO_MID				1500
	#define R_R_MOTOR_SERVO_LOWER_LIMIT     0
	//===========================================================

	//========================== Motor Servo ===================		Although not a servo requires same output
	#define R_L_MOTOR_SERVO_UPPER_LIMIT     5000
	#define R_L_MOTOR_SERVO_MID				1500
	#define R_L_MOTOR_SERVO_LOWER_LIMIT     0
	//===========================================================


	//========================== Head Servo ===================
	#define HEAD_VERT_SERVO_UPPER_LIMIT     2000
	#define HEAD_VERT_SERVO_MID				1500
	#define HEAD_VERT_SERVO_LOWER_LIMIT		1000
	//===========================================================


	//========================== Head Servo ===================
	#define HEAD_HORZ_SERVO_UPPER_LIMIT     2000
	#define HEAD_HORZ_SERVO_MID				1500
	#define HEAD_HORZ_SERVO_LOWER_LIMIT     1000
	//===========================================================


	//========================== Radar Servo ====================
	#define RADAR_SERVO_UPPER_LIMIT     2000
	#define RADAR_SERVO_MID				1500
	#define RADAR_SERVO_LOWER_LIMIT     1000
	//===========================================================

	//=========================================================================================================================



	//====================================== PIDS ==============================================================================
	#define NO_OF_PIDS 4

	#define STEERING_RATE_PID		0
	#define STEER_ANGLE_PID_NO		1
	#define NAVIGATION_PID_NO		2
	#define THROTTLE_SPEED_PID		3
	#define HEAD_X__PID_NO			4
//	#define HEAD_Y_PID_NO			5

	//=========================================================================================================================


	//========================== Navigation Defaults ===========================================================================

	#define NAVIGATION_REVERSE_SPEED	10
	#define WAYPOINT_RADIUS				300.00	// Cm

	#define CROSSTRACK_GAIN				100          // Weighting the cross track error
	#define NAV_ROLL_MAX				20          // 20deg max banking when navigating
	#define NAV_PITCH_MAX				25          // 25deg max Pitch when navigating
	#define FENCE_DISTANCE				600
	#define SAFE_WAYPOINT_DISTANCE      50          // Distance to keep from any waypoint Cm



	//=========================================================================================================================



	//  Magnetic declination from here : http://magnetic-declination.com/
	//  Convert the degree+minutes into decimal degree by ==> degree+(minutes*(1/60))
	//  Note the sign on declination it could be negative or positive (WEST or EAST) */
	#define MAG_DECLINATION  -1.95f   //(**)   Birmingham England


	#define NO_OF_STICK_FUNCTIONS	7
	#define NO_OF_AUX_SWITCHES		2
	#define NO_OF_AUX_FUNCTIONS		10





	typedef struct __attribute__ ((packed))
	{
		int16_t min;        // minimum value, must be more than 1020 with the current implementation
		int16_t max;        // maximum value, must be less than 2000 with the current implementation
		int16_t middle;     // default should be 1500
		int8_t  Rate;       // range [0 - 100]  can be used to adjust a rate 0-100%
		int8_t  Reverse;    // direction
	}ServoConfig_t ;


	typedef struct __attribute__ ((packed))
	{
		int16_t min;        // minimum value, must be more than 1020 with the current implementation
		int16_t max;        // maximum value, must be less than 2000 with the current implementation
		int16_t middle;     // default should be 1500
		int8_t  Reverse;    // direction
	}RadioTrims_t ;



	typedef struct __attribute__ ((packed))
	{
		float  VoltsBattery_1;               // battery voltage
		uint16_t amperage;
	} Analog_t;

	typedef struct __attribute__ ((packed))
	{
		float PVal;
		float IVal;
		float DVal;
	}PidValues_t;

	typedef struct __attribute__((__packed__ ))
	{
		uint16_t Switch_1;
		uint16_t Switch_2;
		uint16_t Switch_3;
		uint16_t Switch_4;
	}SwitchValues_t;






	typedef struct __attribute__((__packed__ ))
	{
		int16_t AcelerometerOffsets[3];
		int16_t MagnetometerOffsets[3];
		int16_t GiroOffsets[3];
		uint16_t CrosstrackGain;
		uint16_t FenceDistance;
		uint16_t SafeWpDistance;
		uint16_t DataFileNo;
		uint16_t CourseFileNo;
		int16_t MagDeclination;
		uint16_t WaypointRadius;
		unsigned char MaxNumberOfWayPoints;
		uint16_t CruiseThrottle;
		float CruiseSpeed;			//Target cruise speed in auto modes Units: m/s Increment: 0.1
		float SteerAccelMax;
		float MaxTurnRate;       //The maximum turning acceleration (in units of gravities) that  can be handled while remaining stable. The nav will keep the lateral acceleration below this level to avoid rolling over or slipping the wheels in turns
		float WheelRadius;
		float TurnRadius;
		uint16_t ReverseSpeed; // Maximum speed that will be used to back away (in m/s)
		uint16_t MaxNavRoll;
		uint16_t MaxNavPitch;
		unsigned char NavigationFlags;
		unsigned char MaxPitchAngle;
		unsigned char MaxRollAngle;
		bool ReadWaypointsFromSDcard;
		bool ClearLog;					// not used
		float MaximumSpeed;	//Maximum speed vehicle can obtain at full throttle.If 0, it will be estimated based on CRUISE_SPEEDand CRUISE_THROTTLE
		float MaxSpeed;					// not used					
		float CruiseThrottle1;
		unsigned char checksum;			 // MUST BE ON LAST POSITION OF STRUCTURE !
	}  NavigationConfig_t;

	
	
	typedef struct  __attribute__((__packed__ ))
	{
		//	unsigned char powerTrigger1;
		int16_t FailsafeThrottleValue;
		uint16_t MinThrottleValue;
		uint16_t MaxThrottleValue;
		char Bat_1_LevelWarning;
		char Bat_2_LevelWarning;
		char Battery_1_Critical;
		char Battery_2_Critical;
		uint16_t Batt_1_Scale;
		uint16_t Batt_2_Scale;
		PidValues_t PidValues[NO_OF_PIDS];
		unsigned char RcRate;
		unsigned char RcExpo;
		ServoConfig_t ServoConf[NO_OF_SERVOS];
		RadioTrims_t RadioTrims[8];
		uint16_t SafeFrontDistance;
		uint16_t SafeRearDistance;
		uint16_t SafeSideDistance;
		float MaxThrottleAcceleration;
		uint16_t SticksSettings[NO_OF_STICK_FUNCTIONS];
		uint32_t AuxSwitchSettings[NO_OF_AUX_FUNCTIONS];
		SwitchValues_t SwitchSettings;
		uint8_t LastFileNumber;
		uint8_t WindSpeedScale;
		int16_t MotorTrims[4];
		float MaxMotorPwm;
	//float WheelRadius;
		unsigned char  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE
	}  MainConfig_t;


	typedef struct  __attribute__((__packed__ ))
	{
		uint16_t ChannnelsMax[18];
		uint16_t ChannnelsMin[18];
		uint16_t FailsafeValues[18];
		uint16_t DeadZone[18];
		uint16_t Trim[18];
		unsigned char  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE

	}RcChannels_t;

	typedef struct __attribute__((__packed__ ))
	{	//======== Startup and general=======
		unsigned char MAIN_INIT_COMPLETED	:1;
		unsigned char ALL_CONFIGS__LOADED	:1;
		unsigned char RSSI_LEVEL			:1;
//		unsigned char SD_CARD_PRESENT		:1;
//		unsigned char SD_CARD_INIALISED		:1;
		unsigned char AVOIDANCE_INHIBIT		:1;
		unsigned char EEPROM_PRESENT	    :1;
		//=========	Sensors =================
		unsigned char ACC_CALIBRATED		:1;
		unsigned char GYRO_CALIBRATED		:1;
		unsigned char MAG_CALIBRATED		:1;
		unsigned char CALIBRATE_MAG			:1;
		unsigned char HEAD_DISTANCE_ALARM   :1;
 		unsigned char FRONT_DISTANCE_ALARM	:1;
 		unsigned char RIGHT_DISTANCE_ALARM	:1;
 		unsigned char LEFT_DISTANCE_ALARM	:1;
 		unsigned char REAR_DISTANCE_ALARM	:1;

		unsigned char VTX_PRESENT			:1;
		unsigned char ENABLE_WHEEL_SENSORS	:1;


		
		//=========	Alarms =================
		unsigned char PANIC_MODE_TRIGGERED	:1;
		unsigned char ARMED					:1;
		unsigned char ANGLE_ALARM			:1;
		unsigned char LOST_RC_SIGNAL		:1;
		unsigned char FAILSAFE_ACTIVE	    :1;
		unsigned char LOGGING_FAIL			:1;
		unsigned char NAVIGATION_FAIL		:1;
		unsigned char RD_MAIN_CONFIG_FAIL	:1;
		unsigned char RD_NAVIGATION_CFG_FAIL:1;
		unsigned char LOAD_WAYPOINTS_FAIL	:1;
		unsigned char RD_FUNCTION_CONFIG_FAIL :1;
		unsigned char IMU_FAIL				:1;
		unsigned char COMPASS_FAIL			:1;
		unsigned char HEAD_LIDAR_FAIL		:1;
		unsigned char FRONT_LIDAR_FAIL		:1;
		unsigned char RIGHT_LIDAR_FAIL		:1;
		unsigned char LEFT_LIDAR_FAIL		:1;
		unsigned char REAR_LIDAR_FAIL		:1;
		unsigned char I2C_INTERFACE_FAIL  :1;
		
		//========= Radio Control ==================
		unsigned char DIRECTION_FORWARD		:1;
		unsigned char RD_RC_CONFIG_FAIL		:1;
		unsigned char TELEMETRY_ACTIVE	    :1;
		unsigned char RC_HEAD_CONTROL	    :1;

		//========= MOTORS ==================
		unsigned char MOTOR_FAIL			:1;
		unsigned char IS_MECHAN_CONTROL		:1;
		unsigned char IN_REVERSE		:1;
		unsigned char RATE_CONTROLED_THROTTLE :1;
		
		//========== Battery ===============
		unsigned char BATTERY_1_WARNING		:1;
		unsigned char BATTERY_2_WARNING		:1;
		unsigned char BATTERY_1_ALARM_CRITICAL	:1;
		unsigned char BATTERY_2_ALARM_CRITICAL	:1;
		unsigned char BATTERY_FAILSAFE		:1;
		
		
		//========== Displays ===============
		unsigned char DISABLE_DEBUG_DISPLAY	:1;
		unsigned char ENABLE_GUI_DISPLAY :1;
		unsigned char ENABLE_GUI_CLI_DISPLAY :1;
		
		//========== Logging ===============
		unsigned char LOGGING_IN_PROGRESS	:1;
		

		//========== Navigation ============
		unsigned char HOME_LOCATION_SET		:1;
		unsigned char AT_HOME_COORDINATES	:1;
		unsigned char CONTROL_BY_AUTO_PILOT :1;
		unsigned char STICK_MIXING			:1;
		unsigned char RETURN_HOME			:1;
		unsigned char AVOID_CONTROL_OVERIDE	:1;
		unsigned char AVOIDANCE_ACTIVE		:1;
		unsigned char GPS_ONLINE			:1;
		unsigned char GPS_3D_LOCK			:1;
		unsigned char GPS_2D_LOCK			:1;
		unsigned char NAV_HOME_COORD_SET	:1;
		unsigned char NAV_RETURN_TO_HOME	:1;
		unsigned char MISSION_DATA_LOADED	:1;
		unsigned char WP_ERROR				:1;
		unsigned char MISSION_COMPLETED		:1;
		unsigned char INIT_DOCKING			:1;
		unsigned char NAV_PROCESSOR_RUNNING	:1;
		unsigned char GEO_FENCE_BREACH		:1;
		unsigned char FENCE_ENABLE			:1;
		
		//========== Debug ============
		unsigned char DEBUG_1				:1;
		unsigned char DEBUG_2				:1;
	} RunningFlags_t;


	typedef struct  __attribute__((__packed__ ))
	{
		unsigned char FunctionFlag_1	:1;
		unsigned char FunctionFlag_2	:1;
		unsigned char FunctionFlag_3	:1;
		unsigned char FunctionFlag_4	:1;
		unsigned char FunctionFlag_5	:1;
		unsigned char FunctionFlag_6	:1;
		unsigned char PassThroughFlag	:1;
		unsigned char MotorReverseFlag	:1;
		unsigned char  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE

	}FunctionFlags_t;

	typedef struct __attribute__ ((packed))
	{
		//	GPS_COORDINATES Coordinates;    //GPS coordinate of the way point
		int32_t  nLatitide;
		int32_t  nLongitude;
		uint8_t  Number;		// Waypoint number
		uint8_t  Action;		// Action to follow
		int16_t  Parameter1;	// Parameter for the wp action
		int16_t  Parameter2;	// Parameter for the wp action
		int16_t  Parameter3;	// Parameter for the wp action
		uint8_t  Flag;			// flags the last wp
		uint32_t altitude;		// Altitude in cm (AGL) not used for boat but needed for gui
		unsigned char ChkSum;	// must be the last byte
	} Waypoint_t;


	#define WP_FLAG_END				0xA5   //Flags that this is the last step
	#define MISSION_FLAG_HOME       0x01   //Returned WP is the home position
	#define MISSION_FLAG_HOLD       0x02   //Returned WP is the hold position
	#define MISSION_FLAG_DO_LAND    0x20   //Land when reached desired point (used in RTH)



	class CConfig
	{
		//variables
		public:
		NavigationConfig_t m_NavigationConfig;
		MainConfig_t m_MainConfig;
		RcChannels_t m_RcChannels;
		RunningFlags_t m_RunningFlags;
		int16_t m_lookupRudderExpo[11];					// lookup table for rudder expo &  rate
		FunctionFlags_t m_FunctionFlags;
		uint16_t m_RealTimeGuiFlags;
		//	OsdConfig_t OsdConfig;
		Waypoint_t m_CourseWaypoints[MAX_WAYPOINTS];
		unsigned char m_LastWaypoint;

		protected:
		private:
		//functions
		public:
		CConfig();
		~CConfig();
		bool Init();
		void InitRunningFlags();
		bool ReadAllConfigs();
		void WriteAllConfigs();
		bool  WriteNavigationConfig();
		bool WriteMainConfig();
		bool WriteRcConfig();
		bool ReadRcConfig();
		bool WriteFunctionFlagsConfig();
		bool ReadFunctionFlagsConfig();
		void WriteWaypoints();
		bool ReadNavigationConfig();
		bool ReadMainConfig();
		bool LoadDefaultRcConfig(bool WriteToFlash);
		bool LoadDefaultNavigationConfig();
		void LoadDefaultConstants();
		bool LoadDefaultConfig(bool WriteToFlash);
		bool LoadDefaultFunctionFlags();
		unsigned char CalculateConfigChecksum(unsigned char *pBlock , unsigned int Size);
		bool WriteCourseWp();
		bool ReadCourseFile();
		void ClearWaypoints();
		bool ReadWaypoints();
		bool WritePage(unsigned char DeviceAddress, unsigned int EEpromAddress, char* Data, unsigned int DataLength);
		bool Read(unsigned char DeviceAddress, uint16_t EEAddress, char* Data, uint16_t DataLength);
		bool AcknowledgePolling();
		
		
		
		protected:
		private:
		CConfig( const CConfig &c );
		CConfig& operator=( const CConfig &c );

	}; //CConfig

	#endif //__CCONFIG_H__

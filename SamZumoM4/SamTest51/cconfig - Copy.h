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
#define VTX_CHANNEL		8
#define CAMERA_CHANNEL	9
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
#define BATTERY_1_WARNING	69	// volts * 10
#define BATTERY_2_WARNING	69
#define BATTERY_1_CRITICAL	65 //critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define BATTERY_2_CRITICAL	65 //critical condition: if vbat ever goes below this value, permanent alarm is triggered
#define BATTERY_1_SCALE		264 // *100 used to allow for Potential divider On bat sensing eg 7.4 to 3.3 for controller battery
#define BATTERY_2_SCALE		1025 // *100 used to allow for Potential divider On bat sensing eg 7.4 to 3.3 for motor battery
//=========================================================================================================================

#define MAXTHROTTLE 2000
#define MINTHROTTLE 1000
#define MINCOMMAND 1500

//================================== FRONT DISTANCE =====================================
#define FRONT_DISTANCE_THRESHOLD		180		// distance in mm
#define AI_SPIN_CLEARANCE_VALUE			90		// distance in mm
#define RC_CLEARENCE_VALUE				200		// distance in mm
#define RADAR_MIN_DISTANCE				0		// distance in mm
#define RADAR_MAX_DISTANCE				350		// distance in mm


//================================== REAR DISTANCE =====================================
#define REAR_DISTANCE_THESHOLD		180		// distance in mm



//===================== Servos ============================================================================================

typedef enum{SERVO_RUDDER,SERVO_MOTOR}ServoNo_t;
#define NO_OF_SERVOS				1
//========================== Motor Servo ===================		Although not a servo requires same output
#define MOTOR_SERVO_UPPER_LIMIT     2000
#define MOTOR_SERVO_MID				1500
#define MOTOR_SERVO_LOWER_LIMIT     1000
//===========================================================


//========================== Rudder Servo ===================
#define RUDDER_SERVO_UPPER_LIMIT     2000
#define RUDDER_SERVO_MID			 1500
#define RUDDER_SERVO_LOWER_LIMIT     1000
//===========================================================


//========================== Radar Servo ====================
#define RADAR_SERVO_UPPER_LIMIT     2000
#define RADAR_SERVO_MID				1500
#define RADAR_SERVO_LOWER_LIMIT     1000
//===========================================================

//=========================================================================================================================


//========================= Throttle ======================================================================================
#define THROTTLE_SAFE				1000
#define STOP_UPPER_LIMIT			1010
#define STOP_LOWER_LIMIT			980
#define MANOEUVER_SPEED_VALUE			1
#define REVERSE_SPEED_VALUE				1
#define NAVIGATION_SPEED_VALUE			8			 // Navigation speed in Knots *10
#define FULL_TURN_SPEED_VALUE				1			 // Min speed allowed in Knots *10
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

//======================= Sails ============================================================================================
#define MAX_SAIL_POSITION	2000
#define MIN_SAIL_POSITION	1000
//=========================================================================================================================

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
	float  VoltsBattery_1;               // battery voltage
	 uint16_t amperage;
} Analog_t;

typedef struct __attribute__ ((packed))
{
	unsigned char PVal;
	unsigned char IVal;
	unsigned char DVal;
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
	unsigned char MaxNumberOfWapoints;
	uint16_t ManoeuverSpeed;
	uint16_t NavigationSpeed;
	uint16_t FullTurnSpeed;
	uint16_t ReverseSpeed;
	uint16_t MaxNavRoll;
	uint16_t MaxNavPitch;
	unsigned char NavigationFlags;
	unsigned char MaxPitchAngle;
	unsigned char MaxRollAngle;
	bool ReadWaypointsFromSDcard;
	bool ClearLog;					// not used
	uint16_t MaximumFullTurnSpeed;
	unsigned char checksum;      // MUST BE ON LAST POSITION OF STRUCTURE !
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
	PidValues_t PidValues;
	unsigned char RudderRate;
	unsigned char RudderExpo;
	ServoConfig_t ServoConf[NO_OF_SERVOS];
	uint16_t SafeFrontDistance;
	uint16_t SafeRearDistance;
	uint16_t SticksSettings[NO_OF_STICK_FUNCTIONS];
	uint32_t AuxSwitchSettings[NO_OF_AUX_FUNCTIONS];
	SwitchValues_t SwitchSettings;
	uint8_t LastFileNumber;
	uint8_t WindSpeedScale;
	unsigned char  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE 
}  MainConfig_t;


typedef struct  __attribute__((__packed__ ))
{
	uint16_t ChannnelsMax[18];
	uint16_t ChannnelsMin[18];
	uint16_t FailsafeValues[18];
	unsigned char  checksum;      // MUST BE ON LAST POSITION OF CONF STRUCTURE 

}RcChannels_t;

typedef struct
{	//======== Startup and general=======
	unsigned char MAIN_INIT_COMPLETED	:1;
	unsigned char SD_CARD_PRESENT		:1;
	unsigned char SD_CARD_INIALISED		:1;
	unsigned char ALL_CONFIGS__LOADED	:1;
	unsigned char MENU_SELECTED		    :1;
	unsigned char LED_DRIVER_PRESENT    :1;
	unsigned char EEPROM_PRESENT	    :1;
	unsigned char RSSI_LEVEL			:1;
	unsigned char INITIALISE_COMPLETE	:1;
	unsigned char MOTOR_ASTERN			:1;
	unsigned char STOP_OBJECT_AVOID		:1;

	//=========	Sensors =================
	unsigned char ULTRA_DISTANCE_READY	:1;
	unsigned char ACC_CALIBRATED		:1;
	unsigned char GYRO_CALIBRATED		:1;
	unsigned char MAG_CALIBRATED		:1;
	unsigned char CALIBRATE_MAG			:1;
	unsigned char RADAR_FAIL			:1;
	unsigned char GPS_FAIL				:1;
	unsigned char COMPASS_FAIL			:1;
	unsigned char STERN_RANGER_FAILED	:1;
	unsigned char COMPASS_VALID			:1;
	unsigned char GIRO_VALID			:1;
	unsigned char ACCEL_VALID			:1;
	unsigned char LIDAR_VALID			:1;
	unsigned char RADAR_VALID			:1;
	unsigned char SONAR_VALID			:1;
	unsigned char VTX_PRESENT			:1;
	unsigned char CAMERA_PRESENT		:1;
	unsigned char STERN_MODULE_VALID	:1;			
	unsigned char I2C_INTERFACE_1_FAIL  :1;
	unsigned char I2C_INTERFACE_0_FAIL  :1;


	
	//=========	Alarms =================
	unsigned char PANIC_MODE_TRIGGERED	:1;
	unsigned char ARMED					:1;
	unsigned char ALARM_RAISED			:1;
	unsigned char ANGLE_ALARM			:1;
	unsigned char FRONT_DISTANCE_ALARM  :1;
	unsigned char REAR_DISTANCE_ALARM	:1;	
	unsigned char TASK_OVERRUN			:1;
	unsigned char MALFUNCTION_DETECTED	:1;
	unsigned char PANIC_MODE			:1;
	unsigned char LOST_RC_SIGNAL		:1;
	unsigned char GEN_MESSAGE_RECEIVED	:1;
	unsigned char FAILSAFE_ACTIVE	    :1;
	unsigned char RC_FAILSAVE_IN_USE    :1;
	unsigned char STERN_MODULE_FAILED	:1;
	unsigned char LOGGING_FAIL			:1;
	unsigned char NAVIGATION_FAIL		:1;
	unsigned char RD_MAIN_CONFIG_FAIL	:1;
	unsigned char RD_NAVIGATION_CFG_FAIL:1;
	unsigned char RD_WAYPOINTS_FAIL		:1;
	unsigned char RD_RC_CONFIG_FAIL		:1;
	unsigned char RD_FUNCTION_CONFIG_FAIL :1;
	unsigned char WR_MAIN_CONFIG_FAIL	:1;
	unsigned char WR_NAVIGATION_CFG_FAIL:1;
	unsigned char WR_WAYPOINTS_FAIL		:1;
	unsigned char WR_RC_CONFIG_FAIL		:1;
	unsigned char WR_FUNCTION_CONFIG_FAIL :1;
	
	
	//========== Displays ===============
	unsigned char ENABLE_GUI			:1;
	unsigned char DISPLAY_INIT_COMPLETED:1;
	
	//========== Battery ===============
	unsigned char BATTERY_1_ALARM		:1;
	unsigned char BATTERY_2_ALARM		:1;
	unsigned char BATTERY_1_ALARM_CRITICAL	:1;
	unsigned char BATTERY_2_ALARM_CRITICAL	:1;
	unsigned char BATTERY_FAILSAFE		:1;
	
	//========== Logging ===============
	unsigned char LOGGING_IN_PROGRESS	:1;
	
	//========== Switches ==============
	unsigned char SEARCHLIGHT_STATE		:1;

	//========== Navigation ============
	unsigned char HOME_LOCATION_SET		:1;
	unsigned char AT_HOME_COORDINATES	:1;
	unsigned char AUTOPILOT_START_OK	:1;
	unsigned char CONTROL_FAULT			:1;
	unsigned char RETURN_HOME			:1;
	unsigned char CONTROL_BY_AUTO_PILOT :1;
	unsigned char CONTROL_OVERIDE		:1;
	unsigned char GPS_INIT				:1;
	unsigned char GPS_3D_LOCK			:1;
	unsigned char GPS_2D_LOCK			:1;
	unsigned char NAV_HOME_COORD_SET	:1;
	unsigned char NAV_RETURN_TO_HOME	:1;
	unsigned char GPS_AVAILABLE			:1;
	unsigned char MISSION_DATA_LOADED	:1;
	unsigned char WP_PAST_SAFTEY_LIMIT	:1;
	unsigned char WP_ERROR				:1;
	unsigned char MISSION_COMPLETED		:1;
	unsigned char INIT_DOCKING			:1;
	unsigned char NAV_PROCESSOR_RUNNING	:1;
//	unsigned char WP_TABLE_UPDATED		:1;
	unsigned char OBJECT_AVOIDANCE_ACTIVE	:1;
	unsigned char GEO_FENCE_BREACH		:1;
	
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

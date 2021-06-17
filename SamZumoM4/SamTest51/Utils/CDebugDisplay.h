/* 
* CDebugDisplay.h
*
* Created: 11/06/2020 12:42:50
* Author: philg
*/


#ifndef __CDEBUGDISPLAY_H__
#define __CDEBUGDISPLAY_H__

#define DEBUG_MAX_LINE_LEN		80




//=====================Display Top =============================================
#define DEBUG_TOP_ROW				1
#define DEBUG_TOP_COL				3
#define DEBUG_ARMED_COL				DEBUG_TOP_COL
#define DEBUG_ARMED_ROW				DEBUG_TOP_ROW
#define DEBUG_BATTERY_V_COL			DEBUG_TOP_COL+25
#define DEBUG_BATTERY_V_ROW			DEBUG_TOP_ROW
#define DEBUG_SD_CARD_ROW			DEBUG_TOP_ROW
#define DEBUG_SD_CARD_COL			DEBUG_TOP_COL+18

//#define AVAILABLE_RAM_COL		38
//============================================================================

//==================== Col 1 =================================================
#define DEBUG_COLUMN_1				2
#define DEBUG_ROW_1					3

#define DEBUG_INIT_1_ROW	DEBUG_ROW_1
#define DEBUG_INIT_1_COL	DEBUG_COLUMN_1
#define DEBUG_INIT_2_ROW	DEBUG_ROW_1+1
#define DEBUG_INIT_2_COL	DEBUG_COLUMN_1
#define DEBUG_INIT_3_ROW	DEBUG_ROW_1+2
#define DEBUG_INIT_3_COL	DEBUG_COLUMN_1


#define DEBUG_DISTANCE_SENSOR_ROW	DEBUG_ROW_1
#define DEBUG_DISTANCE_SENSOR_COL	DEBUG_COLUMN_1

#define DEBUG_ALARMS_ROW			DEBUG_ROW_1+18
#define DEBUG_ALARMS_COL			DEBUG_COLUMN_1

#define DEBUG_I2C_DEVICES_ROW		DEBUG_ROW_1+28
#define DEBUG_I2C_DEVICES_COL		DEBUG_COLUMN_1

//============================================================================

//==================== Col 2 =================================================
#define DEBUG_COLUMN_2				35
#define DEBUG_ROW_2					3

#define DEBUG_IMU_ROW				DEBUG_ROW_2+5
#define DEBUG_IMU_COL				DEBUG_COLUMN_2
#define DEBUG_GPS_ROW				DEBUG_ROW_2 +12
#define DEBUG_GPS_COL				DEBUG_COLUMN_2
#define DEBUG_WAYPOINT_ROW			DEBUG_ROW_2 + 23
#define DEBUG_WAYPOINT_COL			DEBUG_COLUMN_2


//============================================================================

//==================== Col 3 =================================================
#define DEBUG_COLUMN_3				60
#define DEBUG_ROW_3					3

#define DEBUG_MOTOR_ROW				DEBUG_ROW_3+2
#define DEBUG_MOTOR_COL				DEBUG_COLUMN_3+1


#define DEBUG_NAVIGATION_ROW		DEBUG_ROW_3+20
#define DEBUG_NAVIGATION_COL		DEBUG_COLUMN_3+20

#define DEBUG_RADIO_ROW				DEBUG_ROW_3
#define DEBUG_RADIO_COL				DEBUG_COLUMN_3



//============================================================================

//============================================================================


//==================== Col 4 Debug Info =================================================



#define DEBUG_COLUMN_4				10   //20 //86
#define DEBUG_ROW_4					3

#define DEBUG_WHEEL_SENSORS_ROW		DEBUG_ROW_4
#define DEBUG_WHEEL_SENSORS_COL		DEBUG_COLUMN_4


#define DEBUG_RUNNING_FLAGS_ROW		DEBUG_ROW_4
#define DEBUG_RUNNING_FLAGS_COL		DEBUG_COLUMN_4

#define DEBUG_MAG_CAL_VALUES_ROW	DEBUG_ROW_3+13
#define DEBUG_MAG_CAL_VALUES_COL	DEBUG_COLUMN_4

#define DEBUG_MPU_CAL_VALUES_ROW	DEBUG_ROW_4+13
#define DEBUG_MPU_CAL_VALUES_COL	DEBUG_COLUMN_4

#define DEBUG_DEBUG_TIMINGS_ROW		DEBUG_ROW_3+13
#define DEBUG_DEBUG_TIMINGS_COL		DEBUG_COLUMN_4

//==================== Col 4 Debug Info =================================================
#define DEBUG_COLUMN_5				115
#define DEBUG_ROW_5					3
#define DEBUG_SYSTEM_TIMES_ROW		DEBUG_ROW_5
#define DEBUG_SYSTEM_TIMES_COL		DEBUG_COLUMN_5

//==================== Col 6 Debug Info =================================================
#define DEBUG_COLUMN_6				116
#define DEBUG_ROW_6					3

#define DEBUG_IMU_RAW_VALUES_ROW		DEBUG_ROW_6+40
#define DEBUG_IMU_RAW_VALUES_COL		DEBUG_COLUMN_6

//========================Page Bottom ========================================

#define DEBUG_BOTTOM_ROW				41
#define DEBUG_BOTTOM_COL				3


//============================================================================

#define DEBUG_INFOMATION_MESSAGES_ROW	DEBUG_BOTTOM_ROW
#define DEBUG_INFOMATION_MESSAGES_COL	DEBUG_BOTTOM_COL
#define DEBUG_DEBUG_MESSAGES_ROW		50
#define DEBUG_MESSAGES_COL		3
#define DEBUG_MESSAGES_ROW_MAX	20

typedef enum
{
	DISPLAY_INACTIVE,
	UPDATE_ACTIVE,
	PRINTF_ACTIVE
}DebugDisplayActive_t;


class CDebugDisplay
{
//variables
public:
	bool m_PrintfActive;

	CUart *m_Serial;
//	CSoftwareSerial *m_Serial;
protected:
private:

//functions
public:
//	CDebugDisplay();
//	CDebugDisplay(CSoftwareSerial *Serial);
	CDebugDisplay(CUart *Serial);
	~CDebugDisplay();
	void Init();
	void InitScreen();
	void DisplayLiveData();
	void SetDisplayActive(DebugDisplayActive_t Type);
	void DisplayGps();
	void DisplayConfigFail(void);
	void GpsVersion();
	void DisplayDistance();
	void DisplayLidars();
	void DisplayWheelSensorsCal();
	void DisplayWheelSensors();
	void DisplayMotors();
	void DisplayImu();
	void Printf(unsigned char Row,unsigned char Col, unsigned char ClearLen,const char *fmt, ... );
	void Printf(const char *fmt, ... );
	void ClearLine(unsigned char Row,unsigned char Col,unsigned char Length);
	void SetDisplayRowCol(unsigned char Row,unsigned char Col);
	void SetCursorState(bool OnOff);
	void WriteChar(uint8_t Char);
	void WriteBytes(uint8_t *Bytes,uint8_t Size);
protected:
private:
	CDebugDisplay( const CDebugDisplay &c );
	CDebugDisplay& operator=( const CDebugDisplay &c );

}; //CDebugDisplay

#endif //__CDEBUGDISPLAY_H__

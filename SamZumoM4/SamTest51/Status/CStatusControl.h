/* 
* CLedControl.h
*
* Created: 27/04/2017 12:47:07
* Author: phil
*/


#ifndef __CSTATUSCONTROL_H__
#define __CSTATUSCONTROL_H__

#include "stdio.h"



typedef enum {NORMAL,ALARM,BATTERY,IN_NAVIGATION,IMU_CALIBRATION,ALL_OFF,ARMED}LedSequence_t;

// wait 2 seconds before assuming a tone is done and continuing the continuous tone
#define STATUS_CONTROL_REPEAT_TONE_REPEAT_TIME_MS 4000
#define STATUS_CONTROL_TONE_BUF_SIZE 100


	#define STATUS_CONTROL_MAIN_CONFIG_FAIL						0
	#define STATUS_CONTROL_NAVIGATION_CONFIG_FAIL			1
	#define STATUS_CONTROL_READ_FUNCTION_CONFIG_FAIL	2
	#define STATUS_CONTROL_BATTERY_1_CRITICAL					3
	#define STATUS_CONTROL_BATTERY_1_WARN						4
	#define STATUS_CONTROL_ACC_NOT_CALIBRATED				5
	#define STATUS_CONTROL_IMU_FAIL										6	
	#define STATUS_CONTROL_LIDAR_FAILED								7		
	#define STATUS_CONTROL_GPS_FAIL									8		
	#define STATUS_CONTROL_VTX_SETTINGS							9		
	#define STATUS_CONTROL_CAMERA_FAIL								10		
	#define STATUS_CONTROL_I2C_FAIL_BUS_0							11		
	#define STATUS_CONTROL_I2C_FAIL_BUS_1							12		
	#define STATUS_CONTROL_NAVIGATION_UPDATE_FAILED		13		
	#define STATUS_CONTROL_NO_MISSION_FILE_LOADED			14		
	#define STATUS_CONTROL_HOME_WAYPOINT_NOT_SET			15		
	#define STATUS_CONTROL_WAYPOINT_1_ERROR					16		
	#define STATUS_CONTROL_RC_DATA_LOSS							17		
	#define STATUS_CONTROL_NO_SDCARD								18		
	#define STATUS_CONTROL_STARTUP										19		
	#define STATUS_CONTROL_LOUD_1										20		
	#define STATUS_CONTROL_LOUD_2										21		
	#define STATUS_CONTROL_LOUD_3										22		
	#define STATUS_CONTROL_LOUD_4										23		
	#define STATUS_CONTROL_LOUD_5										24		
	#define STATUS_CONTROL_LOUD_6										25		
	#define STATUS_CONTROL_LOUD_7										26		
	#define STATUS_CONTROL_NEG_FEEDBACK							27
	#define STATUS_CONTROL_NEU_FEEDBACK							28
	#define STATUS_CONTROL_POS_FEEDBACK							29




class CStatusControl
{
//variables
public:

	

protected:
private:
   int8_t m_RepeatTonePlaying;
   int8_t m_TonePlaying;
   uint32_t m_ToneBeginningMs;
   struct Tone
   {
	   const char *ToneSequenceStr;
	   const uint8_t Repeat : 1;
   };

   const static Tone Tones[];
   char ToneSquenceBuffer[STATUS_CONTROL_TONE_BUF_SIZE];
  //  bool _have_played_ready_tone : 1;

//functions
public:
	CStatusControl();
	~CStatusControl();
	void Init();
	void Update();
	void UpdateAlarms();
	void UpdateSound();
	void Startup();
	void PlayTone(const uint8_t tone_index);
	void PlayTune(const char *str);
	void StopContTone();
	void CheckForRepeatTone();
protected:
private:
	CStatusControl( const CStatusControl &c );
	CStatusControl& operator=( const CStatusControl &c );

}; //CStatusControl

#endif //__CSTATUSCONTROL_H__

/* 
* CRadioControl.h
*
* Created: 21/04/2019 19:38:04
* Author: phil
*/


#ifndef __CRADIOCONTROL_H__
#define __CRADIOCONTROL_H__

#include "cconfig.h"

#define NO_OF_RC_CHANS 16
#define FAILSAFE_DETECT_TRESHOLD  985

#define RC_MINCHECK 1200
#define RC_MAXCHECK 1850
#define RC_MID_UPPER 1590
#define RC_MID_LOWER 1420
#define RC_MIN		1000
#define RC_MID		1500
#define RC_MAX		1200


typedef enum  
{
	PILOT_STEER_TYPE_DEFAULT = 0,
	PILOT_STEER_TYPE_TWO_PADDLES = 1,
	PILOT_STEER_TYPE_DIR_REVERSED_WHEN_REVERSING = 2,
	PILOT_STEER_TYPE_DIR_UNCHANGED_WHEN_REVERSING = 3,
}RadioSteer_t;




// axillary switch handling ( stored as 2-bits!):
enum AuxSwitchPosition_t : uint8_t
{
	TRIGGER_LOW,	// Indicates auxiliary switch is in the low position (pwm <1200)
	TRIGGER_MID,	// Indicates auxiliary switch is in the middle position (pwm >1200, <1800)
	TRIGGER_HIGH	// Indicates auxiliary switch is in the high position (pwm >1800)
};


enum rc
{
	RC_AILE,
	RC_ELEV,			// PITCH ELEV
	RC_THROTTLE,			// ROLL  AILE
	RC_RUDDER,			// YAW
	RC_CHANNEL_5,
	RC_CHANNEL_6,
	RC_CHANNEL_7,
	RC_CHANNEL_8,
	RC_CHANNEL_9,
	RC_CHANNEL_10,
	RC_CHANNEL_11,
	RC_CHANNEL_12,
	RC_CHANNEL_13,
	RC_CHANNEL_14,
	RC_CHANNEL_15,
	RC_CHANNEL_16
};

#define THROTTLE_CHANNEL	RC_THROTTLE
#define STEER_CHANNEL_X		RC_AILE
#define STEER_CHANNEL_Y		RC_ELEV
#define THROTTLE_CHANNEL	RC_THROTTLE
#define STEER_CHANNEL		RC_AILE

#define THR_LO  (1<<(2*RC_THROTTLE))
#define THR_CE  (3<<(2*RC_THROTTLE))
#define THR_HI  (2<<(2*RC_THROTTLE))
#define ROL_LO  (1<<(2*RC_AILE))
#define ROL_CE  (3<<(2*RC_AILE))
#define ROL_HI  (2<<(2*RC_AILE))
#define PIT_LO  (1<<(2*RC_ELEV))
#define PIT_CE  (3<<(2*RC_ELEV))
#define PIT_HI  (2<<(2*RC_ELEV))
#define YAW_LO  (1<<(2*RC_RUDDER))
#define YAW_CE  (3<<(2*RC_RUDDER))
#define YAW_HI  (2<<(2*RC_RUDDER))


#define STICK_LOW	1
#define STICK_MID	2
#define STICK_HIGH	4

#define CALIBRATE_STICKS_NONE	0
#define CALIBRATE_STICKS		1
#define CALIBRATE_STICKS_SAVE	2			


#define CALIBRATE_ACCELEROMETER		1
#define CALIBRATE_GIRO				2
#define CALIBRATE_MAGNETOMETER		3
#define NO_FUNCTION					0

// de-bounce counters
typedef struct
{
	uint8_t Count;
	uint8_t NewPosition;
} DebounceCounter_t;



class CRadioControl
{
	//variables
public:
	unsigned char m_DoMenuFuctionIndex;
	bool m_FowardObsticalStop;
	bool m_ReversObsticalStop;
	uint16_t m_RadioDataIn[NO_OF_RC_CHANS];
//	uint16_t RadiocValues[8] ;	// interval [1000;2000]usede in interupt
	uint16_t m_ThrottleValue;
	uint16_t m_SteerValue;
	bool m_FailSafe;
	uint8_t m_LQI;
	DebounceCounter_t DebounceCounter[16];
	uint32_t m_PreviousSwitchPositions;
	uint16_t m_Rssi;
	int16_t m_ControlIn[3];
	RadioSteer_t m_SteerType;
	bool m_IsDirectionForward = true; 
protected:

private:
	uint32_t m_FailsafeStartTime;
	//functions
public:
	CRadioControl();
	~CRadioControl();
	void Init();
	void ComputeRCValues();
	int16_t PwmToRange( uint8_t Channel) const;
	int16_t PwmToRangeDz(uint16_t _dead_zone, uint8_t Channel) const;
	int16_t PwmToAngle(uint16_t Channel) const;
	int16_t PwmToAngleDz(uint16_t DeadZone, uint8_t Channel) const;
	bool GetFailSafe();
	bool MidCheck(unsigned char Stick);
	void CheckStickFunctions();
	uint16_t CheckConfigurableFunctions(uint8_t *StickValues);
	void CheckAuxSwitchFunctions();
	void SetAuxFunction(uint16_t Channel,uint8_t Position);
	void DoAuxFunction(uint8_t FunctionNo,uint8_t Command);
	void ReadAuxChannel(uint8_t Channel);
	bool Read3PosSwitch(uint8_t Channel,AuxSwitchPosition_t *Position);
	void SetStickFunction(uint16_t Number);
	void Calibrate(uint16_t *ReceivedData,bool Save);

	void Update(uint32_t Time);
	void GetRadioInput(float &steering_out, float &throttle_out);
	int16_t stick_mixing(uint16_t Channel,const int16_t servo_in);
	void CheckModeSwitches();
protected:
private:
	CRadioControl( const CRadioControl &c );
	CRadioControl& operator=( const CRadioControl &c );
	AuxSwitchPosition_t GetPreviousSwitchPosition(uint8_t Channel) const
	{
		return (AuxSwitchPosition_t)((m_PreviousSwitchPositions >> (Channel*2)) & 0x3);
	}
	
	void SetPreviousSwitchPosition(const uint8_t Channel, AuxSwitchPosition_t value)
	{
		m_PreviousSwitchPositions &= ~(0x3 << (Channel*2));
		m_PreviousSwitchPositions |= (value << (Channel*2));
	}
	void UpdateRcControl();

}; //CRadioControl
#endif //__CRADIOCONTROL_H__

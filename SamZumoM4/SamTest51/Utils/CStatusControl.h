/* 
* CLedControl.h
*
* Created: 27/04/2017 12:47:07
* Author: phil
*/


#ifndef __CSTATUSCONTROL_H__
#define __CSTATUSCONTROL_H__

#include "stdio.h"

typedef enum{ STARBOARD_LED,PORT_LED}FunctionLeds_t;


#define NUMBER_OF_LEDS		1


#define ENABLE_HEADLIGHT	10
#define HEADLIGHT_DIR		11
#define HEADLIGHT_ON		11
#define HEADLIGHT_OFF		11


// #define PORT_RED				0
// #define STARBOARD_LED			1



#define RGB_LED_OFF     0
#define RGB_LED_LOW     1
#define RGB_LED_MEDIUM  2
#define RGB_LED_HIGH    3



typedef enum {NORMAL,ALARM,BATTERY,IN_NAVIGATION,IMU_CALIBRATION,ALL_OFF,ARMED}LedSequence_t;


#define DEFINE_COLOUR_SEQUENCE(S0, S1, S2, S3, S4, S5, S6, S7, S8, S9)  \
((S0) << (0*3) | (S1) << (1*3) | (S2) << (2*3) | (S3) << (3*3) | (S4) << (4*3) | (S5) << (5*3) | (S6) << (6*3) | (S7) << (7*3) | (S8) << (8*3) | (S9) << (9*3))

#define DEFINE_COLOUR_SEQUENCE_SLOW(colour) DEFINE_COLOUR_SEQUENCE(colour,colour,colour,colour,colour,OFF,OFF,OFF,OFF,OFF)
#define DEFINE_COLOUR_SEQUENCE_FAILSAFE(colour) DEFINE_COLOUR_SEQUENCE(YELLOW,YELLOW,YELLOW,YELLOW,YELLOW,colour,colour,colour,colour,colour)
#define DEFINE_COLOUR_SEQUENCE_SOLID(colour) DEFINE_COLOUR_SEQUENCE(colour,colour,colour,colour,colour,colour,colour,colour,colour,colour)
#define DEFINE_COLOUR_SEQUENCE_ALTERNATE(colour1, colour2) DEFINE_COLOUR_SEQUENCE(colour1,colour2,colour1,colour2,colour1,colour2,colour1,colour2,colour1,colour2)
#define DEFINE_COLOUR_SEQUENCE_TRISTATE(colour1, colour2,colour3) DEFINE_COLOUR_SEQUENCE(colour1,colour2,colour3,colour1,colour2,colour3,colour1,colour2,colour3,colour2)

#define OFF    0
#define BLUE   1
#define GREEN  2
#define RED    4
#define YELLOW (RED|GREEN)
#define WHITE (RED|GREEN|BLUE)





typedef struct
{
	bool OnOff;
	bool FlashState;
	bool IsFlash;
	uint8_t Colour;
}Led_t;


#define BLINK_INTERVAL  90



class CStatusControl
{
//variables
public:
	Led_t m_Led;
	uint32_t CurrentColourSequence = 0;
	uint8_t m_Brightness;
	const uint32_t SequenceBatteryFailPanic = DEFINE_COLOUR_SEQUENCE_ALTERNATE(RED,OFF);
	const uint32_t SequencePanicArmed = DEFINE_COLOUR_SEQUENCE_ALTERNATE(RED,YELLOW);
	const uint32_t SequenceBatteryWarning = DEFINE_COLOUR_SEQUENCE_SLOW(RED);
	const uint32_t GeneralAlarm = DEFINE_COLOUR_SEQUENCE_ALTERNATE(YELLOW,OFF);

	

	const uint32_t SequenceArmedGps = DEFINE_COLOUR_SEQUENCE_SOLID(GREEN);
	const uint32_t SequenceArmedNoGps = DEFINE_COLOUR_SEQUENCE_SOLID(BLUE);
	const uint32_t SequenceUnarmedGps = DEFINE_COLOUR_SEQUENCE_SOLID(YELLOW);
	const uint32_t SequenceUnarmedNoGps = DEFINE_COLOUR_SEQUENCE_ALTERNATE(YELLOW,OFF);
	const uint32_t SequenceAllOff = DEFINE_COLOUR_SEQUENCE_SOLID(OFF);
 	const uint32_t SequenceInitialising = DEFINE_COLOUR_SEQUENCE_ALTERNATE(RED,BLUE);

	const uint32_t SequenceInAuto = DEFINE_COLOUR_SEQUENCE_TRISTATE(RED,GREEN,YELLOW);
	const uint32_t SequenceGyroCal = DEFINE_COLOUR_SEQUENCE_TRISTATE(OFF,GREEN,YELLOW);
	const uint32_t SequenceAccelCal = DEFINE_COLOUR_SEQUENCE_TRISTATE(RED,OFF,YELLOW);
	const uint32_t SequenceMagCal = DEFINE_COLOUR_SEQUENCE_TRISTATE(RED,GREEN,OFF);
// 	const uint32_t sequence_prearm_failing = DEFINE_COLOUR_SEQUENCE(YELLOW,YELLOW,OFF,OFF,YELLOW,YELLOW,OFF,OFF,OFF,OFF);
// 	const uint32_t sequence_disarmed_good_dgps = DEFINE_COLOUR_SEQUENCE_ALTERNATE(GREEN,OFF);
// 	const uint32_t sequence_disarmed_good_gps = DEFINE_COLOUR_SEQUENCE_SLOW(GREEN);
// 	const uint32_t sequence_disarmed_bad_gps = DEFINE_COLOUR_SEQUENCE_SLOW(BLUE);
protected:
private:

//functions
public:
	CStatusControl();
	~CStatusControl();
	void Init();
	uint8_t GetBrightness(void) const;
	void Update();
	void UpdateAlarms();
	void UpdateLedStatus();
	void UpdateStartup();
	void SetLedSequence(uint32_t Sequence);
	void UpdateLedSequence();
	void TestLeds();
	void SetAllLedsOff();
	void UpdateLed();
protected:
private:
	CStatusControl( const CStatusControl &c );
	CStatusControl& operator=( const CStatusControl &c );

}; //CStatusControl

#endif //__CSTATUSCONTROL_H__

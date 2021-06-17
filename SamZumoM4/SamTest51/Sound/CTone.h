/* 
* CTone.h
*
* Created: 08/06/2021 12:40:50
* Author: philg
*/


#ifndef __CTONE_H__
#define __CTONE_H__
//#define SET_TONE_PORT_ENABLE	PinPeripheral.SetPinMode(PORTA,1,OUTPUT);
//#define TONE_LOW			PinPeripheral.SetPin(PORTA,1,LOW);
//#define TONE_HIGH			PinPeripheral.SetPin(PORTA,1,HIGH);

#define TONE_WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.SYNCBUSY.bit.ENABLE);
#define TONE_TC         TC3
#define TONE_TC_IRQn    TC3_IRQn
#define TONE_TC_GCLK_ID	TC3_GCLK_ID
#define TONE_TC_TOP     0xFFFF
#define TONE_TC_CHANNEL 0
class CTone
{
//variables
public:
	volatile uint32_t *portToggleRegister;
	volatile uint32_t *portClearRegister;
	volatile uint32_t portBitMask;
	volatile int64_t toggleCount;
	volatile bool toneIsActive = false;
	volatile bool firstTimeRunning = false;
	EPortType m_Port;
	uint8_t m_PortPin;


protected:
private:

//functions
public:
	CTone();
	~CTone();
	void PlayTone (EPortType Port, uint8_t outputPin, uint32_t frequency, uint32_t duration);
	inline void resetTC (Tc* TCx);
	void toneAccurateClock (uint32_t accurateSystemCoreClockFrequency);
	void noTone (uint32_t outputPin);
	void Tone_Handler (void);
protected:
private:
	CTone( const CTone &c );
	CTone& operator=( const CTone &c );

}; //CTone

#endif //__CTONE_H__

/* 
* CPinPeripheral.h
*
* Created: 14/05/2020 19:20:19
* Author: philg
*/


#ifndef __CPINPERIPHERAL_H__
#define __CPINPERIPHERAL_H__
#include "stdio.h"
#include "Defines.h"


// LEDs
#define PIN_LED_13           (13u)
#define PIN_LED_RXL          (25u)
#define PIN_LED_TXL          (26u)
#define PIN_LED              PIN_LED_13
#define PIN_LED2             PIN_LED_RXL
#define PIN_LED3             PIN_LED_TXL
#define LED_BUILTIN          PIN_LED_13
#define PIN_NEOPIXEL         (8)

#define LOW             (0x0)
#define HIGH            (0x1)

#define INPUT           (0x0)
#define OUTPUT          (0x1)
#define INPUT_PULLUP    (0x2)
#define INPUT_PULLDOWN  (0x3)


// static const uint8_t DAC0 = PIN_DAC0;
// static const uint8_t DAC1 = PIN_DAC1;


// Other pins
//#define PIN_ATN              (31ul)
//static const uint8_t ATN = PIN_ATN;



class CPinPeripheral
{
//variables
public:
protected:
private:

//functions
public:
	CPinPeripheral();
	~CPinPeripheral();
	int SetPeripheral( uint32_t ulPin, EPioType_t ulPeripheral );
	void SetPeripheral(EPortType port ,uint32_t PortNo, EPioType_t ulPeripheral );
	void SetPinMode( EPortType port,uint32_t PortPin, uint32_t Mode );
//	void SetPin( uint32_t ulPin, bool ulVal );
//	void SetPin(EPortType port, uint32_t PortPin, uint32_t Value );
	void SetPin(EPortType port, uint32_t PortPin, bool Value );
	void TogglePin(EPortType port, uint32_t PortPin );
	int ReadPin( uint32_t ulPin );
	int ReadPin(EPortType port, uint32_t PortPin );
protected:
private:
	CPinPeripheral( const CPinPeripheral &c );
	CPinPeripheral& operator=( const CPinPeripheral &c );

}; //CPinPeripheral

#endif //__CPINPERIPHERAL_H__

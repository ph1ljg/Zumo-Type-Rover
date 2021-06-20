/* 
* PCF8574.h
*
* Created: 23/09/2020 10:42:43
* Author: philg
*/


#ifndef __PCF8574_H__
#define __PCF8574_H__


#ifdef TWO_WHEEL_ZUMO
#define PCF8574_ADDDRESS  0x20
#endif

#ifdef FOUR_WHEEL_ZUMO
#define PCF8724_ADDDRESS  0x27
#endif



class CPCF8574
{
//variables
public:
protected:
private:
	uint8_t m_PinRegister;

//functions
public:
	CPCF8574();
	~CPCF8574();
	bool PCF8574OutData (unsigned char Data);
	bool SetOutput(uint8_t PinNo,bool State);
	int Test();
protected:
private:
	CPCF8574( const CPCF8574 &c );
	CPCF8574& operator=( const CPCF8574 &c );

}; //PCF8574

#endif //__PCF8574_H__

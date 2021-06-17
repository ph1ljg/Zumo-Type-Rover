/*
* Cpwm.h
*
* Created: 20/05/2020 14:04:04
* Author: philg
*/


#ifndef __CPWM_H__
#define __CPWM_H__
#include "stdio.h"
#include "samd51j19a.h"
#include "Defines.h"

#define GetTCNumber( x ) ( (x) >> 8 )
#define GetTCChannelNumber( x ) ( (x) & 0xff )
#define GetTC( x ) ( g_apTCInstances[(x) >> 8] )

typedef struct
{
	Tc* TCx;
	uint32_t Value;
	bool IsForward;
	uint8_t Channel;
}MotorChannel_t;




class Cpwm
{
//variables
public:
	uint16_t m_PWMChannel;

protected:
private:
	Tc* m_TCx;
	Tcc* m_TCCx;
	MotorChannel_t m_Motors[2];
	uint8_t MotorIndex =0;
//functions
public:
	bool Write( EPortType port,uint32_t PortPin, uint32_t value);
	void SetValue(uint32_t Value);
	bool GetTimerAndChannelNo(uint16_t PortPin,uint32_t &tcNum,uint8_t &tcChannel,eTCChannel &PWMChannel);
	Cpwm();
	~Cpwm();
	bool Init( EPortType port,uint32_t PortPin, uint32_t Value,eTCChannel PWMChannel);
	bool GetTimer(uint32_t tcNum,eTCChannel PWMChannel);
protected:
private:
	Cpwm( const Cpwm &c );
	Cpwm& operator=( const Cpwm &c );

}; //Cpwm

#endif //__CPWM_H__

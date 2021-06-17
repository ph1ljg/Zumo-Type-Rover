/* 
* PCF8574.cpp
*
* Created: 23/09/2020 10:42:43
* Author: philg
*/

#include "Includes.h"

// default constructor
CPCF8574::CPCF8574()
{
	m_PinRegister = 0;
} //PCF8574

// default destructor
CPCF8574::~CPCF8574()
{
} //~PCF8574


bool CPCF8574::PCF8574OutData (unsigned char Data)
{
	if(!I2c.WriteByte(PCF8574_ADDDRESS,Data))
		return(false);
// 	if(!I2C.WriteByte(LCD_ADDRESS, Output))
// 	return(0);
	return(true);
}

bool CPCF8574::SetOutput(uint8_t PinNo,bool State)
{
	if(State)
		m_PinRegister |= (1<<PinNo);
	else
		m_PinRegister &= ~(1<<PinNo);
		
	return(PCF8574OutData(m_PinRegister));	
		 	
}

int CPCF8574::Test()
{
	uint8_t Index =1;
	PCF8574OutData (0);
	while(1)
	{
		SetOutput(Index,true);
		Core.delay(100);
		SetOutput(Index,false);
		if(Index <8)
		{
			Index++;
		}
		else
		{
			Index = 1;
			PCF8574OutData (0);
		}

		Core.delay(50);		
		
	}
}
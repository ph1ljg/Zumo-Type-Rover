/* 
* CBuzzer.cpp
*
* Created: 03/10/2020 17:37:20
* Author: philg
*/


#include "includes.h"

// default constructor
CBuzzer::CBuzzer()
{
} //CBuzzer

// default destructor
CBuzzer::~CBuzzer()
{
} //~CBuzzer


void CBuzzer::Init()
{
	PinPeripheral.SetPinMode(PORTA,5,OUTPUT);
	PinPeripheral.SetPin(PORTA,5,LOW);
	m_LastState  = false;

}
void CBuzzer::Tone()
{
	for (int n = 0;n<50;n++)
	{
	Buzzer.ToggleOutput();
	Core.delay(1);	
	}
	
}


void CBuzzer::ToggleOutput()
{
	if(m_LastState)
	{
		PinPeripheral.SetPin(PORTA,5,LOW);
		m_LastState = false;
	}
	else
	{
		PinPeripheral.SetPin(PORTA,5,HIGH);
		m_LastState = true;
		
	}
}
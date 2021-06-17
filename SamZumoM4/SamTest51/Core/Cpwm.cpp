/*
* Cpwm.cpp
*
* Created: 20/05/2020 14:04:04
* Author: philg
*/
#include "Includes.h"

// static uint32_t Lastime1 = 0;
// static uint32_t Lastime2 = 0;
// static uint32_t Lastime3 = 0;
// //static uint32_t Lastime4 = 0;
// static uint32_t time1;
// static uint32_t time2;
// static uint32_t time3;
//static uint32_t time4;

uint32_t Chan1;

static bool tcEnabled[TCC_INST_NUM+TC_INST_NUM];

const void* g_apTCInstances[TCC_INST_NUM+TC_INST_NUM]={ TCC0, TCC1, TCC2, TCC3, TCC4, TC0, TC1, TC2, TC3, TC4, TC5 } ;
const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM] = { TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID, TCC3_GCLK_ID, TCC4_GCLK_ID, TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID, TC5_GCLK_ID } ;

Cpwm Pwm;

// default constructor
Cpwm::Cpwm()
{
} //Cpwm

// default destructor
Cpwm::~Cpwm()
{
} //~Cpwm

bool Cpwm::Init( EPortType port,uint32_t PortPin, uint32_t Value,eTCChannel PWMChannel)
{
	uint32_t tcNum;
	uint8_t tcChannel;
	
	tcNum = GetTCNumber(PWMChannel);
	tcChannel = GetTCChannelNumber(PWMChannel);

	m_PWMChannel = (uint16_t) PWMChannel & ~(1<<8);
	switch(PortPin)
	{
	case 16:	// D5 PA16
		PinPeripheral.SetPeripheral(port,PortPin, PIO_TIMER_ALT);	// PWM F
		break;
	case 14: // D4 PA14
		PinPeripheral.SetPeripheral(port,PortPin, PIO_TCC_PDEC);	// PWM G
		break;
	case 20: // D10 PA20
		PinPeripheral.SetPeripheral(port,PortPin, PIO_TCC_PDEC);	// PWM G
		//PinPeripheral.SetPeripheral(port,PortPin, PIO_TIMER_ALT);	// PWM F
		break;
	case 21:	// D11 PA21
		PinPeripheral.SetPeripheral(port,PortPin, PIO_TCC_PDEC);	// PWM G
//		PinPeripheral.SetPeripheral(port,PortPin, PIO_TIMER_ALT);	// PWM F
		break;
	}
	
	if (!tcEnabled[tcNum])
	{
		tcEnabled[tcNum] = true;
		GCLK->PCHCTRL[GCLK_CLKCTRL_IDs[tcNum]].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);		//use clock generator 0
		
		if (tcNum >= TCC_INST_NUM)												// Set PORT
		{
			m_TCx = (Tc*) GetTC(PWMChannel);						// -- Configure TC
			m_TCx->COUNT8.CTRLA.bit.SWRST = 1;									//reset
			while (m_TCx->COUNT8.SYNCBUSY.bit.SWRST);
			m_TCx->COUNT8.CTRLA.bit.ENABLE = 0;									// Disable TCx
			while (m_TCx->COUNT8.SYNCBUSY.bit.ENABLE);
			
			m_TCx->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV64; // Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/256
			m_TCx->COUNT8.WAVE.reg = TC_WAVE_WAVEGEN_NPWM;

			while (m_TCx->COUNT8.SYNCBUSY.bit.CC0);								// Set the initial value
			m_TCx->COUNT8.CC[tcChannel].reg = (uint8_t) Value;
			while (m_TCx->COUNT8.SYNCBUSY.bit.CC0);
			m_TCx->COUNT8.PER.reg = 0xFF;											// Set PER to maximum counter value (resolution : 0xFF)
			while (m_TCx->COUNT8.SYNCBUSY.bit.PER);
			
			m_TCx->COUNT8.CTRLA.bit.ENABLE = 1;									// Enable TCx
			while (m_TCx->COUNT8.SYNCBUSY.bit.ENABLE);
		}
		else
		{
			
			m_TCCx = (Tcc*) GetTC(PWMChannel);				// -- Configure TCC

			m_TCCx->CTRLA.bit.SWRST = 1;
			while (m_TCCx->SYNCBUSY.bit.SWRST);

			
			m_TCCx->CTRLA.bit.ENABLE = 0;									// Disable TCCx
			while (m_TCCx->SYNCBUSY.bit.ENABLE);
			
			m_TCCx->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV256_Val | TCC_CTRLA_PRESCSYNC_GCLK;		// Set prescaler to 1/256

			
			m_TCCx->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;											// Set TCx as normal PWM
			while ( m_TCCx->SYNCBUSY.bit.WAVE );

			while (m_TCCx->SYNCBUSY.bit.CC0 || m_TCCx->SYNCBUSY.bit.CC1);
			
			m_TCCx->CC[tcChannel].reg = (uint32_t) Value;										// Set the initial value
			while (m_TCCx->SYNCBUSY.bit.CC0 || m_TCCx->SYNCBUSY.bit.CC1);
			
			m_TCCx->PER.reg = 5000; //0x310;	//3771														// Set PER to maximum counter value
			while (m_TCCx->SYNCBUSY.bit.PER);
			
			m_TCCx->DBGCTRL.reg  =1;
			//m_TCCx->DRVCTRL.bit.INVEN4 =1;
			m_TCCx->CTRLA.bit.ENABLE = 1;
			while (m_TCCx->SYNCBUSY.bit.ENABLE);											// Enable TCCx
		}
		SetValue(Value);
		return(true);
	}
	GetTimer(tcNum,PWMChannel);
	SetValue(Value);
	return(false);
}

bool Cpwm::GetTimer(uint32_t tcNum,eTCChannel PWMChannel)
{
	if (tcNum >= TCC_INST_NUM)
	{
		m_TCx = (Tc*) GetTC(PWMChannel);
		return(true);
	}
	else
	{
		m_TCCx = (Tcc*) GetTC(PWMChannel);
		return(true);
	}
	return(false);
}


void Cpwm::SetValue(uint32_t Value)
{
	while (m_TCCx->SYNCBUSY.bit.CTRLB);

	while (m_TCCx->SYNCBUSY.bit.CC0 || m_TCCx->SYNCBUSY.bit.CC1);
	m_TCCx->CCBUF[m_PWMChannel].reg = (uint32_t) Value;
	Chan1 = m_TCCx->CCBUF[m_PWMChannel].reg;
	while (m_TCCx->SYNCBUSY.bit.CC0 || m_TCCx->SYNCBUSY.bit.CC1);

	m_TCCx->CTRLBCLR.bit.LUPD = 1;
	while (m_TCCx->SYNCBUSY.bit.CTRLB);
}




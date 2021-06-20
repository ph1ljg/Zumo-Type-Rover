/*
* CTone.cpp
*
* Created: 01/06/2021 09:23:15
* Author: philg
*/


#include "Includes.h"
#include "CTone.h"


// default constructor
CTone::CTone()
{
} //CTone

// default destructor
CTone::~CTone()
{
} //~CTone


//#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.SYNCBUSY.bit.ENABLE);

uint32_t toneMaxFrequency = F_CPU / 2;
uint32_t lastOutputPin = 0xFFFFFFFF;



void TC3_Handler ()
{
	Tone.Tone_Handler();
}
//	 __attribute__ ((weak, alias("Tone.Tone_Handler")));

// void CTone::PlayTone()
// {
// 	SET_TONE_PORT_ENABLE;
// }

inline void CTone::resetTC (Tc* TCx)
{
	// Disable TCx
	TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	TONE_WAIT_TC16_REGS_SYNC(TCx)

	// Reset TCx
	TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	TONE_WAIT_TC16_REGS_SYNC(TCx)
	while (TCx->COUNT16.CTRLA.bit.SWRST);
}

void CTone::toneAccurateClock (uint32_t accurateSystemCoreClockFrequency)
{
	toneMaxFrequency = accurateSystemCoreClockFrequency / 2;
}

void CTone::PlayTone (EPortType Port, uint8_t outputPin, uint32_t frequency, uint32_t duration)
{
	m_Port = Port;
	m_PortPin = outputPin;
	// Configure interrupt request
	NVIC_DisableIRQ(TONE_TC_IRQn);
	NVIC_ClearPendingIRQ(TONE_TC_IRQn);
	
	if(!firstTimeRunning)
	{
		firstTimeRunning = true;
		
		NVIC_SetPriority(TONE_TC_IRQn, 0);

		GCLK->PCHCTRL[TONE_TC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}

	//if it's a rest, set to 1Hz (below audio range)
	frequency = (frequency > 0 ? frequency : 1);
	
	if (toneIsActive && (outputPin != lastOutputPin))
	noTone(lastOutputPin);

	//
	// Calculate best prescaler divider and comparator value for a 16 bit TC peripheral
	//

	uint32_t prescalerConfigBits;
	uint32_t ccValue;

	ccValue = toneMaxFrequency / frequency - 1;
	prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1;
	
	uint8_t i = 0;
	
	while(ccValue > TONE_TC_TOP)
	{
		ccValue = toneMaxFrequency / frequency / (2<<i) - 1;
		i++;
		if(i == 4 || i == 6 || i == 8) //DIV32 DIV128 and DIV512 are not available
		i++;
	}
	
	switch(i-1)
	{
		case 0: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV2; 
				break;
		case 1: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV4; 
				break;
		case 2: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV8; 
				break;
		case 3: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV16; 
				break;
		case 5: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV64; 
				break;
		case 7: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV256; 
				break;
		case 9: prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1024; 
				break;
		default: 
				break;
	}

	toggleCount = (duration > 0 ? frequency * duration * 2 / 1000UL : -1);

	resetTC(TONE_TC);
	TONE_TC->COUNT16.DBGCTRL.bit.DBGRUN = true;
	uint16_t tmpReg = 0;
	tmpReg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
	
	TONE_TC->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;  // Set TONE_TC mode as match frequency
	tmpReg |= prescalerConfigBits;
	TONE_TC->COUNT16.CTRLA.reg |= tmpReg;
	TONE_WAIT_TC16_REGS_SYNC(TONE_TC)

	TONE_TC->COUNT16.CC[TONE_TC_CHANNEL].reg = (uint16_t) ccValue;
	TONE_WAIT_TC16_REGS_SYNC(TONE_TC)


	// Enable the TONE_TC interrupt request
	TONE_TC->COUNT16.INTENSET.bit.MC0 = 1;
	
	if (outputPin != lastOutputPin)
	{
		lastOutputPin = outputPin;
		PinPeripheral.SetPinMode( Port, outputPin, OUTPUT );
		PinPeripheral.SetPin(Port,outputPin, LOW);
		toneIsActive = true;
	}

	// Enable TONE_TC
	TONE_TC->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	TONE_WAIT_TC16_REGS_SYNC(TONE_TC)
	
	NVIC_EnableIRQ(TONE_TC_IRQn);
}

void CTone::noTone (uint32_t outputPin)
{
	resetTC(TONE_TC);
	PinPeripheral.SetPin(m_Port,m_PortPin,LOW);
	toneIsActive = false;
}

void CTone::Tone_Handler (void)
{
	if (toggleCount != 0)
	{
		
		PinPeripheral.TogglePin(m_Port,m_PortPin);		// Toggle the output pin			
		if (toggleCount > 0)
		--toggleCount;

		
		TONE_TC->COUNT16.INTFLAG.bit.MC0 = 1;	// Clear the interrupt
	}
	else
	{
		resetTC(TONE_TC);
		PinPeripheral.SetPin(m_Port,m_PortPin,LOW);
		toneIsActive = false;
	}
}



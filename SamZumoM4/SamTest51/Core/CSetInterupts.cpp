/* 
* CSetInterupts.cpp
*
* Created: 21/07/2020 18:41:20
* Author: philg
*/
#include "Includes.h"


// default constructor
CSetInterupts::CSetInterupts()
{
} //CSetInterupts

// default destructor
CSetInterupts::~CSetInterupts()
{
} //~CSetInterupts



/* Configure I/O interrupt sources */
void CSetInterupts::Init()
{
  ///EIC MCLK is enabled by default
  for (uint32_t i = 0; i <= 15; i++)     // EIC_0_IRQn = 12 ... EIC_15_IRQn = 27
  {
    uint8_t irqn = EIC_0_IRQn + i;
    NVIC_DisableIRQ((IRQn_Type)irqn);
    NVIC_ClearPendingIRQ((IRQn_Type)irqn);
    NVIC_SetPriority((IRQn_Type)irqn, 3);
    NVIC_EnableIRQ((IRQn_Type)irqn);
  }
  
  GCLK->PCHCTRL[EIC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);


  // Enable EIC
  EIC->CTRLA.bit.ENABLE = 1;
  while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
}

void CSetInterupts::SetInterrupt(EPortType port ,uint32_t pin,EExt_Interrupts ExtIntNumber, EExt_Interrupts InType, uint32_t mode)
{
//	static int enabled = 0;
	uint32_t config;
	uint32_t pos;

//	EExt_Interrupts in = EXTERNAL_INT_0;

//	uint32_t inMask = (1UL << InType);

	if (InType == EXTERNAL_INT_NMI)
	{
		EIC->NMIFLAG.bit.NMI = 1; // Clear flag
		switch (mode) 
		{
		case LOW:
			EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_LOW;
			break;
		case HIGH:
			EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_HIGH;
			break;
		case CHANGE:
			EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_BOTH;
			break;
		case FALLING:
			EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_FALL;
			break;
			case RISING:
			EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_RISE;
			break;
		}

	} 
	else 
	{ 
		PinPeripheral.SetPeripheral(port,pin, PIO_EXTINT);		// Not NMI, is external interrupt Assign pin to EIC
		
		if (ExtIntNumber > EXTERNAL_INT_7)								// Look for right CONFIG register to be addressed 
		{
			config = 1;
			pos = (ExtIntNumber - 8) << 2;
		} 
		else 
		{
			config = 0;
			pos = ExtIntNumber << 2;
		}

		EIC->CTRLA.bit.ENABLE = 0;
		while (EIC->SYNCBUSY.bit.ENABLE == 1) { }

		EIC->CONFIG[config].reg &=~ (EIC_CONFIG_SENSE0_Msk << pos); // Reset sense mode, important when changing trigger mode during runtime
		switch (mode)
		{
		case LOW:
			EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_LOW_Val << pos;
			break;
		case HIGH:
			EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_HIGH_Val << pos;
			break;
		case CHANGE:
			EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_BOTH_Val << pos;
			break;
		case FALLING:
			EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
			break;
		case RISING:
			EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
			break;
		}
	}
	// Enable the interrupt
	EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << ExtIntNumber);

	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.bit.ENABLE == 1) { }
}


void CSetInterupts::SetEnable(EExt_Interrupts ExtIntNumber,bool Enable)
{
	if(Enable)
			EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << ExtIntNumber);
	else
			EIC->INTENCLR.reg = EIC_INTENSET_EXTINT(1 << ExtIntNumber);
	
}
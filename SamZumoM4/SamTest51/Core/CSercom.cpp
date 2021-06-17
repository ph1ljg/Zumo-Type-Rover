/*
* CSercom.cpp
*
* Created: 22/05/2020 14:23:52
* Author: philg
*/


#include "includes.h"





// default constructor
CSercom::CSercom()
{

	// #if SERCOM_SPI_FREQ_REF == F_CPU       // F_CPU clock = GCLK0
	//   m_ClockSource = SERCOM_CLOCK_SOURCE_FCPU;
	//  #elif SERCOM_SPI_FREQ_REF == 48000000  // 48 MHz clock = GCLK1 (standard)
	//   m_ClockSource = SERCOM_CLOCK_SOURCE_48M;
	//  #elif SERCOM_SPI_FREQ_REF == 100000000 // 100 MHz clock = GCLK2
	//   m_ClockSource = SERCOM_CLOCK_SOURCE_100M;
	//  #endif

} //CSercom

// default destructor
CSercom::~CSercom()
{
} //~CSercom



void CSercom::InitClockNVIC( void* Ser ,SercomClockSource m_ClockSource)
{
	int8_t idx = GetSercomIndex((Sercom*)Ser);
	if(idx < 0)	return; // We got a problem here	for(uint8_t i=0; i<4; i++)	{		NVIC_ClearPendingIRQ(sercomData[idx].irq[i]);		NVIC_SetPriority(sercomData[idx].irq[i], SERCOM_NVIC_PRIORITY);		NVIC_EnableIRQ(sercomData[idx].irq[i]);	}	// SPI DMA speed is dictated by the "slow clock" so BOTH are set to the same clock source (clk_slow isn't sourced from XOSC32K as in prior versions of SAMD core).	// This might have power implications for sleep code.	SetClockSource(idx, m_ClockSource, true);  // true  = core clock
	SetClockSource(idx, m_ClockSource, false); // false = slow clock

}


uint8_t CSercom::GetSercomIndex(Sercom *Ser)
{
	for(uint8_t i=0; i<(sizeof(sercomData) / sizeof(sercomData[0])); i++)	{		if(Ser == sercomData[i].sercomPtr)		return i;	}	return -1;}


uint8_t CSercom::CalculateBaudrateSynchronous(uint32_t baudrate,uint32_t RefFrequency)
{
	uint16_t b = RefFrequency / (2 * baudrate);
	if(b > 0)
	b--; // Don't -1 on baud calc if already at 0
	return b;
}



void CSercom::SetClockSource(int8_t idx, SercomClockSource src, bool core)
{

	if(src == SERCOM_CLOCK_SOURCE_NO_CHANGE) return;
	
	uint8_t clk_id = core ? sercomData[idx].id_core : sercomData[idx].id_slow;
	
	GCLK->PCHCTRL[clk_id].bit.CHEN = 0;     // Disable timer
	while(GCLK->PCHCTRL[clk_id].bit.CHEN);  // Wait for disable
	
	
	// From startup.c:
	// GCLK0 = F_CPU
	// GCLK1 = 48 MHz
	// GCLK2 = 100 MHz
	// GCLK3 = XOSC32K
	// GCLK4 = 12 MHz
	if(src == SERCOM_CLOCK_SOURCE_FCPU)
	{
		GCLK->PCHCTRL[clk_id].reg =   GCLK_PCHCTRL_GEN_GCLK0_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}
	else if(src == SERCOM_CLOCK_SOURCE_48M)
	{
		GCLK->PCHCTRL[clk_id].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}
	else if(src == SERCOM_CLOCK_SOURCE_100M)
	{
		GCLK->PCHCTRL[clk_id].reg =	GCLK_PCHCTRL_GEN_GCLK2_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}
	else if(src == SERCOM_CLOCK_SOURCE_32K)
	{
		GCLK->PCHCTRL[clk_id].reg =	GCLK_PCHCTRL_GEN_GCLK3_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}
	else if(src == SERCOM_CLOCK_SOURCE_12M)
	{
		GCLK->PCHCTRL[clk_id].reg =	GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos);
	}
	
	while(!GCLK->PCHCTRL[clk_id].bit.CHEN); // Wait for clock enable
}


void CSercom::EnableClock(const void *const hw,uint8_t SerComNo)
{
	switch(SerComNo)
	{
		case 0:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBAMASK.reg |= MCLK_APBAMASK_SERCOM0;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 1:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBAMASK.reg |= MCLK_APBAMASK_SERCOM1;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 2:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBBMASK.reg |= MCLK_APBBMASK_SERCOM2;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 3:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBBMASK.reg |= MCLK_APBBMASK_SERCOM3;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 4:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBDMASK.reg |= MCLK_APBDMASK_SERCOM4;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 5:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 6:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBDMASK.reg |= MCLK_APBDMASK_SERCOM6;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
		case 7:
		//	MCLK_CRITICAL_SECTION_ENTER();
		((Mclk *)hw)->APBDMASK.reg |= MCLK_APBDMASK_SERCOM7;
		//	MCLK_CRITICAL_SECTION_LEAVE();
		break;
	}
}



void CSercom::EnableMainClockSercom5(const void *const hw)
{
	//	MCLK_CRITICAL_SECTION_ENTER();
	((Mclk *)hw)->APBDMASK.reg |= MCLK_APBDMASK_SERCOM5;
	//	MCLK_CRITICAL_SECTION_LEAVE();
}



void CSercom::hri_mclk_write_APBDMASK_SERCOM5_bit(const void *const hw, bool value)
{
	uint32_t tmp;
	//	MCLK_CRITICAL_SECTION_ENTER();
	tmp = ((Mclk *)hw)->APBDMASK.reg;
	tmp &= ~MCLK_APBDMASK_SERCOM5;
	tmp |= value << MCLK_APBDMASK_SERCOM5_Pos;
	((Mclk *)hw)->APBDMASK.reg = tmp;
	//	MCLK_CRITICAL_SECTION_LEAVE();
}

void CSercom::GclkWritePCHCTRL_reg(const void *const hw, uint8_t index, uint32_t data)
{
	//	GCLK_CRITICAL_SECTION_ENTER();
	((Gclk *)hw)->PCHCTRL[index].reg = data;
	//	GCLK_CRITICAL_SECTION_LEAVE();
}
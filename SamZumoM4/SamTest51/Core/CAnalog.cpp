/* 
* CAnalog.cpp
*
* Created: 19/05/2020 19:20:29
* Author: philg
*/

#include "Includes.h"


// default constructor
CAnalog::CAnalog()
{
	m_DacEnabled[0] = false;
	m_DacEnabled[1] = false;
} //CAnalog

// default destructor
CAnalog::~CAnalog()
{
} //~CAnalog

void CAnalog::Init()
{
	// Initialize Analog Controller
	// Setting clock
	//set to 1/(1/(48000000/32) * 6) = 250000 SPS
	GCLK->PCHCTRL[ADC0_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
	GCLK->PCHCTRL[ADC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 1 (48Mhz)
	Adc *adcs[] = {ADC0, ADC1};
	for(int i=0; i<2; i++)
	{
		adcs[i]->CTRLA.bit.PRESCALER =	ADC_CTRLA_PRESCALER_DIV32_Val;
		adcs[i]->CTRLB.bit.RESSEL =		ADC_CTRLB_RESSEL_12BIT_Val;

		while( adcs[i]->SYNCBUSY.reg &	ADC_SYNCBUSY_CTRLB );			// wait for sync
		adcs[i]->SAMPCTRL.reg = 5;										// sampling Time Length
		while( adcs[i]->SYNCBUSY.reg &	ADC_SYNCBUSY_SAMPCTRL );		// wait for sync

		adcs[i]->INPUTCTRL.reg =		ADC_INPUTCTRL_MUXNEG_GND;		// No Negative input (Internal Ground)
		while( adcs[i]->SYNCBUSY.reg &	ADC_SYNCBUSY_INPUTCTRL );		// wait for sync
		
		adcs[i]->AVGCTRL.reg =			ADC_AVGCTRL_SAMPLENUM_1 |		// Averaging (see data sheet table in AVGCTRL register description) 1 sample only (no oversampling nor averaging)
		ADC_AVGCTRL_ADJRES(0x0ul);										// Adjusting result by 0

		while( adcs[i]->SYNCBUSY.reg & ADC_SYNCBUSY_AVGCTRL );			// wait for sync
	}
	AnalogReference(AR_DEFAULT) ;									// Analog Reference is  (2.5v)
  }
	 


void CAnalog::InitDAC()
{
	GCLK->PCHCTRL[DAC_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK4_Val | (1 << GCLK_PCHCTRL_CHEN_Pos); //use clock generator 4 (12mhz)
	while (GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN == 0);
  
	while ( DAC->SYNCBUSY.bit.SWRST == 1 ); // Wait for synchronization of registers between the clock domains
	DAC->CTRLA.bit.SWRST = 1;
	while ( DAC->SYNCBUSY.bit.SWRST == 1 ); // Wait for synchronization of registers between the clock domains
  
	DAC->CTRLB.reg = DAC_CTRLB_REFSEL_VREFPU; // TODO: fix this once silicon bug is fixed
  
	//set refresh rates
	DAC->DACCTRL[0].bit.REFRESH = 2;
	DAC->DACCTRL[1].bit.REFRESH = 2;
	
}

/*
 * Internal Reference is at 1.0v
 * External Reference should be between 1v and VDDANA-0.6v=2.7v
 *
 * Warning :  input/output voltage  is 3.3 volts maximum
 */
void CAnalog::AnalogReference(eAnalogReference mode)
{
	while(ADC0->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
	while(ADC1->SYNCBUSY.reg & ADC_SYNCBUSY_REFCTRL); //wait for sync
	
	//TODO: fix gains
	switch (mode)
	{
	case AR_INTERNAL1V0:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V0_Val;									// select 1.0V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					 
		break;
	case AR_INTERNAL1V1:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V1_Val;									// select 1.1V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;  
	break;
		case AR_INTERNAL1V2:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V2_Val;									// select 1V2
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;  
		break;
	case AR_INTERNAL1V25:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_1V25_Val;								// select 1.25V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; 
	break;
		case AR_INTERNAL2V0:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V0_Val;									// select 2.0V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; 
		break;
	case AR_INTERNAL2V2:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V2_Val;									// select 2.2V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// 
		break;
		case AR_INTERNAL2V4:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V4_Val;									// select 2.4V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; 
		break;
	case AR_INTERNAL2V5:
		SUPC->VREF.bit.SEL = SUPC_VREF_SEL_2V5_Val;									// select 2.5V
		SUPC->VREF.bit.VREFOE = 1;													//	Turn on for use with ADC
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val;					// Use SUPC.VREF
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTREF_Val; 
		break;
	case AR_EXTERNAL:
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;					// AREF is jumpered to VCC, so 3.3V
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;
		break;

	case AR_INTERNAL1V65:
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;					// 1/2 VDDANA = 1.65
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;  
		break;
		
	case AR_DEFAULT:
	default:
		ADC0->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;					// VDDANA = 3V3
		ADC1->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val; 
		break;
	}
	
}


float CAnalog::AnalogReadVolts(Adc *adc,EPortType port,uint32_t PortBitNo)
{
	float Value;
	uint32_t AdcUnits = AnalogRead(adc,port,PortBitNo);

	Value = (float)(AdcUnits* 0.00078947368421);
	return( Value);
}


//ADC0 ADC1
uint32_t CAnalog::AnalogRead(Adc *adc,EPortType port,uint32_t PortBitNo)
{
	uint32_t valueRead = 0;
	_EAnalogChannel AnalogChannel;

	PinPeripheral.SetPeripheral(port,PortBitNo, PIO_ANALOG);

	while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_INPUTCTRL ); //wait for sync
	
	switch(PortBitNo)
	{
		case 9:
			if(port == PORTB)
				AnalogChannel = ADC_Channel3;
			break;
		case 4:
			break;
		default:	
			return(0);
			break;
	}
	adc->INPUTCTRL.bit.MUXPOS = AnalogChannel; // Selection for the positive ADC input
  



// Control A
//  Bit 1 ENABLE: Enable  0: The ADC is disabled.  1: The ADC is enabled.
// Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
// value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
// (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.

//  Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
// 	configured. The first conversion after the reference is changed must not be used.

	while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );				//wait for sync
	adc->CTRLA.bit.ENABLE = 0x01;									// Enable ADC
	  
	while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );				// Start conversion  wait for sync
  
	adc->SWTRIG.bit.START = 1;
	 
	adc->INTFLAG.reg = ADC_INTFLAG_RESRDY;						 // Clear the Data Ready flag

	adc->SWTRIG.bit.START = 1;									// Start conversion again, since The first conversion after the reference is changed must not be used.
	  
	while (adc->INTFLAG.bit.RESRDY == 0);							// Waiting for conversion to complete		
	valueRead = adc->RESULT.reg;									//  Store the value

	while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );				//wait for sync
	adc->CTRLA.bit.ENABLE = 0x00;									// Disable ADC
	while( adc->SYNCBUSY.reg & ADC_SYNCBUSY_ENABLE );				//wait for sync
  
	return mapResolution(valueRead, _ADCResolution, _readResolution);
}

void CAnalog::DisableDAC(uint8_t channel)
{
	if(m_DacEnabled[channel])
	{
		m_DacEnabled[channel] = false;
				
		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->CTRLA.bit.ENABLE = 0;										// disable DAC
				
		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->DACCTRL[channel].bit.ENABLE = 0;
				
		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->CTRLA.bit.ENABLE = 1;										// enable DAC
	}
			
	while (DAC->SYNCBUSY.bit.ENABLE);

}





void CAnalog::AnalogWrite(uint32_t Channel, uint32_t value)
{
	uint32_t PortNo;	

	value = mapResolution(value, _writeResolution, _dacResolution);

	if(Channel == 0)
		PortNo = 2;
	else
		PortNo = 5;	

	PinPeripheral.SetPeripheral(PORTA,PortNo, PIO_ANALOG);

	if(!m_DacEnabled[Channel])
	{
		m_DacEnabled[Channel] = true;

		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->CTRLA.bit.ENABLE = 0;     // disable DAC

		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->DACCTRL[Channel].bit.ENABLE = 1;

		while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
		DAC->CTRLA.bit.ENABLE = 1;     // enable DAC

		if(Channel == 0)
		{
			while ( !DAC->STATUS.bit.READY0 );
			while (DAC->SYNCBUSY.bit.DATA0);
			DAC->DATA[0].reg = value;
		}
		else if(Channel == 1)
		{
			while ( !DAC->STATUS.bit.READY1 );
			while (DAC->SYNCBUSY.bit.DATA1);
			DAC->DATA[1].reg = value;
		}

		Core.delayMicroseconds(10000);
	}

	//ERROR!
	while(!DAC->DACCTRL[Channel].bit.ENABLE);

	if(Channel == 0)
	{
		while ( !DAC->STATUS.bit.READY0 );
		while (DAC->SYNCBUSY.bit.DATA0);
		DAC->DATA[0].reg = value;						// DAC on 10 bits.
	}
	else if(Channel == 1)
	{
		while ( !DAC->STATUS.bit.READY1 );
		while (DAC->SYNCBUSY.bit.DATA1);
		DAC->DATA[1].reg = value;  // DAC on 10 bits.
	}
	return;

}

inline uint32_t CAnalog::mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
	if (from == to) 
		return value;
	if (from > to) 
		return value >> (from-to);
	return value << (to-from);
}


void CAnalog::AnalogReadCorrection (Adc *adc,int offset, uint16_t gain)
{
	// Set correction values
	adc->OFFSETCORR.reg = ADC_OFFSETCORR_OFFSETCORR(offset);
	adc->GAINCORR.reg = ADC_GAINCORR_GAINCORR(gain);

	// Enable digital correction logic
	adc->CTRLB.bit.CORREN = 1;
	while(adc->STATUS.bit.ADCBUSY);
}
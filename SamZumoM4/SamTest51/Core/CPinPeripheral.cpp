/* 
* CPinPeripheral.cpp
*
* Created: 14/05/2020 19:20:19
* Author: philg
*/

#include "sam.h"
#include "Defines.h"
#include "CCore.h"
#include "CPinPeripheral.h"



volatile uint32_t TestD;
volatile PortGroup GroupD;
// default constructor
CPinPeripheral::CPinPeripheral()
{
} //CPinPeripheral

// default destructor
CPinPeripheral::~CPinPeripheral()
{
} //~CPinPeripheral



void CPinPeripheral::SetPeripheral(EPortType port ,uint32_t PortNo, EPioType_t ulPeripheral )
{
	uint32_t TempReg;
	
	if ( ulPeripheral == PIO_INPUT )					// Configure pin mode, if requested
		SetPinMode( port,PortNo, INPUT ) ;
	else if ( ulPeripheral == PIO_INPUT_PULLUP )
			SetPinMode(port, PortNo, INPUT_PULLUP ) ;
	else if ( ulPeripheral == PIO_OUTPUT )
				SetPinMode(port, PortNo, OUTPUT ) ;
	else
	{
		if ( PortNo & 1 ) // is pin odd?
		{
			
			TempReg = (PORT->Group[port].PMUX[PortNo >> 1].reg) &  0xF ;		// Get whole current setup for both odd and even pins and remove odd one
			
			PORT->Group[port].PMUX[PortNo >> 1].reg = TempReg|PORT_PMUX_PMUXO( ulPeripheral ) ;	// Set new muxing	
			
			PORT->Group[port].PINCFG[PortNo].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;	// Enable port mux
		}
		else // even pin
		{
			TempReg = (PORT->Group[port].PMUX[PortNo >> 1].reg) &  0xF0 ;
			PORT->Group[port].PMUX[PortNo >> 1].reg = TempReg|PORT_PMUX_PMUXE( ulPeripheral ) ;
			PORT->Group[port].PINCFG[PortNo].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR ; // Enable port mux
		}
	}
	
}

void CPinPeripheral::SetPinMode( EPortType port,uint32_t PortPin, uint32_t Mode )
{
	uint32_t PinMask = (1ul << PortPin);

	switch ( Mode )																				// Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
	{
	case INPUT:
		PORT->Group[port].PINCFG[PortPin].reg=(uint8_t)(PORT_PINCFG_INEN) ;						// Set pin to input mode
		PORT->Group[port].DIRCLR.reg = PinMask ;
		break ;
	case INPUT_PULLUP:
		PORT->Group[port].PINCFG[PortPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;	// Set pin to input mode with pull-up resistor enabled
		PORT->Group[port].DIRCLR.reg = PinMask ;
		PORT->Group[port].OUTSET.reg = PinMask ;												// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
		break ;
	case INPUT_PULLDOWN:
		PORT->Group[port].PINCFG[PortPin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;	// Set pin to input mode with pull-down resistor enabled
		PORT->Group[port].DIRCLR.reg = PinMask ;
		PORT->Group[port].OUTCLR.reg = PinMask ;												// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
		break ;
	case OUTPUT:
		PORT->Group[port].PINCFG[PortPin].reg=(uint8_t)(PORT_PINCFG_INEN) ;						// enable input, to support reading back values, with pullups disabled
		PORT->Group[port].DIRSET.reg = PinMask ;												// Set pin to output mode
		break ;
	default:
		// do nothing
		break ;
	}
}

void CPinPeripheral::SetPin(EPortType port, uint32_t PortPin, bool Value )
{
	uint32_t pinMask = (1ul << PortPin);

	if ( (PORT->Group[port].DIRSET.reg & pinMask) == 0 )
		PORT->Group[port].PINCFG[PortPin].bit.PULLEN = ((Value == LOW) ? 0 : 1) ; // the pin is not an output, disable pull-up if val is LOW, otherwise enable pull-up

	if( Value == LOW )
		PORT->Group[port].OUTCLR.reg = pinMask;
	else
		PORT->Group[port].OUTSET.reg = pinMask;
	return ;
}

void CPinPeripheral::TogglePin(EPortType port, uint32_t PortPin )
{
	uint32_t pinMask = (1ul << PortPin);
	PORT->Group[port].OUTTGL.reg = pinMask;
	
}



int CPinPeripheral::ReadPin(EPortType port, uint32_t PortPin )
{
	if ( (PORT->Group[port].IN.reg & (1ul << PortPin)) != 0 )
		return HIGH ;
	return LOW ;
}







/*
int CPinPeripheral::SetPeripheral( uint32_t ulPin, EPioType ulPeripheral )
{
	// Handle the case the pin isn't usable as PIO
	if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
	{
		return -1 ;
	}

	switch ( ulPeripheral )
	{
		case PIO_DIGITAL:
		case PIO_INPUT:
		case PIO_INPUT_PULLUP:
		case PIO_OUTPUT:
		// Configure pin mode, if requested
		if ( ulPeripheral == PIO_INPUT )
		{
			SetPinMode( ulPin, INPUT ) ;
		}
		else
		{
			if ( ulPeripheral == PIO_INPUT_PULLUP )
			{
				SetPinMode( ulPin, INPUT_PULLUP ) ;
			}
			else
			{
				if ( ulPeripheral == PIO_OUTPUT )
				{
					SetPinMode( ulPin, OUTPUT ) ;
				}
				else
				{
					// PIO_DIGITAL, do we have to do something as all cases are covered?
				}
			}
		}
		break ;

		case PIO_ANALOG:
		case PIO_SERCOM:
		case PIO_SERCOM_ALT:
		case PIO_TIMER:
		case PIO_TIMER_ALT:
		case PIO_EXTINT:
		case PIO_TCC_PDEC:
		case PIO_COM:
		case PIO_SDHC:
		case PIO_I2S:
		case PIO_PCC:
		case PIO_GMAC:
		case PIO_AC_CLK:
		case PIO_CCL:

// 		#if 0
// 		// Is the pio pin in the lower 16 ones?
// 		// The WRCONFIG register allows update of only 16 pin max out of 32
// 		if ( g_APinDescription[ulPin].ulPin < 16 )
// 		{
// 			PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
// 			PORT_WRCONFIG_WRPINCFG |
// 			PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin ) ;
// 		}
// 		else
// 		{
// 			PORT->Group[g_APinDescription[ulPin].ulPort].WRCONFIG.reg = PORT_WRCONFIG_HWSEL |
// 			PORT_WRCONFIG_WRPMUX | PORT_WRCONFIG_PMUXEN | PORT_WRCONFIG_PMUX( ulPeripheral ) |
// 			PORT_WRCONFIG_WRPINCFG |
// 			PORT_WRCONFIG_PINMASK( g_APinDescription[ulPin].ulPin - 16 ) ;
// 		}
// 		#else



		TestD =  g_APinDescription[ulPin].ulPin;
		if ( TestD & 1 ) // is pin odd?
		{
			volatile uint32_t temp ;
		
			
			// Get whole current setup for both odd and even pins and remove odd one
			temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
			// Set new muxing
			PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXO( ulPeripheral ) ;
			// Enable port mux
			PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR;
		}
		else // even pin
		{
			uint32_t temp ;
//			GroupD = PORT->Group[g_APinDescription[ulPin].ulPort];
//			TestD = Group.PMUX[g_APinDescription[ulPin].ulPin >> 1;
			temp = (PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
			PORT->Group[g_APinDescription[ulPin].ulPort].PMUX[g_APinDescription[ulPin].ulPin >> 1].reg = temp|PORT_PMUX_PMUXE( ulPeripheral ) ;
			PORT->Group[g_APinDescription[ulPin].ulPort].PINCFG[g_APinDescription[ulPin].ulPin].reg |= PORT_PINCFG_PMUXEN | PORT_PINCFG_DRVSTR ; // Enable port mux
		}
//		#endif
		break ;

		case PIO_NOT_A_PIN:
		return -1l ;
		break ;
	}

	return 0l ;
}



void CPinPeripheral::SetPinMode( uint32_t ulPin, uint32_t ulMode )
{
	
 	if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )									// Handle the case the pin isn't usable as PIO
 		return ;

	EPortType port = g_APinDescription[ulPin].ulPort;
	uint32_t pin = g_APinDescription[ulPin].ulPin;
	uint32_t pinMask = (1ul << pin);

	switch ( ulMode )																			// Set pin mode according to chapter '22.6.3 I/O Pin Configuration'
	{
	case INPUT:
		PORT->Group[port].PINCFG[pin].reg=(uint8_t)(PORT_PINCFG_INEN) ;							// Set pin to input mode
		PORT->Group[port].DIRCLR.reg = pinMask ;
		break ;
	case INPUT_PULLUP:
		PORT->Group[port].PINCFG[pin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;		// Set pin to input mode with pull-up resistor enabled
		PORT->Group[port].DIRCLR.reg = pinMask ;
		PORT->Group[port].OUTSET.reg = pinMask ;												// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
		break ;
	case INPUT_PULLDOWN:
		PORT->Group[port].PINCFG[pin].reg=(uint8_t)(PORT_PINCFG_INEN|PORT_PINCFG_PULLEN) ;		// Set pin to input mode with pull-down resistor enabled
		PORT->Group[port].DIRCLR.reg = pinMask ;
		PORT->Group[port].OUTCLR.reg = pinMask ;												// Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.6 Data Output Value Clear')
		break ;
	case OUTPUT:
		PORT->Group[port].PINCFG[pin].reg=(uint8_t)(PORT_PINCFG_INEN) ;							// enable input, to support reading back values, with pullups disabled
		PORT->Group[port].DIRSET.reg = pinMask ;												// Set pin to output mode
		break ;
	default:
		// do nothing
		break ;
	}
}

void CPinPeripheral::SetPin( uint32_t ulPin, uint32_t ulVal )
{
	// Handle the case the pin isn't usable as PIO 
	if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
	{
		return ;
	}

	EPortType port = g_APinDescription[ulPin].ulPort;
	uint32_t pin = g_APinDescription[ulPin].ulPin;
	uint32_t pinMask = (1ul << pin);

	if ( (PORT->Group[port].DIRSET.reg & pinMask) == 0 ) 
	{
		// the pin is not an output, disable pull-up if val is LOW, otherwise enable pull-up
		PORT->Group[port].PINCFG[pin].bit.PULLEN = ((ulVal == LOW) ? 0 : 1) ;
	}

	if( ulVal == LOW )
		PORT->Group[port].OUTCLR.reg = pinMask;
	else
		PORT->Group[port].OUTSET.reg = pinMask;

	return ;
}

int CPinPeripheral::ReadPin( uint32_t ulPin )
{
	// Handle the case the pin isn't usable as PIO
	if ( g_APinDescription[ulPin].ulPinType == PIO_NOT_A_PIN )
		return LOW ;

	if ( (PORT->Group[g_APinDescription[ulPin].ulPort].IN.reg & (1ul << g_APinDescription[ulPin].ulPin)) != 0 )
		return HIGH ;

	return LOW ;
}


*/
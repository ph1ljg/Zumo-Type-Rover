/*
* CSpi.cpp
*
* Created: 18/05/2020 11:30:11
* Author: philg
*/

#include "includes.h"


// default constructor
CSpi::CSpi()
{
} //CSpi

// default destructor
CSpi::~CSpi()
{
} //~CSpi


#define SPI_IMODE_NONE   0
#define SPI_IMODE_EXTINT 1
#define SPI_IMODE_GLOBAL 2

const SPISettings DEFAULT_SPI_SETTINGS = SPISettings();

CSpi::CSpi(const void *const   s, EPortType MOSIport,  EPortType MISOport,EPortType CLKport,uint8_t PinNoMISO, uint8_t PinNoSCK, uint8_t PinNoMOSI, SercomSpiTXPad PadTx, SercomRXPad PadRx)
{
	initialized = false;
	m_SercomRegStruct = (Sercom *)s;
	m_MosiPort = MOSIport;
	m_MisoPort = MISOport;
	m_SckPort = CLKport;
	m_PinNoMiso = PinNoMISO;
	m_PinNoSCK = PinNoSCK;
	m_PinNoMosi = PinNoMOSI;

	// SERCOM pads
	m_PadTx=PadTx;
	m_PadRx=PadRx;
}

void CSpi::Init()
{
	if (initialized)
		return;
	interruptMode = SPI_IMODE_NONE;
	interruptSave = 0;
	interruptMask = 0;
	initialized = true;


	PinPeripheral.SetPeripheral(m_MisoPort,m_PinNoMiso, PIO_SERCOM);
	PinPeripheral.SetPeripheral(m_ClkPort,m_PinNoSCK, PIO_SERCOM);
	PinPeripheral.SetPeripheral(m_MosiPort,m_PinNoMosi, PIO_SERCOM);


	DisableSPI();
	ResetSPI();
	SercomCommon.InitClockNVIC(m_SercomRegStruct,SERCOM_CLOCK_SOURCE_48M);
	m_MainClock = SERCOM_CLOCK_SOURCE_48M;
	// master mode
	m_SercomRegStruct->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3) | SERCOM_SPI_CTRLA_DOPO(m_PadTx) |SERCOM_SPI_CTRLA_DIPO(m_PadRx) |	LSBFIRST << SERCOM_SPI_CTRLA_DORD_Pos;
	//Setting the CTRLB register
	m_SercomRegStruct->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(SPI_CHAR_SIZE_8_BITS) |	SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

	while( m_SercomRegStruct->SPI.SYNCBUSY.bit.CTRLB == 1 );

	InitSPIClock(SERCOM_SPI_MODE_0, SPI_SPEED_SLOW);

	EnableSPI();

}

void CSpi::SetSpiClock(uint32_t Speed)
{
	InitSPIClock(SERCOM_SPI_MODE_0,Speed);
}
void CSpi::Config()
{
	DisableSPI();

	InitSPI(m_PadTx, m_PadRx, SPI_CHAR_SIZE_8_BITS, MSBFIRST);
	InitSPIClock(SERCOM_SPI_MODE_0, SPI_CLOCK_RATE);

	EnableSPI();
}


void CSpi::InitSPI(SercomSpiTXPad mosi, SercomRXPad miso, uint8_t charSize, uint8_t dataOrder)
{
	
	CSercom Com;
	ResetSPI();
	Com.InitClockNVIC(m_SercomRegStruct,SERCOM_CLOCK_SOURCE_48M);
	// master mode
	m_SercomRegStruct->SPI.CTRLA.reg = SERCOM_SPI_CTRLA_MODE(0x3) | SERCOM_SPI_CTRLA_DOPO(mosi) |SERCOM_SPI_CTRLA_DIPO(miso) |	dataOrder << SERCOM_SPI_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	m_SercomRegStruct->SPI.CTRLB.reg = SERCOM_SPI_CTRLB_CHSIZE(charSize) |	SERCOM_SPI_CTRLB_RXEN; //Active the SPI receiver.

	while( m_SercomRegStruct->SPI.SYNCBUSY.bit.CTRLB == 1 );
}


void CSpi::EnableSPI()
{
	//Setting the enable bit to 1
	m_SercomRegStruct->SPI.CTRLA.bit.ENABLE = 1;

	while(m_SercomRegStruct->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}
}


void CSpi::ResetSPI()
{
	m_SercomRegStruct->SPI.CTRLA.bit.SWRST = 1;			//Setting the Software Reset bit to 1
	while(m_SercomRegStruct->SPI.CTRLA.bit.SWRST || m_SercomRegStruct->SPI.SYNCBUSY.bit.SWRST);	//Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
}

void CSpi::InitSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate)
{
	CSercom Com;
	int cpha, cpol;

	if((clockMode & (0x1ul)) == 0 )												//Extract data from clockMode
	cpha = 0;
	else
	cpha = 1;

	if((clockMode & (0x2ul)) == 0)
	cpol = 0;
	else
	cpol = 1;
	
	m_SercomRegStruct->SPI.CTRLA.reg |= ( cpha << SERCOM_SPI_CTRLA_CPHA_Pos ) |	( cpol << SERCOM_SPI_CTRLA_CPOL_Pos );			//Setting the CTRLA register
	
	m_SercomRegStruct->SPI.BAUD.reg = Com.CalculateBaudrateSynchronous(baudrate,m_MainClock);		//Synchronous arithmetic
}


void CSpi::DisableSPI()
{
	while(m_SercomRegStruct->SPI.SYNCBUSY.bit.ENABLE)
	{
		//Waiting then enable bit from SYNCBUSY is equal to 0;
	}

	//Setting the enable bit to 0
	m_SercomRegStruct->SPI.CTRLA.bit.ENABLE = 0;
}







void CSpi::end()
{
	ResetSPI();
	initialized = false;
}

void CSpi::SetDataOrderSPI(eDataOrder dataOrder)
{
	DisableSPI();											//Register enable-protected
	m_SercomRegStruct->SPI.CTRLA.bit.DORD = dataOrder;
	EnableSPI();
}

eDataOrder CSpi::GetDataOrder()
{
	return (m_SercomRegStruct->SPI.CTRLA.bit.DORD ? SPI_LSB_FIRST : SPI_MSB_FIRST);
}

void CSpi::SetBaudrate(uint8_t Divider)
{
	CSercom  Com;
	DisableSPI(); // Register is enable-protected
	m_SercomRegStruct->SPI.BAUD.reg = Com.CalculateBaudrateSynchronous(m_MainClock/Divider,m_MainClock);
	EnableSPI();
}

void CSpi::SetClockModeSPI(SercomSpiClockMode clockMode)
{
	int cpha, cpol;
	if((clockMode & (0x1ul)) == 0)
	cpha = 0;
	else
	cpha = 1;

	if((clockMode & (0x2ul)) == 0)
	cpol = 0;
	else
	cpol = 1;
	
	DisableSPI();				//Register enable-protected

	m_SercomRegStruct->SPI.CTRLA.bit.CPOL = cpol;
	m_SercomRegStruct->SPI.CTRLA.bit.CPHA = cpha;

	EnableSPI();
}

uint8_t CSpi::Transfer(uint8_t data)
{
	m_SercomRegStruct->SPI.DATA.bit.DATA = data; // Writing data into Data register

	while(m_SercomRegStruct->SPI.INTFLAG.bit.RXC == 0); // Waiting Complete Reception

	return m_SercomRegStruct->SPI.DATA.bit.DATA;  // Reading data
}

bool CSpi::IsBufferOverflowError()
{
	return m_SercomRegStruct->SPI.STATUS.bit.BUFOVF;
}

bool CSpi::IsDataRegisterEmpty()
{
	return m_SercomRegStruct->SPI.INTFLAG.bit.DRE;			//DRE : Data Register Empty
}


uint16_t CSpi::Transfer16(uint16_t data)
{
	union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } t;

	t.val = data;

	if (GetDataOrder() == SPI_LSB_FIRST)
	{
		t.lsb = Transfer(t.lsb);
		t.msb = Transfer(t.msb);
	}
	else
	{
		t.msb = Transfer(t.msb);
		t.lsb = Transfer(t.lsb);
	}

	return t.val;
}

void CSpi::Transfer(void *buf, size_t count)
{
	uint8_t *buffer = reinterpret_cast<uint8_t *>(buf);
	for (size_t i=0; i<count; i++)
	{
		*buffer = Transfer(*buffer);
		buffer++;
	}
}




void CSpi::BeginTransaction()
{
	if (interruptMode != SPI_IMODE_NONE)
	{
		if (interruptMode & SPI_IMODE_GLOBAL)
		{
			interruptSave = __get_PRIMASK() ? 0 : 1;
			__disable_irq();
		}
		else if (interruptMode & SPI_IMODE_EXTINT)
		EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(interruptMask);
	}
	Config();
}



void CSpi::endTransaction(void)
{
	if (interruptMode != SPI_IMODE_NONE)
	{
		//     if (interruptMode & SPI_IMODE_GLOBAL)
		//     {
		//       if (interruptSave)
		//         interrupts();
		//     }
		//     else if (interruptMode & SPI_IMODE_EXTINT)
		//       EIC->INTENSET.reg = EIC_INTENSET_EXTINT(interruptMask);
	}
}

void CSpi::setBitOrder(BitOrder order)
{
	if (order == LSBFIRST)
	SetDataOrderSPI(SPI_LSB_FIRST);
	else
	SetDataOrderSPI(SPI_MSB_FIRST);
}

void CSpi::SetDataMode(uint8_t mode)
{
	switch (mode)
	{
		case SPI_MODE0:
		SetClockModeSPI(SERCOM_SPI_MODE_0);
		break;
		case SPI_MODE1:
		SetClockModeSPI(SERCOM_SPI_MODE_1);
		break;
		case SPI_MODE2:
		SetClockModeSPI(SERCOM_SPI_MODE_2);
		break;
		case SPI_MODE3:
		SetClockModeSPI(SERCOM_SPI_MODE_3);
		break;
		default:
		break;
	}
}

void CSpi::setClockDivider(uint8_t div)
{
	if(div < SPI_MIN_CLOCK_DIVIDER)
	{
		SetBaudrate(SPI_MIN_CLOCK_DIVIDER);
	}
	else
	{
		SetBaudrate(div);
	}
}


void CSpi::attachInterrupt()
{
	// Should be enableInterrupt()
}

void CSpi::detachInterrupt()
{
	// Should be disableInterrupt()
}



// Set the SPI device's SERCOM clock CORE and SLOW clock sources. SercomClockSource values are an enumeration in CSercom.h.
void CSpi::setClockSource(SercomClockSource Clock)
{
	SercomCommon.InitClockNVIC( (void*) m_SercomRegStruct ,Clock);
	m_MainClock = Clock;
}



// #ifndef interruptsStatus
// #define interruptsStatus() __interruptsStatus()
// static inline unsigned char __interruptsStatus(void) __attribute__((always_inline, unused));
// static inline unsigned char __interruptsStatus(void)
// {
// 	// PRIMASK	0 = no effect	1 = prevents the activation of all exceptions with configurable priority.
// 	return (__get_PRIMASK() ? 0 : 1);
// }
// #endif






void CSpi::usingInterrupt(int PortNumber)
{
	EExt_Interrupts ExtInt;
	uint8_t irestore = __get_PRIMASK() ? 0 : 1;

	if(!GetInterruptNo(PortNumber,ExtInt))
	return;

	
	__disable_irq();
	
	if (ExtInt >= EXTERNAL_NUM_INTERRUPTS)
	interruptMode = SPI_IMODE_GLOBAL;
	else
	{
		interruptMode |= SPI_IMODE_EXTINT;
		interruptMask |= (1 << ExtInt);
	}

	if (irestore)
	__enable_irq();
}



void CSpi::notUsingInterrupt(int PortNumber)
{
	EExt_Interrupts ExtInt;
	if (interruptMode & SPI_IMODE_GLOBAL)
	return; // can't go back, as there is no reference count
	if(!GetInterruptNo(PortNumber,ExtInt))
	return;
	
	uint8_t irestore = __get_PRIMASK() ? 0 : 1;
	__disable_irq();
	
	
	interruptMask &= ~(1 << ExtInt);

	if (interruptMask == 0)
	interruptMode = SPI_IMODE_NONE;

	if (irestore)
	__enable_irq();
}



bool CSpi::GetInterruptNo(int PortNo,EExt_Interrupts &ExtInt)
{
	bool RetVal = false;
	if(PortNo == 17)
	{
		RetVal = true;
		ExtInt = EXTERNAL_INT_1;
	}
	else if(PortNo == 22)
	{
		RetVal = true;
		ExtInt = EXTERNAL_INT_6;
	}
	else if(PortNo == 23)
	{
		RetVal = true;
		ExtInt = EXTERNAL_INT_7;
	}
	return(RetVal);
}





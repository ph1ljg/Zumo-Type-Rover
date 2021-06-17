/*
* CUart.cpp
*
* Created: 14/05/2020 12:23:49
* Author: philg
*/

#include "includes.h"




static volatile uint32_t UartDebug;

//extern CUart Uart0;


unsigned char TxBuffer0[SERIAL0_TX_BUFFER_SIZE];
unsigned char RxBuffer0[SERIAL0_RX_BUFFER_SIZE];
unsigned char TxBuffer1[SERIAL1_TX_BUFFER_SIZE];
unsigned char RxBuffer1[SERIAL1_RX_BUFFER_SIZE];



// default constructor
CUart::CUart()
{
} //CUart

// default destructor
CUart::~CUart()
{
} //~CUart



#define NO_RTS_PIN 255
#define NO_CTS_PIN 255
#define RTS_RX_THRESHOLD 10

uint8_t UartByte = 0;
uint8_t UartByte2 = 0;




void CUart::RxInterruptHandler()
{
	static uint8_t count = 0;
	uint8_t data;
	if (isFrameErrorUART())
	{
		m_SercomRegStruct->USART.DATA.bit.DATA;					// frame error, next byte is invalid so read and discard it
		m_SercomRegStruct->USART.STATUS.bit.FERR = 1;			// clear error
		return;
	}

	if (m_SercomRegStruct->USART.INTFLAG.bit.RXC)
	{
		data = m_SercomRegStruct->USART.DATA.bit.DATA;		if(m_RecieveQue.m_Len < m_RecieveQue.Size)		{			m_RecieveQue.Buffer[m_RecieveQue.m_Head] = data;			m_RecieveQue.m_Head++;			if(m_RecieveQue.m_Head >= m_RecieveQue.Size)			m_RecieveQue.m_Head = 0;			m_RecieveQue.m_Len++;		}
		else
		m_SercomRegStruct->USART.DATA.bit.DATA;   // cannot store so just read
	}
	if (m_SercomRegStruct->USART.INTFLAG.bit.ERROR)								// Is Error
	{
		count++;
		if(m_SercomRegStruct->USART.STATUS.bit.COLL)
		{
			m_CommsError |= COLLISION_ERROR;
		}
		if(m_SercomRegStruct->USART.STATUS.bit.ISF)
		{
			m_CommsError |= SYNC_FIELD_ERROR;
		}
		if(m_SercomRegStruct->USART.STATUS.bit.BUFOVF)
		{
			m_CommsError |= BUFFER_OVERFLOW_ERROR;
		}
		if(m_SercomRegStruct->USART.STATUS.bit.FERR)
		{
			m_CommsError |= FRAME_ERROR;
		}
		if(m_SercomRegStruct->USART.STATUS.bit.PERR)
		{
			m_CommsError |= PARITY_ERROR;
		}

		m_SercomRegStruct->USART.INTFLAG.bit.ERROR = 1;
		m_SercomRegStruct->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;		//clear status
	}
}


void CUart::TxInterruptHandler()
{
	uint8_t data;
	if (m_SercomRegStruct->USART.INTFLAG.bit.DRE)
	{
		if (m_TransmitQue.m_Len > 0)
		{
			data = m_TransmitQue.Buffer[m_TransmitQue.m_Tail];
			m_SercomRegStruct->USART.DATA.reg = (uint16_t)data;
			m_TransmitQue.m_Tail++;
			if(m_TransmitQue.m_Tail >= m_TransmitQue.Size)
			m_TransmitQue.m_Tail = 0;
			m_TransmitQue.m_Len--;
		}
		else
		SetTxDREInterrupt(false);		// disable DataRegister Tx  Interrupt
	}

	if (m_SercomRegStruct->USART.INTFLAG.bit.ERROR)								// Is Error
	{
		m_SercomRegStruct->USART.INTFLAG.bit.ERROR = 1;
		// TODO: if (sercom->isBufferOverflowErrorUART()) ....
		// TODO: if (sercom->isParityErrorUART()) ....
		m_SercomRegStruct->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;		//clear status
	}
	
}


void CUart::IrqHandler()
{
	uint8_t data;

	if (isFrameErrorUART())
	{
		m_SercomRegStruct->USART.DATA.bit.DATA;					// frame error, next byte is invalid so read and discard it
		m_SercomRegStruct->USART.STATUS.bit.FERR = 1;			// clear error
		return;
	}

	if (m_SercomRegStruct->USART.INTFLAG.bit.RXC)
	{
		data = m_SercomRegStruct->USART.DATA.bit.DATA;		if(m_RecieveQue.m_Len < m_RecieveQue.Size)		{			m_RecieveQue.Buffer[m_RecieveQue.m_Head] = data;			m_RecieveQue.m_Head++;			if(m_RecieveQue.m_Head >= m_RecieveQue.Size)			m_RecieveQue.m_Head = 0;			m_RecieveQue.m_Len++;		}
		//		while(!m_SercomRegStruct->USART.INTFLAG.bit.DRE);
		//		m_SercomRegStruct->USART.DATA.reg = (uint16_t)data;
	}
	
	// Tx
	if (m_SercomRegStruct->USART.INTFLAG.bit.DRE)
	{
		if (m_TransmitQue.m_Len > 0)
		{
			data = m_TransmitQue.Buffer[m_TransmitQue.m_Tail];
			m_TransmitQue.m_Tail++;
			if(m_TransmitQue.m_Tail >= m_TransmitQue.Size)
			m_TransmitQue.m_Tail = 0;
			m_TransmitQue.m_Len--;
			if(data != 61)
			printf("fred");
			m_SercomRegStruct->USART.DATA.reg = (uint16_t)data;
		}
		else
		SetTxDREInterrupt(false);		// disable DataRegister Tx  Interrupt
	}

	if (m_SercomRegStruct->USART.INTFLAG.bit.ERROR)								// Is Error
	{
		m_SercomRegStruct->USART.INTFLAG.bit.ERROR = 1;
		// TODO: if (sercom->isBufferOverflowErrorUART()) ....
		// TODO: if (sercom->isParityErrorUART()) ....
		m_SercomRegStruct->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;		//clear status
	}
}






CUart::CUart(const void *const   s, EPortType port,uint8_t PortPinRx, uint8_t PortPinTx, SercomPad_t PadRX, SercomPad_t PadTX,uint8_t UartNo)
{
	m_SercomRegStruct = (Sercom *)s;
	
	m_PortPinRX = PortPinRx;
	m_PortPinTX = PortPinTx;
	m_Port = port;
	m_SercomPadRX = PadRX;
	m_SercomPadTX = PadTX;
	m_UartNo = UartNo;
	m_CommsError =0;
	switch(UartNo)
	{
	case 0:
		m_RecieveQue.Size = SERIAL0_RX_BUFFER_SIZE;
		m_TransmitQue.Size = SERIAL0_TX_BUFFER_SIZE;
		m_TransmitQue.Buffer = TxBuffer0;
		m_RecieveQue.Buffer = RxBuffer0;
		break;
	case 1:
		m_RecieveQue.Size = SERIAL1_RX_BUFFER_SIZE;
		m_TransmitQue.Size = SERIAL1_TX_BUFFER_SIZE;
		m_TransmitQue.Buffer = TxBuffer1;
		m_RecieveQue.Buffer = RxBuffer1;
		break;
	default:
		m_RecieveQue.Size = 0;
		m_TransmitQue.Size = 0;
		m_TransmitQue.Buffer = NULL;
		m_RecieveQue.Buffer = NULL;
		
	}
}


void CUart::Init(unsigned long baudrate)
{
	uint16_t config;
	if(m_UartNo == 1)
		config = SERIAL_8E2;
	else
		config = SERIAL_8N1;
	PinPeripheral.SetPeripheral(m_Port,m_PortPinRX, PIO_SERCOM);
	PinPeripheral.SetPeripheral(m_Port,m_PortPinTX, PIO_SERCOM);

	SercomCommon.InitClockNVIC(m_SercomRegStruct,SERCOM_CLOCK_SOURCE_48M);
	
	InitClock(m_UartNo);
	
	if (!IsSyncing( SERCOM_USART_SYNCBUSY_SWRST))
	{
		uint32_t mode = GetUartMode();
		UartDebug = mode;
		if (GetCTRLAReg( SERCOM_USART_CTRLA_ENABLE))
		{
			ClearCTRLA_ENABLE_bit();
			WaitForSync(SERCOM_USART_SYNCBUSY_ENABLE);
		}
		WriteCTRLAReg( SERCOM_USART_CTRLA_SWRST | mode);
	}
	WaitForSync( SERCOM_USART_SYNCBUSY_SWRST);

	initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baudrate);
	initFrame(extractCharSize(config), LSB_FIRST, extractParity(config), extractNbStopBit(config));
	InitPads(m_SercomPadTX, m_SercomPadRX);
	ClearBuffer(&m_RecieveQue);
	ClearBuffer(&m_TransmitQue);
	Enable();

}


void CUart::DisplayPrintf(const char *fmt,  ... )
{
	char buf[80];
	uint8_t Len;
	uint8_t i;
	va_list ap;

	va_start(ap, fmt);
	Len = vsprintf(buf, fmt, ap);
	va_end(ap);
	for(i=0;i<Len;i++)
	{
		if (buf[i] == '\n')
		WriteChar('\r');
		WriteChar(buf[i]);
	}
}


void CUart::end()
{
	Reset();
	ClearBuffer(&m_RecieveQue);
	ClearBuffer(&m_TransmitQue);
}


void CUart::FlushTx()
{
	while(BufferAvailable(&m_TransmitQue)){}; // wait until TX buffer is empty

	Flush();
}


void CUart::StartTx()
{
	uint8_t Data;
	TxRead(&Data);
	WriteTxUnbuffered(Data);
	SetTxDREInterrupt(true);

}


bool CUart::WriteTxBuffer(uint8_t *Buffer, uint16_t Size,bool StartTx)
{
	uint8_t i;
	for(i=0;i<Size;i++)
	{
		if(!Write(&m_TransmitQue, Buffer[i]))
			return(false);
	}
	if(StartTx)
	{
		SetTxDREInterrupt(true);
	}
	return(true);
}



bool CUart::WriteChar( uint8_t Data)
{
	return(WriteTxBuffer(&Data,1,true));
}

size_t CUart::WriteChar1(const uint8_t data)
{
	if (isDataRegisterEmptyUART() && BufferAvailable(&m_TransmitQue) == 0)
	{
		WriteTxUnbuffered(data);
	}
	else
	{
		// spin lock until a spot opens up in the buffer
		while(IsFull(&m_TransmitQue))
		{
			uint8_t interruptsEnabled = ((__get_PRIMASK() & 0x1) == 0);

			if (interruptsEnabled)
			{
				uint32_t exceptionNumber = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk);

				if (exceptionNumber == 0 ||  NVIC_GetPriority((IRQn_Type)(exceptionNumber - 16)) > SERCOM_NVIC_PRIORITY)
				{
					// no exception or called from an ISR with lower priority,
					// wait for free buffer spot via IRQ
					continue;
				}
			}

			// interrupts are disabled or called from ISR with higher or equal priority than the SERCOM IRQ
			// manually call the UART IRQ handler when the data register is empty
			if (isDataRegisterEmptyUART())
			{
				IrqHandler();
			}
		}

		SetTxDREInterrupt(true);
		Write(&m_TransmitQue, data);
	}
	return 1;
}



bool CUart::IsFull(CirBuffer_t *Buffer)
{
	if(Buffer->m_Len >= Buffer->Size)
		return(true);
	return(false);
}

void CUart::ClearTxBuffer()
{
	ClearBuffer(&m_TransmitQue);
}

void CUart::ClearRxBuffer()
{
	ClearBuffer(&m_RecieveQue);
}

void CUart::ClearBuffer(CirBuffer_t *Buffer)
{
	uint16_t i;
	Buffer->m_Head = 0;
	Buffer->m_Len = 0;
	Buffer->m_Tail = 0;
	for(i=0;i<Buffer->Size;i++)
	Buffer->Buffer[i] =0;
}

uint16_t CUart::Write(CirBuffer_t *InBuffer,unsigned char ch)
{
	if(InBuffer->m_Len < InBuffer->Size)	{		InBuffer->Buffer[InBuffer->m_Head] = ch;		InBuffer->m_Head++;		if(InBuffer->m_Head >= InBuffer->Size)		InBuffer->m_Head = 0;		__disable_irq();		InBuffer->m_Len++;		__enable_irq();		return(true);	}	else	return(false);}



bool CUart::Read(unsigned char *Data)
{
	if(m_RecieveQue.m_Len > 0)
	{
		*Data = m_RecieveQue.Buffer[m_RecieveQue.m_Tail];
		m_RecieveQue.m_Tail++;
		if(m_RecieveQue.m_Tail >= m_RecieveQue.Size)
		m_RecieveQue.m_Tail = 0;
		m_SercomRegStruct->USART.INTENSET.reg |= SERCOM_USART_INTENCLR_RXC;
		__disable_irq();		m_RecieveQue.m_Len--;
		__enable_irq();		m_SercomRegStruct->USART.INTENSET.reg |= SERCOM_USART_INTENSET_RXC;
		
		return(true);
	}
	else
	return(false);
}

bool CUart::TxRead(unsigned char *Data)
{
	if(m_TransmitQue.m_Len > 0)
	{
		*Data = m_TransmitQue.Buffer[m_TransmitQue.m_Tail];
		m_TransmitQue.Buffer[m_TransmitQue.m_Tail] = 0xff;
		m_TransmitQue.m_Tail++;
		if(m_TransmitQue.m_Tail >= m_TransmitQue.Size)
		m_TransmitQue.m_Tail = 0;
		m_SercomRegStruct->USART.INTENSET.reg |= SERCOM_USART_INTENCLR_DRE;
		m_TransmitQue.m_Len--;
		m_SercomRegStruct->USART.INTENSET.reg |= SERCOM_USART_INTENCLR_DRE;
		
		return(true);
	}
	else
	return(false);
}


uint16_t CUart::TxBufferSize()
{
	return( m_TransmitQue.m_Len);
}


uint16_t CUart::TxAvailable()
{
	return(m_TransmitQue.Size - m_TransmitQue.m_Len);
}

uint16_t CUart::Available()
{
	return(BufferAvailable(&m_RecieveQue));
}

uint16_t CUart::BufferAvailable(CirBuffer_t* Buffer)
{
	return(Buffer->m_Len);
}

SercomNumberStopBit CUart::extractNbStopBit(uint16_t config)
{
	switch(config & HARDSER_STOP_BIT_MASK)
	{
		case HARDSER_STOP_BIT_1:
		default:
		return SERCOM_STOP_BIT_1;

		case HARDSER_STOP_BIT_2:
		return SERCOM_STOP_BITS_2;
	}
}

SercomUartCharSize CUart::extractCharSize(uint16_t config)
{
	switch(config & HARDSER_DATA_MASK)
	{
		case HARDSER_DATA_5:
		return UART_CHAR_SIZE_5_BITS;

		case HARDSER_DATA_6:
		return UART_CHAR_SIZE_6_BITS;

		case HARDSER_DATA_7:
		return UART_CHAR_SIZE_7_BITS;

		case HARDSER_DATA_8:
		default:
		return UART_CHAR_SIZE_8_BITS;

	}
}

SercomParityMode CUart::extractParity(uint16_t config)
{
	switch(config & HARDSER_PARITY_MASK)
	{
		case HARDSER_PARITY_NONE:
		default:
		return SERCOM_NO_PARITY;

		case HARDSER_PARITY_EVEN:
		return SERCOM_EVEN_PARITY;

		case HARDSER_PARITY_ODD:
		return SERCOM_ODD_PARITY;
	}
}


void CUart::initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate)
{

	//Setting the CTRLA register
	m_SercomRegStruct->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE(mode) |SERCOM_USART_CTRLA_SAMPR(sampleRate);

	//Setting the Interrupt register
	m_SercomRegStruct->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  SERCOM_USART_INTENSET_ERROR;


	if ( mode == UART_INT_CLOCK )
	{
		uint16_t sampleRateValue;

		if (sampleRate == SAMPLE_RATE_x16)
		sampleRateValue = 16;
		else
		sampleRateValue = 8;

		// Asynchronous fractional mode (Table 24-2 in datasheet)  BAUD = fref / (sampleRateValue * fbaud) (multiply by 8, to calculate fractional piece)
		uint32_t baudTimes8 = (SERCOM_FREQ_REF * 8) / (sampleRateValue * baudrate);

		m_SercomRegStruct->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
		m_SercomRegStruct->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);
	}
}
void CUart::initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits)
{
	//Setting the CTRLA register
	m_SercomRegStruct->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM((parityMode == SERCOM_NO_PARITY ? 0 : 1) ) | dataOrder << SERCOM_USART_CTRLA_DORD_Pos;

	//Setting the CTRLB register
	m_SercomRegStruct->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(charSize) |	nbStopBits << SERCOM_USART_CTRLB_SBMODE_Pos |	(parityMode == SERCOM_NO_PARITY ? 0 : parityMode) <<
	SERCOM_USART_CTRLB_PMODE_Pos; //If no parity use default value
}




void CUart::InitPads(SercomPad_t txPad, SercomPad_t rxPad)
{
	UartDebug = 0;
	//Setting the CTRLA register
	m_SercomRegStruct->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(txPad) |	SERCOM_USART_CTRLA_RXPO(rxPad);

	// Enable Transceiver and Receiver
	UartDebug |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN;
	m_SercomRegStruct->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;
}

void CUart::Reset()
{
	// Start the Software Reset
	m_SercomRegStruct->USART.CTRLA.bit.SWRST = 1 ;

	while ( m_SercomRegStruct->USART.CTRLA.bit.SWRST || m_SercomRegStruct->USART.SYNCBUSY.bit.SWRST )
	{
		// Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
	}
}

void CUart::Enable()
{
	//Setting  the enable bit to 1
	m_SercomRegStruct->USART.CTRLA.bit.ENABLE = 0x1u;

	//Wait for then enable bit from SYNCBUSY is equal to 0;
	while(m_SercomRegStruct->USART.SYNCBUSY.bit.ENABLE);
}

void CUart::Flush()
{
	// Skip checking transmission completion if data register is empty
	if(isDataRegisterEmptyUART())
	return;

	// Wait for transmission to complete
	while(!m_SercomRegStruct->USART.INTFLAG.bit.TXC);
}

void CUart::ClearStatus()
{
	//Reset (with 0) the STATUS register
	m_SercomRegStruct->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
}

bool CUart::AvailableData()
{
	//RXC : Receive Complete
	return m_SercomRegStruct->USART.INTFLAG.bit.RXC;
}

bool CUart::isUARTError()
{
	return m_SercomRegStruct->USART.INTFLAG.bit.ERROR;
}

void CUart::acknowledgeUARTError()
{
	m_SercomRegStruct->USART.INTFLAG.bit.ERROR = 1;
}

bool CUart::isBufferOverflowErrorUART()
{
	//BUFOVF : Buffer Overflow
	return m_SercomRegStruct->USART.STATUS.bit.BUFOVF;
}

bool CUart::isFrameErrorUART()
{
	//FERR : Frame Error
	return m_SercomRegStruct->USART.STATUS.bit.FERR;
}

void CUart::clearFrameErrorUART()
{
	// clear FERR bit writing 1 status bit
	m_SercomRegStruct->USART.STATUS.bit.FERR = 1;
}

bool CUart::isParityErrorUART()
{
	//PERR : Parity Error
	return m_SercomRegStruct->USART.STATUS.bit.PERR;
}

bool CUart::isDataRegisterEmptyUART()
{
	//DRE : Data Register Empty
	return m_SercomRegStruct->USART.INTFLAG.bit.DRE;
}

uint8_t CUart::readDataUART()
{
	return m_SercomRegStruct->USART.DATA.bit.DATA;
}

int CUart::WriteTxUnbuffered(uint8_t *data,uint8_t Size)
{
	for(uint8_t i =0 ;i<Size;i++)
	WriteTxUnbuffered(data[i]);
	
	return(true);
}

int CUart::WriteTxUnbuffered(uint8_t data)
{
	// Wait for data register to be empty
	while(!m_SercomRegStruct->USART.INTFLAG.bit.DRE)
	{
	}

	//Put data into DATA register
	m_SercomRegStruct->USART.DATA.reg = (uint16_t)data;
	return 1;
}

void CUart::SetTxDREInterrupt(bool Set)
{
	if(Set)
	m_SercomRegStruct->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;
	else
	m_SercomRegStruct->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;
}

void CUart::SetTxRXCInterrupt(bool Set)
{
	if(Set)
	m_SercomRegStruct->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC;
	else
	m_SercomRegStruct->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
}

bool CUart::IsSyncing( uint32_t reg)
{
	return (m_SercomRegStruct->USART.SYNCBUSY.reg & reg);
}


void CUart::Write_CTRLA_reg( uint32_t data)
{
	SERCOM_CRITICAL_SECTION_ENTER();
	m_SercomRegStruct->USART.CTRLA.reg = data;
	WaitForSync( SERCOM_USART_SYNCBUSY_SWRST | SERCOM_USART_SYNCBUSY_ENABLE);
	SERCOM_CRITICAL_SECTION_LEAVE();
}

void CUart::WaitForSync( uint32_t reg)
{
	while (m_SercomRegStruct->USART.SYNCBUSY.reg & reg)
	{
	};
}

uint32_t CUart::GetCTRLAReg(uint32_t mask)
{
	uint32_t tmp;
	WaitForSync( SERCOM_USART_SYNCBUSY_SWRST | SERCOM_USART_SYNCBUSY_ENABLE);
	tmp = m_SercomRegStruct->USART.CTRLA.reg;
	tmp &= mask;
	return tmp;
}

uint32_t CUart::GetUartMode()
{
	uint32_t RetVal  = m_SercomRegStruct->USART.CTRLA.reg & SERCOM_USART_CTRLA_MODE_Msk ;
	return(RetVal);
}



void CUart::ClearCTRLA_ENABLE_bit()
{
	SERCOM_CRITICAL_SECTION_ENTER();
	m_SercomRegStruct->USART.CTRLA.reg &= ~SERCOM_USART_CTRLA_ENABLE;
	WaitForSync( SERCOM_USART_SYNCBUSY_SWRST | SERCOM_USART_SYNCBUSY_ENABLE);
	SERCOM_CRITICAL_SECTION_LEAVE();
}

void CUart::WriteCTRLAReg( uint32_t data)
{
	SERCOM_CRITICAL_SECTION_ENTER();
	m_SercomRegStruct->USART.CTRLA.reg = data;
	WaitForSync( SERCOM_USART_SYNCBUSY_SWRST | SERCOM_USART_SYNCBUSY_ENABLE);
	SERCOM_CRITICAL_SECTION_LEAVE();
}

void CUart::InitClock(uint8_t UartNo)
{
	switch(UartNo)
	{
	case 0:
		SercomCommon.GclkWritePCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_CORE, CONF_GCLK_SERCOM5_CORE_UART_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		SercomCommon.GclkWritePCHCTRL_reg(GCLK, SERCOM5_GCLK_ID_SLOW, CONF_GCLK_SERCOM5_SLOW_UART_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		SercomCommon.EnableClock((const void *const)MCLK,5);
		break;
	case 1:
		SercomCommon.GclkWritePCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_CORE, CONF_GCLK_SERCOM3_CORE_UART_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		SercomCommon.GclkWritePCHCTRL_reg(GCLK, SERCOM3_GCLK_ID_SLOW, CONF_GCLK_SERCOM3_SLOW_UART_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
		SercomCommon.EnableClock((const void *const)MCLK,3);
		break;
	}
}


void CUart::Uart_1_InitClock(void)
{
}

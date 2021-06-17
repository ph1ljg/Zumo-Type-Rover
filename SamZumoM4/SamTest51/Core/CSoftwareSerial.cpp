/*
* CSoftwareSerial.cpp
*
* Created: 07/06/2020 13:04:03
* Author: philg
*/

#include "Includes.h"

uint8_t ReceiveBuffer1[PORT_1_RECEIVE_BUFFER_LEN];
uint8_t TransmitBuffer1[PORT_1_TRANSMIT_BUFFER_LEN];

uint8_t ReceiveBuffer2[PORT_2_RECEIVE_BUFFER_LEN];
uint8_t TransmitBuffer2[PORT_2_TRANSMIT_BUFFER_LEN];
// default constructor
CSoftwareSerial::CSoftwareSerial(EPortType port,uint8_t PortPinRx, uint8_t PortPinTx,bool Inverse,bool HalfDuplex,uint8_t PortNo)
{
	m_TxTickCount = 0;
	m_RxTickCount = 0;
	m_TxRegister = 0;
	m_TransmitBitCount = 0;
	m_ActiveIn = false;
	m_ActiveOut = false;
	m_RxRegister = 0;
	m_RxBitCount = -1;
	m_InverseLogic = Inverse;
	m_IsHalfDuplex = HalfDuplex;
	m_PortNo = PortNo;
	m_PortPinRX = PortPinRx;
	m_PortPinTX = PortPinTx;
	m_Port = port;
	if(!m_IsHalfDuplex)
	{
		if(PortPinRx != DIRECTION_INHIBIT)
		m_ActiveIn = true;
		if(PortPinTx != DIRECTION_INHIBIT)
		m_ActiveOut = true;
	}
	else
		SetDirection(RECIEVE);
	
	m_TxInProgress = false;
} //CSoftwareSerial

// default destructor
CSoftwareSerial::~CSoftwareSerial()
{
	delete m_ReciveBuffer.Buffer;
} //~CSoftwareSerial



void CSoftwareSerial::Init(uint32_t BaudRate)
{
	m_Baud = BaudRate;
	//	NVIC_SetPriority(TC3_IRQn, SS_INTERRUPT_PRIORITY);
	m_RxTickCount = 1;
	m_RxBitCount = -1;
	// First write, then set output. If this is done  the other way around, the pin would be output low for a short while before switching to
	// output. Now, it is input with pullup for a short while, which	is fine. With inverse logic, either order is fine.

	if(m_IsHalfDuplex)
		SetDirection(RECIEVE);
	else
	{
		//	PinPeripheral.SetPinMode(m_Port,m_PortPinTX,m_InverseLogic ? INPUT_PULLDOWN : INPUT_PULLUP);
		PinPeripheral.SetPinMode(m_Port,m_PortPinTX,OUTPUT);
		PinPeripheral.SetPin(m_Port,m_PortPinTX,m_InverseLogic ? LOW : HIGH);
		PinPeripheral.SetPinMode(m_Port,m_PortPinRX,m_InverseLogic ? INPUT_PULLDOWN : INPUT_PULLUP);
	}
	
	switch(m_PortNo)
	{
	case 1:
		m_ReciveBuffer.Size = PORT_1_RECEIVE_BUFFER_LEN;
		m_TransmitBuffer.Size = PORT_1_TRANSMIT_BUFFER_LEN;
		m_ReciveBuffer.Buffer = ReceiveBuffer1;
		m_TransmitBuffer.Buffer = TransmitBuffer1;
		ClearBuffer(&m_ReciveBuffer);
		ClearBuffer(&m_TransmitBuffer);
		m_Timer = TC1;
		NVIC_SetPriority(TC1_IRQn, SS_INTERRUPT_PRIORITY_1);
		break;
	case 2:
		m_ReciveBuffer.Buffer = new uint8_t[PORT_2_RECEIVE_BUFFER_LEN];
		m_ReciveBuffer.Size = PORT_2_RECEIVE_BUFFER_LEN;
		m_TransmitBuffer.Size = PORT_2_TRANSMIT_BUFFER_LEN;
		m_ReciveBuffer.Buffer = ReceiveBuffer2;
		m_TransmitBuffer.Buffer = TransmitBuffer2;
		ClearBuffer(&m_ReciveBuffer);
		ClearBuffer(&m_TransmitBuffer);
		m_Timer = TC2;
		NVIC_SetPriority(TC2_IRQn, SS_INTERRUPT_PRIORITY_2);
		break;
	}
	m_ActiveOut = false;
	SetBaud(BaudRate);
}

void CSoftwareSerial::SetInterupt(bool OffOn)
{
	switch(m_PortNo)
	{
	case 1:
		if(OffOn)
			NVIC_EnableIRQ(TC1_IRQn);
		else
			NVIC_DisableIRQ(TC1_IRQn);
		break;
	case 2:
		if(OffOn)
			NVIC_EnableIRQ(TC2_IRQn);
		else
			NVIC_DisableIRQ(TC2_IRQn);
		break;
	default:
		break;
	}

	
}

void CSoftwareSerial::Enable(bool YesNo)
{
	if(YesNo)
		SetInterupt(true);
	else
		SetInterupt(false);
}

void CSoftwareSerial::handle_interrupt()
{
	if (m_ActiveIn)
		recv();
	if (m_ActiveOut)
		send();
}

void TC1_Handler()
{
		DebugSerial.m_Timer->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
		DebugSerial.handle_interrupt();
	
}

void TC2_Handler()
{
	TeleSoftwareSerial.m_Timer->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
	TeleSoftwareSerial.handle_interrupt();
}


void CSoftwareSerial::send()
{
	static enum{CheckForData,TransmitByteN,TransmitByteI,StopBit}TxState = CheckForData;
	bool TXBit;

	if (--m_TxTickCount > 0)
	return;
	
	switch(TxState)
	{
		case CheckForData:
		uint8_t Byte;
		if(m_TransmitBuffer.Len >0)
		{
			Byte= m_TransmitBuffer.Buffer[m_TransmitBuffer.Tail];
			m_TransmitBuffer.Tail++;
			if(m_TransmitBuffer.Tail >= m_TransmitBuffer.Size)
				m_TransmitBuffer.Tail = 0;
			m_TransmitBuffer.Len--;
			m_TxRegister = Byte << 1 | 0x200;			// add start and stop bits.
			m_TransmitBitCount = 0;
			m_TxTickCount = OVERSAMPLE;
			if(m_InverseLogic)
			TxState = TransmitByteI;
			else
			TxState = TransmitByteN;
		}
		else
		{
			if (m_IsHalfDuplex )
			SetDirection(RECIEVE);
			else
			m_ActiveOut = false;
		}
		break;
		case TransmitByteN:
		if (m_TransmitBitCount++ < 10 )
		{
			m_TxInProgress = true;
			PinPeripheral.SetPin(m_Port,m_PortPinTX,m_TxRegister & 1);			// send data (including start and stop bits)
			m_TxRegister >>= 1;
			m_TxTickCount = OVERSAMPLE;
		}
		else
		TxState = StopBit;
		break;

		case TransmitByteI:
		if (m_TransmitBitCount++ < 10 )
		{
			m_TxInProgress = true;
			TXBit = (m_TxRegister &0x01) ^ 1;
			PinPeripheral.SetPin(m_Port,m_PortPinTX,TXBit);			// send data (including start and stop bits)
			m_TxRegister >>= 1;
			m_TxTickCount = OVERSAMPLE;
		}
		else
		TxState = StopBit;
		break;

		case StopBit:
		if (m_TransmitBitCount++ > 10 + OVERSAMPLE*5)
		{
			TxState = CheckForData;
		}
		break;
	}
}


//======================================================
// The receive routine called by the interrupt handler
//======================================================
void CSoftwareSerial::recv()
{
	uint8_t inbit;

	if (--m_RxTickCount > 0)
	return;
	inbit =	PinPeripheral.ReadPin(m_Port, m_PortPinRX);

	if(m_InverseLogic)
	{
		inbit = (~inbit)& 0x01; 
	}
	if (m_RxBitCount == -1)
	{
		// waiting for start bit
		if (inbit)
		m_RxTickCount = 1;
		else
		{
			// got start bit
			m_RxBitCount = 0;
			m_RxTickCount = OVERSAMPLE + 1;
			m_RxRegister = 0;
		}
	}
	else if (m_RxBitCount < 8)
	{
		// data bits
		m_RxRegister >>= 1;
		if (inbit)
		m_RxRegister |= 0x80;
		m_RxBitCount++;
		m_RxTickCount = OVERSAMPLE;
	}
	else
	{
		if (inbit)
		{
			// stop bit read complete add to buffer
			if(m_ReciveBuffer.Len < m_ReciveBuffer.Size)			{//				if(m_InverseLogic)//				m_RxRegister = ~m_RxRegister;
				m_ReciveBuffer.Buffer[m_ReciveBuffer.Head] = m_RxRegister;				m_ReciveBuffer.Head++;				if(m_ReciveBuffer.Head >= m_ReciveBuffer.Size)				m_ReciveBuffer.Head = 0;				m_ReciveBuffer.Len++;			}
			
			else
			m_BufferOverflow = true;
		}
		m_RxTickCount = 1;
		m_RxBitCount = -1;
	}
}




bool  CSoftwareSerial::SetDirection(bool TxRx)
{
	if(TxRx == TRANSMIT)
	{
		PinPeripheral.SetPinMode(m_Port,m_PortPinTX,OUTPUT);
		PinPeripheral.SetPin(m_Port,m_PortPinTX,m_InverseLogic ? LOW : HIGH);
		m_ActiveIn = false;
		m_ActiveOut = true;
	}
	else
	{
		PinPeripheral.SetPinMode(m_Port,m_PortPinTX,m_InverseLogic ? INPUT_PULLDOWN : INPUT_PULLUP);
		m_RxBitCount = -1;
		m_RxTickCount = 2;
		m_ActiveOut = false;
		m_ActiveIn = true;

	}
	return(true);
}


bool CSoftwareSerial::Write(uint8_t* Buffer,uint8_t Size,bool TransmitBuffer)
{
	for(uint8_t i = 0;i<Size;i++)
	{
		if(!WriteBuffer(&m_TransmitBuffer,Buffer[i],TransmitBuffer))
		return(false);
	}
	return(true);
}

bool CSoftwareSerial::WriteBufferBeforeTx(uint8_t* Buffer,uint8_t Size)
{
	while(m_ActiveOut)
	{};						// wait for previous transmit to complete
	for(uint8_t i = 0;i<Size;i++)	{		if(m_TransmitBuffer.Len < m_TransmitBuffer.Size)		{			m_TransmitBuffer.Buffer[m_TransmitBuffer.Head] = Buffer[i];			m_TransmitBuffer.Head++;			if(m_TransmitBuffer.Head >= m_TransmitBuffer.Size)			m_TransmitBuffer.Head = 0;			m_TransmitBuffer.Len++;		}
		else		return(false);	}	if (m_IsHalfDuplex)
	SetDirection(TRANSMIT);
	else
	m_ActiveOut = true;
	return(true);	}

bool CSoftwareSerial::WriteTxBuffer(uint8_t *Buffer, uint16_t Size,bool StartTx)
{
	if(!Write( Buffer,Size,StartTx))
		return(false);
	if (m_IsHalfDuplex)
		SetDirection(TRANSMIT);
	else if(StartTx)
		m_ActiveOut = true;
	return(true);
}

bool CSoftwareSerial::WriteChar( uint8_t Data)
{
	return(WriteBuffer(&m_TransmitBuffer,Data,true));
}



uint16_t CSoftwareSerial::WriteBuffer(SSCirBuffer_t *Buffer, uint8_t ch,bool TransmitBuffer)
{
//	while(m_ActiveOut)
//	{
//	}						// wait for previous transmit to complete
	if(Buffer->Len < Buffer->Size)	{		Buffer->Buffer[Buffer->Head] = ch;		Buffer->Head++;		if(Buffer->Head >= Buffer->Size)		Buffer->Head = 0;		SetInterupt(false);		Buffer->Len++;		SetInterupt(true);	}	else		return(false);	
	if (m_IsHalfDuplex)
		SetDirection(TRANSMIT);
	else 
		if(TransmitBuffer)
			m_ActiveOut = true;
		return(true);}


// Read data from buffer
bool CSoftwareSerial::Read(uint8_t *Data)
{
	if(ReadBuffer(&m_ReciveBuffer,Data))
	return(true);
	return(false);
}

int CSoftwareSerial::Available()
{
	return (m_ReciveBuffer.Len);
}
int CSoftwareSerial::TxAvailable()
{
	return (m_TransmitBuffer.Size - m_TransmitBuffer.Len);
}



void CSoftwareSerial::ClearRxBuffer()
{
	ClearBuffer(&m_ReciveBuffer);
}

void CSoftwareSerial::ClearTxBuffer()
{
	ClearBuffer(&m_TransmitBuffer);
}



uint8_t CSoftwareSerial::Peek(SSCirBuffer_t *Buffer)
{
	return(Buffer->Buffer[Buffer->Tail]);
}







void CSoftwareSerial::SetBaud(uint32_t Baud)
{
	SetInterupt(false);
	if (Baud != 0)
	{
		
		m_Timer->COUNT16.INTENCLR.reg = TC_INTENCLR_OVF;								// disable overflow interrupt

		
		const uint8_t clockID = GCLK_CLKCTRL_IDs[TCC_INST_NUM + m_PortNo];					// TC clock are proceeded by TCC ones
		GCLK->PCHCTRL[clockID].bit.CHEN = false;
		while(GCLK->PCHCTRL[clockID].bit.CHEN) ;
		GCLK->PCHCTRL[clockID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;	// 120MHz startup code programmed
		while(!GCLK->PCHCTRL[clockID].bit.CHEN){} ;
		
		m_Timer->COUNT16.CTRLA.bit.ENABLE = false;										// Stop timer, just in case, to be able to reconfigure it
		while(m_Timer->COUNT16.SYNCBUSY.bit.ENABLE){} ;
		
		m_Timer->COUNT16.CTRLA.bit.SWRST = true;										// Reset timer
		while(m_Timer->COUNT16.SYNCBUSY.bit.SWRST){} ;
		
		m_Timer->COUNT16.WAVE.reg = TC_WAVE_WAVEGEN_MFRQ;								// Wave mode, reset counter on overflow on 0 (I use count down to prevent double buffer use)
		m_Timer->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_PRESCALER_DIV1;
		m_Timer->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_DIR;
		while(m_Timer->COUNT16.SYNCBUSY.bit.CTRLB) ;
		
		m_Timer->COUNT16.COUNT.reg = m_Timer->COUNT16.CC[0].reg = 120000000L / (Baud * OVERSAMPLE);	// Set compare value
		
		m_Timer->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;									// Enable interrupt on compare, reset pending interrupt
		m_Timer->COUNT16.INTENSET.reg = TC_INTENSET_OVF;								// enable overflow interrupt

		m_Timer->COUNT16.CTRLA.bit.ENABLE = true;										// And start timer
		while(m_Timer->COUNT16.SYNCBUSY.bit.ENABLE){} ;

		m_Timer->COUNT16.INTFLAG.reg = TC_INTFLAG_OVF;
		m_Timer->COUNT16.DBGCTRL.bit.DBGRUN = 1;
		SetInterupt(false);
	}
}


bool CSoftwareSerial::IsFull(SSCirBuffer_t *Buffer)
{
	if(Buffer->Len >= Buffer->Size)
	return(true);
	return(false);
}
void CSoftwareSerial::ClearBuffer(SSCirBuffer_t *Buffer)
{
	uint16_t i;
	Buffer->Head = 0;
	Buffer->Len = 0;
	Buffer->Tail = 0;
	for(i=0;i<Buffer->Size;i++)
		Buffer->Buffer[i] =0;
}


bool CSoftwareSerial::ReadBuffer(SSCirBuffer_t *Buffer,unsigned char *Data)
{
	if(Buffer->Len > 0)
	{
		*Data = Buffer->Buffer[Buffer->Tail];
		Buffer->Tail++;
		if(Buffer->Tail >= Buffer->Size)
		Buffer->Tail = 0;
		SetInterupt(false);		Buffer->Len--;
		SetInterupt(true);		return(true);
	}
	else
	return(false);
}

void CSoftwareSerial::TestPorts()
{
	while(1)
	{
		PinPeripheral.SetPinMode(m_Port,m_PortPinRX,OUTPUT);
		PinPeripheral.SetPinMode(m_Port,m_PortPinTX,OUTPUT);

		PinPeripheral.SetPin(m_Port,m_PortPinRX,HIGH);
		PinPeripheral.SetPin(m_Port,m_PortPinRX,LOW);

		PinPeripheral.SetPin(m_Port,m_PortPinTX,HIGH);
		PinPeripheral.SetPin(m_Port,m_PortPinTX,LOW);
		
		Core.delay(500);
	}

}
void CSoftwareSerial::TestTx()
{
	uint8_t Buffer[] = {'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q'};
	while(1)
	{
		WriteTxBuffer(Buffer,16,true);
		Core.delay(100);
	}
}
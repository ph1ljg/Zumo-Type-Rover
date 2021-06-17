/* 
* CCSbusDecoderDecoder.cpp
*
* Created: 16/04/2019 18:26:24
* Author: phil
*/
#include "Includes.h"

// 100000 Baud 
// 8E2 configuration, 
// 1 start bit
// 8 Data bits
// 1 Even parity bit
// 2 Stop bits

// SBUS Protocol
// 1 Header byte 00001111b (0x0F)
// 16 * 11 bit channels -> 22 bytes
// 1 Byte with two digital channels (channel 17 and 18) and "frame lost" and "failsafe" flags
// 1 Footer byte 00000000b (0x00)
// Each byte is composed of 8 bits with IDs as follows [7 6 5 4 3 2 1 0] where bit 0 is the least significant bit. 
// The data of the 16 channels are distributed onto the 22 data bytes starting with the least significant bit of channel 1 as follows (using the notation CHANNEL.BIT_ID):
// data byte 0: [1.7 1.6 1.5 1.4 1.3 1.2 1.1 1.0]
// data byte 1: [2.4 2.3 2.2 2.1 2.0 1.10 1.9 1.8]
// data byte 2: [3.1 3.0 2.10 2.9 2.8 2.7 2.6 2.5]
// data byte 3: ...
// The digital channels and flag bytes is composed as:
// flag byte: [0 0 0 0 failsafe frame_lost ch18 ch17]
// Since the least significant bit is sent first over the serial port, the following bit sequence is transmitted:
// shhhhhhhhpss | s 1.0 1.1 1.2 1.3 1.4 1.5 1.6 1.7 pss | s 1.8 1.9 1.10 2.1 2.2 2.3 2.4 pss | ...

// Each of the 16 channels use values in the range of 192 - 1792 

// 	SBUS frames are 25 bytes long, and always start with
// 	0x0f, but there is no other framing information to
// 	prevent  getting out of sync. All we have are the
// 	timing guarantees
//           
// 	In this case we have read a partial frame, or lost some
// 	bytes. A 25 bytes frame at 100000 baud takes 2.5ms. By
// 	delaying 3.5ms here we will get any remaining bytes for
// 	this frame. We shouldn't get the start of the next frame
// 	as frames happen at most every 7ms


volatile  uint16_t SbusDebug;


// default constructor
CSbusDecoder::CSbusDecoder(CUart* Uart)
{
	m_Uart = Uart;
	PreviousByte = SBUS_FOOTER_1;
	m_ValidPacket = false;
} //CCSbusDecoderDecoder

// default destructor
CSbusDecoder::~CSbusDecoder()
{
} //~CCSbusDecoderDecoder

// interrupt driven
bool CSbusDecoder::ParseInputData(uint16_t ControlData[])
{
	uint8_t InByte;
	uint8_t LastByte;
	static uint8_t ByteCount = 0;
	static enum {Header,Data} ParserState = Header;
	SbusDebug = m_Uart->Available();
	if(SbusDebug  <1)
	{
		return(false);	
	}
	while(m_Uart->Read(&InByte))
	{
		switch(ParserState)
		{
		case Header:
			if ((InByte == SBUS_HEADER) && (LastByte == SBUS_FOOTER_1))
			{
				ParserState = Data;
				ByteCount = 0;
			}
			break;
		case Data:
			if ((ByteCount) < PACKET_SIZE-1)				// strip off the data
			{
				m_SbusPacket[ByteCount] = InByte;
				ByteCount++;

			}
			else			// check the end byte
			{
				ParserState = Header;
				LastByte = InByte;
				if ((InByte == SBUS_FOOTER_1) )
				{
					if(Decode2(ControlData))
					{
						m_ValidPacket = true;
						return(true);
					}
					else
						m_ValidPacket = false;
				}
				else
				if(InByte == SBUS_HEADER && m_SbusPacket[23] == SBUS_FOOTER_1)
						ByteCount = 0;
				else		
					m_ValidPacket = false;
			}
			break;
		default:
			break;
		}
		LastByte = InByte;
	}
	return(false);
}

// 192 - 1792 center 800 3E0
// bool CSbusDecoder::Decode(uint16_t ControlData[])
// {
// 			// 16 channels of 11 bit data
// 	ControlData[0]  = (uint16_t) ((m_SbusPacket[0]    |m_SbusPacket[1] <<8)                     & 0x07FF);
// 	ControlData[1]  = (uint16_t) ((m_SbusPacket[1]>>3 |m_SbusPacket[2] <<5)                     & 0x07FF);
// 	ControlData[2]  = (uint16_t) ((m_SbusPacket[2]>>6 |m_SbusPacket[3] <<2 |m_SbusPacket[4]<<10)	 & 0x07FF);
// 	ControlData[3]  = (uint16_t) ((m_SbusPacket[4]>>1 |m_SbusPacket[5] <<7)                     & 0x07FF);
// 	ControlData[4]  = (uint16_t) ((m_SbusPacket[5]>>4 |m_SbusPacket[6] <<4)                     & 0x07FF);
// 	ControlData[5]  = (uint16_t) ((m_SbusPacket[6]>>7 |m_SbusPacket[7] <<1 |m_SbusPacket[8]<<9) 	 & 0x07FF);
// 	ControlData[6]  = (uint16_t) ((m_SbusPacket[8]>>2 |m_SbusPacket[9] <<6)                     & 0x07FF);
// 	ControlData[7]  = (uint16_t) ((m_SbusPacket[9]>>5 |m_SbusPacket[10]<<3)                     & 0x07FF);
// 	ControlData[8]  = (uint16_t) ((m_SbusPacket[11]   |m_SbusPacket[12]<<8)                     & 0x07FF);
// 	ControlData[9]  = (uint16_t) ((m_SbusPacket[12]>>3|m_SbusPacket[13]<<5)                     & 0x07FF);
// 	ControlData[10] = (uint16_t) ((m_SbusPacket[13]>>6|m_SbusPacket[14]<<2 |m_SbusPacket[15]<<10) & 0x07FF);
// 	ControlData[11] = (uint16_t) ((m_SbusPacket[15]>>1|m_SbusPacket[16]<<7)                     & 0x07FF);
// 	ControlData[12] = (uint16_t) ((m_SbusPacket[16]>>4|m_SbusPacket[17]<<4)                     & 0x07FF);
// 	ControlData[13] = (uint16_t) ((m_SbusPacket[17]>>7|m_SbusPacket[18]<<1 |m_SbusPacket[19]<<9)  & 0x07FF);
// 	ControlData[14] = (uint16_t) ((m_SbusPacket[19]>>2|m_SbusPacket[20]<<6)                     & 0x07FF);
// 	ControlData[15] = (uint16_t) ((m_SbusPacket[20]>>5|m_SbusPacket[21]<<3)                     & 0x07FF);
// 		
// 	// count lost frames
// 	if (m_SbusPacket[22] & SBUS_LOST_FRAME)
// 		m_LostFrame = true;
// 	else
// 		m_LostFrame = false;
// 
// 	// failsafe state
// 	if (m_SbusPacket[22] & SBUS_FAILSAFE)
// 		m_FailSafe = true;
// 	else
// 	{
// 		return(true);
// 		m_FailSafe = false;
// 	}
// 	return(false);
// }
// 
// 
// 192 - 1792 center 800 3E0
bool CSbusDecoder::Decode2(uint16_t ControlData[])
{
	const Channels11Bit_t* channels = (const Channels11Bit_t*)m_SbusPacket;
	ControlData[0] = CHANNEL_SCALE(channels->ch0);
	ControlData[1] = CHANNEL_SCALE(channels->ch1);
	ControlData[2] = CHANNEL_SCALE(channels->ch2);
	ControlData[3] = CHANNEL_SCALE(channels->ch3);
	ControlData[4] = CHANNEL_SCALE(channels->ch4);
	ControlData[5] = CHANNEL_SCALE(channels->ch5);
	ControlData[6] = CHANNEL_SCALE(channels->ch6);
	ControlData[7] = CHANNEL_SCALE(channels->ch7);
	ControlData[8] = CHANNEL_SCALE(channels->ch8);
	ControlData[9] = CHANNEL_SCALE(channels->ch9);
	ControlData[10] = CHANNEL_SCALE(channels->ch10);
	ControlData[11] = CHANNEL_SCALE(channels->ch11);
	ControlData[12] = CHANNEL_SCALE(channels->ch12);
	ControlData[13] = CHANNEL_SCALE(channels->ch13);
	ControlData[14] = CHANNEL_SCALE(channels->ch14);
	ControlData[15] = CHANNEL_SCALE(channels->ch15);

	
	m_LostFrame = false;
	m_FailSafe = false;
	
	// failsafe state
	 if (m_SbusPacket[SBUS_FLAGS_BYTE] & SBUS_FAILSAFE_BIT)	
	{																												
		m_FailSafe = true;
		m_LostFrame = true;
		return(false);
	}
	 
	 else if (m_SbusPacket[SBUS_FLAGS_BYTE] & SBUS_FRAMELOST_BIT) // count lost frames
	{																														//This flag indicates a skipped frame only, not a total link loss! Handling  as fail-safe greatly reduces the 
		m_LostFrame = true;
		return(false);																						//reliability and range of the radio link,* e.g. by prematurely issuing return home
	}
	return(true);

}





void CSbusDecoder::Init()
{
}

//===================================================================================
// Byte[23]: 
// Bit 7: digital channel 17 (0x80)
// Bit 6: digital channel 18 (0x40)
// Bit 5: frame lost (0x20)
// Bit 4: failsafe activated (0x10)
// Bit 0 - 3: n/a
// Byte[24]: SBUS End Byte, 0x0
//===================================================================================
// bool CSbusDecoder::Update() 
// {
// 	static uint16_t Count_1 = 0;
// 	m_FailSafe = true;
// 	if(UpdateChannels())
// 	{			// 16 channels of 11 bit data
// 		m_RcChannels[0]  = (uint16_t) ((m_SbusPacket[0]    |m_SbusPacket[1] <<8)                     & 0x07FF);
// 		m_RcChannels[1]  = (uint16_t) ((m_SbusPacket[1]>>3 |m_SbusPacket[2] <<5)                     & 0x07FF);
// 		m_RcChannels[2]  = (uint16_t) ((m_SbusPacket[2]>>6 |m_SbusPacket[3] <<2 |m_SbusPacket[4]<<10)	 & 0x07FF);
// 		m_RcChannels[3]  = (uint16_t) ((m_SbusPacket[4]>>1 |m_SbusPacket[5] <<7)                     & 0x07FF);
// 		m_RcChannels[4]  = (uint16_t) ((m_SbusPacket[5]>>4 |m_SbusPacket[6] <<4)                     & 0x07FF);
// 		m_RcChannels[5]  = (uint16_t) ((m_SbusPacket[6]>>7 |m_SbusPacket[7] <<1 |m_SbusPacket[8]<<9) 	 & 0x07FF);
// 		m_RcChannels[6]  = (uint16_t) ((m_SbusPacket[8]>>2 |m_SbusPacket[9] <<6)                     & 0x07FF);
// 		m_RcChannels[7]  = (uint16_t) ((m_SbusPacket[9]>>5 |m_SbusPacket[10]<<3)                     & 0x07FF);
// 		m_RcChannels[8]  = (uint16_t) ((m_SbusPacket[11]   |m_SbusPacket[12]<<8)                     & 0x07FF);
// 		m_RcChannels[9]  = (uint16_t) ((m_SbusPacket[12]>>3|m_SbusPacket[13]<<5)                     & 0x07FF);
// 		m_RcChannels[10] = (uint16_t) ((m_SbusPacket[13]>>6|m_SbusPacket[14]<<2 |m_SbusPacket[15]<<10) & 0x07FF);
// 		m_RcChannels[11] = (uint16_t) ((m_SbusPacket[15]>>1|m_SbusPacket[16]<<7)                     & 0x07FF);
// 		m_RcChannels[12] = (uint16_t) ((m_SbusPacket[16]>>4|m_SbusPacket[17]<<4)                     & 0x07FF);
// 		m_RcChannels[13] = (uint16_t) ((m_SbusPacket[17]>>7|m_SbusPacket[18]<<1 |m_SbusPacket[19]<<9)  & 0x07FF);
// 		m_RcChannels[14] = (uint16_t) ((m_SbusPacket[19]>>2|m_SbusPacket[20]<<6)                     & 0x07FF);
// 		m_RcChannels[15] = (uint16_t) ((m_SbusPacket[20]>>5|m_SbusPacket[21]<<3)                     & 0x07FF);
// 		
// 		// count lost frames
// 		if (m_SbusPacket[22] & SBUS_LOST_FRAME) 
// 			m_LostFrame = true;
// 		else 
// 			m_LostFrame = false;
// 
// 		// failsafe state
// 		if (m_SbusPacket[22] & SBUS_FAILSAFE)
// 		{ 
// 			m_FailSafe = true;
// 			if(++Count_1 > 15)	
// 				Count_1 = 15;
// 			else
// 				Count_1++;
// 		}
// 		else
// 		{
// 			m_FailSafe = false;
// 			if(Count_1 >0)
// 				Count_1--;
// 		}
// 	} 
// 
// 	if(Count_1 >12)
// 		return false;
// 	return(true);	
// }
// 


// bool CSbusDecoder::UpdateChannels()
// {
// 	uint8_t i;
// 	bool RetVal = false;
// //	m_Uart->US_IDR |= US_INTERRUPT_RXRDY;
// 	if(m_ValidPacket)
// 	{
// 		for(i=0;i<PACKET_SIZE;i++)
// 		{
// 			m_SbusPacket[i] = InterSbusPacket[i];
// 			InterSbusPacket[i] = 0xff;
// 		}
// 		RetVal = true;	
// 	}
// //	m_Uart->US_IER |= US_INTERRUPT_RXRDY;
// 	return(RetVal);
// }
// 



 void CSbusDecoder::Test()
{
	uint16_t channels[] {800,800,800,800,800,800,800,800,800,800,800,800,800,192,800,800};
	uint8_t i;
	uint16_t packet[25];
	CMyMath Math;
	for(i=0;i<16;i++)
	{
		channels[i] = Math.Map(channels[i],1000,2000,192,1792);			// convert to sbus values
	}
	
	// CSbusDecoder header
	m_SbusPacket[0] =	SBUS_HEADER;								// assemble the CSbusDecoder packet 
	m_SbusPacket[1] =	(uint8_t) ((channels[0] & 0x07FF));
	m_SbusPacket[2] =	(uint8_t) ((channels[0] & 0x07FF)>>8 | (channels[1] & 0x07FF)<<3);
	m_SbusPacket[3] =	(uint8_t) ((channels[1] & 0x07FF)>>5 | (channels[2] & 0x07FF)<<6);
	m_SbusPacket[4] = (uint8_t) ((channels[2] & 0x07FF)>>2);
	m_SbusPacket[5] = (uint8_t) ((channels[2] & 0x07FF)>>10 | (channels[3] & 0x07FF)<<1);
	m_SbusPacket[6] = (uint8_t) ((channels[3] & 0x07FF)>>7 | (channels[4] & 0x07FF)<<4);
	m_SbusPacket[7] = (uint8_t) ((channels[4] & 0x07FF)>>4 | (channels[5] & 0x07FF)<<7);
	m_SbusPacket[8] = (uint8_t) ((channels[5] & 0x07FF)>>1);
	m_SbusPacket[9] = (uint8_t) ((channels[5] & 0x07FF)>>9 | (channels[6] & 0x07FF)<<2);
	m_SbusPacket[10]= (uint8_t) ((channels[6] & 0x07FF)>>6 | (channels[7] & 0x07FF)<<5);
	m_SbusPacket[11]= (uint8_t) ((channels[7] & 0x07FF)>>3);
	m_SbusPacket[12]= (uint8_t) ((channels[8] & 0x07FF));
	m_SbusPacket[13]= (uint8_t) ((channels[8] & 0x07FF)>>8 | (channels[9] & 0x07FF)<<3);
	m_SbusPacket[14]= (uint8_t) ((channels[9] & 0x07FF)>>5 | (channels[10] & 0x07FF)<<6);
	m_SbusPacket[15]= (uint8_t) ((channels[10] & 0x07FF)>>2);
	m_SbusPacket[16]= (uint8_t) ((channels[10] & 0x07FF)>>10 | (channels[11] & 0x07FF)<<1);
	m_SbusPacket[17]= (uint8_t) ((channels[11] & 0x07FF)>>7 | (channels[12] & 0x07FF)<<4);
	m_SbusPacket[18]= (uint8_t) ((channels[12] & 0x07FF)>>4 | (channels[13] & 0x07FF)<<7);
	m_SbusPacket[19]= (uint8_t) ((channels[13] & 0x07FF)>>1);
	m_SbusPacket[20]= (uint8_t) ((channels[13] & 0x07FF)>>9 | (channels[14] & 0x07FF)<<2);
	m_SbusPacket[21]= (uint8_t) ((channels[14] & 0x07FF)>>6 | (channels[15] & 0x07FF)<<5);
	m_SbusPacket[22]= (uint8_t) ((channels[15] & 0x07FF)>>3);
	m_SbusPacket[23] = 0x00;									// flags
	m_SbusPacket[24] = SBUS_FOOTER_1;
	
	Decode(packet);
	//m_Uart->WriteTxBuffer(packet,25,true);

}










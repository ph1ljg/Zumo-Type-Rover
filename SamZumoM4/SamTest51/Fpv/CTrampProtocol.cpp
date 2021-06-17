/* 
* CVtx.cpp
*
* Created: 02/05/2019 13:59:30
* Author: phil
*/

#include "Includes.h"


// default constructor
CTrampProtocol::CTrampProtocol(CSoftwareSerial *Serial)
{
	m_Serial = Serial;
} //CVtx

// default destructor
CTrampProtocol::~CTrampProtocol()
{
} //~CVtx

void CTrampProtocol::Init()
{
	//TrampSoftwareSerial.Init(9600);
}

// returns completed response code or 0
char CTrampProtocol::trampReceive()
{
	uint8_t InByte;
	while (m_Serial->Read(&InByte)) 
	{
		m_ResponseBuffer[m_ReceivePosition++] = InByte;

		switch (trampReceiveState) 
		{
		case S_WAIT_LEN:
			if (InByte == 0x0F) 
			{
				trampReceiveState = S_WAIT_CODE;
			} 
			else 
			{
				m_ReceivePosition = 0;
			}
			break;

		case S_WAIT_CODE:
			if (trampIsValidResponseCode(InByte)) 
			{
				trampReceiveState = S_DATA;
			} 
			else 
			{
				trampResetReceiver();
			}
			break;

		case S_DATA:
			if (m_ReceivePosition == 16) 
			{
				uint8_t cksum = trampChecksum(m_ResponseBuffer);

				trampResetReceiver();

				if ((m_ResponseBuffer[14] == cksum) && (m_ResponseBuffer[15] == 0)) 
				{
					return trampHandleResponse();
				}
			}
			break;
		default:
			trampResetReceiver();
			break;
		}
	}

	return 0;
}

void CTrampProtocol::trampSetPitMode(uint8_t onoff)
{
    trampCmdU16('I', onoff ? 0 : 1);
}


void CTrampProtocol::trampResetReceiver(void)
{
	trampReceiveState = S_WAIT_LEN;
	m_ReceivePosition = 0;
}


// returns completed response code
char CTrampProtocol::trampHandleResponse(void)
{
	const uint8_t respCode = m_ResponseBuffer[1];

	switch (respCode) 
	{
	case 'r':
		{
			const uint16_t min_freq = m_ResponseBuffer[2]|(m_ResponseBuffer[3] << 8);
			if (min_freq != 0) 
			{
				trampRFFreqMin = min_freq;
				trampRFFreqMax = m_ResponseBuffer[4]|(m_ResponseBuffer[5] << 8);
				trampRFPowerMax = m_ResponseBuffer[6]|(m_ResponseBuffer[7] << 8);
				m_DataReceived = true;
				return('r');
			}
		}
		break;
	case 'v':
		{
			const uint16_t freq = m_ResponseBuffer[2]|(m_ResponseBuffer[3] << 8);
			if (freq != 0) 
			{
				trampCurFreq = freq;
				trampConfiguredPower = m_ResponseBuffer[4]|(m_ResponseBuffer[5] << 8);
				trampPitMode = m_ResponseBuffer[7];
				trampPower = m_ResponseBuffer[8]|(m_ResponseBuffer[9] << 8);


// 				if (trampConfFreq == 0)  
// 					trampConfFreq  = trampCurFreq;
// 				if (trampConfPower == 0) 
// 					trampConfPower = trampPower;
				m_DataReceived =true;
				return('v');
			}

		}
		break;

		case 's':
		{
			const uint16_t temp = (int16_t)(m_ResponseBuffer[6]|(m_ResponseBuffer[7] << 8));
			if (temp != 0) 
			{
				trampTemperature = temp;
				m_DataReceived = true;
				return('s');
			}
		}
		break;
	}

	return 0;
}



uint8_t CTrampProtocol::trampChecksum(uint8_t *trampBuf)
{
	uint8_t cksum = 0;

	for (int i = 1 ; i < 14 ; i++) 
	{
		cksum += trampBuf[i];
	}

	return cksum;
}

void CTrampProtocol::trampCmdU16(uint8_t cmd, uint16_t param)
{
	memset(m_RequestBuffer, 0, sizeof(m_RequestBuffer));
	m_RequestBuffer[0] = 15;
	m_RequestBuffer[1] = cmd;
	m_RequestBuffer[2] = param & 0xff;
	m_RequestBuffer[3] = (param >> 8) & 0xff;
	m_RequestBuffer[14] = trampChecksum(m_RequestBuffer);
	m_Serial->WriteBufferBeforeTx( m_RequestBuffer, 16);
	Core.delay(1000);
}



void CTrampProtocol::trampSendFreq(uint16_t freq)
{
	if (freq != trampCurFreq)
	{
		trampCmdU16('F', freq);
	}
}


bool CTrampProtocol::trampIsValidResponseCode(uint8_t code)
{
	if (code == 'r' || code == 'v' || code == 's') 
		return true;
	else 
		return false;
}

void CTrampProtocol::GetReceiveData()
{
	Core.delay(60);
	while(1)
	{
		if(trampReceive() == 0)
			 break;
	}	
}


bool CTrampProtocol::trampQueryR(void)
{
	m_DataReceived = false;
	trampQuery('r');
	GetReceiveData();
	return(m_DataReceived);
}

bool CTrampProtocol::trampQueryV(void)
{
	m_DataReceived = false;
	trampQuery('v');
	GetReceiveData();
	return(m_DataReceived);
}

bool CTrampProtocol::trampQueryI(void)
{
	m_DataReceived = false;
	trampQuery('I');
	GetReceiveData();
	return(m_DataReceived);
}




bool CTrampProtocol::trampQueryS(void)
{
	m_DataReceived = false;
	trampQuery('s');
	GetReceiveData();
	return(m_DataReceived);
}

void CTrampProtocol::trampQuery(uint8_t cmd)
{
	trampResetReceiver();
	trampCmdU16(cmd, 0);
}

void CTrampProtocol::trampSendRFPower(uint16_t level)
{
	trampCmdU16('P', level);
}



void CTrampProtocol::Test()
{
	m_Serial->Init(9600);
	trampQueryR();
	trampQueryV();
	trampQueryS();
	trampSendRFPower(25);
	trampQueryR();
	trampSendRFPower(100);
	trampQueryR();

}







/* 
* CSeeedLidar.cpp
*
* Created: 17/06/2019 17:49:34
* Author: phil
*/
#include "Includes.h"
#include "CTfMini.h"





// default constructor
CTFMini::CTFMini()
{
	m_MeasurementValid = false;
} //CTFMiniLidar

// default destructor
CTFMini::~CTFMini()
{
} //~CTFminiLidar
unsigned char dta[100];
unsigned char len = 0;



void CTFMini::Init()
{
	//SetDefault();
	GetUpdateDistanceI2c();
}


void CTFMini::UpDate()
{
	int NumMeasurementAttempts = 0;
	m_MeasurementValid = false;
	
	while (!UpdateDistance())  
	{
		NumMeasurementAttempts += 1;

		if (NumMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS) 
		{
			  m_ErrorState = ERROR_SERIAL_TOOMANYTRIES;
			  m_Distance = 0xffff;
			  m_Strength = 0xffff;
			  return;
	   }
	}

	if (m_ErrorState == MEASUREMENT_OK) 
  		m_MeasurementValid = true;
}


bool CTFMini::SetDefault()
{
	uint8_t Buffer[7];
	Buffer[0]= 0x00;
	Buffer[1]= 0x70;
	Buffer[2]= 0x02;

	if(!I2c.WriteBytes(TFMINI_ADDRESS,Buffer,3))
	{
//		printf("%d",Buffer[1]);
		return(true);
	}
	
	Buffer[0]= 0x00;
	Buffer[1]= 0x27;
	Buffer[2]= 0x00;

	if(!I2c.WriteBytes(TFMINI_ADDRESS,Buffer,3))
	{
//		printf("%d",Buffer[1]);
		return(true);
	}

	return(false);
}


bool CTFMini::GetUpdateDistanceI2c()
{
	uint8_t Buffer[10] = {0x59,0x59, 0X00, 0x00,0x00,0x00,0x00,0x00,0x00};
	
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,9);	
	I2c.ReadBytes(TFMINPLUS_ADDRESS,m_ResponseFrame.Buffer,sizeof(TfMiniFrame_t));
	if(ValidateChecksum(m_ResponseFrame.Buffer,sizeof(TfMiniFrame_t)))
	{
		m_Results.Distance = m_ResponseFrame.Response.Distance;
		m_Results.Strength = m_ResponseFrame.Response.Strength;
		m_Results.MeasurementValid = true;
		return(true);
	}
	else
		m_Results.MeasurementValid = false;

	return(false);
}


bool CTFMini::ValidateChecksum( uint8_t dataBuffer[], size_t length) 
{
	uint8_t sum = GenerateChecksum(dataBuffer, length- 1);
	uint8_t checksum = dataBuffer[length-1];
	if (sum == checksum) 
		return true;
	else 
		return(false);
}



uint8_t CTFMini::GenerateChecksum(const uint8_t buffer[], size_t length) 
{
	uint16_t sum = 0x0000;
	for (uint8_t i = 0; i < length; i++) 
		sum += buffer[i];

	return (uint8_t)sum;
}




bool CTFMini::UpdateDistance()
{
#ifdef TF_SERIAL
	static enum {Header_1,Header_2,GetData}RxState = Header_1;
	static uint8_t Index = 0;
	static uint8_t RxBuffer[7];
	uint8_t i = 0;
	uint8_t InByte;
	uint8_t checksum = 0;
	bool RetVal = false;  
	while(Serial.Available()) 
	{
		Serial.Read(&InByte);
		 
		switch(RxState)
		{
			case Header_1:
			if(InByte == 0x59)
				RxState = Header_2;
			break;
		case Header_2:	
			if(InByte == 0x59)
			{
				RxState = GetData;
				memset(RxBuffer,0,7);
				Index = 0;
			}
			else
			{
				RxState = Header_1;
			}
			break;
		case GetData:	
			if(Index < TFMINI_FRAME_SIZE-1)
			{
				RxBuffer[Index++] = InByte;
				break;
			}
			else
			{
				Index =0;
				RxState = Header_1;
				checksum = 0x59 + 0x59;
				for(i = 0; i < TFMINI_FRAME_SIZE-2; i++)
				{
					checksum += RxBuffer[i];
				}
				if(InByte == checksum)
				{
					m_Distance =(uint16_t) (RxBuffer[1] << 8) + RxBuffer[0];
					m_Strength = (uint16_t)(RxBuffer[3] << 8) + RxBuffer[2];
					m_Quality = RxBuffer[5];
					m_ErrorState = MEASUREMENT_OK;
					return(true);
				}
				else
				{
					m_ErrorState = ERROR_SERIAL_BADCHECKSUM;
					ErrCount++;
				}
			}	
		}
	}
#endif
	 return(false);
}


// Set the TF Mini into the correct measurement mode

void CTFMini::SetStandardOutputMode() 
{
#ifdef TF_SERIAL
	// Set to "standard" output mode (this is found in the debug documents)

	Serial.Sendchar((uint8_t)0x42);

	Serial.Sendchar((uint8_t)0x57);

	Serial.Sendchar((uint8_t)0x02);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x01);

	Serial.Sendchar((uint8_t)0x06);
#endif

}



// Set configuration mode

void CTFMini::SetConfigMode() 
{

#ifdef TF_SERIAL
	// advanced parameter configuration mode

	Serial.Sendchar((uint8_t)0x42);

	Serial.Sendchar((uint8_t)0x57);

	Serial.Sendchar((uint8_t)0x02);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x01);

	Serial.Sendchar((uint8_t)0x02);
#endif

}



// Set single scan mode (external trigger)

void CTFMini::SetSingleScanMode() 
{
#ifdef TF_SERIAL

	SetConfigMode();

	// setting trigger source to external

	Serial.Sendchar((uint8_t)0x42);

	Serial.Sendchar((uint8_t)0x57);

	Serial.Sendchar((uint8_t)0x02);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x40);
#endif
}



// Send external trigger

void CTFMini::ExternalTrigger() 
{
#ifdef TF_SERIAL

	SetConfigMode();

	// send trigger

	Serial.Sendchar((uint8_t)0x42);

	Serial.Sendchar((uint8_t)0x57);

	Serial.Sendchar((uint8_t)0x02);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x00);

	Serial.Sendchar((uint8_t)0x41);
#endif
}
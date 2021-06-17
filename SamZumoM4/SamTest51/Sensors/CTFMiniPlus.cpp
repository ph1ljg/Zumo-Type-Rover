/* 
* CTFMiniPlus.cpp
*
* Created: 13/07/2020 18:55:38
* Author: philg
*/

#include "Includes.h"

// default constructor
CTFMiniPlus::CTFMiniPlus()
{
	NumMeasurementAttempts =0;
} //CTFMiniPlus

// default destructor
CTFMiniPlus::~CTFMiniPlus()
{
} //~CTFMiniPlus

volatile uint16_t ErrCount =0;


void CTFMiniPlus::Init()
{
	
	TfminiPlusData_t Data;
	SetOutputFormat(0x066A); 
	
	UpdateDistance();
}

void CTFMiniPlus::UpDate()
{
	int NumMeasurementAttempts = 0;
	m_Results.MeasurementValid = false;
	
	while (!UpdateDistance())
	{
		NumMeasurementAttempts += 1;

		if (NumMeasurementAttempts > TFMINI_MAX_MEASUREMENT_ATTEMPTS)
		{
			m_Results.Distance = 0xffff;
			m_Results.Strength = 0xffff;
			return;
		}
	}

}


bool CTFMiniPlus::GetRange(TfminiPlusData_t &Data)
{
	uint8_t Buffer[5] = {CMD_FRAME_MARKER, 0X05, 0x00,0x01,0x60};
	
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,m_ResponseFrame.Buffer,sizeof(TfMiniPlusFrame_t));
	if(ValidateChecksum(m_ResponseFrame.Buffer,sizeof(TfMiniPlusFrame_t)))
	{
		Data.Distance = m_ResponseFrame.Response.Distance*10;
		Data.Strength = m_ResponseFrame.Response.Strength;
		Data.MeasurementValid = true;
		Data.Temperature = m_ResponseFrame.Response.Temp;
		return(true);
	}
	else
	Data.MeasurementValid = false;

	return(false);
}





bool CTFMiniPlus::UpdateDistance()
{
	uint32_t Version;
	uint8_t Buffer[5] = {CMD_FRAME_MARKER, 0X05, 0x00,0x01,0x65};

	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);	
	I2c.ReadBytes(TFMINPLUS_ADDRESS,m_ResponseFrame.Buffer,sizeof(TfMiniPlusFrame_t));
	if(ValidateChecksum(m_ResponseFrame.Buffer,sizeof(TfMiniPlusFrame_t)))
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





bool CTFMiniPlus::GetDistance(uint16_t *Distance) 
{
	*Distance = m_Results.Distance;
	return(m_Results.MeasurementValid);
}

bool CTFMiniPlus::GetSensorRawTemperature(uint16_t *Temp) 
{
	*Temp = m_Results.Temperature;
	return(m_Results.MeasurementValid);
}

bool CTFMiniPlus::GetSensorTemperature(double * Temp) 
{
	*Temp = m_Results.Temperature/ 8.0 - 256;
	return(m_Results.MeasurementValid);
}

bool CTFMiniPlus::GetSignalStrength(uint16_t * Stren) 
{
	*Stren = m_Results.Strength;
	return(m_Results.MeasurementValid);
}

bool CTFMiniPlus::GetVersion(uint32_t &Version) 
{
	uint8_t CommandResponse[7];

	I2c.WriteBytes(TFMINPLUS_ADDRESS,GetVersionCommand,4);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,7);
	Version = (CommandResponse[3]<<16) | (CommandResponse[4]<<8) | (CommandResponse[5]);
	return(ValidateChecksum(CommandResponse,7));
}

bool CTFMiniPlus::SystemReset() 
{
	uint8_t CommandResponse[5];
	I2c.WriteBytes(TFMINPLUS_ADDRESS,ResetCommand,4);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,5);

	if(CommandResponse[3] == 0x00) 
		return true;

	return false;
}

bool CTFMiniPlus::SetFrameRate(uint16_t FrameRate) 
{
	uint8_t NewRateHigh = (uint8_t)(FrameRate >> 8);
	uint8_t NewRateLow = (uint8_t)FrameRate;
	uint8_t Buffer[6] = {CMD_FRAME_MARKER, 0x06, 0x03, NewRateLow, NewRateHigh};
	uint8_t CommandResponse[6];

	Buffer[5] = GenerateChecksum(Buffer, 4);
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,6);
	if(ValidateChecksum(CommandResponse,6))
	{
		if(Buffer[3] == CommandResponse[3] && Buffer[4] == CommandResponse[4])
			return(true);
	}
	return false;
}

void CTFMiniPlus::TriggerDetection() 
{
	I2c.WriteBytes(TFMINPLUS_ADDRESS,TriggerDetectionCommand,4);
}

// for I2C 5A 05 0A 01 6A

bool CTFMiniPlus::SetInterface(uint8_t Interface)
{
//	uint8_t CommandResponse[5];
	uint8_t Buffer[5] = {CMD_FRAME_MARKER, 0X05, 0x0A,Interface};
	Buffer[4] = GenerateChecksum(Buffer, 4);
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);
//	DebugDisplay.Printf("Code %02x %02x %02x %02x %02x",Buffer[0],Buffer[1],Buffer[2],Buffer[3],Buffer[4]);
	return true;

}



bool CTFMiniPlus::SetOutputFormat(uint16_t Format) 
{
	uint8_t CommandResponse[5];
	uint8_t FormatH = (uint8_t)(Format >> 8);
	uint8_t FormatL = (uint8_t)Format;
	uint8_t Buffer[5] = {CMD_FRAME_MARKER, 0X05, 0x05,FormatH, FormatL};

	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,5);

	
	if(Buffer[3] == CommandResponse[3] && Buffer[4] == CommandResponse[4])
			return(true);
	return false;
}

bool CTFMiniPlus::SetBaudRate(uint32_t baud) 
{
	uint8_t CommandResponse[8];
	uint8_t baud_b4 = (uint8_t)(baud >> 24);
	uint8_t baud_b3 = (uint8_t)(baud >> 16);
	uint8_t baud_b2 = (uint8_t)(baud >> 8);
	uint8_t baud_b1 = (uint8_t)baud;
	uint8_t Buffer[8] = {CMD_FRAME_MARKER, 0x08, 0x06, baud_b1, baud_b2, baud_b3, baud_b4};
	Buffer[7] = GenerateChecksum(Buffer, 7);
	
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,8);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,8);

	if(CommandResponse[3] == Buffer[3])
		return true;
	return false;
}

bool CTFMiniPlus::SetEnabled(bool state) 
{
	uint8_t CommandResponse[5];
	uint8_t Buffer[5] = {CMD_FRAME_MARKER, 0x05, 0x07};
	if (state) 
	{
		Buffer[3] = 0x00;
		Buffer[4] = 0x66;
	} 
	else 
	{
		Buffer[3] = 0x01;
		Buffer[4] = 0x67;
	}
	I2c.WriteBytes(TFMINPLUS_ADDRESS,Buffer,5);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,5);


	if(CommandResponse[3] == Buffer[3])
		return true;
	return false;
}

bool CTFMiniPlus::RestoreFactorySettings() 
{
	uint8_t CommandResponse[4];
	I2c.WriteBytes(TFMINPLUS_ADDRESS,RestoreFactorySettingsCommand,4);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,4);

	if(CommandResponse[3] == RestoreFactorySettingsCommand[3])
		return true;
	return false;
}

bool CTFMiniPlus::SaveSettings() 
{
	uint8_t CommandResponse[4];
	I2c.WriteBytes(TFMINPLUS_ADDRESS,SaveSettingsCommand,4);
	I2c.ReadBytes(TFMINPLUS_ADDRESS,CommandResponse,4);

	if(CommandResponse[3] == SaveSettingsCommand[3])
		return true;
	return false;
}


bool CTFMiniPlus::ValidateChecksum( uint8_t dataBuffer[], size_t length) 
{
	uint8_t sum = GenerateChecksum(dataBuffer, length- 1);
	uint8_t checksum = dataBuffer[length-1];
	if (sum == checksum) 
		return true;
	else 
		return(false);
}



uint8_t CTFMiniPlus::GenerateChecksum(const uint8_t buffer[], size_t length) 
{
	uint16_t sum = 0x0000;
	for (uint8_t i = 0; i < length; i++) 
		sum += buffer[i];

	return (uint8_t)sum;
}



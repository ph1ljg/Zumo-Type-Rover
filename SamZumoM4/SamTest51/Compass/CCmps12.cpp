/* 
* CCmps12.cpp
*
* Created: 17/07/2020 14:45:17
* Author: philg
*/
#include "Includes.h"



// default constructor
CCmps12::CCmps12()
{
	m_CmpsData.Valid= false;

} //CCmps12

// default destructor
CCmps12::~CCmps12()
{
} //~CCmps12



void CCmps12::Init()
{
	

}


bool CCmps12::Update()
{
	bool RetVal = true;	
	uint8_t State =0;
	switch(State++)
	{
		case 0:
			if(!GetBearing_16_Bit())
				RetVal = false;
			break;
		case 1:
			if(!GetRoll())
				RetVal = false;
			break;
		case 2:
			if(!GetPitch())
				RetVal = false;
			State = 0;	
			break;
	}
	
// 	if(!GetMagRaw())
// 		RetVal = false;
// 	if(!GetAccelRaw())
// 		RetVal = false;
// 	if(!GetGyroRaw())
// 		RetVal = false;

	if(!RetVal)
		Config.m_RunningFlags.COMPASS_FAIL = true;
	else	
		Config.m_RunningFlags.COMPASS_FAIL = false;
	return(RetVal);	
}


bool CCmps12::GetVersion(uint8_t *Version)
{
	if(GetResponse(SOFTWARE_VERSION,Version,1))
		return(true);
	return(false);
}

bool CCmps12::GetBearing_8_Bit()
{
	uint8_t Bearing;
	if(GetResponse(COMPASS_BEARING_8,&Bearing,1))
		return(true);
	return(false);
}



//Bearing (16 bit), 0-3599
bool CCmps12::GetBearing_16_Bit()
{
	uint8_t RetVal[2];
	if(GetResponse(COMPASS_16_HIGH,RetVal,2))					// returns high byte first
	{
		m_CmpsData.HeadingDeg = (RetVal[0] <<8) | RetVal[1];
		return(true);
	}
	return(false);
}

// Pitch angle +/- 0-90°
bool CCmps12::GetPitch()
{
	int8_t Pitch;
	if(GetResponse(COMPASS_PITCH,(uint8_t*)&Pitch,1))
	{
		m_CmpsData.Pitch = Pitch;
		return(true);
	}
	return(false);
}

// Roll angle +/- 0-90°
bool CCmps12::GetRoll()
{
	int8_t Roll;
	if(GetResponse(COMPASS_ROLL,(uint8_t*)&Roll,1))
	{
		m_CmpsData.Roll = Roll;
		return(true);
	}
	return(false);
}

bool CCmps12::UpdateRawData()
{
	bool RetVal = true;
	if(!GetMagRaw())
		RetVal = false;
	if(!GetAccelRaw())	
		RetVal = false;
	if(!GetGyroRaw())
		RetVal = false;
	return(RetVal);
}
// Raw magnetic data, 16 bit signed: X high, X low, Y high, Y low, Z high, Z low
bool CCmps12::GetMagRaw()
{
	uint8_t MagData[6];
	int16_t x, y, z;

	if(GetResponse(MAGNETOMETER_X_HIGH,MagData,6))
	{
		x = ((int16_t)MagData[0]) | (((int16_t)MagData[1]) << 8);
		y = ((int16_t)MagData[2]) | (((int16_t)MagData[3]) << 8);
		z = ((int16_t)MagData[4]) | (((int16_t)MagData[5]) << 8);

		m_CmpsRawData.MagRaw.x = ((double)x) ;
		m_CmpsRawData.MagRaw.y = ((double)y) ;
		m_CmpsRawData.MagRaw.z = ((double)z) ;
		return(true);
	}
	return(false);
}

// Raw accelerometer data, 16 bit signed: X high, X low, Y high, Y low, Z high, Z low
bool CCmps12::GetAccelRaw()
{
	uint8_t AccelData[6];
	int16_t x, y, z;
	if(GetResponse(RAW_ACCEL_X_HIGH,AccelData,6))
	{
		x = ((int16_t)AccelData[0]) | (((int16_t)AccelData[1]) << 8);
		y = ((int16_t)AccelData[2]) | (((int16_t)AccelData[3]) << 8);
		z = ((int16_t)AccelData[4]) | (((int16_t)AccelData[5]) << 8);

		m_CmpsRawData.AccelRaw.x = ((double)x) ;
		m_CmpsRawData.AccelRaw.y = ((double)y) ;
		m_CmpsRawData.AccelRaw.z = ((double)z) ;
		return(true);
	}
	return(false);
}

bool CCmps12::GetGyroRaw()
{
	uint8_t GiroData[6];
	int16_t x, y, z;
	if(GetResponse(GIRO_X_RAW_HIGH,GiroData,6))
	{
		x= (int16_t)((GiroData[0] <<8) | GiroData[1]);
		y= (int16_t)((GiroData[2] <<8) | GiroData[3]);
		z=(int16_t)((GiroData[4] <<8) | GiroData[5]);
		if(x>-4 && x<4)
			x=0;
		if(y>-4 && y<4)
			y=0;
		if(z>-4 && z<4)
			z=0;
		m_CmpsRawData.GiroRaw.x = (float)x;
		m_CmpsRawData.GiroRaw.y = (float)y;
		m_CmpsRawData.GiroRaw.z = (float)z;
		return(true);
	}
	return(false);
 }

int16_t CCmps12::GetGiroYaw()
{
	uint8_t GiroData[2];
	if(GetResponse(GIRO_Z_RAW_HIGH,GiroData,2))
		return((int16_t)(GiroData[0] <<8) | GiroData[1]);
	else
		return(0);	

}

// BNO055 reported temperature  scaled in °C
bool CCmps12::GetTemp()
{
	uint8_t TempData[2];
	if(GetResponse(BNO_TEMPERATURE_HIGH,(uint8_t*)&TempData,2))
	{
		m_CmpsData.Temperature = (uint16_t)(TempData[0] <<8) | TempData[1];
		return(true);
	}
	return(false);
}




// Bits 0 and 1 reflect the calibration status (0 un-calibrated, 3 fully calibrated)
bool CCmps12::UpdateCalState()
{
	uint8_t Value;
	if(!GetResponse(CALIBRATION_STATE,&Value,1))
		return(false);
	if(Value&0x03)
		m_CmpsData.MagInCal = true;

	if(Value&0x0C)
		m_CmpsData.AccelInCal = true;

	if( Value&0x30)
		m_CmpsData.GyroInCal = true;
		
	if(Value&0xC0)
		m_CmpsData.SystemInCal = true;
		
	return(true);
}

//Bearing (16 bit)
bool CCmps12::GetBoschBearing(uint16_t *Bearing)
{
	uint8_t RetVal[2];

	if(GetResponse(BNO_REARING_HIGH,RetVal,2))					// (0-5759)
	{
		*Bearing = (RetVal[0] <<8) | RetVal[1];
		*Bearing /=16;							// divide by 16 for degrees
		return(true);

	}
	return(false);
}


//Pitch angle (16 bit)  +/- 0-180°
bool CCmps12::GetPitch_180(uint16_t *Pitch)
{
	uint8_t RetVal[2];

	if(GetResponse(PITCH_ANGLE_16_HIGH, RetVal,1))					// (0-5759)
	{
		*Pitch = (RetVal[0] <<8) | RetVal[1];
		return(true);

	}
	return(false);
}


bool CCmps12::StoreCalibration()
{
	if(SendCommand(COMMAND_REG,STORE_CALIBRATION_1))
	{
		Core.delay(20);
		if(SendCommand(COMMAND_REG,STORE_CALIBRATION_2))
		{
			Core.delay(20);
			if(SendCommand(COMMAND_REG,STORE_CALIBRATION_3))
				return(true);
		}
	}
	return(false);
}

bool CCmps12::DeleteCalibration()
{
	if(SendCommand(COMMAND_REG,DELETE_CALIBRATION_1))
	{
		Core.delay(20);
		if(SendCommand(COMMAND_REG,DELETE_CALIBRATION_2))
		{
			Core.delay(20);
			if(SendCommand(COMMAND_REG,DELETE_CALIBRATION_3))
			return(true);
		}
	}
	return(false);
}


bool CCmps12::SendCommand(uint8_t Register ,uint8_t Command)
{
	return(I2c.WriteRegisterByte(CMPS12_ADDRESS,Register ,Command));
}

bool CCmps12::GetResponse(uint8_t Register, uint8_t *Response,uint8_t Size)
{
	if(I2c.ReadRegister(CMPS12_ADDRESS,Register,Response,Size))
	return(true);
	return(false);
}

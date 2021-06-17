/* 
* CBN0055.cpp
*
* Created: 20/08/2020 12:45:21
* Author: philg
*/


#include "Includes.h"
#include "quaternion.h"
#include "CQuatToDcm.h"

	uint8_t UsRegister;

// default constructor
CBN0055::CBN0055()
{
} //CBN0055

// default destructor
CBN0055::~CBN0055()
{
} //~CBN0055

//		 mode values
//  OPERATION_MODE_CONFIG,
//  OPERATION_MODE_ACCONLY,
//  OPERATION_MODE_MAGONLY,
//  OPERATION_MODE_GYRONLY,
//  OPERATION_MODE_ACCMAG,
//  OPERATION_MODE_ACCGYRO,
//  OPERATION_MODE_MAGGYRO,
//  OPERATION_MODE_AMG,
//  OPERATION_MODE_IMUPLUS,
//  OPERATION_MODE_COMPASS,
//  OPERATION_MODE_M4G,
//  OPERATION_MODE_NDOF_FMC_OFF,
//  OPERATION_MODE_NDOF]
bool CBN0055::Init(BN0055_OPMODE_t mode) 
{
	uint8_t id = ReadByte(BN0055_CHIP_ID_ADDR);		// Make sure we have the right device
	if (id != BN0055_ID) 
  
	Core.delay(1000); // hold on for boot
	id = ReadByte(BN0055_CHIP_ID_ADDR);
	if (id != BN0055_ID) 
	{
		return false; 
	}
	SetMode(OPERATION_MODE_CONFIG);						// Switch to config mode (just in case since this is the default)

	WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x20);				//  Reset
	
	Core.delay(200);										//  Delay increased to 30ms 
	while (ReadByte(BN0055_CHIP_ID_ADDR) != BN0055_ID) 
	{
		Core.delay(10);
	}
	Core.delay(50);

	WriteByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);		//Set to normal power mode
	Core.delay(10);

	WriteByte(BNO055_PAGE_ID_ADDR, 0);
	WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x0);
	WriteByte(BNO055_UNIT_SEL_ADDR,0x03);	// Acceleration, Linear m/s/s ,Angular Rate Dps,Euler Angles Degrees,Temperature C,+Pitch clockwise
	Core.delay(10);
	SetMode(mode);										//Set the requested operating mode (see section 3.3)
	Core.delay(20);
	UsRegister = ReadByte(BNO055_UNIT_SEL_ADDR);
	SetExtCrystalUse(true); 

//	DisplaySysStatus();
	return true;
}

void CBN0055::SetMode(BN0055_OPMODE_t mode) 
{
  m_OpMode = mode;
  WriteByte(BNO055_OPR_MODE_ADDR, m_OpMode);
  Core.delay(30);
}

// Changes the chip's axis remap
void CBN0055::SetAxisRemap(   BN0055_REMAP_AXIS_t remapcode) 
{
  BN0055_OPMODE_t modeback = m_OpMode;
  SetMode(OPERATION_MODE_CONFIG);
  Core.delay(25);
  WriteByte(BNO055_AXIS_MAP_CONFIG_ADDR, remapcode);
  Core.delay(10);
  SetMode(modeback);					//Set the requested operating mode (see section 3.3)
  Core.delay(20);
}

//=========================================================================
// Changes the chip's axis signs
// remap sign possible values
// [REMAP_SIGN_P0
// REMAP_SIGN_P1 (default)
// REMAP_SIGN_P2
// REMAP_SIGN_P3
// REMAP_SIGN_P4
// REMAP_SIGN_P5
// REMAP_SIGN_P6
// REMAP_SIGN_P7]
//======================================================================== 
void CBN0055::SetAxisSign(BN0055_AXIS_REMAP_SIGN_t remapsign) 
{
  BN0055_OPMODE_t modeback = m_OpMode;

  SetMode(OPERATION_MODE_CONFIG);
  Core.delay(25);
  WriteByte(BNO055_AXIS_MAP_SIGN_ADDR, remapsign);
  Core.delay(10);
  SetMode(modeback);					// Set the requested operating mode (see section 3.3)
  Core.delay(20);
}


// Use the external 32.768KHz crystal
void CBN0055::SetExtCrystalUse(bool usextal) 
{
  BN0055_OPMODE_t modeback = m_OpMode;

  SetMode(OPERATION_MODE_CONFIG);		//Switch to config mode (just in case since this is the default)
  Core.delay(25);
  WriteByte(BNO055_PAGE_ID_ADDR, 0);
  if (usextal) 
    WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x80);
  else 
    WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x00);
  
  Core.delay(10);
  SetMode(modeback);					// Set the requested operating mode (see section 3.3)
  Core.delay(20);
}


// Gets the latest system status info
void CBN0055::GetSystemStatus(uint8_t *system_status,uint8_t *self_test_result,uint8_t *system_error) 
{
  WriteByte(BNO055_PAGE_ID_ADDR, 0);

// System Status (see section 4.3.58)
// 0 = Idle
// 1 = System Error
// 2 = Initializing Peripherals
// 3 = System Iniitalization
// 4 = Executing Self-Test
// 5 = Sensor fusio algorithm running
// 6 = System running without fusion algorithms

  if (system_status != 0)
    *system_status = ReadByte(BNO055_SYS_STAT_ADDR);

// Self Test Results
// 1 = test passed, 0 = test failed
// 
// Bit 0 = Accelerometer self test
// Bit 1 = Magnetometer self test
// Bit 2 = Gyroscope self test
// Bit 3 = MCU self test
// 
// 0x0F = all good!

  if (self_test_result != 0)
    *self_test_result = ReadByte(BNO055_SELFTEST_RESULT_ADDR);

// System Error (see section 4.3.59)
// 0 = No error
// 1 = Peripheral initialization error
// 2 = System initialization error
// 3 = Self test result failed
// 4 = Register map value out of range
// 5 = Register map address out of range
// 6 = Register map write error
// 7 = BNO low power mode not available for selected operat ion mode
// 8 = Accelerometer power mode not available
// 9 = Fusion algorithm configuration error
// A = Sensor configuration error
// 
  if (system_error != 0)
    *system_error = ReadByte(BNO055_SYS_ERR_ADDR);

  Core.delay(200);
}

void CBN0055::DisplaySysStatus()
{
	uint8_t system_status;
	uint8_t self_test_result;
	uint8_t system_error;
	uint8_t Row = 10;
	uint8_t Col = 10;
	GetSystemStatus(&system_status,&self_test_result,&system_error);
	
	switch(system_status)
	{
    case 0:
		DebugDisplay.Printf(Row++,Col,1,"Idle");
		break;
	case 1:
		DebugDisplay.Printf(Row++,Col,1,"System Error");
		break;
     case 2:
		DebugDisplay.Printf(Row++,Col,1,"Initializing Peripherals");
		break;
     case 3:
		DebugDisplay.Printf(Row++,Col,1,"System Iniitalization");
		break;
     case 4:
		DebugDisplay.Printf(Row++,Col,1,"Executing Self-Test");
		break;
     case 5:
		DebugDisplay.Printf(Row++,Col,1,"Sensor fusion algorithm running");
		break;
     case 6:
		DebugDisplay.Printf(Row++,Col,1,"System running without fusion algorithms");
		break;
	}
	
	Row++;
	if(self_test_result &(1<<0))
		DebugDisplay.Printf(Row++,Col,1,"Acc Passed");
	
	if(self_test_result &(1<<1))
		DebugDisplay.Printf(Row++,Col,1,"Mag Passed");
	if(self_test_result &(1<<2))
		DebugDisplay.Printf(Row++,Col,1,"Gyro Passed");
	if(self_test_result &(1<<3))
		DebugDisplay.Printf(Row++,Col,1,"All good");
	
	Row++;
	switch(system_status)
	{
	case 0:
		DebugDisplay.Printf(Row++,Col,1,"No error");
		break;
	case 1:
		DebugDisplay.Printf(Row++,Col,1,"Peripheral initialization error");
		break;
	case 2:
		DebugDisplay.Printf(Row++,Col,1,"System initialization error");
		break;
	case 3:
		DebugDisplay.Printf(Row++,Col,1,"Self test result failed");
		break;
	case 4:
		DebugDisplay.Printf(Row++,Col,1,"Register map value out of ranges");
		break;
	case 5:
		DebugDisplay.Printf(Row++,Col,1,"Register map address out of range");
		break;
	case 6:
		DebugDisplay.Printf(Row++,Col,1,"Register map write error");
		break;
	case 7:
		DebugDisplay.Printf(Row++,Col,1,"BNO low power mode not available for selected operation mode");
		break;
	case 8:
		DebugDisplay.Printf(Row++,Col,1,"Accelerometer power mode not available");
		break;
	case 9:
		DebugDisplay.Printf(Row++,Col,1,"Fusion algorithm configuration error");
	break;
	case 10:
		DebugDisplay.Printf(Row++,Col,1,"Sensor configuration error");
		break;
	}
	Core.delay(4000);
}

// Gets the chip revision numbers
void CBN0055::GetRevInfo(BN0055_rev_info_t *info) 
{
  uint8_t a, b;

  memset(info, 0, sizeof(BN0055_rev_info_t));

  info->accel_rev = ReadByte(BNO055_ACCEL_REV_ID_ADDR);		// Check the accelerometer revision
  info->mag_rev = ReadByte(BNO055_MAG_REV_ID_ADDR);			// Check the magnetometer revision
  info->gyro_rev = ReadByte(BNO055_GYRO_REV_ID_ADDR);			// Check the gyroscope revision
  info->bl_rev = ReadByte(BNO055_BL_REV_ID_ADDR);				// Check the SW revision

  a = ReadByte(BNO055_SW_REV_ID_LSB_ADDR);
  b = ReadByte(BNO055_SW_REV_ID_MSB_ADDR);
  info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}
//================================================================================================================================================
// Gets current calibration state.  Each value should be a uint8_t pointer and it will be set to 0 if not calibrated and 3 if fully calibrated.
// See section 34.3.54
// Current system calibration status, depends on status of all sensors,
//================================================================================================================================================
void CBN0055::GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag) 
{
  uint8_t calData = ReadByte(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) 
  {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) 
  {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) 
  {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) 
  {
    *mag = calData & 0x03;
  }
}

// Gets the temperature in degrees celsius
int8_t CBN0055::GetTemp() 
{
  int8_t temp = (int8_t)(ReadByte(BNO055_TEMP_ADDR));
  return temp;
}
  // Possible vector values can be:

//==========================================================================
// Gets a vector reading from the specified source vector_type
// possible vector type values
  //  VECTOR_ACCELEROMETER		- m/s^2
  //  VECTOR_MAGNETOMETER		- uT
  //  VECTOR_GYROSCOPE				- rad/s
  //  VECTOR_EULER						- degrees
  //  VECTOR_LINEARACCEL			- m/s^2
  //  VECTOR_GRAVITY					- m/s^2
//==========================================================================
 bool CBN0055::GetVector(Vector3f &Vect,BN0055_VECTOR_t vector_type) 
{
	Vector3f Test;
	uint8_t buffer[6];
	memset(buffer, 0, 6);

	int16_t x, y, z;
	x = y = z = 0;

	if(!ReadBuffer((Bno055Reg_t)vector_type, buffer, 6))		// Read vector data (6 bytes)
		return(false);

	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

	// Convert the value to an appropriate range (section 3.6.4) and assign the value to the Vector type
	switch (vector_type) 
	{
	case VECTOR_MAGNETOMETER:			// 1uT = 16 LSB
		Vect.x = ((double)x) / 16.0;
		Vect.y = ((double)y) / 16.0;
		Vect.z = ((double)z) / 16.0;
		break;
	case VECTOR_GYROSCOPE:				// 1dps = 16 LSB 
		Vect.x = ((double)x) / 900.0;		// 1 Rps = 900 LSB   // fusion mode
		Vect.y = ((double)y) / 900.0;
		Vect.z = ((double)z) / 900.0;
//		GuiFunctions.SetPidOutputValues( Vect.x,GUI_PID_OUTPUT_INPUT);
//		GuiFunctions.SetPidOutputValues( Vect.y,GUI_PID_OUTPUT_OUTPUT);
//		GuiFunctions.SetPidOutputValues( Vect.z,GUI_PID_OUTPUT_ERROR);

		break;
	case VECTOR_EULER:
		Vect.x = ((double)z) / 16.0;	// 1 degree = 16 LSB
		Vect.y = ((double)y) / 16.0;
		Vect.z = ((double)x) / 16.0;
		break;
	case VECTOR_ACCELEROMETER:			// 1m/s^2 = 100 LSB
		Vect.x = ((double)x) / 100.0;
		Vect.y = ((double)y) / 100.0;
		Vect.z = ((double)z) / 100.0;
	//	Vect.z /= 2048.0;
		break;
	case VECTOR_LINEARACCEL:			// 1m/s^2 = 100 LSB
		Vect.x = ((double)x) / 100.0;
		Vect.y = ((double)y) / 100.0;
		Vect.z = ((double)z) / 100.0;
		break;
	case VECTOR_GRAVITY:				// 1m/s^2 = 100 LSB 
		Vect.x = ((double)x) / 100.0;
		Vect.y = ((double)y) / 100.0;
		Vect.z = ((double)z) / 100.0;
		break;
	}
	return(true);
}


 bool CBN0055::GetData(Vector3f &Gyro,Vector3f &LinVel) 
{
	
	uint8_t buffer[25];
	Vector3f Eular;
	
	memset(buffer, 0, 6);

	int16_t x, y, z, w;
	x = y = z = w = 0;

	if(!I2c.ReadRegister(BN0055_ADDRESS_B,BNO055_GYRO_DATA_X_LSB_ADDR,buffer,25))
		return(false);

	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

	Gyro.x = ((double)x) / 900.0;
	Gyro.y = ((double)y) / 900.0;
	Gyro.z = ((double)z) / 900.0;

	x = ((int16_t)buffer[6]) | (((int16_t)buffer[7]) << 8);
	y = ((int16_t)buffer[8]) | (((int16_t)buffer[9]) << 8);
	z = ((int16_t)buffer[10]) | (((int16_t)buffer[11]) << 8);

	Eular.x = ((double)z) / 16.0;	// 1 degree = 16 LSB
	Eular.y = ((double)y) / 16.0;
	Eular.z = ((double)x) / 16.0;

	x = y = z = w = 0;
	w =		((int16_t)buffer[12]) | (((int16_t)buffer[13]) << 8);
	x =		((int16_t)buffer[14]) | (((int16_t)buffer[15]) << 8);
	y =		((int16_t)buffer[16]) | (((int16_t)buffer[17]) << 8);
	z =		((int16_t)buffer[18]) | (((int16_t)buffer[19]) << 8);


	x = ((int16_t)buffer[20]) | (((int16_t)buffer[21]) << 8);
	y = ((int16_t)buffer[22]) | (((int16_t)buffer[23]) << 8);
	z = ((int16_t)buffer[24]) | (((int16_t)buffer[25]) << 8);


	LinVel.x = ((double)x) / 100.0;
	LinVel.y = ((double)y) / 100.0;
	LinVel.z = ((double)z) / 100.0;
}

// Reads the sensor's offset registers into a byte array
// calibData   Calibration offset (buffer size should be 22)
// return true if read is successful
bool CBN0055::getSensorOffsets(uint8_t *calibData) 
{
  if (isFullyCalibrated()) 
  {
    BN0055_OPMODE_t lastMode = m_OpMode;
    SetMode(OPERATION_MODE_CONFIG);

    ReadBuffer(ACCEL_OFFSET_X_LSB_ADDR, calibData, NUM_BN0055_OFFSET_REGISTERS);

    SetMode(lastMode);
    return true;
  }
  return false;
}

// Reads the sensor's offset registers into an offset struct
// offsets_type
// type of offsets
// return true if read is successful
 
bool CBN0055::getSensorOffsets(BN0055_SENSOR_OFFSETS_t &CalOffsets) 
{
  if (isFullyCalibrated()) 
  {
    BN0055_OPMODE_t lastMode = m_OpMode;
    SetMode(OPERATION_MODE_CONFIG);
    Core.delay(25);

// Accel offset range depends on the G-range:
// +/-2g  = +/- 2000 mg
// +/-4g  = +/- 4000 mg
// +/-8g  = +/- 8000 mg
// +/-1§g = +/- 16000 mg 
    CalOffsets.accel_offset_x = (ReadByte(ACCEL_OFFSET_X_MSB_ADDR) << 8) |(ReadByte(ACCEL_OFFSET_X_LSB_ADDR));
    CalOffsets.accel_offset_y = (ReadByte(ACCEL_OFFSET_Y_MSB_ADDR) << 8) |(ReadByte(ACCEL_OFFSET_Y_LSB_ADDR));
    CalOffsets.accel_offset_z = (ReadByte(ACCEL_OFFSET_Z_MSB_ADDR) << 8) |(ReadByte(ACCEL_OFFSET_Z_LSB_ADDR));

    // Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB 
    CalOffsets.mag_offset_x =   (ReadByte(MAG_OFFSET_X_MSB_ADDR) << 8) | (ReadByte(MAG_OFFSET_X_LSB_ADDR));
    CalOffsets.mag_offset_y =   (ReadByte(MAG_OFFSET_Y_MSB_ADDR) << 8) | (ReadByte(MAG_OFFSET_Y_LSB_ADDR));
    CalOffsets.mag_offset_z =   (ReadByte(MAG_OFFSET_Z_MSB_ADDR) << 8) | (ReadByte(MAG_OFFSET_Z_LSB_ADDR));

//  Gyro offset range depends on the DPS range:
//  2000 dps = +/- 32000 LSB
//  1000 dps = +/- 16000 LSB
//  500 dps = +/- 8000 LSB
//  250 dps = +/- 4000 LSB
//  125 dps = +/- 2000 LSB
//  where 1 DPS = 16 LSB 
    CalOffsets.gyro_offset_x =  (ReadByte(GYRO_OFFSET_X_MSB_ADDR) << 8) | (ReadByte(GYRO_OFFSET_X_LSB_ADDR));
    CalOffsets.gyro_offset_y =  (ReadByte(GYRO_OFFSET_Y_MSB_ADDR) << 8) | (ReadByte(GYRO_OFFSET_Y_LSB_ADDR));
    CalOffsets.gyro_offset_z =  (ReadByte(GYRO_OFFSET_Z_MSB_ADDR) << 8) | (ReadByte(GYRO_OFFSET_Z_LSB_ADDR));

    // Accelerometer radius = +/- 1000 LSB 
    CalOffsets.accel_radius =   (ReadByte(ACCEL_RADIUS_MSB_ADDR) << 8) | (ReadByte(ACCEL_RADIUS_LSB_ADDR));

    // Magnetometer radius = +/- 960 LSB 
    CalOffsets.mag_radius =     (ReadByte(MAG_RADIUS_MSB_ADDR) << 8) | (ReadByte(MAG_RADIUS_LSB_ADDR));

    SetMode(lastMode);
    return true;
  }
  return false;
}

// Writes an array of calibration values to the sensor's offset
// calibData         calibration data
void CBN0055::setSensorOffsets(const uint8_t *calibData) 
{
  BN0055_OPMODE_t lastMode = m_OpMode;
  SetMode(OPERATION_MODE_CONFIG);
  Core.delay(25);

// 	Note: Configuration will take place only when user writes to the last byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
// 	Therefore the last byte must be written whenever the user wants to  changes the configuration. 

	WriteByte(ACCEL_OFFSET_X_LSB_ADDR, calibData[0]);
	WriteByte(ACCEL_OFFSET_X_MSB_ADDR, calibData[1]);
	WriteByte(ACCEL_OFFSET_Y_LSB_ADDR, calibData[2]);
	WriteByte(ACCEL_OFFSET_Y_MSB_ADDR, calibData[3]);
	WriteByte(ACCEL_OFFSET_Z_LSB_ADDR, calibData[4]);
	WriteByte(ACCEL_OFFSET_Z_MSB_ADDR, calibData[5]);

	WriteByte(MAG_OFFSET_X_LSB_ADDR, calibData[6]);
	WriteByte(MAG_OFFSET_X_MSB_ADDR, calibData[7]);
	WriteByte(MAG_OFFSET_Y_LSB_ADDR, calibData[8]);
	WriteByte(MAG_OFFSET_Y_MSB_ADDR, calibData[9]);
	WriteByte(MAG_OFFSET_Z_LSB_ADDR, calibData[10]);
	WriteByte(MAG_OFFSET_Z_MSB_ADDR, calibData[11]);

	WriteByte(GYRO_OFFSET_X_LSB_ADDR, calibData[12]);
	WriteByte(GYRO_OFFSET_X_MSB_ADDR, calibData[13]);
	WriteByte(GYRO_OFFSET_Y_LSB_ADDR, calibData[14]);
	WriteByte(GYRO_OFFSET_Y_MSB_ADDR, calibData[15]);
	WriteByte(GYRO_OFFSET_Z_LSB_ADDR, calibData[16]);
	WriteByte(GYRO_OFFSET_Z_MSB_ADDR, calibData[17]);

	WriteByte(ACCEL_RADIUS_LSB_ADDR, calibData[18]);
	WriteByte(ACCEL_RADIUS_MSB_ADDR, calibData[19]);

	WriteByte(MAG_RADIUS_LSB_ADDR, calibData[20]);
	WriteByte(MAG_RADIUS_MSB_ADDR, calibData[21]);

	SetMode(lastMode);
}


// Writes to the sensor's offset registers from an offset struct
// offsets_type
// accel_offset_x = acceleration offset x
// accel_offset_y = acceleration offset y
// accel_offset_z = acceleration offset z
//  
// mag_offset_x   = magnetometer offset x
// mag_offset_y   = magnetometer offset y
// mag_offset_z   = magnetometer offset z
//  
// gyro_offset_x  = gyroscope offset x
// gyro_offset_y  = gyroscope offset y
// gyro_offset_z  = gyroscope offset z
 
void CBN0055::setSensorOffsets(    const BN0055_SENSOR_OFFSETS_t &offsets_type) 
{
  BN0055_OPMODE_t lastMode = m_OpMode;
  SetMode(OPERATION_MODE_CONFIG);
  Core.delay(25);

// Note: Configuration will take place only when user writes to the last byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
// Therefore the last byte must be written whenever the user wants to changes the configuration. 

	WriteByte(ACCEL_OFFSET_X_LSB_ADDR, (offsets_type.accel_offset_x) & 0x0FF);
	WriteByte(ACCEL_OFFSET_X_MSB_ADDR, (offsets_type.accel_offset_x >> 8) & 0x0FF);
	WriteByte(ACCEL_OFFSET_Y_LSB_ADDR, (offsets_type.accel_offset_y) & 0x0FF);
	WriteByte(ACCEL_OFFSET_Y_MSB_ADDR, (offsets_type.accel_offset_y >> 8) & 0x0FF);
	WriteByte(ACCEL_OFFSET_Z_LSB_ADDR, (offsets_type.accel_offset_z) & 0x0FF);
	WriteByte(ACCEL_OFFSET_Z_MSB_ADDR, (offsets_type.accel_offset_z >> 8) & 0x0FF);

	WriteByte(MAG_OFFSET_X_LSB_ADDR, (offsets_type.mag_offset_x) & 0x0FF);
	WriteByte(MAG_OFFSET_X_MSB_ADDR, (offsets_type.mag_offset_x >> 8) & 0x0FF);
	WriteByte(MAG_OFFSET_Y_LSB_ADDR, (offsets_type.mag_offset_y) & 0x0FF);
	WriteByte(MAG_OFFSET_Y_MSB_ADDR, (offsets_type.mag_offset_y >> 8) & 0x0FF);
	WriteByte(MAG_OFFSET_Z_LSB_ADDR, (offsets_type.mag_offset_z) & 0x0FF);
	WriteByte(MAG_OFFSET_Z_MSB_ADDR, (offsets_type.mag_offset_z >> 8) & 0x0FF);

	WriteByte(GYRO_OFFSET_X_LSB_ADDR, (offsets_type.gyro_offset_x) & 0x0FF);
	WriteByte(GYRO_OFFSET_X_MSB_ADDR, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
	WriteByte(GYRO_OFFSET_Y_LSB_ADDR, (offsets_type.gyro_offset_y) & 0x0FF);
	WriteByte(GYRO_OFFSET_Y_MSB_ADDR, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
	WriteByte(GYRO_OFFSET_Z_LSB_ADDR, (offsets_type.gyro_offset_z) & 0x0FF);
	WriteByte(GYRO_OFFSET_Z_MSB_ADDR, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

	WriteByte(ACCEL_RADIUS_LSB_ADDR, (offsets_type.accel_radius) & 0x0FF);
	WriteByte(ACCEL_RADIUS_MSB_ADDR, (offsets_type.accel_radius >> 8) & 0x0FF);

	WriteByte(MAG_RADIUS_LSB_ADDR, (offsets_type.mag_radius) & 0x0FF);
	WriteByte(MAG_RADIUS_MSB_ADDR, (offsets_type.mag_radius >> 8) & 0x0FF);

	SetMode(lastMode);
}

// Checks of all cal status values are set to 3 (fully calibrated)
// return status of calibration
bool CBN0055::isFullyCalibrated() 
{
  uint8_t system, gyro, accel, mag;
  GetCalibration(&system, &gyro, &accel, &mag);

  switch (m_OpMode) 
  {
  case OPERATION_MODE_ACCONLY:
    return (accel == 3);
  case OPERATION_MODE_MAGONLY:
    return (mag == 3);
  case OPERATION_MODE_GYRONLY:
  case OPERATION_MODE_M4G: /* No magnetometer calibration required. */
    return (gyro == 3);
  case OPERATION_MODE_ACCMAG:
  case OPERATION_MODE_COMPASS:
    return (accel == 3 && mag == 3);
  case OPERATION_MODE_ACCGYRO:
  case OPERATION_MODE_IMUPLUS:
    return (accel == 3 && gyro == 3);
  case OPERATION_MODE_MAGGYRO:
    return (mag == 3 && gyro == 3);
  default:
    return (system == 3 && gyro == 3 && accel == 3 && mag == 3);
  }
}

// Enter Suspend mode (i.e., sleep)
void CBN0055::EnterSuspendMode() 
{
  BN0055_OPMODE_t modeback = m_OpMode;

  // Switch to config mode (just in case since this is the default) 
  SetMode(OPERATION_MODE_CONFIG);
  Core.delay(25);
  WriteByte(BNO055_PWR_MODE_ADDR, 0x02);
  SetMode(modeback);		// Set the requested operating mode (see section 3.3)
  Core.delay(20);
}

// Enter Normal mode (i.e., wake)
void CBN0055::enterNormalMode() 
{
  BN0055_OPMODE_t modeback = m_OpMode;

  SetMode(OPERATION_MODE_CONFIG);		// Switch to config mode (just in case since this is the default)
  Core.delay(25);
  WriteByte(BNO055_PWR_MODE_ADDR, 0x00);
  SetMode(modeback);					// Set the requested operating mode (see section 3.3)
  Core.delay(20);
}


// Gets a quaternion reading from the specified source
// return quaternion reading
ImuMath::Quaternion CBN0055::getQuat()
{
	uint8_t buffer[8];
	memset(buffer, 0, 8);

	int16_t x, y, z, w;
	x = y = z = w = 0;

	/* Read quat data (8 bytes) */
	ReadBuffer(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
	x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
	y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
	z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

	// 	 Assign to Quaternion
	// 	 See 	 http://ae-bst.resource.bosch.com/media/products/dokumente/bno055/BST_BNO055_DS000_12~1.pdf
	// 	 3.6.5.5 Orientation (Quaternion)
	
	const double scale = (1.0 / (1 << 14));
	ImuMath::Quaternion quat(scale * w, scale * x, scale * y, scale * z);
	return quat;
}

const Matrix3f CBN0055::GetDcm()
{
	Matrix3f Dcm;
	uint8_t buffer[8];
	memset(buffer, 0, 8);
	float Quat[4];
	CQuatToDcm QuatToDcm;
	int16_t x, y, z, w;
	x = y = z = w = 0;

	/* Read quat data (8 bytes) */
	ReadBuffer(BNO055_QUATERNION_DATA_W_LSB_ADDR, buffer, 8);
	w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
	x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
	y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
	z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);
	
	const double scale = (1.0 / (1 << 14)); //2^14 LSB

	Quat[0] = scale * w;
	Quat[1] = scale * x;
	Quat[2] = scale * y;
	Quat[3] = scale* z;
	QuatToDcm.Convert(Quat, Dcm);
	return(Dcm);
}



bool CBN0055::WriteByte(Bno055Reg_t reg, uint8_t value) 
{
	return(I2c.WriteRegisterByte(BN0055_ADDRESS_B,reg ,value));
}

uint8_t CBN0055::ReadByte(Bno055Reg_t reg) 
{
  uint8_t value = 0;
	if(I2c.ReadRegister(BN0055_ADDRESS_B,reg,&value,1))
		return(value);
	return(0);	
}

bool CBN0055::ReadBuffer(Bno055Reg_t reg, uint8_t *buffer,uint8_t len) 
{
	if(I2c.ReadRegister(BN0055_ADDRESS_B,reg,buffer,len))
		return(true);
	return(false);		
}

void CBN0055::Test()
{
	if(Init())
	{
		while(1)
		{
//			Test1();
			Test2();
		}
	}
}


void CBN0055::Test1()
{
	Vector3f Vect1;
	Vector3f Vect2;
	Vector3f Vect3;
	double xPos = 0, yPos = 0, headingVel = 0;
	static uint32_t tStart =0;
//	uint32_t period = Core.micros() - tStart;
//	uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board
	uint16_t PRINT_DELAY_MS = 500; // how often to print the data
	static	uint16_t printCount = 0; //counter to avoid printing every 10MS sample

	//velocity = accel*dt (dt in seconds)
	//position = 0.5*accel*dt^2
	double ACCEL_VEL_TRANSITION =  (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
	double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
	double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees


	
	
	GetVector(Vect1,VECTOR_EULER);
	GetVector(Vect2,VECTOR_GYROSCOPE);
	GetVector(Vect3,VECTOR_LINEARACCEL);
	
	xPos = xPos + ACCEL_POS_TRANSITION * Vect3.x;
	yPos = yPos + ACCEL_POS_TRANSITION * Vect3.y;

	// velocity of sensor in the direction it's facing
	headingVel = ACCEL_VEL_TRANSITION * Vect3.x / cos(DEG_2_RAD * Vect1.x);

	if (printCount++ * BNO055_SAMPLERATE_DELAY_MS >= PRINT_DELAY_MS) 
	{
		//enough iterations have passed that we can print the latest data
		DebugDisplay.Printf("Heading: %f\n",Vect1.x);
		DebugDisplay.Printf("Position: %f,%f\n",xPos,yPos);
		DebugDisplay.Printf("Speed:1 %f\n",headingVel);
		printCount = 0;
 	}
 	else
 		printCount++;
 	while ((Core.micros() - tStart) < (BNO055_SAMPLERATE_DELAY_MS * 1000))
 	{
	}	
}

void CBN0055::Test2()
{
	// Get the four calibration values (0..3) Any sensor data reporting 0 should be ignored,  3 means 'fully calibrated" 
	uint8_t Row = 10;
	uint8_t Col = 10;
	Vector3f Vect1;
	Vector3f Vect2;
	Vector3f Vect3;
	double xPos = 0, yPos = 0, headingVel = 0;
	static uint32_t LastTime = Core.millis();
	
	uint32_t period = Core.millis() - LastTime;


	//velocity = accel*dt (dt in seconds)
	//position = 0.5*accel*dt^2
	double ACCEL_VEL_TRANSITION =  (double)period / 1000.0;
	double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;
	double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees

	uint8_t system, gyro, accel, mag;
	system = gyro = accel = mag = 0;
	GetCalibration(&system, &gyro, &accel, &mag);

	// Display the individual values 
	DebugDisplay.Printf(Row++,Col,8,"Register %d", system);
	DebugDisplay.Printf(Row++,Col,8," Giro: %d",gyro);
	DebugDisplay.Printf(Row++,Col,8," Accel: %d",accel);
	DebugDisplay.Printf(Row++,Col,8," Mag: %d",mag);
	if(gyro >=3 && accel >=3 && mag >=3)
	{
			GetVector(Vect1,VECTOR_EULER);
			GetVector(Vect2,VECTOR_GYROSCOPE);
			GetVector(Vect3,VECTOR_LINEARACCEL);
			
			xPos = xPos + ACCEL_POS_TRANSITION * Vect3.x;
			yPos = yPos + ACCEL_POS_TRANSITION * Vect3.y;

			// velocity of sensor in the direction it's facing
			headingVel = ACCEL_VEL_TRANSITION * Vect3.x / cos(DEG_2_RAD * Vect1.x);

			//enough iterations have passed that we can print the latest data
			DebugDisplay.Printf(Row++,Col,8,"Heading: %f",Vect1.x);
			DebugDisplay.Printf(Row++,Col,8,"Position: %f,%f",xPos,yPos);
			DebugDisplay.Printf(Row++,Col,8,"Speed:1 %f",headingVel);

	}
	Core.delay(1000);
}
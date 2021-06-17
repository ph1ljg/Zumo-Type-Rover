/* 
* CCmps12.h
*
* Created: 17/07/2020 14:45:17
* Author: philg
*/

#ifndef __CCMPS12_H__
#define __CCMPS12_H__


#define CMPS12_X 0
#define CMPS12_Y 1
#define CMPS12_Z 2

#define CMPS12_ADDRESS			0x60
#define COMPASS_TIMEOUT			1000

#define COMMAND_REG				0x00
#define SOFTWARE_VERSION		0x00  //  Software version
#define COMPASS_BEARING_8		0x01  //  Compass Bearing as a byte, i.e. 0-255 for a full circle
#define COMPASS_16_HIGH			0x02  // Compass Bearing High Byte , i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
#define COMPASS_16_LOW			0x03  // Compass Bearing Low Byte, i.e. 0-3599 for a full circle, representing 0-359.9 degrees.
#define COMPASS_PITCH			0x04  // Pitch angle - signed byte giving angle in degrees from the horizontal plane (+/- 90°)
#define COMPASS_ROLL			0x05  // Roll angle - signed byte giving angle in degrees from the horizontal plane (+/- 90°)


#define MAGNETOMETER_X_HIGH		0x06	// Magnetometer X axis raw output, 16 bit signed integer (high byte)
#define MAGNETOMETER_X_LOW		0x07	// Magnetometer X axis raw output, 16 bit signed integer (low byte)
#define MAGNETOMETER_Y_HIGH		0x08	// Magnetometer Y axis raw output, 16 bit signed integer (high byte)
#define MAGNETOMETER_Y_LOW		0x09	// Magnetometer Y axis raw output, 16 bit signed integer (Low byte)
#define MAGNETOMETER_Z_HIGH		0x0A	// Magnetometer Z axis raw output, 16 bit signed integer (high byte)
#define MAGNETOMETER_Z_LOW		0x0B	// Magnetometer Z axis raw output, 16 bit signed integer (Low byte)


// accelerometer registers
#define RAW_ACCEL_X_HIGH		0x0C	// Accelerometer X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
#define RAW_ACCEL_X_LOW			0x0D	// Accelerometer X axis raw output, 16 bit signed integer with register 16 being the upper 8 bits
#define RAW_ACCEL_Y_HIGH		0x0E	// Accelerometer Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
#define RAW_ACCEL_Y_LOW			0x0F	// Accelerometer Y axis raw output, 16 bit signed integer with register 18 being the upper 8 bits
#define RAW_ACCEL_Z_HIGH		0x10	// Accelerometer Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
#define RAW_ACCEL_Z_LOW			0x11	// Accelerometer Z axis raw output, 16 bit signed integer with register 20 being the upper 8 bits

#define GIRO_X_RAW_HIGH			0x12	// Gyro X axis raw output, 16 bit signed integer (register 0x12 high byte)
#define GIRO_X_RAW_LOW			0x13	// Gyro X axis raw output, 16 bit signed integer (register 0x12 high byte)
#define GIRO_Y_RAW_HIGH			0x14	// Gyro Y axis raw output, 16 bit signed integer (register 0x14 high byte)
#define GIRO_Y_RAW_LOW			0x15	// Gyro Y axis raw output, 16 bit signed integer (register 0x14 high byte)
#define GIRO_Z_RAW_HIGH			0x16	// Gyro Z axis raw output, 16 bit signed integer (register 0x16 high byte)
#define GIRO_Z_RAW_LOW			0x17	// Gyro Z axis raw output, 16 bit signed integer (register 0x16 high byte)

#define BNO_TEMPERATURE_HIGH	0x18	// Temperature of the BNO055 in degrees centigrade (register 0x18 high byte)
#define BNO_TEMPERATURE_LOW		0x19	// Temperature of the BNO055 in degrees centigrade (register 0x18 high byte)

#define BNO_REARING_HIGH		0x1A	// Compass Bearing 16 bit This is the angle Bosch generate in the BNO055 (0-5759),divide by 16 for degrees
#define BNO_REARING_LOW			0x1B	// Compass Bearing 16 bit This is the angle Bosch generate in the BNO055 (0-5759),divide by 16 for degrees

#define PITCH_ANGLE_16_HIGH		0x1C	// Pitch angle 16 bit - signed byte giving angle in degrees from the horizontal plane (+/-180°)
#define PITCH_ANGLE_16_LOW		0x1D	// Pitch angle 16 bit - signed byte giving angle in degrees from the horizontal plane (+/-180°)


#define CALIBRATION_STATE		 0x1E	// Calibration state, bits 0 and 1 reflect the calibration status (0 un-calibrated, 3 fully calibrated)


#define	STORE_CALIBRATION_1		0xF0	//store a profile write  to the command register 0xF0, 0xF5, 0xF6 with a 20ms delay after each of the three bytes.
#define	STORE_CALIBRATION_2		0xF5
#define	STORE_CALIBRATION_3		0xF6

#define DELETE_CALIBRATION_1	0xE0	// erase the stored profile so your module powers into a default state write to the commandwith a 20ms delay after each of the three bytes.
#define DELETE_CALIBRATION_2	0xE5
#define DELETE_CALIBRATION_3	0xE2

#define CHANGE_I2C_ADDRESS_1	0xA0	//	write the following to the command register 0 at address Present Address (0xA0, 0xAA, 0xA5, 0xC2 ) with a 20ms delay
										//	after each of the first three bytes. These commands must be sent in the correct sequence 
#define CHANGE_I2C_ADDRESS_2	0xAA
#define CHANGE_I2C_ADDRESS_3	0xA5



typedef struct __attribute__ ((packed))
{
	uint16_t HeadingDeg;
	int8_t Roll;
	int8_t Pitch;
	uint16_t Temperature;
	bool GyroInCal;
	bool AccelInCal;
	bool MagInCal;
	bool SystemInCal;
	bool Valid;
}CmpsData_t;

typedef struct __attribute__ ((packed))
{
	Vector3f  AccelRaw;
	Vector3f GiroRaw;
	Vector3f MagRaw;
	bool Valid;
}CmpsRawData_t;



class CCmps12
{
//variables
public:
	CmpsData_t 	m_CmpsData;
	CmpsRawData_t m_CmpsRawData;
protected:
private:

//functions
public:
	CCmps12();
	~CCmps12();
	void Init();
	bool Update();
	bool SendCommand(uint8_t Register ,uint8_t Command);
	bool GetResponse(uint8_t Register, uint8_t *Response,uint8_t Size);
	bool GetVersion(uint8_t *Version);
	bool GetBearing_8_Bit();
	bool GetBearing_16_Bit();
	bool GetPitch();
	bool GetRoll();
	bool UpdateRawData();
	bool GetMagRaw();
	bool GetAccelRaw();
	bool GetGyroRaw();
	int16_t GetGiroYaw();
	bool GetTemp();
	bool UpdateCalState();
	bool GetCalibrationState();
	bool GetBoschBearing(uint16_t *Bearing);
	bool GetPitch_180(uint16_t *Pitch);
	bool StoreCalibration();
	bool DeleteCalibration();
protected:
private:
	CCmps12( const CCmps12 &c );
	CCmps12& operator=( const CCmps12 &c );

}; //CCmps12

#endif //__CCMPS12_H__

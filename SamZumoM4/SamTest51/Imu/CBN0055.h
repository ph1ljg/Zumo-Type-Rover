/* 
* CBN0055.h
*
* Created: 20/08/2020 12:45:22
* Author: philg
*/


#ifndef __CBN0055_H__
#define __CBN0055_H__
#include "quaternion.h"

#define BN0055_ADDRESS_B (0x28)				// BNO055 Address B 

#define BN0055_ID (0xA0)					// BNO055 ID
#define BNO055_SAMPLERATE_DELAY_MS 10

#define NUM_BN0055_OFFSET_REGISTERS (22)	// Offsets registers

// A structure to represent offsets 
typedef struct 
{
  int16_t accel_offset_x; /**< x acceleration offset */
  int16_t accel_offset_y; /**< y acceleration offset */
  int16_t accel_offset_z; /**< z acceleration offset */

  int16_t mag_offset_x; /**< x magnetometer offset */
  int16_t mag_offset_y; /**< y magnetometer offset */
  int16_t mag_offset_z; /**< z magnetometer offset */

  int16_t gyro_offset_x; /**< x gyroscope offset */
  int16_t gyro_offset_y; /**< y gyroscope offset */
  int16_t gyro_offset_z; /**< z gyroscope offset */

  int16_t accel_radius; /**< acceleration radius */

  int16_t mag_radius; /**< magnetometer radius */
} BN0055_SENSOR_OFFSETS_t;

// BNO055 Registers
 typedef enum 
 {
    BNO055_PAGE_ID_ADDR = 0X07,				//Page id register definition
    BN0055_CHIP_ID_ADDR = 0x00,				// PAGE0 REGISTER DEFINITION START
    BNO055_ACCEL_REV_ID_ADDR = 0x01,
    BNO055_MAG_REV_ID_ADDR = 0x02,
    BNO055_GYRO_REV_ID_ADDR = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR = 0x05,
    BNO055_BL_REV_ID_ADDR = 0X06,

   
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08,		// Accel data register
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D,

    
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E,			// Mag data register 
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13,

   
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14,			 // Gyro data registers
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19,

    
    BNO055_EULER_H_LSB_ADDR = 0X1A,				// Euler data registers
    BNO055_EULER_H_MSB_ADDR = 0X1B,
    BNO055_EULER_R_LSB_ADDR = 0X1C,
    BNO055_EULER_R_MSB_ADDR = 0X1D,
    BNO055_EULER_P_LSB_ADDR = 0X1E,
    BNO055_EULER_P_MSB_ADDR = 0X1F,

   
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20,		// Quaternion data registers
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27,

    
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28,		// Linear acceleration data registers
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D,

    
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E,			// Gravity data registers
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33,

    
    BNO055_TEMP_ADDR = 0X34,						// Temperature data register 

    
    BNO055_CALIB_STAT_ADDR = 0X35,					// Status registers
    BNO055_SELFTEST_RESULT_ADDR = 0X36,
    BNO055_INTR_STAT_ADDR = 0X37,

    BNO055_SYS_CLK_STAT_ADDR = 0X38,
    BNO055_SYS_STAT_ADDR = 0X39,
    BNO055_SYS_ERR_ADDR = 0X3A,

    
    BNO055_UNIT_SEL_ADDR = 0X3B,					// Unit selection register
    BNO055_DATA_SELECT_ADDR = 0X3C,

    
    BNO055_OPR_MODE_ADDR = 0X3D,					// Mode registers
    BNO055_PWR_MODE_ADDR = 0X3E,

    BNO055_SYS_TRIGGER_ADDR = 0X3F,
    BNO055_TEMP_SOURCE_ADDR = 0X40,

    
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41,				// Axis remap registers
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42,

   
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43,			// SIC registers
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54,

    
    ACCEL_OFFSET_X_LSB_ADDR = 0X55,					// Accelerometer Offset registers
    ACCEL_OFFSET_X_MSB_ADDR = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A,

    
    MAG_OFFSET_X_LSB_ADDR = 0X5B,					// Magnetometer Offset registers
    MAG_OFFSET_X_MSB_ADDR = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR = 0X60,

    
    GYRO_OFFSET_X_LSB_ADDR = 0X61,					// Gyroscope Offset registers
    GYRO_OFFSET_X_MSB_ADDR = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR = 0X66,

    
    ACCEL_RADIUS_LSB_ADDR = 0X67,					// Radius registers 
    ACCEL_RADIUS_MSB_ADDR = 0X68,
    MAG_RADIUS_LSB_ADDR = 0X69,
    MAG_RADIUS_MSB_ADDR = 0X6A
  } Bno055Reg_t;

  
  typedef enum										// BNO055 power settings 
  {
    POWER_MODE_NORMAL = 0X00,
    POWER_MODE_LOWPOWER = 0X01,
    POWER_MODE_SUSPEND = 0X02
  } BN0055_powermode_t;

  
  typedef enum 
  {													// Operation mode settings
    OPERATION_MODE_CONFIG = 0X00,
    OPERATION_MODE_ACCONLY = 0X01,
    OPERATION_MODE_MAGONLY = 0X02,
    OPERATION_MODE_GYRONLY = 0X03,
    OPERATION_MODE_ACCMAG = 0X04,
    OPERATION_MODE_ACCGYRO = 0X05,
    OPERATION_MODE_MAGGYRO = 0X06,
    OPERATION_MODE_AMG = 0X07,
    OPERATION_MODE_IMUPLUS = 0X08,
    OPERATION_MODE_COMPASS = 0X09,
    OPERATION_MODE_M4G = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B,
    OPERATION_MODE_NDOF = 0X0C
  } BN0055_OPMODE_t;

typedef enum 
{													// Remap settings 
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
  } BN0055_REMAP_AXIS_t;

  
  typedef enum 
  {													// Remap Signs 
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
  } BN0055_AXIS_REMAP_SIGN_t;

  typedef struct 
  {													// A structure to represent revisions
    uint8_t accel_rev; /**< acceleration rev */
    uint8_t mag_rev;   /**< magnetometer rev */
    uint8_t gyro_rev;  /**< gyroscrope rev */
    uint16_t sw_rev;   /**< SW rev */
    uint8_t bl_rev;    /**< bootloader rev */
  } BN0055_rev_info_t;

  typedef enum 
  {													// Vector Mappings
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY = BNO055_GRAVITY_DATA_X_LSB_ADDR,
    VECTOR_YAW_RATE = BNO055_GRAVITY_DATA_X_LSB_ADDR
  } BN0055_VECTOR_t;


class CBN0055
{
//variables
public:
protected:
private:
	 BN0055_OPMODE_t m_OpMode;
//functions
public:
	CBN0055();
	~CBN0055();
	bool Init(BN0055_OPMODE_t mode = OPERATION_MODE_NDOF);
	void SetMode(BN0055_OPMODE_t mode);
	void SetAxisRemap( BN0055_REMAP_AXIS_t remapcode);
	void SetAxisSign(BN0055_AXIS_REMAP_SIGN_t remapsign);
	void SetExtCrystalUse(bool usextal);
	void GetSystemStatus(uint8_t *system_status,uint8_t *self_test_result,uint8_t *system_error);
	void DisplaySysStatus();
	void GetRevInfo(BN0055_rev_info_t *info);
	void GetCalibration(uint8_t *sys, uint8_t *gyro, uint8_t *accel, uint8_t *mag);
	int8_t GetTemp();
	bool GetVector(Vector3f &Vect,BN0055_VECTOR_t vector_type);
	bool GetData(Vector3f &Gyro,Vector3f &LinVel);
	ImuMath::Quaternion getQuat();
	const Matrix3f GetDcm();
	bool getSensorOffsets(BN0055_SENSOR_OFFSETS_t &offsets_type);
	bool getSensorOffsets(uint8_t *calibData);
	void setSensorOffsets( const BN0055_SENSOR_OFFSETS_t &offsets_type);
	void setSensorOffsets(const uint8_t *calibData);
	bool isFullyCalibrated();
	void EnterSuspendMode();
	void enterNormalMode();
	bool WriteByte(Bno055Reg_t reg, uint8_t value);
	uint8_t ReadByte(Bno055Reg_t reg);
	bool ReadBuffer(Bno055Reg_t reg, uint8_t *buffer,uint8_t len);
	void Test();
	void Test1();
	void Test2();
protected:
private:
	CBN0055( const CBN0055 &c );
	CBN0055& operator=( const CBN0055 &c );

}; //CBN0055
#endif //__CBN0055_H__

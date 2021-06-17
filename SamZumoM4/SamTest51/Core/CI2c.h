/* 
* CI2c.h
*
* Created: 18/05/2020 14:22:53
* Author: philg
*/


#ifndef __CI2C_H__
#define __CI2C_H__
#include "stddef.h"
#include  "stdio.h"
#include "Defines.h"
#include "CSercom.h"
#define  I2C_BUFFER_SIZE 10

#define I2c_FREQ_REF       48000000ul

//#define TWI_CLOCK 400000  // 100000
#define TWI_CLOCK  100000

typedef enum
{
	I2C_SLAVE_OPERATION = 0x4u,
	I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

typedef enum
{
	WIRE_WRITE_FLAG = 0x0ul,
	WIRE_READ_FLAG
} SercomWireReadWriteFlag;

typedef enum
{
	WIRE_MASTER_ACT_NO_ACTION = 0,
	WIRE_MASTER_ACT_REPEAT_START,
	WIRE_MASTER_ACT_READ,
	WIRE_MASTER_ACT_STOP
} SercomMasterCommandWire;

typedef enum
{
	WIRE_MASTER_ACK_ACTION = 0,
	WIRE_MASTER_NACK_ACTION
} SercomMasterAckActionWire;

typedef enum
{
	WIRE_UNKNOWN_STATE = 0x0ul,
	WIRE_IDLE_STATE,
	WIRE_OWNER_STATE,
	WIRE_BUSY_STATE
} SercomWireBusState;



/*
 * I2c Interfaces
 */

#define PIN_SDA         (12u)
#define PIN_SCL         (13u)


#define I2C_IT_HANDLER      SERCOM2_Handler
#define I2C_IT_HANDLER_0    SERCOM2_0_Handler
#define I2C_IT_HANDLER_1    SERCOM2_1_Handler
#define I2C_IT_HANDLER_2    SERCOM2_2_Handler
#define I2C_IT_HANDLER_3    SERCOM2_3_Handler

static const uint8_t SDA = PIN_SDA;
static const uint8_t SCL = PIN_SCL;





typedef struct
{
	uint16_t m_Head,m_Tail,Len, Size;
	unsigned char *Buffer;
	uint8_t Error;
} I2cCirBuffer_t;



class CI2c
{
//variables
public:
		uint8_t TxBuffer[I2C_BUFFER_SIZE];
		uint8_t RxBuffer[I2C_BUFFER_SIZE];
		uint16_t m_I2cErrorsTotal;

protected:
private:
//	SERCOM * sercom;
	uint8_t m_PortNoSDA;
	uint8_t m_PortNoSCL;
	Sercom* m_SercomRegStruct;
	EPortType m_Port;



//functions
public:
	CI2c();
	CI2c(const void *const s, EPortType port, uint8_t pinSDA, uint8_t pinSCL);
	void init(void);
	void Reset();
	void Enable();
	void disableWIRE();
	void initSlaveWIRE( uint8_t ucAddress, bool enableGeneralCall );
	void InitMaster( uint32_t baudrate );
	void prepareNackBitWIRE( void );
	void prepareAckBitWIRE( void );
	void prepareCommandBitsWire(uint8_t cmd);
	bool startTransmissionWIRE(uint8_t address, SercomWireReadWriteFlag flag);
	bool sendDataMasterWIRE(uint8_t data);
	bool sendDataSlaveWIRE(uint8_t data);
	bool isMasterWIRE( void );
	bool isSlaveWIRE( void );
	bool isBusIdleWIRE( void );
	bool isBusOwnerWIRE( void );
	bool isDataReadyWIRE( void );
	bool isStopDetectedWIRE( void );
	bool isRestartDetectedWIRE( void );
	bool isAddressMatch( void );
	bool isMasterReadOperationWIRE( void );
	bool isRXNackReceivedWIRE( void );
	int Available( void );
//	int availableWIRE( void );
	uint8_t readDataWIRE( void );
	void SetClock(uint32_t baudrate);
	bool ReadRegister(uint8_t SlaveAddress, uint8_t Register, uint8_t *Bytes, uint8_t Size);
	bool ReadRegisterAdd16(uint8_t SlaveAddress, uint16_t Register, uint8_t *Bytes, uint8_t Size);
	uint8_t ReadBytes(uint8_t address, uint8_t *Bytes,uint8_t Size);
	bool WriteByte(uint8_t SlaveAddress , uint8_t Data);
	bool WriteBytes(uint8_t SlaveAddress , uint8_t *Data,unsigned int DataLength );
	bool WriteRegisterByte(uint8_t SlaveAddress,uint8_t SlaveRegister , uint8_t Data);
	bool WriteRegisterBytes(uint8_t SlaveAddress,uint8_t SlaveRegister , uint8_t *Data,unsigned int DataLength);
	bool WriteRegisterBytesAdd16(uint8_t SlaveAddress,uint16_t SlaveRegister , uint8_t *Data,unsigned int DataLength);
	void UpdateErrors();
	~CI2c();
protected:
private:
	CI2c( const CI2c &c );
	CI2c& operator=( const CI2c &c );

}; //CI2c

#endif //__CI2C_H__

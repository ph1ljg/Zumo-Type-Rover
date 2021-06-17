/* 
* CQSpiFlash.h
*
* Created: 25/05/2020 14:43:03
* Author: philg
*/


#ifndef __CQSPIFLASH_H__
#define __CQSPIFLASH_H__
#include "QSpiFlashDevices.h"


// On-board QSPI Flash
#define EXTERNAL_FLASH_DEVICES   GD25Q16C
#define EXTERNAL_FLASH_USE_QSPI

//QSPI Pins
#define PIN_QSPI_SCK    (10u)		// PORTB 10
#define PIN_QSPI_CS     (11u)		// PORTB 11 
#define PIN_QSPI_IO0    (8u)		// PORTA 8
#define PIN_QSPI_IO1    (9u)		// PORTA 9 
#define PIN_QSPI_IO2    (10u)		// PORTA 10 
#define PIN_QSPI_IO3    (11u)		// PORTA 11




#if !defined(VARIANT_QSPI_BAUD_DEFAULT)
  // TODO: meaningful value for this
  #define VARIANT_QSPI_BAUD_DEFAULT 5000000
#endif




class CQSpiFlash
{
//variables
public:
  /// Constant that is (mostly) true to all external flash devices
  enum 
  {
	  QSPI_FLASH_BLOCK_SIZE  = 64*1024,
	  QSPI_FLASH_SECTOR_SIZE = 4*1024,
	  QSPI_FLASH_PAGE_SIZE   = 256
  };

protected:
private:
	external_flash_device const * _flash_dev;

	void _wait_for_flash_ready(void)
	{
		while ( readStatus() & 0x03 ) {}		// both WIP and WREN bit should be clear
	}


//functions
public:
	CQSpiFlash();
	~CQSpiFlash();
	bool Init(void);
	bool end(void);
	void GetManufacturerInfo (uint8_t *manufID, uint8_t *deviceID);
	uint32_t GetJEDECID (void);
	uint8_t readStatus(void);
	uint8_t readStatus2(void);
	bool writeEnable(void);
	uint32_t readBuffer (uint32_t address, uint8_t *buffer, uint32_t len);
	uint32_t writeBuffer (uint32_t addr, uint8_t *data, uint32_t len);
	uint8_t read8(uint32_t addr);
	uint16_t read16(uint32_t addr);
	uint32_t read32(uint32_t addr);
	bool chipErase(void);
	bool eraseSector (uint32_t sectorNumber);
	bool eraseBlock (uint32_t blockNumber);
	void Test();
protected:
private:
	CQSpiFlash( const CQSpiFlash &c );
	CQSpiFlash& operator=( const CQSpiFlash &c );

}; //CQSpiFlash
extern CQSpiFlash QSpiFlash;
#endif //__CQSPIFLASH_H__

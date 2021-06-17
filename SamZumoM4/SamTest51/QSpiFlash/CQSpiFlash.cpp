/* 
* CQSpiFlash.cpp
*
* Created: 25/05/2020 14:43:03
* Author: philg
*/

#include "Includes.h"


// default constructor
CQSpiFlash::CQSpiFlash()
{
  _flash_dev = NULL;

} //CQSpiFlash

// default destructor
CQSpiFlash::~CQSpiFlash()
{
} //~CQSpiFlash


/// List of all possible flash devices used by Adafruit boards
static const external_flash_device possible_devices[] =
{
  GD25Q16C, GD25Q64C,    // Main devices current Adafruit
  S25FL116K, S25FL216K,
  W25Q16FW, W25Q64JV_IQ, // Only a handful of production run

  MX25R6435F,            // Nordic PCA10056
};

/// Flash device list count
enum
{
  EXTERNAL_FLASH_DEVICE_COUNT = sizeof(possible_devices)/sizeof(possible_devices[0])
};



//======================================================================================================================================
//	Initialize QSPI peripheral and external flash device. It will  also detect all known flash devices on the board and enable quad mode 
//	if  found. QSPI is also set to max possible speed with device.
//	SCK PORTB 10
//	CS  PORTB 11 
// 	IO0 PORTA 8
// 	IO1 PORTA 9
// 	IO2 PORTA 10 
// 	IO3 PORTA 11
// 	per PIO_COM
//=======================================================================================================================================
bool CQSpiFlash::Init(void)
{

	QSPI0.Init(10, 11, PIN_QSPI_IO0, PIN_QSPI_IO1, PIN_QSPI_IO2, PIN_QSPI_IO3);

	uint8_t jedec_ids[3];
	QSPI0.readCommand(QSPI_CMD_READ_JEDEC_ID, jedec_ids, 3);

	for (uint8_t i = 0; i < EXTERNAL_FLASH_DEVICE_COUNT; i++) 
	{
		const external_flash_device* possible_device = &possible_devices[i];
		if (jedec_ids[0] == possible_device->manufacturer_id &&	jedec_ids[1] == possible_device->memory_type &&	jedec_ids[2] == possible_device->capacity) 
		{
			_flash_dev = possible_device;
			break;
		}
	}

	if (_flash_dev == NULL) return false;

  // We don't know what state the flash is in so wait for any remaining writes and then reset.

  // The write in progress bit should be low.
  while ( readStatus() & 0x01 ) {}

  // The suspended write/erase bit should be low.
  while ( readStatus2() & 0x80 ) {}

  QSPI0.runCommand(QSPI_CMD_ENABLE_RESET);
  QSPI0.runCommand(QSPI_CMD_RESET);

  // Wait 30us for the reset
  Core.delayMicroseconds(30);

  // Speed up to max device frequency
  QSPI0.setClockSpeed(_flash_dev->max_clock_speed_mhz*1000000UL);

  // Enable Quad Mode if available
  if (_flash_dev->quad_enable_bit_mask)
  {
    // Verify that QSPI mode is enabled.
    uint8_t status = _flash_dev->single_status_byte ? readStatus() : readStatus2();

    // Check the quad enable bit.
    if ((status & _flash_dev->quad_enable_bit_mask) == 0)
    {
        writeEnable();

        uint8_t full_status[2] = {0x00, _flash_dev->quad_enable_bit_mask};

        if (_flash_dev->write_status_register_split) 
		{
            QSPI0.writeCommand(QSPI_CMD_WRITE_STATUS2, full_status + 1, 1);
        } 
		else if (_flash_dev->single_status_byte) 
		{
            QSPI0.writeCommand(QSPI_CMD_WRITE_STATUS, full_status + 1, 1);
        } 
		else {
            QSPI0.writeCommand(QSPI_CMD_WRITE_STATUS, full_status, 2);
        }
    }
  }

  // Turn off sector protection if needed
//  if (_flash_dev->has_sector_protection)
//  {
//    writeEnable();
//
//    uint8_t data[1] = {0x00};
//    QSPI0.writeCommand(QSPI_CMD_WRITE_STATUS, data, 1);
//  }

  // Turn off writes in case this is a microcontroller only reset.
  QSPI0.runCommand(QSPI_CMD_WRITE_DISABLE);

  _wait_for_flash_ready();

  // Adafruit_SPIFlash variables
//   currentAddr = 0;
//   addrsize = 24;
//   totalsize = _flash_dev->total_size;
//   pagesize = QSPI_FLASH_PAGE_SIZE;
//   pages = totalsize/QSPI_FLASH_PAGE_SIZE;

//  type is ignored

	return true;
}

/**
 * Disable qspi peripheral clock
 * @return true if success
 */
bool CQSpiFlash::end(void)
{
  // TODO implement later
  return true;
}

/**************************************************************************/
/*! 
   @brief read the manufacturer ID and device ID
   @param manufID pointer to where to put the manufacturer ID
   @param deviceID pointer to where to put the device ID
*/
/**************************************************************************/
void CQSpiFlash::GetManufacturerInfo (uint8_t *manufID, uint8_t *deviceID)
{
  uint32_t jedec_id = GetJEDECID();

	*deviceID = (uint8_t) (jedec_id & 0xffUL);
	*manufID  = (uint8_t) (jedec_id >> 16);
}

/**************************************************************************/
/*! 
   @brief read JEDEC ID information from the device
   @returns the read id as a uint32
*/
/**************************************************************************/
uint32_t CQSpiFlash::GetJEDECID (void)
{
	uint8_t ids[3];
	QSPI0.readCommand(QSPI_CMD_READ_JEDEC_ID, ids, 3);

	return (ids[0] << 16) | (ids[1] << 8) | ids[2];
}

/**************************************************************************/
/*! 
   @brief read the generic status register.
   @returns the status register reading
*/
/**************************************************************************/
uint8_t CQSpiFlash::readStatus(void)
{
	uint8_t r;
	QSPI0.readCommand(QSPI_CMD_READ_STATUS, &r, 1);
	return r;
}

/**
 * Read the second status register
 * @return status2 register
 */
uint8_t CQSpiFlash::readStatus2(void)
{
	uint8_t r;
	QSPI0.readCommand(QSPI_CMD_READ_STATUS2, &r, 1);
	return r;
}

/**
 * Execute Write Enable command
 * @return true if success
 */
bool CQSpiFlash::writeEnable(void)
{
  return QSPI0.runCommand(QSPI_CMD_WRITE_ENABLE);
}

/**
 * Read data from external flash contents. Typically it is implemented by quad read command 0x6B
 * @param address   address to read
 * @param buffer    buffer to hold data
 * @param len       number of byte to read
 * @return true if success
 */
uint32_t CQSpiFlash::readBuffer (uint32_t address, uint8_t *buffer, uint32_t len)
{
  if (!_flash_dev) return 0;

  _wait_for_flash_ready();

  return QSPI0.readMemory(address, buffer, len) ? len : 0;
}

/**
 * Write data to external flash contents, flash sector must be previously erased by \ref eraseSector() first.
 * Typically it uses quad write command 0x32
 * @param addr       address to read
 * @param data       writing data
 * @param len        number of byte to read
 * @return true if success
 */
uint32_t CQSpiFlash::writeBuffer (uint32_t addr, uint8_t *data, uint32_t len)
{
  if (!_flash_dev) return 0;

  uint32_t remain = len;

	//write one page at a time
	while(remain)
	{
	  _wait_for_flash_ready();
	  writeEnable();

	  uint16_t const toWrite = MIN(remain,(uint32_t) QSPI_FLASH_PAGE_SIZE);

		if ( !QSPI0.writeMemory(addr, data, toWrite) ) break;

		remain -= toWrite;
		data += toWrite;
		addr += toWrite;
	}

	return len - remain;
}

/**
 * Read one byte from flash device
 * @param addr address to read
 * @return the data byte read
 */
uint8_t CQSpiFlash::read8(uint32_t addr)
{
	uint8_t ret;
	return readBuffer(addr, &ret, sizeof(ret)) ? 0xff : ret;
}

/**
 * Read uint16_t from flash device
 * @param addr address to read
 * @return the data uint16_t read
 */
uint16_t CQSpiFlash::read16(uint32_t addr)
{
	uint16_t ret;
	return readBuffer(addr, (uint8_t*) &ret, sizeof(ret)) ? 0xffff : ret;
}

/**
 * Read uint32_t from flash device
 * @param addr address to read
 * @return the data uint32_t read
 */
uint32_t CQSpiFlash::read32(uint32_t addr)
{
	uint32_t ret;
	return readBuffer(addr, (uint8_t*) &ret, sizeof(ret)) ? 0xffffffff : ret;
}

/**************************************************************************/
/*! 
    @brief perform a chip erase. All data on the device will be erased.
    @return true if success
*/
/**************************************************************************/
bool CQSpiFlash::chipErase(void)
{
  if (!_flash_dev) return false;

  // We need to wait for any writes to finish
  _wait_for_flash_ready();

	writeEnable();

	return QSPI0.runCommand(QSPI_CMD_ERASE_CHIP);
}

/**************************************************************************/
/*! 
    @brief erase a sector of flash
    @param sectorNumber the sector number to erase. The address erased will be (sectorNumber * 4096)
    @return true if success
*/
/**************************************************************************/
bool CQSpiFlash::eraseSector (uint32_t sectorNumber)
{
  if (!_flash_dev) return false;

  // Before we erase the sector we need to wait for any writes to finish
  _wait_for_flash_ready();

  writeEnable();

	return QSPI0.eraseCommand(QSPI_CMD_ERASE_SECTOR, sectorNumber * QSPI_FLASH_SECTOR_SIZE);
}

/**
 * Erase a 64KB block of flash
 * @param blockNumber Address to be erased
 * @return true if success
 */
bool CQSpiFlash::eraseBlock  (uint32_t blockNumber)
{
  if (!_flash_dev) return false;

  // Before we erase the sector we need to wait for any writes to finish
  _wait_for_flash_ready();

  writeEnable();

  return QSPI0.eraseCommand(QSPI_CMD_ERASE_BLOCK, blockNumber * QSPI_FLASH_BLOCK_SIZE);
}


void CQSpiFlash::Test()
{
	#define PROGSIZE 512

	uint8_t progData[PROGSIZE];
	uint8_t readData[PROGSIZE];
	if(!Init())
	{
		DebugDisplay.Printf("Could not find flash on QSPI bus!\n");
		while(1);
	}
	DebugDisplay.Printf("Found QSPI Flash\n");

	for (int i=0; i<PROGSIZE; i++)
		progData[i] = i;

	DebugDisplay.Printf("Erasing...\n");
	chipErase();
	DebugDisplay.Printf("done!\n");

	DebugDisplay.Printf("Writing...\n");
	writeBuffer(0, progData, sizeof(progData));
	DebugDisplay.Printf("done!\n");

	DebugDisplay.Printf("Reading...\n");
	readBuffer(0, readData, sizeof(readData));
	DebugDisplay.Printf("done!\n");

	for (int i=0; i<PROGSIZE; i++)
	{
		if(readData[i] != progData[i])
		{
			DebugDisplay.Printf("ERROR: mismatch found at index %u\n",i);
			while(1);
		}
	}

	DebugDisplay.Printf("Verify successful!\n");
	while(1);
}


/* 
* CQSpi.h
*
* Created: 25/05/2020 18:45:47
* Author: philg
*/


#ifndef __CQSPI_H__
#define __CQSPI_H__

enum
{
	QSPI_CMD_QUAD_READ         = 0x6B, // 1 line address, 4 line data

	QSPI_CMD_READ_JEDEC_ID     = 0x9f,

	QSPI_CMD_PAGE_PROGRAM      = 0x02,
	QSPI_CMD_QUAD_PAGE_PROGRAM = 0x32, // 1 line address, 4 line data

	QSPI_CMD_READ_STATUS       = 0x05,
	QSPI_CMD_READ_STATUS2      = 0x35,

	QSPI_CMD_WRITE_STATUS      = 0x01,
	QSPI_CMD_WRITE_STATUS2     = 0x31,

	QSPI_CMD_ENABLE_RESET      = 0x66,
	QSPI_CMD_RESET             = 0x99,

	QSPI_CMD_WRITE_ENABLE      = 0x06,
	QSPI_CMD_WRITE_DISABLE     = 0x04,

	QSPI_CMD_ERASE_SECTOR      = 0x20,
	QSPI_CMD_ERASE_BLOCK       = 0xD8,
	QSPI_CMD_ERASE_CHIP        = 0xC7,
};



class CQSpi
{
//variables
public:
protected:
private:

//functions
public:
	CQSpi();
	~CQSpi();
	void Init(int sck, int cs, int io0, int io1, int io2, int io3);
	bool _run_instruction(uint8_t command, uint32_t iframe, uint32_t addr, uint8_t *buffer, uint32_t size);
	bool runCommand(uint8_t command);
	bool readCommand(uint8_t command, uint8_t* response, uint32_t len);
	bool writeCommand(uint8_t command, uint8_t const* data, uint32_t len);
	bool eraseCommand(uint8_t command, uint32_t address);
	bool readMemory(uint32_t addr, uint8_t *data, uint32_t len);
	bool writeMemory(uint32_t addr, uint8_t *data, uint32_t len);
	void setClockDivider(uint8_t uc_div);
	void setClockSpeed(uint32_t clock_hz);
protected:
private:
	CQSpi( const CQSpi &c );
	CQSpi& operator=( const CQSpi &c );

}; //CQSpi

extern CQSpi QSPI0; ///< default QSPI instance

#endif //__CQSPI_H__

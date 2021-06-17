/*
* CSpi.h
*
* Created: 18/05/2020 11:30:11
* Author: philg
*/


#ifndef __CSPI_H__
#define __CSPI_H__


typedef enum
{
	SPI_CHAR_SIZE_8_BITS = 0x0ul,
	SPI_CHAR_SIZE_9_BITS
} SercomSpiCharSize;


//extern SERCOM sercom1;
// SPI Interfaces
#define SPI_INTERFACES_COUNT 1

#define PIN_SPI_MISO         (22u)
#define PIN_SPI_MOSI         (23u)
#define PIN_SPI_SCK          (17u)
#define PERIPH_SPI           sercom1
#define PAD_SPI_TX           SPI_PAD_3_SCK_1
#define PAD_SPI_RX           SERCOM_RX_PAD_2




const uint8_t SS	  = 9 ;	// SERCOM1 last PAD is present on d9 but HW SS isn't used. Set here only for reference.
const uint8_t MOSI = PIN_SPI_MOSI ;
const uint8_t MISO = PIN_SPI_MISO ;
const uint8_t SCK  = PIN_SPI_SCK ;



#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

#define SPI_MODE0 0x02
#define SPI_MODE1 0x00
#define SPI_MODE2 0x03
#define SPI_MODE3 0x01

// SAMD51 has configurable MAX_SPI, else use peripheral clock default.
// Update: changing MAX_SPI via compiler flags is DEPRECATED, because
// this affects ALL SPI peripherals including some that should NOT be
// changed (e.g. anything using SD card). Use the setClockSource()
// function instead. This is left here for compatibility with interim code.
#if !defined(MAX_SPI)
#define MAX_SPI 24000000
#endif
#define SPI_MIN_CLOCK_DIVIDER 1

#define SPI_CLOCK_RATE 4000000


#define SPI_SPEED_SLOW    1000000
#define SPI_SPEED_FAST    6000000


typedef enum
{
	SPI_MSB_FIRST = 0,
	SPI_LSB_FIRST
} eDataOrder;



class SPISettings
{
	public:
	SPISettings(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
	{
		if (__builtin_constant_p(clock))
		{
			init_AlwaysInline(clock, bitOrder, dataMode);
		}
		else
		{
			init_MightInline(clock, bitOrder, dataMode);
		}
	}

	// Default speed set to 4MHz, SPI mode set to MODE 0 and Bit order set to MSB first.
	
	SPISettings() { init_AlwaysInline(4000000, MSBFIRST, SPI_MODE0); }

	uint32_t clockFreq;
	SercomSpiClockMode dataMode;
	SercomDataOrder bitOrder;

	
	private:
	void init_MightInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode)
	{
		init_AlwaysInline(clock, bitOrder, dataMode);
	}

	void init_AlwaysInline(uint32_t clock, BitOrder bitOrder, uint8_t dataMode) __attribute__((__always_inline__))
	{
		this->clockFreq = clock; // Clipping handled in SERCOM.cpp

		this->bitOrder = (bitOrder == MSBFIRST ? MSB_FIRST : LSB_FIRST);

		switch (dataMode)
		{
			case SPI_MODE0:
			this->dataMode = SERCOM_SPI_MODE_0; break;
			case SPI_MODE1:
			this->dataMode = SERCOM_SPI_MODE_1; break;
			case SPI_MODE2:
			this->dataMode = SERCOM_SPI_MODE_2; break;
			case SPI_MODE3:
			this->dataMode = SERCOM_SPI_MODE_3; break;
			default:
			this->dataMode = SERCOM_SPI_MODE_0; break;
		}
	}

	friend class SPIClass;
};

class CSpi
{
	//variables
	public:
	protected:
	private:
	//	SERCOM *_p_sercom;
	EPortType m_MosiPort;
	EPortType m_MisoPort;
	EPortType m_SckPort;
	uint8_t m_PinNoMiso;
	uint8_t m_PinNoMosi;
	uint8_t m_PinNoSCK;

	SercomSpiTXPad m_PadTx;
	Sercom* m_SercomRegStruct;
	SercomRXPad m_PadRx;

	bool initialized;
	uint8_t interruptMode;
	char interruptSave;
	uint32_t interruptMask;
	uint32_t m_MainClock;
	EPortType m_Port;
	EPortType m_ClkPort;
	//functions
	public:
	CSpi();
	CSpi(const void *const s, EPortType MOSIport, EPortType MISOport,EPortType CLKport,uint8_t PinNoMISO, uint8_t PinNoSCK, uint8_t PinNoMOSI, SercomSpiTXPad PadTx, SercomRXPad PadRx);
	void Init();
	void SetSpiClock(uint32_t Speed);
	void Config();
	void InitSPI(SercomSpiTXPad mosi, SercomRXPad miso, uint8_t charSize, uint8_t dataOrder);
	void EnableSPI();
	void ResetSPI();
	void InitSPIClock(SercomSpiClockMode clockMode, uint32_t baudrate);
	void DisableSPI();
	void end();
	void SetDataOrderSPI(eDataOrder dataOrder);
	eDataOrder GetDataOrder();
	void SetBaudrate(uint8_t Divider);
	void usingInterrupt(int interruptNumber);
	void SetClockModeSPI(SercomSpiClockMode clockMode);
	uint8_t Transfer(uint8_t data);
	bool IsBufferOverflowError();
	bool IsDataRegisterEmpty();
	void notUsingInterrupt(int interruptNumber);
	bool GetInterruptNo(int IntNo,EExt_Interrupts &ExtInt);
	void BeginTransaction();
	void endTransaction(void);
	void setBitOrder(BitOrder order);
	void SetDataMode(uint8_t mode);
	void setClockDivider(uint8_t div);
	void Transfer(void *buf, size_t count);
	void attachInterrupt();
	void detachInterrupt();
	void setClockSource(SercomClockSource clk);
	uint16_t Transfer16(uint16_t data);
	~CSpi();
	protected:
	private:
	CSpi( const CSpi &c );
	CSpi& operator=( const CSpi &c );

}; //CSpi

#endif //__CSPI_H__

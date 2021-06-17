/*
* CUart.h
*
* Created: 14/05/2020 12:23:49
* Author: philg
*/


#ifndef __CUART_H__
#define __CUART_H__
#include "stddef.h"
#include "CSercom.h"


#define SERIAL0_RX_BUFFER_SIZE 800
#define SERIAL0_TX_BUFFER_SIZE 600
#define SERIAL1_RX_BUFFER_SIZE 600
#define SERIAL1_TX_BUFFER_SIZE 800


#define COLLISION_ERROR			1
#define SYNC_FIELD_ERROR		2
#define BUFFER_OVERFLOW_ERROR	4
#define FRAME_ERROR				8
#define PARITY_ERROR			16



typedef enum
{
	UART_EXT_CLOCK = 0,
	UART_INT_CLOCK = 0x1u
} SercomUartMode;

typedef enum
{
	UART_CHAR_SIZE_8_BITS = 0,
	UART_CHAR_SIZE_9_BITS,
	UART_CHAR_SIZE_5_BITS = 0x5u,
	UART_CHAR_SIZE_6_BITS,
	UART_CHAR_SIZE_7_BITS
} SercomUartCharSize;


typedef enum
{
	UART_TX_PAD_0 = 0x0ul,  // Only for UART
	UART_TX_PAD_2 = 0x1ul,  // Only for UART
	UART_TX_RTS_CTS_PAD_0_2_3 = 0x2ul,  // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
	SERCOM_EVEN_PARITY = 0,
	SERCOM_ODD_PARITY,
	SERCOM_NO_PARITY
} SercomParityMode;

#define NO_PARITY 0
typedef enum
{
	SERCOM_STOP_BIT_1 = 0,
	SERCOM_STOP_BITS_2
} SercomNumberStopBit;


typedef enum
{
	SAMPLE_RATE_x16 = 0x1,  // Fractional
	SAMPLE_RATE_x8  = 0x3,  // Fractional
} SercomUartSampleRate;


#define HARDSER_PARITY_EVEN (0x1ul)
#define HARDSER_PARITY_ODD	(0x2ul)
#define HARDSER_PARITY_NONE (0x3ul)
#define HARDSER_PARITY_MASK	(0xFul)

#define HARDSER_STOP_BIT_1		(0x10ul)
#define HARDSER_STOP_BIT_1_5	(0x20ul)
#define HARDSER_STOP_BIT_2	 	(0x30ul)
#define HARDSER_STOP_BIT_MASK	(0xF0ul)

#define HARDSER_DATA_5	 	(0x100ul)
#define HARDSER_DATA_6	 	(0x200ul)
#define HARDSER_DATA_7	 	(0x300ul)
#define HARDSER_DATA_8	 	(0x400ul)
#define HARDSER_DATA_MASK	(0xF00ul)

#define NO_RTS_PIN 255
#define NO_CTS_PIN 255
#define RTS_RX_THRESHOLD 10


// Serial1
// #define PIN_SERIAL1_RX       (0ul)
// #define PIN_SERIAL1_TX       (1ul)
// #define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
// #define PAD_SERIAL1_TX       (UART_TX_PAD_0)
#define SERIAL_8N1	(HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_8)
#define SERIAL_8E2	(HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_8)
typedef struct
{
	uint16_t m_Head,m_Tail,m_Len, Size;
	unsigned char *Buffer;
	uint8_t Error;
} CirBuffer_t;


class CUart
{
	//variables
public:
	CirBuffer_t m_RecieveQue;
	CirBuffer_t m_TransmitQue;
	//	SERCOM * sercom;
	Sercom* m_SercomRegStruct;
protected:
private:
	//	 SERCOM *sercom;
	uint8_t m_PortPinRX;
	uint8_t m_PortPinTX;
	SercomPad_t m_SercomPadRX;
	SercomPad_t m_SercomPadTX;
	uint8_t uc_pinRTS;
	uint8_t uc_pinCTS;
	volatile uint32_t* pul_outsetRTS;
	volatile uint32_t* pul_outclrRTS;
	uint32_t ul_pinMaskRTS;
	uint8_t m_UartNo;
	uint8_t m_CommsError;
	//functions
public:
	CUart();
	CUart(const void *const s, EPortType port,uint8_t PortPinRx, uint8_t PortPinTx, SercomPad_t PadRX, SercomPad_t PadTX,uint8_t UartNo);
	void Init(unsigned long baudrate);
	void DisplayPrintf(const char *fmt, ... );
	void end();
	void FlushTx();
	void StartTx();
	bool WriteTxBuffer(uint8_t *Buffer, uint16_t Size,bool StartTx = true);
	void IrqHandler();
	bool WriteChar( uint8_t data);
	size_t WriteChar1(const uint8_t data);
	bool IsFull(CirBuffer_t *Buffer);
	void ClearTxBuffer();
	void ClearRxBuffer();
	void ClearBuffer(CirBuffer_t *Buffer);
	uint16_t Write(CirBuffer_t *Buffer,unsigned char ch);
	bool Read(unsigned char *Data);
	bool TxRead(unsigned char *Data);
	uint16_t TxBufferSize();
	uint16_t TxAvailable();
	uint16_t Available();
	uint16_t BufferAvailable(CirBuffer_t* Buffer);
	SercomNumberStopBit extractNbStopBit(uint16_t config);
	SercomUartCharSize extractCharSize(uint16_t config);
	SercomParityMode extractParity(uint16_t config);
	void initUART(SercomUartMode mode, SercomUartSampleRate sampleRate, uint32_t baudrate);
	void initFrame(SercomUartCharSize charSize, SercomDataOrder dataOrder, SercomParityMode parityMode, SercomNumberStopBit nbStopBits);
	void InitPads(SercomPad_t txPad, SercomPad_t rxPad);
	void Reset();
	void Enable();
	void Flush();
	void ClearStatus();
	bool AvailableData();
	bool isUARTError();
	void acknowledgeUARTError();
	bool isBufferOverflowErrorUART();
	bool isFrameErrorUART();
	void clearFrameErrorUART();
	bool isParityErrorUART();
	bool isDataRegisterEmptyUART();
	uint8_t readDataUART();
	int WriteTxUnbuffered(uint8_t data);
	int WriteTxUnbuffered(uint8_t *data,uint8_t Size);
	void SetTxDREInterrupt(bool Set);
	void SetTxRXCInterrupt(bool Set);
	bool IsSyncing( uint32_t reg);
	void Write_CTRLA_reg( uint32_t data);
	void WaitForSync( uint32_t reg);
	uint32_t GetCTRLAReg(uint32_t mask);
	uint32_t GetUartMode();
	void ClearCTRLA_ENABLE_bit();
	void WriteCTRLAReg( uint32_t data);
	void InitClock(uint8_t UartNo);
	void Uart_1_InitClock(void);
	~CUart();
	void RxInterruptHandler();
	void TxInterruptHandler();
	protected:
	private:
	CUart( const CUart &c );
	CUart& operator=( const CUart &c );
	EPortType m_Port;
}; //CUart

#endif //__CUART_H__

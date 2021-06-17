/*
* CSoftwareSerial.h
*
* Created: 07/06/2020 13:04:04
* Author: philg
*/
#ifndef __CSOFTWARESERIAL_H__
#define __CSOFTWARESERIAL_H__


#include "Defines.h"
#include "CCirBuffer.h"
#define OVERSAMPLE 3
#define SS_INTERRUPT_PRIORITY_1 4
#define SS_INTERRUPT_PRIORITY_2 5

#define SS_INVERSE true
#define SS_NORMAL false

#define FULL_DUPLEX false
#define HALF_DUPLEX true
#define PORT_1_RECEIVE_BUFFER_LEN 400
#define PORT_1_TRANSMIT_BUFFER_LEN 400

#define PORT_2_RECEIVE_BUFFER_LEN 500
#define PORT_2_TRANSMIT_BUFFER_LEN 400


#define TX_ACTIVE	true;
#define TX_INACTIVE	false;
#define RX_ACTIVE	true;
#define RX_INACTIVE	false;

typedef struct
{
	uint16_t Head,Tail,Len, Size;
	uint8_t *Buffer;
} SSCirBuffer_t;




#define DIRECTION_INHIBIT 255
#define TRANSMIT true
#define RECIEVE false


class CSoftwareSerial
{
	//variables
	public:
	uint16_t m_BufferOverflow:1;
	Tc* m_Timer;
	SSCirBuffer_t m_ReciveBuffer;
	SSCirBuffer_t m_TransmitBuffer;


	protected:
	private:
	uint16_t m_InverseLogic:1;
	uint16_t m_IsHalfDuplex:1;
	uint16_t _output_pending:1;

	int32_t m_TxTickCount;
	int32_t m_RxTickCount;
	int32_t m_TransmitBitCount;
	uint32_t m_RxRegister;
	int32_t m_RxBitCount;
	uint32_t cur_speed;
	uint32_t m_Baud;
	uint8_t m_PortPinRX;
	uint8_t m_PortPinTX;
	EPortType m_Port;
	bool m_ActiveIn;
	bool m_ActiveOut;
	bool m_TxInProgress;
	uint16_t m_TxRegister;
	uint8_t m_PortNo;
	unsigned char *m_Buffer;
	
	//functions
	public:
	CSoftwareSerial();
	CSoftwareSerial(EPortType port,uint8_t PortPinRx, uint8_t PortPinTx,bool Inverse,bool HalfDuplex,uint8_t PortNo);
	~CSoftwareSerial();
	void TestPorts();
	void TestTx();
	void Init(uint32_t BaudRate);
	void SetInterupt(bool OffOn);
	void Enable(bool YesNo);
	void handle_interrupt();
	void recv();
	bool SetDirection(bool TxRx);
	void send();
	bool Write(uint8_t* Buffer,uint8_t Size,bool TransmitBuffer= true);
	bool WriteBufferBeforeTx(uint8_t* Buffer,uint8_t Size);
	bool WriteTxBuffer(uint8_t *Buffer, uint16_t Size,bool StartTx);
	bool Read(uint8_t *Data);
	int Available();
	int TxAvailable();
	void ClearRxBuffer();
	void ClearTxBuffer();
	uint8_t Peek(SSCirBuffer_t *Buffer);
	void SetBaud(uint32_t Baud);
	bool IsFull(SSCirBuffer_t *Buffer);
	void ClearBuffer(SSCirBuffer_t *Buffer);
	bool WriteChar( uint8_t Data);
	uint16_t WriteBuffer(SSCirBuffer_t *Buffer, uint8_t ch,bool TransmitBuffer = true);
	bool ReadBuffer(SSCirBuffer_t *Buffer,unsigned char *Data);
	protected:
	private:
	CSoftwareSerial( const CSoftwareSerial &c );
	CSoftwareSerial& operator=( const CSoftwareSerial &c );

}; //CSoftwareSerial

#endif //__CSOFTWARESERIAL_H__

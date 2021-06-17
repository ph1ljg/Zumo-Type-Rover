/* 
* CSbusDecoder.h
*
* Created: 16/04/2019 18:26:25
* Author: phil
*/

#ifndef __CSBUSDECODER_H__
#define __CSBUSDECODER_H__


#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		22
#define SBUS_FAILSAFE_BIT	0x04
#define SBUS_FRAMELOST_BIT	0x08

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

// pre-calculate the floating point stuff as far as possible at compile time 
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))


#define CHANNEL_SCALE(x) ((int32_t(x) * SBUS_TARGET_RANGE) / SBUS_RANGE_RANGE + SBUS_SCALE_OFFSET)

#define SBUS_TIMEOUT_US 7000
#define SBUS_HEADER		0x0F
#define SBUS_FOOTER_1	0x00
#define SBUS_FOOTER_2	0x04
#define SBUS_2_MASK		0x0F


#define PACKET_SIZE		24

typedef   struct   __attribute__ ((packed))
{
        uint32_t ch0 : 11;
        uint32_t ch1 : 11;
        uint32_t ch2 : 11;
        uint32_t ch3 : 11;
        uint32_t ch4 : 11;
        uint32_t ch5 : 11;
        uint32_t ch6 : 11;
        uint32_t ch7 : 11;
        uint32_t ch8 : 11;
        uint32_t ch9 : 11;
        uint32_t ch10 : 11;
        uint32_t ch11 : 11;
        uint32_t ch12 : 11;
        uint32_t ch13 : 11;
        uint32_t ch14 : 11;
        uint32_t ch15 : 11;
    } Channels11Bit_t;



class CSbusDecoder
{
//variables
public:
	uint16_t m_RcChannels[16];
	bool m_FailSafe;
	bool m_LostFrame;
	CUart* m_Uart;
	bool m_ValidPacket;
protected:
private:
	static const uint8_t _numChannels = 16;
	uint8_t PreviousByte, CurrentByte;
	uint8_t InterSbusPacket[PACKET_SIZE];
	uint8_t m_SbusPacket[PACKET_SIZE];
//functions
public:
	CSbusDecoder(CUart* Uart);
	~CSbusDecoder();
	bool ParseInputData(uint16_t ControlData[]);
	bool Decode(uint16_t ControlData[]);
	bool Decode2(uint16_t ControlData[]);
	void Init();
	bool Update();
	bool UpdateChannels();
	void Test();
protected:
	CSbusDecoder( const CSbusDecoder &c );
	CSbusDecoder& operator=( const CSbusDecoder &c );
private:

}; //CSbusDecoder

#endif //__CSBUSDECODER_H__

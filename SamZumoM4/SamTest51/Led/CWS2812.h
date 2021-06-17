/* 
* CNeoPixel.h
*
* Created: 10/06/2020 14:42:36
* Author: philg
*/


#ifndef __CWS2812_H__
#define __CWS2812_H__

#define WS_BLUE			240
#define WS_LIGHT_BLUE	180
#define WS_GREEN		0
#define WS_ORANGE		30
#define WS_LIGHT_RED	25
#define WS_RED			120
#define WS_PURPLE		280
#define WS_YELLOW		60
#define WS_LIGHT_GREEN	70
#define WS_CYAN			180
#define WS_MAGENTA		300

// ==== WS2812B================================
// T0H 0 code high voltage time 0.4us ±150ns
// T0L 0 code low voltage time 0.85us ±150ns
// T1H 1 code high voltage time 0.8us ±150ns
// T1L 1 code low voltage time 0.45us ±150ns
// RES low voltage time Above 50µ
//=============================================

// ==== WS2812=================================
// T0H 0code,high voltage time 0.35us ±150ns
// T0L 0code,low voltage time 0.8us ±150ns
// T1H 1code,high voltage time 0.7us ±150ns
// T1L 1code,low voltage time 0.6us ±150ns
// RES low voltage time Above50µs
//=============================================



#define NEO_KHZ800 0x0000 ///< 800 KHz data transmission
#define NEO_KHZ400 0x0100 ///< 400 KHz data transmission
extern CCore Core;

class CWS2812
{
//variables
public:
protected:
private:
	EPortType m_PORT;
	uint8_t m_PortPin;
	uint8_t *pixels;
	uint8_t m_LedCount;
	uint32_t m_EndTime;
	uint8_t m_NoOfBytes;
	bool m_Is800KHz;
//functions
public:
	CWS2812();
	CWS2812(uint8_t NoOfLeds,EPortType port,uint8_t PortPin);
	~CWS2812();
	bool Init();
	void SetLed(uint8_t Led,uint8_t Colour,bool OnOff,bool Update = false);
	void Update();
	void SetRgb( unsigned char Red, unsigned char Green, unsigned char Blue,unsigned char LedNo);
	void SendFrame();
	void SendData();
	void Test();
	void SetLedHsv (int hue, unsigned char sat, unsigned char val,unsigned char LedNo);
	void TestDelay(unsigned char Bit);


	  bool canShow(void)
	   {
		  if (m_EndTime > Core.micros()) 
		  {
			  m_EndTime = Core.micros();
		  }
		  return (Core.micros() - m_EndTime) >= 300L;
	  }

protected:
private:
	CWS2812( const CWS2812 &c );
	CWS2812& operator=( const CWS2812 &c );

}; //CNeoPixel

#endif //__CNEOPIXEL_H__

/* 
* CNeoPixel.cpp
*
* Created: 10/06/2020 14:42:36
* Author: philg
*/

#include "Includes.h"


volatile uint8_t TestColour[6];

volatile uint8_t GColour;
volatile uint8_t RColour;
volatile uint8_t BColour;


// default constructor

CWS2812::CWS2812(uint8_t NoOfLeds,EPortType port,uint8_t PortPin)
{
	m_PORT = port;
	m_PortPin = PortPin;
	m_LedCount = NoOfLeds;
	if(pixels) 
		free(pixels); // Free  data 
}

// default destructor
CWS2812::~CWS2812()
{
} //CWS2812


 //~CWS2812
bool CWS2812::Init()
{
//	m_Is800KHz =false;
	m_Is800KHz =true;
	PinPeripheral.SetPinMode(m_PORT,m_PortPin,OUTPUT);
	PinPeripheral.SetPin(m_PORT,m_PortPin,LOW);
	if(pixels) 
		free(pixels); // Free existing data (if any)

	
	m_NoOfBytes = m_LedCount *  3;				//  Allocate new data
	if((pixels = (uint8_t *)malloc(m_NoOfBytes)))
	{
		memset(pixels, 0, m_NoOfBytes);		//  Clear all pixels
	}
	else
		return(false);	
	SetLed(0,0,false,true);
	return(true);
}

void CWS2812::SetLed(uint8_t Led,uint8_t Colour,bool OnOff,bool Update)
{
	if(OnOff)
	{
		switch(Colour)
		{
			case WS_BLUE:
				SetRgb (0, 0, 255,Led);
				break;
			case WS_GREEN:
				SetRgb (0, 255, 0,Led);
				break;
			case WS_RED:
				SetRgb (255, 0, 0,Led);
				break;
		}
	}
	else	
		SetRgb (0, 0, 0,Led);
	if(Update)
		SendFrame();	
}

void CWS2812::Update()
{
	SendFrame();
}


// Composition of 24bitdata: 
// G7 G6 G5 G4 G3 G2 G1 G0 R7 R6 R5 R4 R3 R2 R1 R0 B7 B6 B5 B4 B3 B2 B1 B0
//  Note:Follow the order of GRB to sent data and the high bit sent at first. 

void CWS2812::SetRgb( unsigned char Red, unsigned char Green, unsigned char Blue,unsigned char LedNo)
{
	uint8_t *p;
	p = &pixels[LedNo * 3];
	p[0]	= Green;
	p[1]	= Red;
	p[2]	= Blue;

	GColour = Green;
	RColour = Red;
	BColour = Blue;
}






//=============================================================================
// 0 = 0.35us   1.36us  +- 150 ns
// 1 = 1.36us   0.35us  +- 150


//     0.35 us
//   |------| 0.7 us |   logic 0
//   |      |_________|

//   |---------|0.6 us|   logic 1
//   | 0.8 us |_______|

//   |  Above 50us   |   reset
//   |_______________|

// 0 = 0.35us  0.7 us  +- 150 ns
// 1 = 0.8 us   0.6 us  +- 150





// T0H 0code,high voltage time 0.4us ±150ns 
// T1H 1code,high voltage time 0.85us ±150ns 
// T0L 0code,low voltage time 0.85us ±150ns 
// T1L 1code,low voltage time 0.4us ±150ns RES lowvoltage time Above50µs






/*
void CWS2812::SendFrame(void) 
{

  if(!pixels) 
	return;

  // Data latch = 300+ microsecond pause in the output stream. Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed. This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  while(!canShow());

  uint8_t  *ptr, *end, p, bitMask, portNum, bit;
  uint32_t  pinMask;

  portNum =  m_PORT;
  pinMask =  1ul << m_PortPin;
  ptr     =  pixels;
  end     =  ptr + m_NoOfBytes;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint32_t *set = &(PORT->Group[portNum].OUTSET.reg), *clr = &(PORT->Group[portNum].OUTCLR.reg);

  uint32_t t0, t1, top, ticks, saveLoad = SysTick->LOAD, saveVal = SysTick->VAL;

  if(m_Is800KHz) 
  {
    top =       (uint32_t)(F_CPU * 0.00000125); // Bit hi + lo = 1.25 uS
    t0  = top - (uint32_t)(F_CPU * 0.00000040); // 0 = 0.4 uS hi
    t1  = top - (uint32_t)(F_CPU * 0.00000080); // 1 = 0.8 uS hi
  } 
  else 
  { // 400 KHz bitstream
    top =       (uint32_t)(F_CPU * 0.00000250); // Bit hi + lo = 2.5 uS
    t0  = top - (uint32_t)(F_CPU * 0.00000050); // 0 = 0.5 uS hi
    t1  = top - (uint32_t)(F_CPU * 0.00000120); // 1 = 1.2 uS hi
  }

  SysTick->LOAD = top;               // Config SysTick for NeoPixel bit freq
  SysTick->VAL  = top;               // Set to start value (counts down)
  (void)SysTick->VAL;                // Dummy read helps sync up 1st bit
  __disable_irq();
  while(1) 
  {
    *set  = pinMask;                 // Set output high
    ticks =  t0 ;// t0; // SysTick threshold,
    while(SysTick->VAL > ticks);     // wait for it
    *clr  = pinMask;                 // Set output low
    while(SysTick->VAL <= ticks);    // Wait for rollover to 'top'
  }
  __enable_irq();
  SysTick->LOAD = saveLoad;          // Restore SysTick rollover to 1 ms
  SysTick->VAL  = saveVal;           // Restore SysTick value

  m_EndTime = Core.micros(); // Save EOD time for latch on next call
}
*/
void CWS2812::SendFrame(void) 
{

  if(!pixels) 
	return;

  // Data latch = 300+ microsecond pause in the output stream. Rather than
  // put a delay at the end of the function, the ending time is noted and
  // the function will simply hold off (if needed) on issuing the
  // subsequent round of data until the latch time has elapsed. This
  // allows the mainline code to start generating the next frame of data
  // rather than stalling for the latch.
  //while(!canShow());

  uint8_t  *ptr, *end, p, bitMask, portNum;  //, bit;
  uint32_t  pinMask;

  portNum =  m_PORT;
  pinMask =  1ul << m_PortPin;
  ptr     =  pixels;
  end     =  ptr + m_NoOfBytes;
  p       = *ptr++;
  bitMask =  0x80;

  volatile uint32_t *set = &(PORT->Group[portNum].OUTSET.reg), *clr = &(PORT->Group[portNum].OUTCLR.reg);

  uint32_t t0, t1, top, ticks, saveLoad = SysTick->LOAD, saveVal = SysTick->VAL;

  if(m_Is800KHz) 
  {
    top =       (uint32_t)(F_CPU * 0.00000125); // Bit hi + lo = 1.25 uS
    t0  = top - (uint32_t)(F_CPU * 0.00000040); // 0 = 0.4 uS hi
    t1  = top - (uint32_t)(F_CPU * 0.00000080); // 1 = 0.8 uS hi
  } 
  else 
  { // 400 KHz bitstream
    top =       (uint32_t)(F_CPU * 0.00000250); // Bit hi + lo = 2.5 uS
    t0  = top - (uint32_t)(F_CPU * 0.00000050); // 0 = 0.5 uS hi
    t1  = top - (uint32_t)(F_CPU * 0.00000120); // 1 = 1.2 uS hi
  }

  SysTick->LOAD = top;               // Config SysTick for NeoPixel bit freq
  SysTick->VAL  = top;               // Set to start value (counts down)
  (void)SysTick->VAL;                // Dummy read helps sync up 1st bit

 __disable_irq();
 for(;;) 
  {
    *set  = pinMask;                 // Set output high
    ticks = (p & bitMask) ? t1 : t0; // SysTick threshold,
    while(SysTick->VAL > ticks);     // wait for it
    *clr  = pinMask;                 // Set output low
    if(!(bitMask >>= 1)) 
	{									// Next bit for this byte...done?
      if(ptr >= end) 
		break;          // If last byte sent, exit loop
      p       = *ptr++;              // Fetch next byte
      bitMask = 0x80;                // Reset bitmask
    }
    while(SysTick->VAL <= ticks);    // Wait for rollover to 'top'
  }
  __enable_irq();

  SysTick->LOAD = saveLoad;          // Restore SysTick rollover to 1 ms
  SysTick->VAL  = saveVal;           // Restore SysTick value

//  m_EndTime = Core.micros(); // Save EOD time for latch on next call
}



void CWS2812::Test()
{
	while(1)
	{

//		SetLed(0,WS_BLUE,true);
//		SetLed(1,WS_GREEN,true);
//		SendFrame();
//		Core.delay(2000);

//		SetLed(0,WS_BLUE,true);
//		SetLed(1,WS_RED,true);
//		SendFrame();
//		Core.delay(1000);
//		SetLed(0,WS_RED,true);
//		SetLed(1,WS_RED,true);
//		SendFrame();
//		Core.delay(1000);
		SetRgb( 0x00, 0x00, 0xff,0);
		SendFrame();
		Core.delay(1000);
	}
}

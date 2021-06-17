/*
* CLedDriver.h
*
* Created: 04/03/2021 10:15:47
* Author: philg
*/


#ifndef __CLEDDRIVER_H__
#define __CLEDDRIVER_H__

#define Ws2812 1
#define ToshLed 2



class CLedDriver
{
	//variables
public:
	uint8_t m_LedInUse = ToshLed;
protected:
private:

	//functions
public:
	CLedDriver();
	~CLedDriver();
	void Init();
	void SetRGB(uint8_t Red,uint8_t Green,uint8_t Blue);
	void SetLedHsv (int hue, unsigned char sat, unsigned char val);
protected:
private:
	CLedDriver(const CLedDriver& c);
	CLedDriver& operator=(const CLedDriver& c);

}; //CLedDriver

#endif //__CLEDDRIVER_H__

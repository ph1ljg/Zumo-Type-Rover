/*
* CLedDriver.cpp
*
* Created: 04/03/2021 10:15:47
* Author: philg
*/

#include "Includes.h"
#include "CLedDriver.h"
#include "CToshibaLedDriver.h"



CToshibaLedDriver ToshibaLedDriver;


// default constructor
CLedDriver::CLedDriver()
{
} //CLedDriver

// default destructor
CLedDriver::~CLedDriver()
{
} //~CLedDriver


void CLedDriver::Init()
{
	WS2812.Init();
	ToshibaLedDriver.Init();
	ToshibaLedDriver.SetLedColour(0,0,0);

}


void CLedDriver::SetRGB(uint8_t Red,uint8_t Green,uint8_t Blue)
{
	if(m_LedInUse == Ws2812)
	{
		WS2812.SetRgb( Red, Green, Blue, 0);
		WS2812.Update();
	}
	else
		ToshibaLedDriver.SetLedColour(Red,Green,Blue);

}


//============================ Function set_led_hsv ============================
void CLedDriver::SetLedHsv (int hue, unsigned char sat, unsigned char val)
{
	unsigned int R =0, G =0, B =0;
	unsigned char sector;
	unsigned char rel_pos;
	unsigned int scale_factor;
	unsigned int const mmd = 65025;        // maximum modulation depth
	unsigned int top;
	unsigned int bottom;
	unsigned int slope;
	
	unsigned int a;
	unsigned int b;
	unsigned int c;
	unsigned int d;
	hue = hue % 360;
	sector = hue / 60;
	rel_pos = hue - (sector * 60);
	top = val * 255;
	bottom = val * (255 - sat);                                         // (val*255) - (val*255)*(sat/255)
	slope = (unsigned int)(val) * (unsigned int)(sat) / 120;			// dy/dx = (top-bottom)/(2*60) -- val*sat: modulation_depth dy
	a = bottom + slope * rel_pos;
	b = bottom + (unsigned int)(val) * (unsigned int)(sat) / 2 + slope * rel_pos;
	c = top - slope * rel_pos;
	d = top - (unsigned int)(val) * (unsigned int)(sat) / 2 - slope * rel_pos;


	switch(sector)
	{
	case 0:
		R = c;
		G = a;
		B = bottom;
		break;
	case 1:
		R = d;
		G = b;
		B = bottom;
		break;
	case 2:
		R = bottom;
		G = c;
		B = a;
		break;
	case 3:
		R = bottom;
		G = d;
		B = b;
		break;
	case 4:
		R = a;
		G = bottom;
		B = c;
		break;
	case 5:
		R = b;
		G = bottom;
		B = d;
		break;
	}

	scale_factor = mmd / 255;
	//	scale_factor = 1;

	R = (unsigned char) (R / scale_factor);
	G = (unsigned char) (G / scale_factor);
	B = (unsigned char) (B / scale_factor);

	SetRGB ( R, G, B);
}

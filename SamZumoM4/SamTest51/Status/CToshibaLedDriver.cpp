/* 
* CToshibaLedDriver.cpp
*
* Created: 28/03/2021 15:02:08
* Author: philg
*/

#include "Includes.h"
#include "CToshibaLedDriver.h"

// default constructor
CToshibaLedDriver::CToshibaLedDriver()
{
} //CToshibaLedDriver

// default destructor
CToshibaLedDriver::~CToshibaLedDriver()
{
} //~CToshibaLedDriver


void CToshibaLedDriver::Init()
{
	I2c.WriteRegisterByte(TOSHIBA_LED_I2C_ADDR, TOSHIBA_LED_ENABLE, 0x03);

	// update the red, green and blue values to zero
	uint8_t Val[] = { TOSHIBA_LED_OFF, TOSHIBA_LED_OFF, TOSHIBA_LED_OFF };

	I2c.WriteRegisterBytes(TOSHIBA_LED_I2C_ADDR, TOSHIBA_LED_PWM0,Val, sizeof(Val));


}


void CToshibaLedDriver::SetLedColour(uint8_t  Red, uint8_t  Green, uint8_t  Blue)
{
	/* 4-bit for each color */
	//	uint8_t Val[3] = { (uint8_t)(Blue >> 4), (uint8_t)(Green / 16), (uint8_t)(Red / 16) };
//	uint8_t Val[3] = { Blue, Green, Red  };
	uint8_t Val[3] = { Red, Green, Blue  };

	I2c.WriteRegisterBytes(TOSHIBA_LED_I2C_ADDR, TOSHIBA_LED_PWM0, Val, sizeof(Val));

}

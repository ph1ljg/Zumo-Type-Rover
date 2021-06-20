/* 
* CToshibaLedDriver.h
*
* Created: 28/03/2021 15:02:08
* Author: philg
*/


#ifndef __CTOSHIBALEDDRIVER_H__
#define __CTOSHIBALEDDRIVER_H__

#define TOSHIBA_LED_BRIGHT  0xFF    // full brightness
#define TOSHIBA_LED_MEDIUM  0x80    // medium brightness
#define TOSHIBA_LED_DIM     0x11    // dim
#define TOSHIBA_LED_OFF     0x00    // off

#define TOSHIBA_LED_I2C_ADDR 0x55    // default I2C bus address

#define TOSHIBA_LED_PWM0    0x01    // pwm0 register
#define TOSHIBA_LED_PWM1    0x02    // pwm1 register
#define TOSHIBA_LED_PWM2    0x03    // pwm2 register
#define TOSHIBA_LED_ENABLE  0x04    // enable register




class CToshibaLedDriver
{
//variables
public:
protected:
private:

//functions
public:
	CToshibaLedDriver();
	~CToshibaLedDriver();
	void Init();
	void SetLedColour(uint8_t Red, uint8_t Green, uint8_t Blue);

protected:
private:
	CToshibaLedDriver( const CToshibaLedDriver &c );
	CToshibaLedDriver& operator=( const CToshibaLedDriver &c );

}; //CToshibaLedDriver

#endif //__CTOSHIBALEDDRIVER_H__

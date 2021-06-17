/*
 * SamTest51.cpp
 *
 * Created: 25/06/2019 12:24:02
 * Author : phil
 */ 

#include "Includes.h"
//#include "CHeadControl.h"
#include <arm_math.h>
/** Address for ARM CPACR */
#define ADDR_CPACR 0xE000ED88

/** CPACR Register */
#define REG_CPACR  (*((volatile uint32_t *)ADDR_CPACR))

#define cpu_irq_enable()                     \
do {                                       \
	__DMB();                               \
	__enable_irq();                        \
} while (0)

#define cpu_irq_disable()                    \
do {                                       \
	__disable_irq();                       \
	__DMB();                               \
} while (0)

typedef uint32_t irqflags_t;
#define cpu_irq_is_enabled()    (__get_PRIMASK() == 0)

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags)
{
	return (flags);
}

static inline irqflags_t cpu_irq_save(void)
{
	volatile irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}
static inline void cpu_irq_restore(irqflags_t flags)
{
	if (cpu_irq_is_enabled_flags(flags))
	cpu_irq_enable();
}

/**
 * \brief Enable FPU
 */
static void fpu_enable(void)
{
	irqflags_t flags;
	flags = cpu_irq_save();
	REG_CPACR |=  (0xFu << 20);
	__DSB();
	__ISB();
	cpu_irq_restore(flags);
}

//============================================================================================================================================
//Feather	PORT	SAMD	FUNCTION 1		Function 2		Function 3      Function 4		Function 5										Used for
// VREF		PA3		4		ADC0/AIN    [1]																													Head Vertical servo
// A0			PA2		3		ADC0/AIN    [0]	DAC/VOUT    [0]																						OSD Reset			
// A1			PA5		14		ADC0/AIN    [5]	SERCOM0/PAD [1]	TC0/WO      [1]																Buzzer
// A4			PA4		13		ADC0/AIN    [4]	SERCOM0/PAD [0]	TC0/WO      [0]	EIC/EXTINT [4]										Left Wheel Encoder
// A2			PB8		11		ADC0/AIN    [2]	ADC1/AIN    [0]	SERCOM4/PAD [0]	TC4/WO     [0]										OSD CS
// A3			PB9		12		ADC0/AIN    [3]	ADC1/AIN    [1]	SERCOM4/PAD [1]	TC4/WO     [1]										Battery Volts
// A5			PA6		15		ADC0/AIN    [6]	SERCOM0/PAD [2]	TC1/WO	    [0]															Head Horizontal servo
// D4 		PA14	31		SERCOM2/PAD [2]	SERCOM4/PAD [2]	TC3/WO      [0]	TCC2/WO    [0]	TCC1/WO [2]					Rear Right Motor PWM
// D5			PA16	35		SERCOM1/PAD [0]	TC2/WO      [0]	TCC1/WO     [0]	TCC0/WO	   [4]											Rear Left Motor  PWM 
// D6			PA18	37		SERCOM1/PAD [2]	SERCOM3/PAD [2]	TC3/WO      [0]	TCC1/WO    [2]	TCC0/WO [6]					Right Wheel Encoder		EIC/EXTINT[2]	
// D9			PA19	38		SERCOM1/PAD [3]	SERCOM3/PAD [3]	TC3/WO      [1]	TCC1/WO    [3]	TCC0/WO [7]					FrSky Telemetry
// D10		PA20	41		SERCOM5/PAD [2]	SERCOM3/PAD [2]	TC7/WO      [0]	TCC1/WO    [4]	TCC0/WO [0]					Front Right Motor PWM
// D11		PA21	42		SERCOM5/PAD [3]	SERCOM3/PAD [3]	TC7/WO      [1]	TCC1/WO    [5]	TCC0/WO [1]					Front Left Motor  PWM
// D12		PA22	43		SERCOM3/PAD [0]	SERCOM5/PAD [1]	TC4/WO      [0]	TCC1/WO    [6]	TCC0/WO [2]					Receiver Tx
// D13		PA23	44		SERCOM3/PAD [1]	SERCOM5/PAD [0]	TC4/WO      [1]	TCC1/WO    [7]	TCC0/WO [3]					Receiver Rx
// TX 		PB16	00		SERCOM5/PAD [0]	TC6/WO      [0]	TCC3/WO     [0]	TCC0/WO    [4]	SERIAL  Tx						Gps/debug
// RX 		PB17	40		SERCOM5/PAD [1]	TC6/WO      [1]	TCC3/WO     [1]	TCC0/WO    [5]	SERIAL  Rx						Gps/debug
// SCK		PA17	36		SERCOM1/PAD [1]	SERCOM3/PAD [0]	TC2/WO      [1]	TCC1/WO    [1]	TCC0/WO [5]					SCK
// MISO		PB22	49		SERCOM1/PAD [2]	SERCOM5/PAD [2]	TC7/WO      [0]																MISO
// MOSI 	PB23	50		SERCOM1/PAD [3]	SERCOM5/PAD [3]	TC4/WO      [1]	TCC1/WO    [7]	TCC0/WO [3]					MOSI
// SDA 		PA12	29		SERCOM2/PAD [0]	SERCOM4/PAD [1]	TC2/WO      [0]	TCC0/WO    [6]	TCC1/WO [2]					SDA
// SCL		PA13	30		SERCOM2/PAD [1]	SERCOM4/PAD [0]	TC2/WO      [1]	TCC0/WO    [7]	TCC1/WO [3]					SCL
// USB D	PA25	46		SERCOM3/PAD [3]	SERCOM5/PAD [3]	TC5/WO      [1]	USB/DP													debug TX
// USB D	PA24	45		SERCOM3/PAD [2]	SERCOM5/PAD [2]	TC5/WO      [0]	USB/DM												debug RX
// XX		PB03	64		ADC0/AIN   [15]	SERCOM5/PAD [1]	TC6/WO      [1]																	NEO_PIX
// XX		PB10	QSCK
// XX		PB11	QSCS
// XX		PA08	QIO0
// XX		PA09	QIO1
// XX		PA10	QIO2
// XX		PA11	QIO3
//============================================================================================================================================

CUart Uart((void*)SERCOM5,PORTB,17, 16, SERCOM_PAD_1, SERCOM_PAD_0,0);
CUart SbusUart((void*)SERCOM3,PORTA,23, 22, SERCOM_PAD_1, SERCOM_PAD_0,1);

CSpi Spi ((void*)SERCOM1,PORTB,PORTB,PORTA, PIN_SPI_MISO,  PIN_SPI_SCK,  PIN_SPI_MOSI,  PAD_SPI_TX,  PAD_SPI_RX);
CI2c I2c((void*)SERCOM2,PORTA, PIN_SDA, PIN_SCL);

CSoftwareSerial DebugSerial(PORTA,24, 25,false,false,1);
CSoftwareSerial TeleSoftwareSerial(PORTA,19, 19,SS_INVERSE,HALF_DUPLEX,2);		// Telemetry
//CSoftwareSerial TrampSoftwareSerial(PORTB,5, 5,SS_NORMAL,HALF_DUPLEX,3);		// VTX

extern CServo Servo;
CWS2812 WS2812(1,PORTB,3);
//CWS2812 WS2812(1,PORTA,16);
CPinPeripheral PinPeripheral;
CCore Core;
CZumo Zumo;
CConfig Config;
CDebugDisplay DebugDisplay(&Uart);		// Sbus uses RX only leaving Tx Free
CServo Servo;
CSensors Sensors;
CAhrs Ahrs;
CGps Gps(&Uart);	
CSbusDecoder SbusDecoder(&SbusUart);
CTaskManager TaskManager;
CFrskyTelemetry FrskyTelemetry(&TeleSoftwareSerial);
CGuiFunctions GuiFunctions(&Uart);
//CGuiFunctions GuiFunctions(&DebugSerial);
COsd Osd;
CNavigation Navigation;
CAlarm Alarms;
CStatusControl LedControl;
CHeadControl HeadControl;
CRadioControl RadioControl;
//CNavigationFunctions CNavigationFunctions;
CBuzzer Buzzer;

CSercom SercomCommon;
CAvoidance Avoidance;
CSteering Steering;
CMotors Motors;
CImu Imu;
CAttitudeControl AttitudeControl;
CStatusControl StatusControl;
CTone Tone;


void DoTests();

int main(void)
{
    /* Initialize the SAM system */
    SystemInit();
	fpu_enable();
	Core.Init();

	Zumo.Init();
//	DoTests();
	while (1) 
    {
		if(Config.m_RunningFlags.PANIC_MODE_TRIGGERED )
		{
			Zumo.Panic();
			TaskManager.EnableTask(TASK_START_TASKS,true);
		}
		else
		{
			TaskManager.Scheduler();
			if( Config.m_RunningFlags.I2C_INTERFACE_FAIL )
				Config.m_RunningFlags.PANIC_MODE_TRIGGERED = true;
		}
	}
}


void DoTests()
{
	Motors.TestMotors();
}
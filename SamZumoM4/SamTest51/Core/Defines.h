/*
 * Defines.h
 *
 * Created: 14/05/2020 19:26:29
 *  Author: philg
 */ 


#ifndef DEFINES_H_
#define DEFINES_H_
//#include "MSercom.h"
#include "samd51j19a.h"




// USB
#define PIN_USB_HOST_ENABLE (28ul)
#define PIN_USB_DM          (29ul)
#define PIN_USB_DP          (30ul)

// Declare and implement const and non-const versions of the array subscript
// operator. The object is treated as an array of type_ values. 
#define DEFINE_BYTE_ARRAY_METHODS                                                                   \
    inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }           \
    inline uint8_t operator[](size_t i) const { return reinterpret_cast<const uint8_t *>(this)[i]; }




/* Definitions and types for pins */
typedef enum _EAnalogChannel
{
	No_ADC_Channel=-1,
	ADC_Channel0=0,
	ADC_Channel1=1,
	ADC_Channel2=2,
	ADC_Channel3=3,
	ADC_Channel4=4,
	ADC_Channel5=5,
	ADC_Channel6=6,
	ADC_Channel7=7,
	ADC_Channel8=8,
	ADC_Channel9=9,
	ADC_Channel10=10,
	ADC_Channel11=11,
	ADC_Channel12=12,
	ADC_Channel13=13,
	ADC_Channel14=14,
	ADC_Channel15=15,
	ADC_Channel16=16,
	ADC_Channel17=17,
	ADC_Channel18=18,
	ADC_Channel19=19,
	DAC_Channel0,
	DAC_Channel1,
	ADC_Channel_Bandgap=0x1B,
	ADC_Channel_PTAT=0x1C,
} EAnalogChannel ;

typedef enum eTCChannel
{
	NOT_ON_TIMER=-1,
	TCC0_CH0 = (0<<8)|(0),
	TCC0_CH1 = (0<<8)|(1),
	TCC0_CH2 = (0<<8)|(2),
	TCC0_CH3 = (0<<8)|(3),
	TCC0_CH4 = (0<<8)|(4),
	TCC0_CH5 = (0<<8)|(5),
	TCC0_CH6 = (0<<8)|(6),
	TCC0_CH7 = (0<<8)|(7),
	TCC1_CH0 = (1<<8)|(0),
	TCC1_CH1 = (1<<8)|(1),
	TCC1_CH2 = (1<<8)|(2),
	TCC1_CH3 = (1<<8)|(3),
	TCC1_CH4 = (1<<8)|(4),
	TCC1_CH5 = (1<<8)|(5),
	TCC1_CH6 = (1<<8)|(6),
	TCC1_CH7 = (1<<8)|(7),
	TCC2_CH0 = (2<<8)|(0),
	TCC2_CH1 = (2<<8)|(1),
	TCC2_CH2 = (2<<8)|(2),
	TCC3_CH0 = (3<<8)|(0),
	TCC3_CH1 = (3<<8)|(1),
	TCC4_CH0 = (4<<8)|(0),
	TCC4_CH1 = (4<<8)|(1),
	TC0_CH0 =  (5<<8)|(0),
	TC0_CH1 =  (5<<8)|(1),
	TC1_CH0 =  (6<<8)|(0),
	TC1_CH1 =  (6<<8)|(1),
	TC2_CH0 =  (7<<8)|(0),
	TC2_CH1 =  (7<<8)|(1),
	TC3_CH0 =  (8<<8)|(0),
	TC3_CH1 =  (8<<8)|(1),
	TC4_CH0 =  (9<<8)|(0),
	TC4_CH1 =  (9<<8)|(1),
	TC5_CH0 =  (10<<8)|(0),
	TC5_CH1 =  (10<<8)|(1),
	TC6_CH0 =  (11<<8)|(0),
	TC6_CH1 =  (11<<8)|(1),
	TC7_CH0 =  (12<<8)|(0),
	TC7_CH1 =  (12<<8)|(1),
} ETCChannel ;


typedef ETCChannel EPWMChannel;
extern const uint32_t GCLK_CLKCTRL_IDs[TCC_INST_NUM+TC_INST_NUM];
#define NOT_ON_PWM NOT_ON_TIMER


// Multi-serial objects instantiation
// SERCOM sercom0( SERCOM0 ) ;
// SERCOM sercom1( SERCOM1 ) ;
// SERCOM sercom2( SERCOM2 ) ;
// SERCOM sercom3( SERCOM3 ) ;
// SERCOM sercom4( SERCOM4 ) ;
 


// Serial interfaces

// Serial1
#define PIN_SERIAL1_RX       (0)
#define PIN_SERIAL1_TX       (1)
#define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX       (UART_TX_PAD_0)



typedef enum _EPortType
{
	NOT_A_PORT=-1,
	PORTA=0,
	PORTB=1,
	PORTC=2,
	PORTD=3,
} EPortType ;


typedef enum _EPioType
{
	PIO_NOT_A_PIN=-1,		// Not under control of a peripheral. 
	PIO_EXTINT=0,			// The pin is controlled by the associated signal of peripheral A. 
	PIO_ANALOG,				// The pin is controlled by the associated signal of peripheral B. 
	PIO_SERCOM,				// The pin is controlled by the associated signal of peripheral C. 
	PIO_SERCOM_ALT,			// The pin is controlled by the associated signal of peripheral D. 
	PIO_TIMER,				// The pin is controlled by the associated signal of peripheral E. 
	PIO_TIMER_ALT,			// The pin is controlled by the associated signal of peripheral F. 
	PIO_TCC_PDEC,			// The pin is controlled by the associated signal of peripheral G. 
	PIO_COM,				// The pin is controlled by the associated signal of peripheral H. 
	PIO_SDHC,				// The pin is controlled by the associated signal of peripheral I. 
	PIO_I2S,				// The pin is controlled by the associated signal of peripheral J. 
	PIO_PCC,				// The pin is controlled by the associated signal of peripheral K. 
	PIO_GMAC,				// The pin is controlled by the associated signal of peripheral L. 
	PIO_AC_CLK,				// The pin is controlled by the associated signal of peripheral M. 
	PIO_CCL,				// The pin is controlled by the associated signal of peripheral N. 
	PIO_DIGITAL,			// The pin is controlled by PORT. 
	PIO_INPUT,				// The pin is controlled by PORT and is an input. 
	PIO_INPUT_PULLUP,		// The pin is controlled by PORT and is an input with internal pull-up resistor enabled. 
	PIO_OUTPUT,				// The pin is controlled by PORT and is an output. 
	PIO_PWM=PIO_TIMER,
	PIO_PWM_ALT=PIO_TIMER_ALT,
} EPioType_t ;



typedef enum
{
	EXTERNAL_INT_0 = 0,
	EXTERNAL_INT_1,
	EXTERNAL_INT_2,
	EXTERNAL_INT_3,
	EXTERNAL_INT_4,
	EXTERNAL_INT_5,
	EXTERNAL_INT_6,
	EXTERNAL_INT_7,
	EXTERNAL_INT_8,
	EXTERNAL_INT_9,
	EXTERNAL_INT_10,
	EXTERNAL_INT_11,
	EXTERNAL_INT_12,
	EXTERNAL_INT_13,
	EXTERNAL_INT_14,
	EXTERNAL_INT_15,
	EXTERNAL_INT_NMI,
	EXTERNAL_NUM_INTERRUPTS,
	NOT_AN_INTERRUPT = -1,
	EXTERNAL_INT_NONE = NOT_AN_INTERRUPT,
} EExt_Interrupts ;


/**
 * Pin Attributes to be OR-ed
 */
#define PIN_ATTR_NONE          (0UL<<0)
#define PIN_ATTR_COMBO         (1UL<<0)
#define PIN_ATTR_ANALOG        (1UL<<1)
#define PIN_ATTR_DIGITAL       (1UL<<2)
#define PIN_ATTR_TIMER         (1UL<<4)
#define PIN_ATTR_TIMER_ALT     (1UL<<5)
#define PIN_ATTR_EXTINT        (1UL<<6)
#define PIN_ATTR_ANALOG_ALT	   (1UL<<7)

// these correspond to the mux table
#define PIN_ATTR_PWM_E         (1UL<<3)
#define PIN_ATTR_PWM_F         (1UL<<8)
#define PIN_ATTR_PWM_G         (1UL<<9)
#else
#define PIN_ATTR_PWM           (1UL<<3)

/*
// Types used for the table below 
typedef struct _PinDescription1
{
	EPortType       ulPort ;
	uint32_t        ulPin ;
	EPioType        ulPinType ;
 	uint32_t        ulPinAttribute ;
 	EAnalogChannel  ulADCChannelNumber ; // ADC Channel number in the SAM device 
 	EPWMChannel     ulPWMChannel ;
 	ETCChannel      ulTCChannel ;
 	EExt_Interrupts ulExtInt ;
} PinDescription1 ;
*/
/*
const PinDescription g_APinDescription1[]=
{
	// 0..13 - Digital pins
	// ----------------------
	// 0/1 - SERCOM/UART (Serial1)
	{ PORTB, 17, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH4, NOT_ON_TIMER, EXTERNAL_INT_1 }, // RX: SERCOM5/PAD[1]
	{ PORTB, 16, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH5, NOT_ON_TIMER, EXTERNAL_INT_0 }, // TX: SERCOM5/PAD[0]

	// 2..12
	// Digital Low
	{ NOT_A_PORT, 0,  PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ NOT_A_PORT, 0,  PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTA, 14, PIO_DIGITAL, PIN_ATTR_PWM_E, No_ADC_Channel, TC3_CH0, TC3_CH0, EXTERNAL_INT_14 },
	{ PORTA,  16, PIO_TIMER_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH0, TC2_CH0, EXTERNAL_INT_0  },
	{ PORTA,  18, PIO_TIMER_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH2, TC3_CH0, EXTERNAL_INT_2 },
	{ NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_TIMER, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

	// Digital High
	{ PORTB, 3, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //neopix
	{ PORTA,  19, PIO_TIMER_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH3, TC3_CH1, EXTERNAL_INT_3 },
	{ PORTA,  20, PIO_TIMER_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH0, NOT_ON_TIMER, EXTERNAL_INT_4 },
	{ PORTA, 21, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_5 },
	{ PORTA, 22, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_6 },

	// 13 (LED)
	{ PORTA, 23, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH3, TC4_CH1, EXTERNAL_INT_7 }, // TCC2/WO[1]

	// 14..19 - Analog pins
	// --------------------
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 },
	{ PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },
	{ PORTB,  8, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel2, TC4_CH0, TC4_CH0, EXTERNAL_INT_8 },
	{ PORTB,  9, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel3, TC4_CH1, TC4_CH1, EXTERNAL_INT_9 },
	{ PORTA,  4, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel4, TC0_CH0, TC0_CH0, EXTERNAL_INT_6 },
	{ PORTA,  6, PIO_ANALOG, (PIN_ATTR_ANALOG|PIN_ATTR_PWM_E), ADC_Channel6, TC1_CH0, TC1_CH0, EXTERNAL_INT_10 },

	// A6, D20 - VDiv!
	{ PORTB,  1, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel13, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 },


	// 21..22 I2C pins (SDA/SCL)
	// ----------------------
	{ PORTA, 12, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_12 }, // SDA: SERCOM2/PAD[0]
	{ PORTA, 13, PIO_SERCOM, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_13 }, // SCL: SERCOM2/PAD[1]

//	{ PORTA, 12, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, //neopix
//	{ PORTA, 13, PIO_DIGITAL, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	
	// 23..25 - SPI pins (MISO,MOSI,SCK)
	// ----------------------
	{ PORTB, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // MISO: SERCOM1/PAD[2]
	{ PORTB, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // MOSI: SERCOM1/PAD[3]
	{ PORTA, 17, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, // SCK: SERCOM1/PAD[1]

	// 26..27 - RX/TX LEDS  -- unused
	// --------------------
	{ NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },

	// 28..30 - USB
	// --------------------
	{ NOT_A_PORT, 0, PIO_NOT_A_PIN, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable DOES NOT EXIST ON THIS BOARD
	{ PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
	{ PORTA, 27, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

	// 31 (AREF)
	{ PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

	// ----------------------
	// 32..33 - Alternate use of A0 (DAC output)
	{ PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VOUT0
	{ PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VOUT1

	// ----------------------
	// 34..39 QSPI (SCK, CS, IO0, IO1, IO2, IO3)
	{ PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTA, 8, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTA, 9, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTA, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
	{ PORTA, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE },
} ;

*/

#endif /* DEFINES_H_ */
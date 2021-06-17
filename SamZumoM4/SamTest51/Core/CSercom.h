/*
* CSercom.h
*
* Created: 22/05/2020 14:23:52
* Author: philg
*/


#ifndef __CSERCOM_H__
#define __CSERCOM_H__

// Other SERCOM peripherals always use the 48 MHz clock
#define SERCOM_FREQ_REF       48000000ul
#define SERCOM_NVIC_PRIORITY  ((1<<__NVIC_PRIO_BITS) - 1)



#define CONF_GCLK_SERCOM0_CORE_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM0_SLOW_SRC GCLK_PCHCTRL_GEN_GCLK3_Val

#define CONF_GCLK_SERCOM0_CORE_UART_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM0_SLOW_UART_SRC GCLK_PCHCTRL_GEN_GCLK3_Val

#define CONF_GCLK_SERCOM1_CORE_UART_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM1_SLOW_UART_SRC GCLK_PCHCTRL_GEN_GCLK3_Val

#define CONF_GCLK_SERCOM3_CORE_UART_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM3_SLOW_UART_SRC GCLK_PCHCTRL_GEN_GCLK3_Val

#define CONF_GCLK_SERCOM4_CORE_UART_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM4_SLOW_UART_SRC GCLK_PCHCTRL_GEN_GCLK3_Val

#define CONF_GCLK_SERCOM5_CORE_UART_SRC GCLK_PCHCTRL_GEN_GCLK1_Val
#define CONF_GCLK_SERCOM5_SLOW_UART_SRC GCLK_PCHCTRL_GEN_GCLK3_Val


typedef enum
{
	SERCOM_RX_PAD_0 = 0,
	SERCOM_RX_PAD_1,
	SERCOM_RX_PAD_2,
	SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
	SERCOM_PAD_0 = 0,
	SERCOM_PAD_1,
	SERCOM_PAD_2,
	SERCOM_PAD_3
} SercomPad_t;



typedef enum
{
	SPI_PAD_0_SCK_1 = 0,
	SPI_PAD_2_SCK_3,
	SPI_PAD_3_SCK_1,
	SPI_PAD_0_SCK_3
} SercomSpiTXPad;

typedef enum
{
	SERCOM_SPI_MODE_0 = 0, // CPOL : 0 | CPHA : 0
	SERCOM_SPI_MODE_1,     // CPOL : 0 | CPHA : 1
	SERCOM_SPI_MODE_2,     // CPOL : 1 | CPHA : 0
	SERCOM_SPI_MODE_3      // CPOL : 1 | CPHA : 1
} SercomSpiClockMode;



typedef enum
{
	MSB_FIRST = 0,
	LSB_FIRST
} SercomDataOrder;


typedef enum
{
	SERCOM_CLOCK_SOURCE_FCPU		= 120000000L,   // F_CPU clock (GCLK0)
	SERCOM_CLOCK_SOURCE_48M			= 48000000,     // 48 MHz peripheral clock (GCLK1) (standard)
	SERCOM_CLOCK_SOURCE_100M		= 100000000,    // 100 MHz peripheral clock (GCLK2)
	SERCOM_CLOCK_SOURCE_32K			= 32768,		// XOSC32K clock (GCLK3)
	SERCOM_CLOCK_SOURCE_12M			= 12000000,     // 12 MHz peripheral clock (GCLK4)
	SERCOM_CLOCK_SOURCE_NO_CHANGE	= 0				// Leave clock source setting unchanged
} SercomClockSource;


static const struct
{
	Sercom   *sercomPtr;
	uint8_t   id_core;
	uint8_t   id_slow;
	IRQn_Type irq[4];
} sercomData[] =
{
	{ SERCOM0, SERCOM0_GCLK_ID_CORE, SERCOM0_GCLK_ID_SLOW,	SERCOM0_0_IRQn, SERCOM0_1_IRQn, SERCOM0_2_IRQn, SERCOM0_3_IRQn },
	{ SERCOM1, SERCOM1_GCLK_ID_CORE, SERCOM1_GCLK_ID_SLOW,	SERCOM1_0_IRQn, SERCOM1_1_IRQn, SERCOM1_2_IRQn, SERCOM1_3_IRQn },
	{ SERCOM2, SERCOM2_GCLK_ID_CORE, SERCOM2_GCLK_ID_SLOW,	SERCOM2_0_IRQn, SERCOM2_1_IRQn, SERCOM2_2_IRQn, SERCOM2_3_IRQn },
	{ SERCOM3, SERCOM3_GCLK_ID_CORE, SERCOM3_GCLK_ID_SLOW,	SERCOM3_0_IRQn, SERCOM3_1_IRQn, SERCOM3_2_IRQn, SERCOM3_3_IRQn },
	{ SERCOM4, SERCOM4_GCLK_ID_CORE, SERCOM4_GCLK_ID_SLOW,	SERCOM4_0_IRQn, SERCOM4_1_IRQn, SERCOM4_2_IRQn, SERCOM4_3_IRQn },
	{ SERCOM5, SERCOM5_GCLK_ID_CORE, SERCOM5_GCLK_ID_SLOW,	SERCOM5_0_IRQn, SERCOM5_1_IRQn, SERCOM5_2_IRQn, SERCOM5_3_IRQn },
};




class CSercom
{
	//variables
	public:
	//  uint32_t m_FreqRef; // Frequency corresponding to clockSource
	protected:
	private:
	//	SercomClockSource m_ClockSource;
	

	//functions
	public:
	CSercom();
	~CSercom();
	void InitClockNVIC( void* Ser,SercomClockSource m_ClockSource );
	uint8_t GetSercomIndex(Sercom *Ser);
	uint8_t CalculateBaudrateSynchronous(uint32_t baudrate,uint32_t RefFrequency);
	void SetClockSource(int8_t idx, SercomClockSource src, bool core);
	void EnableClock(const void *const hw,uint8_t SerComNo);
	void EnableMainClockSercom5(const void *const hw);
	void hri_mclk_write_APBDMASK_SERCOM5_bit(const void *const hw, bool value);
	void GclkWritePCHCTRL_reg(const void *const hw, uint8_t index, uint32_t data);
	// 	void SetClockSource(int8_t idx, SercomClockSource src, bool core);
	// 	void InitClockNVIC( Sercom Ser );
	//	uint8_t GetSercomIndex(Sercom Ser);
	protected:
	private:
	CSercom( const CSercom &c );
	CSercom& operator=( const CSercom &c );

}; //CSercom

#endif //__CSERCOM_H__

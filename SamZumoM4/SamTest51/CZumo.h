/* 
* CZumo.h
*
* Created: 14/05/2020 12:35:07
* Author: philg
*/


#ifndef __CZUMO_H__
#define __CZUMO_H__


// #define PIN_SERIAL1_RX       (0ul)
// #define PIN_SERIAL1_TX       (1ul)
// #define PAD_SERIAL1_RX       (SERCOM_RX_PAD_1)
// #define PAD_SERIAL1_TX       (UART_TX_PAD_0)
// 
// #define SPI_INTERFACES_COUNT 1
// 
// #define PIN_SPI_MISO         (23u)
// #define PIN_SPI_MOSI         (24u)
// #define PIN_SPI_SCK          (25u)
// #define PERIPH_SPI           sercom1
// #define PAD_SPI_TX           SPI_PAD_3_SCK_1
// #define PAD_SPI_RX           SERCOM_RX_PAD_2
// 
// static const uint8_t SS	  = 9 ;	// SERCOM1 last PAD is present on d9 but HW SS isn't used. Set here only for reference.
// static const uint8_t MOSI = PIN_SPI_MOSI ;
// static const uint8_t MISO = PIN_SPI_MISO ;
// static const uint8_t SCK  = PIN_SPI_SCK ;



typedef enum
{
	DEBUG_SSERIAL,
	GUI_SSERIAL,
	GUI_SSERIAL_DEBUG,
	TELEMETRY_SSERIAL,
	NO_SSERIAL
}EnableSSerial_t;


class CZumo
{
//variables
public:
	uint16_t m_TotalDistance;
	uint32_t m_TotalCourseTimeMilis;
	uint16_t m_TotalTripTimeSeconds;
	float	m_BatteryCapacityMa;
	uint16_t m_MaxBatteryReading;
	uint32_t m_DevicesOnLine;
	EnableSSerial_t m_SSerialSetTo;

protected:
private:

//functions
public:
	CZumo();
	~CZumo();
	void Init();
	void DelayedSetup();
	void StartTasks();
	void CheckDeviceStatus();
	void SetArmedState(bool State);
	void SetCourseDataVariables(uint32_t dt);
	void SetDebugSerial(EnableSSerial_t Active);
	void SetPanicTasks();
	void Panic();
	void TestLoop();
	void EnableMainClocks();
protected:
private:
	CZumo( const CZumo &c );
	CZumo& operator=( const CZumo &c );

}; //CZumo

#endif //__CZUMO_H__

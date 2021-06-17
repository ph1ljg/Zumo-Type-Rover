/* 
* CVtx.h
*
* Created: 02/05/2019 13:59:31
* Author: phil
*/


#ifndef __CVT_TRAMP_H__
#define __CVT_TRAMP_H__

#include "stdio.h"


#define VTX_TRAMP_MIN_FREQUENCY_MHZ 5000             //min freq in MHz
#define VTX_TRAMP_MAX_FREQUENCY_MHZ 5999             //max freq in MHz

// Maximum number of requests sent to try a config change
#define TRAMP_MAX_RETRIES 2

#define VTX_COMMON_BAND_USER      0
#define VTX_COMMON_BAND_A         1
#define VTX_COMMON_BAND_B         2
#define VTX_COMMON_BAND_E         3
#define VTX_COMMON_BAND_FS        4
#define VTX_COMMON_BAND_RACE      5

// Tramp "---", 25, 200, 400. 600 mW
#define VTX_TRAMP_POWER_OFF       0
#define VTX_TRAMP_POWER_25        1
#define VTX_TRAMP_POWER_100       2
#define VTX_TRAMP_POWER_200       3
#define VTX_TRAMP_POWER_400       4
#define VTX_TRAMP_POWER_600       5

typedef enum {
	TRAMP_STATUS_BAD_DEVICE = -1,
	TRAMP_STATUS_OFFLINE = 0,
	TRAMP_STATUS_ONLINE,
	TRAMP_STATUS_SET_FREQ_PW,
	TRAMP_STATUS_CHECK_FREQ_PW
} trampStatus_e;


typedef enum 
{
	S_WAIT_LEN = 0,   // Waiting for a packet len
	S_WAIT_CODE,      // Waiting for a response code
	S_DATA,           // Waiting for rest of the packet.
} trampReceiveState_e;
	

class CTrampProtocol
{
//variables
public:
	trampReceiveState_e trampReceiveState = S_WAIT_LEN;
	uint8_t m_RequestBuffer[16];
	uint8_t m_ResponseBuffer[16];
	uint32_t trampRFFreqMin;
	uint32_t trampRFFreqMax;
	uint32_t trampRFPowerMax;
	uint16_t trampPower = 0;       // Actual transmitting power
//	uint8_t  trampFreqRetries = 0;
//	uint8_t  trampPowerRetries = 0;
	uint32_t trampCurFreq = 0;
	uint16_t trampConfiguredPower = 0;
	int m_ReceivePosition = 0;
	trampStatus_e trampStatus = TRAMP_STATUS_OFFLINE;
//	uint32_t trampConfFreq = 0;
//	bool trampSetByFreqFlag = false;  //false = set via band/channel
	uint8_t trampPitMode = 0;
	int16_t trampTemperature = 0;
	bool m_DataReceived;
	CSoftwareSerial *m_Serial;

protected:
private:

//functions
public:
	CTrampProtocol(CSoftwareSerial *Serial);
	~CTrampProtocol();
	void Init();
	char trampReceive();
	void trampSetPitMode(uint8_t onoff);
	void trampResetReceiver(void);
	char trampHandleResponse(void);
//	void trampWriteBuf(uint8_t *buf);
	uint8_t trampChecksum(uint8_t *trampBuf);
	void trampCmdU16(uint8_t cmd, uint16_t param);
	void trampSendFreq(uint16_t freq);
	bool trampIsValidResponseCode(uint8_t code);
	void GetReceiveData();
	bool trampQueryR(void);
	bool trampQueryV(void);
	bool trampQueryI(void);
	bool trampQueryS(void);
	void trampQuery(uint8_t cmd);
	void trampSendRFPower(uint16_t level);
	void vtxTrampProcess( );
	void Test();
protected:
private:
	CTrampProtocol( const CTrampProtocol &c );
	CTrampProtocol& operator=( const CTrampProtocol &c );

}; //CTrampProtocol
#endif //__CVT_TRAMP_H__

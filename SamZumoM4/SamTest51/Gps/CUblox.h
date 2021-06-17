/* 
* CUblox.h
*
* Created: 20/11/2015 11:27:44
* Author: Phil2
*/


#ifndef __CUBLOX_H__
#define __CUBLOX_H__

//#include "CGps.h"
#include "Includes.h"
#include "UbloxStructures.h"
//================== UBLOX ============================================================

#define LATITUDE  0
#define LONITUDE  1

//#define UBLOX_DEBUGGING_

#ifdef UBLOX_DEBUGGING_
# define Debug(fmt, args ...)  do {DebugDisplay.Printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); Core.delay(1); } while(0)
#else
# define Debug(fmt, args ...)
#endif

template <typename T, size_t N> char (&_ARRAY_SIZE_HELPER(T (&_arr)[N]))[N];

template <typename T> char (&_ARRAY_SIZE_HELPER(T (&_arr)[0]))[0];

#define ARRAY_SIZE(_arr) sizeof(_ARRAY_SIZE_HELPER(_arr))



#define UBLOX_MAX_PORTS 6



#define CONFIG_RATE_NAV      (1<<0)
#define CONFIG_RATE_POSLLH   (1<<1)
#define CONFIG_RATE_STATUS   (1<<2)
#define CONFIG_RATE_SOL      (1<<3)
#define CONFIG_RATE_VELNED   (1<<4)
#define CONFIG_RATE_DOP      (1<<5)
#define CONFIG_CHIP_NO       (1<<6)
#define CONFIG_NAV_SETTINGS  (1<<7)
#define CONFIG_GNSS          (1<<8)
#define CONFIG_SBAS          (1<<9)
#define CONFIG_TIMEUTC       (1<<10)
#define CONFIG_VERSION       (1<<11) // can use up to 250 bytes only used for info 
#define CONFIG_RATE_PVT      (1<<13)
#define CONFIG_LAST          (1<<18) // this must always be the last bit


#define CONFIG_ALL (CONFIG_RATE_NAV | CONFIG_RATE_POSLLH | CONFIG_RATE_STATUS | CONFIG_RATE_SOL | CONFIG_RATE_VELNED | CONFIG_RATE_DOP   \
| CONFIG_NAV_SETTINGS | CONFIG_GNSS | CONFIG_SBAS | CONFIG_TIMEUTC)

enum config_step 
{
	STEP_RATE_NAV = 0,
	STEP_RATE_POSLLH,
	STEP_RATE_VELNED,
	STEP_PORT,
	STEP_POLL_SVINFO,
	STEP_POLL_SBAS,
	STEP_POLL_NAV,
	STEP_POLL_GNSS,
	STEP_NAV_RATE,
	STEP_POSLLH,
	STEP_STATUS,
	STEP_SOL,
	STEP_VELNED,
	STEP_DOP,
	STEP_SW_VERSION,
	STEP_MSG_TIMEUTC,
	STEP_LAST
};

#define RATE_POSLLH		1
#define RATE_STATUS		1
#define RATE_SOL		1
#define RATE_VELNED		1
#define RATE_DOP		1
#define RATE_TIMEUTC	1


typedef struct __attribute__ ((packed))
{
    uint64_t time_us;
    int8_t   ofsI;
    uint8_t  magI;
    int8_t   ofsQ;
    uint8_t  magQ;
}log_Ubx2_t;




// Receive buffer
typedef union
{
	DEFINE_BYTE_ARRAY_METHODS
	ubx_nav_posllh			posllh;
	ubx_nav_status			status;
    ubx_nav_dop				dop;
	ubx_nav_solution_t		solution;
	ubx_nav_pvt				Pvt;
	ubx_nav_velned_t		velned;
    ubx_cfg_msg_rate_t msg_rate;
    UbxCfgMsgRate6_t msg_rate_6;
    ubx_cfg_nav_settings_t	nav_settings;
	ubx_Nav_TimeUtc_t		ubxNavTimeUtc;
	ubx_cfg_gnss_t			gnss;
	ubx_cfg_nav_rate_t nav_rate;
	ubx_cfg_prt_t prt;
//	ubx_mon_hw_60_t mon_hw_60;
//	ubx_mon_hw_68_t mon_hw_68;		// Neo7 uses a 68 byte message
///	ubx_mon_hw2_t mon_hw2;
	ubx_mon_ver_t mon_ver;
    ubx_cfg_sbas_t sbas;
    ubx_nav_svinfo_header_t svinfo_header;
    ubx_ack_ack_t ack;

} GpsRecieveBuffer_t;






class CUblox
{
//variables
public:
	bool            m_CfgSaved;
	bool			m_CfgNeedsSave;
	uint32_t		m_UnconfiguredMessages;
	uint8_t			m_NextMessage;
	uint32_t        m_LastCfgSentTime;
	uint8_t         m_NumCfgSaveTries;
	GpsRecieveBuffer_t GpsRecieveBuffer;

protected:
private:
	CUart *			m_Uart;
    uint8_t			m_HardwareGeneration;
	uint8_t			m_MessageClass;
	uint8_t			m_MesageId;
	uint8_t			NextFix;
	bool			m_NewPosition;		// Ist there new position information?
	bool			m_NewSpeed;			// Is there new speed information?
	uint16_t		m_PayloadLength;
    uint8_t         m_UbloxPort;
    uint8_t         m_DisableCounter;
    bool			m_NoReceivedHdop;
    uint32_t		m_LastUpdateTime;
    uint32_t        m_LastVelTime;
    uint32_t        m_LastPosTime;


//functions
public:
	CUblox();
	~CUblox();
	void UnexpectedMessage(void);
	void log_mon_hw(void);
	void log_mon_hw2(void);
	void UpdateChecksum(uint8_t *data, uint8_t len, uint8_t &ck_a, uint8_t &ck_b);
	bool GetNewFrame(unsigned char data);
	bool Parse();
	void HandleConfig();
	void HandleACk();
	bool HandleMon();
	void RequestNextConfig();
	void ConfigNavRate(void);
	bool ConfigureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
	void RequestPort(void);
	void RequestNavigationRate(void);
	void ConfigureRate(void);
	void SaveCfg();
	bool RequestMessageRate(uint8_t msg_class, uint8_t msg_id);
	void SendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size);
	bool Init(CUart * Uart);
	void VerifyRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
	void ConfigurePort();
	void ConfigureStart();
	void Read(uint8_t data);

	
protected:
private:
	CUblox( const CUblox &c );
	CUblox& operator=( const CUblox &c );
	static const char _initialisation_blob[];
	uint16_t m_ExtensionSize;
}; //CUblox

#endif //__CUBLOX_H__

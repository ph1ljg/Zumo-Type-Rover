/*
 * UbloxMessaging.h
 *
 * Created: 04/07/2020 10:25:29
 *  Author: philg
 */ 


#ifndef UBLOXMESSAGING_H_
#define UBLOXMESSAGING_H_

#define UBLOX_MAX_GNSS_CONFIG_BLOCKS 7

//Configuration Sub-Sections
#define SAVE_CFG_IO     (1<<0)
#define SAVE_CFG_MSG    (1<<1)
#define SAVE_CFG_INF    (1<<2)
#define SAVE_CFG_NAV    (1<<3)
#define SAVE_CFG_RXM    (1<<4)
#define SAVE_CFG_RINV   (1<<9)
#define SAVE_CFG_ANT    (1<<10)
#define SAVE_CFG_ALL    (SAVE_CFG_IO|SAVE_CFG_MSG|SAVE_CFG_INF|SAVE_CFG_NAV|SAVE_CFG_RXM|SAVE_CFG_RINV|SAVE_CFG_ANT)

#define MEASURE_RATE 600   // measurement rate


struct __attribute__ ((packed)) ubx_nav_dop 
{
	uint32_t time;                                  // GPS msToW
	uint16_t gDOP;
	uint16_t pDOP;
	uint16_t tDOP;
	uint16_t vDOP;
	uint16_t hDOP;
	uint16_t nDOP;
	uint16_t eDOP;
};


//====================================================================================================


typedef enum
{
	NAV_STATUS_NO_FIX					= 0,    // No Fix
	NAV_STATUS_NO_FIX_DEAD_RECKONING	= 1,    // Receiving Dead Reckoning only
	NAV_STATUS__FIX_2D					= 2,    // Receiving valid messages and 2D lock
	NAV_STATUS__FIX_3D					= 3,    // Receiving valid messages and 3D lock
	NAV_STATUS__FIX_3D_DEAD_RECKONING	= 4,	// Receiving valid messages and 3D lock with dead Reckoning
	NAV_STATUS__FIX_TIME				= 5,	// Receiving valid messages and Only Time Fix
	NAV_STATUS_DGPS_USED				= 1
}NAV_Status_t;

typedef enum
{
	NAV_SOL_NO_FIX					= 0,    // No Fix
	NAV_SOL_NO_FIX_DEAD_RECKONING	= 1,    // Receiving Dead Reckoning only
	NAV_SOL__FIX_2D					= 2,    // Receiving valid messages and 2D lock
	NAV_SOL__FIX_3D					= 3,    // Receiving valid messages and 3D lock
	NAV_SOL__FIX_3D_DEAD_RECKONING	= 4,	// Receiving valid messages and 3D lock with dead Reckoning
	NAV_SOL__FIX_TIME				= 5,	// Receiving valid messages and Only Time Fix
	NAV_SOL_FIX_IN_LIMITS			= 1,
	NAV_SOL_DGPS_USED				= 2
}NAV_SOL_t;




    // u-blox UBX protocol essentials
typedef struct __attribute__ ((packed))
{
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
}ubx_header_t;



typedef struct __attribute__ ((packed))
{
	uint8_t clsID;
	uint8_t msgID;
}ubx_ack_ack_t;



typedef struct __attribute__ ((packed))
{
	int8_t ofsI;
	uint8_t magI;
	int8_t ofsQ;
	uint8_t magQ;
	uint8_t cfgSource;
	uint8_t reserved0[3];
	uint32_t lowLevCfg;
	uint32_t reserved1[2];
	uint32_t postStatus;
	uint32_t reserved2;
}ubx_mon_hw2_t;
    
typedef struct __attribute__ ((packed))
{
	char swVersion[30];
	char hwVersion[10];
	char RomVersion[30];
	uint8_t extension;		// [250]; 
}ubx_mon_ver_t;
    
typedef struct __attribute__ ((packed))
{
	uint32_t itow;
	uint8_t numCh;
	uint8_t globalFlags;	//Chip hardware generation	0: Antaris, Antaris 4	1: u-blox 5	2: u-blox 6	3: u-blox 7	4: u-blox 8 / u-blox M8
	uint16_t reserved;
}ubx_nav_svinfo_header_t;


typedef struct __attribute__ ((packed))
{
	uint8_t portID;
}ubx_cfg_prt_t;

typedef struct __attribute__ ((packed))
{
	uint8_t portID;				//01
	uint8_t reserved;			//00
	uint16_t txReady;			//00 00
	uint32_t mode;				//D0 08 00 00 ? 00001000 11010000?
	uint32_t Baud;				// 0x00, 0xC2, 0x01, 0x00          115200
	uint16_t inProtoMask;		// 0x07, 0x00   bit 0 Ubx bit 1 nmea
	uint16_t outProtoMask;		// 0x07, 0x00   bit 0 Ubx bit 1 nmea
	uint16_t reserved4;			//	0x00, 0x00
	uint16_t reserved5;			//	0x00, 0x00
}UbxConfigurePort_t;

//const char  Baud115200[]  ={0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0,			//    speed to 115200
//0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x96};


typedef struct __attribute__ ((packed))
{
	uint8_t mode;
	uint8_t usage;
	uint8_t maxSBAS;
	uint8_t scanmode2;
	uint32_t scanmode1;
}ubx_cfg_sbas_t;



typedef struct __attribute__ ((packed))
{
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}ubx_cfg_nav_rate_t;
   



typedef struct __attribute__ ((packed)) 
{
	uint8_t msg_class;
	uint8_t msg_id;
	uint8_t rate;
}ubx_cfg_msg_rate_t;

typedef struct __attribute__ ((packed)) 
{
	uint8_t msg_class;
	uint8_t msg_id;
	uint8_t rates[6];
}UbxCfgMsgRate6_t;


typedef struct __attribute__ ((packed))  
{
	uint32_t	time;				// GPS msToW
	int32_t		longitude;
	int32_t		latitude;
	int32_t		altitude_ellipsoid;
	int32_t		altitude_msl;
	uint32_t	horizontal_accuracy;
	uint32_t	vertical_accuracy;
}ubx_nav_posllh ;
typedef struct __attribute__ ((packed)) 
{
	uint32_t	time;				// GPS msToW
	uint8_t		fix_type;
	uint8_t		fix_status;
	uint8_t		differential_status;
	uint8_t		res;
	uint32_t	time_to_first_fix;
	uint32_t	uptime;				// milliseconds
}ubx_nav_status ;

typedef struct __attribute__ ((packed)) 
{
	uint32_t	time;
	int32_t		time_nsec;
	int16_t		week;
	uint8_t		fix_type;
	uint8_t		fix_status;
	int32_t		ecef_x;
	int32_t		ecef_y;
	int32_t		ecef_z;
	uint32_t	position_accuracy_3d;
	int32_t		ecef_x_velocity;
	int32_t		ecef_y_velocity;
	int32_t		ecef_z_velocity;
	uint32_t	speed_accuracy;
	uint16_t	position_DOP;
	uint8_t		res;
	uint8_t		satellites;
	uint32_t	res2;
}ubx_nav_solution_t;

typedef  struct  __attribute__ ((packed))   
 {
	 uint32_t itow;
	 uint16_t year;
	 uint8_t month, day, hour, min, sec;
	 uint8_t valid;
	 uint32_t t_acc;
	 int32_t nano;
	 uint8_t fix_type;
	 uint8_t flags;
	 uint8_t flags2;
	 uint8_t num_sv;
	 int32_t lon, lat;
	 int32_t height, h_msl;
	 uint32_t h_acc, v_acc;
	 int32_t velN, velE, velD, gspeed;
	 int32_t head_mot;
	 uint32_t s_acc;
	 uint32_t head_acc;
	 uint16_t p_dop;
	 uint8_t reserved1[6];
	 uint32_t headVeh;
	 uint8_t reserved2[4];
 }ubx_nav_pvt;


typedef struct  __attribute__ ((packed)) 
{
	uint32_t	time;				// GPS msToW
	int32_t		ned_north;
	int32_t		ned_east;
	int32_t		ned_down;
	uint32_t	speed_3d;
	uint32_t	speed_2d;
	int32_t		heading_2d;
	uint32_t	speed_accuracy;
	uint32_t	heading_accuracy;
}ubx_nav_velned_t ;



typedef struct __attribute__ ((packed))
{
	uint16_t mask;
	uint8_t dynModel;
	uint8_t fixMode;
	int32_t fixedAlt;
	uint32_t fixedAltVar;
	int8_t minElev;
	uint8_t drLimit;
	uint16_t pDop;
	uint16_t tDop;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t staticHoldThresh;
	uint8_t res1;
	uint32_t res2;
	uint32_t res3;
	uint32_t res4;
}ubx_cfg_nav_settings_t;

typedef struct __attribute__ ((packed))
{
	uint32_t MillisecondTimeOfWeek;
	uint32_t TimeAccuracyEstimate;
	uint32_t NanosecondsOfSecond;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
	uint8_t valid;
	
}ubx_Nav_TimeUtc_t;


typedef struct  __attribute__ ((packed))
{
	uint8_t msgVer;
	uint8_t numTrkChHw;
	uint8_t numTrkChUse;
	uint8_t numConfigBlocks;
	struct  __attribute__ ((packed))
	{
		uint8_t gnssId;
		uint8_t resTrkCh;
		uint8_t maxTrkCh;
		uint8_t reserved1;
		uint32_t flags;
	} configBlock[UBLOX_MAX_GNSS_CONFIG_BLOCKS];
}ubx_cfg_gnss_t;


enum ubx_hardware_version 
{
	ANTARIS = 0,
	UBLOX_5,
	UBLOX_6,
	UBLOX_7,
	UBLOX_M8
};





// Lea6 uses a 60 byte message
typedef struct __attribute__ ((packed))
{
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t aStatus;
	uint8_t aPower;
	uint8_t flags;
	uint8_t reserved1;
	uint32_t usedMask;
	uint8_t VP[17];
	uint8_t jamInd;
	uint16_t reserved3;
	uint32_t pinIrq;
	uint32_t pullH;
	uint32_t pullL;
}ubx_mon_hw_60_t;

// Neo7 uses a 68 byte message
typedef struct __attribute__ ((packed))
 {
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t aStatus;
	uint8_t aPower;
	uint8_t flags;
	uint8_t reserved1;
	uint32_t usedMask;
	uint8_t VP[25];
	uint8_t jamInd;
	uint16_t reserved3;
	uint32_t pinIrq;
	uint32_t pullH;
	uint32_t pullL;
}ubx_mon_hw_68_t;
    



enum __attribute__ ((packed)) ubs_protocol_bytes 
{
	PREAMBLE1 = 0xb5,
	PREAMBLE2 = 0x62,
	CLASS_NAV = 0x01,
	CLASS_ACK = 0x05,
	CLASS_CFG = 0x06,
	CLASS_MON = 0x0A,
	CLASS_RXM = 0x02,
	MSG_ACK_NACK = 0x00,
	MSG_ACK_ACK = 0x01,
	MSG_POSLLH = 0x2,
	MSG_STATUS = 0x3,
	MSG_DOP = 0x4,
	MSG_SOL = 0x6,
    MSG_PVT = 0x7,
	MSG_VELNED = 0x12,
	MSG_CFG_CFG = 0x09,
	MSG_CFG_RATE = 0x08,
	MSG_CFG_MSG = 0x01,
	MSG_CFG_NAV_SETTINGS = 0x24,
	MSG_CFG_PRT = 0x00,
	MSG_CFG_SBAS = 0x16,
	MSG_CFG_GNSS = 0x3E,
	MSG_MON_HW = 0x09,
	MSG_MON_HW2 = 0x0B,
	MSG_MON_VER = 0x04,
	MSG_NAV_SVINFO = 0x30,
	MSG_RXM_RAW = 0x10,
	MSG_RXM_RAWX = 0x15,
	MSG_TIMEUTC = 0x21

};



enum ubx_gnss_identifier 
{
	GNSS_GPS     = 0x00,
	GNSS_SBAS    = 0x01,
	GNSS_GALILEO = 0x02,
	GNSS_BEIDOU  = 0x03,
	GNSS_IMES    = 0x04,
	GNSS_QZSS    = 0x05,
	GNSS_GLONASS = 0x06
};
   
typedef struct  __attribute__ ((packed))
{
	uint32_t clearMask;
	uint32_t saveMask;
	uint32_t loadMask;
}UbxCfgCfg_t;

typedef struct  __attribute__ ((packed))
{
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}UbxCfgNavRate_t;
   

// u-blox UBX protocol essentials
typedef struct  __attribute__ ((packed))
{
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
}UbxHeader_t;






//========================================== messages=================================================
typedef struct  __attribute__ ((packed))
{
	uint8_t msg_class;
	uint8_t msg_id;
}ubx_cfg_msg_t;




#endif /* UBLOXMESSAGING_H_ */
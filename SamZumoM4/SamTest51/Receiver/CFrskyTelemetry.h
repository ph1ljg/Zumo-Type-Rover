/* 
* CFrskyTelemetry.h
*
* Created: 18/04/2020 10:03:45
* Author: Phil2
*/


#ifndef __CFRSKYTELEMETRY_H__
#define __CFRSKYTELEMETRY_H__
#include "Includes.h"

typedef enum MAV_SEVERITY
{
	MAV_SEVERITY_EMERGENCY	= 0,	// System is unusable. This is a "panic" condition. 
	MAV_SEVERITY_ALERT		= 1,	// Action should be taken immediately. Indicates error in non-critical systems. 
	MAV_SEVERITY_CRITICAL	= 2,	// Action must be taken immediately. Indicates failure in a primary system. 
	MAV_SEVERITY_ERROR		= 3,	// Indicates an error in secondary/redundant systems. 
	MAV_SEVERITY_WARNING	= 4,	// Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning. 
	MAV_SEVERITY_NOTICE		= 5,	// An unusual event has occured, though not an error condition. This should be investigated for the root cause. 
	MAV_SEVERITY_INFO		= 6,	// Normal operational messages. Useful for logging. No action is required for these messages.
	MAV_SEVERITY_DEBUG		= 7,	// Useful non-operational messages that can assist in debugging. These should not occur during normal operation. 
	MAV_SEVERITY_ENUM_END	= 8,	
} MAV_SEVERITY;



#define FRSKY_TELEM_PAYLOAD_STATUS_CAPACITY          5 // size of the message buffer queue (max number of messages waiting to be sent)

/*
for FrSky D protocol (D-receivers)
*/
// FrSky sensor hub data IDs
#define DATA_ID_GPS_ALT_BP          0x01
#define DATA_ID_TEMP1               0x02
#define DATA_ID_FUEL                0x04
#define DATA_ID_TEMP2               0x05
#define DATA_ID_GPS_ALT_AP          0x09
#define DATA_ID_BARO_ALT_BP         0x10
#define DATA_ID_GPS_SPEED_BP        0x11
#define DATA_ID_GPS_LONG_BP         0x12
#define DATA_ID_GPS_LAT_BP          0x13
#define DATA_ID_GPS_COURS_BP        0x14
#define DATA_ID_GPS_SPEED_AP        0x19
#define DATA_ID_GPS_LONG_AP         0x1A
#define DATA_ID_GPS_LAT_AP          0x1B
#define DATA_ID_BARO_ALT_AP         0x21
#define DATA_ID_GPS_LONG_EW         0x22
#define DATA_ID_GPS_LAT_NS          0x23
#define DATA_ID_CURRENT             0x28
#define DATA_ID_VARIO               0x30
#define DATA_ID_VFAS                0x39

#define START_STOP_D                0x5E
#define BYTESTUFF_D                 0x5D

/*
for FrSky SPort and SPort Passthrough (OpenTX) protocols (X-receivers)
*/
// FrSky Sensor IDs
#define SENSOR_ID_VARIO             0x00 // Sensor ID  0
#define SENSOR_ID_FAS               0x22 // Sensor ID  2
#define SENSOR_ID_GPS               0x83 // Sensor ID  3
#define SENSOR_ID_SP2UR             0xC6 // Sensor ID  6
#define SENSOR_ID_28                0x1B // Sensor ID 28

// FrSky data IDs
#define GPS_LONG_LATI_FIRST_ID      0x0800
#define DIY_FIRST_ID                0x5000

#define START_STOP_SPORT            0x7E
#define BYTESTUFF_SPORT             0x7D
#define SPORT_DATA_FRAME            0x10
/*
for FrSky SPort Passthrough
*/
// data bits preparation
// for parameter data
#define PARAM_ID_OFFSET             24
#define PARAM_VALUE_LIMIT           0xFFFFFF
// for gps status data
#define GPS_SATS_LIMIT              0xF
#define GPS_STATUS_LIMIT            0x3
#define GPS_STATUS_OFFSET           4
#define GPS_HDOP_OFFSET             6
#define GPS_ADVSTATUS_OFFSET        14
#define GPS_ALTMSL_OFFSET           22
// for battery data
#define BATT_VOLTAGE_LIMIT          0x1FF
#define BATT_CURRENT_OFFSET         9
#define BATT_TOTALMAH_LIMIT         0x7FFF
#define BATT_TOTALMAH_OFFSET        17
// for autopilot status data
#define AP_CONTROL_MODE_LIMIT       0x1F
#define AP_SIMPLE_OFFSET            5
#define AP_SSIMPLE_OFFSET           6
#define FR_FLYING_OFFSET            7
#define FR_ARMED_OFFSET             8
#define FR_BATT_FS_OFFSET           9
#define AP_EKF_FS_OFFSET            10
#define AP_IMU_TEMP_MIN             19.0f
#define AP_IMU_TEMP_MAX             82.0f
#define AP_IMU_TEMP_OFFSET          26
// for home position related data
#define HOME_ALT_OFFSET             12
#define HOME_BEARING_LIMIT          0x7F
#define HOME_BEARING_OFFSET         25
// for velocity and yaw data
#define VELANDYAW_XYVEL_OFFSET      9
#define VELANDYAW_YAW_LIMIT         0x7FF
#define VELANDYAW_YAW_OFFSET        17
// for attitude (roll, pitch) and range data
#define ATTIANDRNG_ROLL_LIMIT       0x7FF
#define ATTIANDRNG_PITCH_LIMIT      0x3FF
#define ATTIANDRNG_PITCH_OFFSET     11
#define ATTIANDRNG_RNGFND_OFFSET    21
// for fair scheduler
#define TIME_SLOT_MAX               11


typedef struct __attribute__ ((packed))
{
	uint8_t FrameType;
	uint16_t ID;
	uint32_t Data;
	uint8_t crc;
}TeleFrame_t;

typedef union __attribute__ ((packed))
{
	TeleFrame_t Frame;
	uint8_t Message[8];
}TxFrame_t;



typedef struct __mavlink_statustext_t
{
	uint8_t severity;	//  Severity of status. Relies on the definitions within RFC-5424.*/
	char text[50];		//  Status text message, without null termination character*/
	} mavlink_statustext_t;




class CFrskyTelemetry
{
//variables
public:
	static CObjectArray<mavlink_statustext_t> _statustext_queue;
protected:
private:
	uint16_t _crc;

	uint32_t m_CheckErrorMessageTimer = 0;
	uint32_t m_MessageTimer = 0;
	uint8_t m_ParamID =0;
	CSoftwareSerial* m_Uart;


   
   struct
   {
	   int32_t vario_vspd;
	   char lat_ns, lon_ew;
	   uint16_t latdddmm;
	   uint16_t latmmmm;
	   uint16_t londddmm;
	   uint16_t lonmmmm;
	   uint16_t alt_gps_meters;
	   uint16_t alt_gps_cm;
	   uint16_t alt_nav_meters;
	   uint16_t alt_nav_cm;
	   int16_t speed_in_meter;
	   uint16_t speed_in_centimeter;
	   uint16_t yaw;
   } _SPort_data;

    struct
    {
	    uint8_t new_byte;
	    bool send_attiandrng;
	    bool send_latitude;
	    uint32_t MessageTimer;
	    bool MessageInProgress;
	    uint32_t params_timer;
	    uint32_t ap_status_timer;
	    uint32_t batt_timer;
	    uint32_t batt_timer2;
	    uint32_t gps_status_timer;
	    uint32_t home_timer;
	    uint32_t velandyaw_timer;
	    uint32_t gps_latlng_timer;
    } m_Passthrough_s;
      

   struct
   {
	   const uint32_t packet_min_period[TIME_SLOT_MAX] = {
		   
		   28,     //0x5000 text,      25Hz
		   38,     //0x5006 attitude   20Hz
		   280,    //0x800  GPS        3Hz
		   280,    //0x800  GPS        3Hz
		   250,    //0x5005 vel&yaw    4Hz
		   500,    //0x5001 AP status  2Hz
		   500,    //0x5002 GPS status 2Hz
		   500,    //0x5004 home       2Hz
		   500,    //0x5008 batt 2     2Hz
		   500,    //0x5003 batt 1     2Hz
		   1000    //0x5007 parameters 1Hz
	   };
   } _sport_config;

   struct
   {
	   bool sport_status;
	   bool gps_refresh;
	   bool vario_refresh;
	   uint8_t fas_call;
	   uint8_t gps_call;
	   uint8_t vario_call;
	   uint8_t various_call;
   } _SPort;
   
   struct
   {
	   uint32_t last_200ms_frame;
	   uint32_t last_1000ms_frame;
   } _D;
   
   struct
   {
	   uint32_t chunk; // a "chunk" (four characters/bytes) at a time of the queued message to be sent
	   uint8_t repeats; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
	   uint8_t char_index; // index of which character to get in the message
   } _msg_chunk;
   TxFrame_t m_TxFrame;
//functions
public:
	CFrskyTelemetry(CSoftwareSerial* Uart);
	~CFrskyTelemetry();
	void Init();
	void update_avg_packet_rate();
	void passthrough_wfq_adaptive_scheduler(void);
	void Update(void);
	void calc_crc(uint8_t byte);
	void send_crc(void);
	void send_byte(uint8_t byte);
	void send_uint32(uint16_t id, uint32_t data);
	bool get_next_msg_chunk(void);
	void QueueMessage(MAV_SEVERITY severity, const char *text);
	void CheckErrorMessageFlags(void);
	void CheckMessages(void);
	void GetNavigationState();
	uint32_t CalcParam(void);
	uint32_t calc_gps_latlng(bool *send_latitude);
	uint32_t CalcGpsStatus(void);
	uint32_t CalcBatteryState(uint8_t instance);
	uint32_t CalcStatus(void);
	uint32_t CalcHome(void);
	uint32_t calc_velandyaw(void);
	uint32_t CalcAttAndRng(void);
	uint16_t PrepareNumber(int32_t number, uint8_t digits, uint8_t power);
	uint8_t GetMode(NavigationState_t State);
	float format_gps(float dec);
protected:
private:
	CFrskyTelemetry( const CFrskyTelemetry &c );
	CFrskyTelemetry& operator=( const CFrskyTelemetry &c );

}; //CFrskyTelemetry

#endif //__CFRSKYTELEMETRY_H__

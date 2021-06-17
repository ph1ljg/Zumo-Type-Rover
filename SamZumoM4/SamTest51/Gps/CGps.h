/* 
* CGpsModule.h
*
* Created: 24/04/2017 12:05:12
* Author: phil
*/


#ifndef __CGPSMODULE_H__
#define __CGPSMODULE_H__

#include "includes.h"
#include "CUart.h"


/// GPS status codes used for telemetry
typedef enum
{
	GPS_FAILED				= 0,
	GPS_NO_FIX,
	GPS_OK_FIX_2D,
	GPS_OK_FIX_3D,
	GPS_OK_FIX_DGPS,
	GPS_OK_FIX_3D_RTK_FLOAT,
	GPS_OK_FIX_3D_RTK_FIXED,
	GPS_OK_FIX_TIME,
	GPS_TEMP_LOS,
}GPS_Status_t;



// GPS navigation engine settings. Not all GPS receivers support this
enum GPS_Engine_Setting 
{
	GPS_ENGINE_NONE        = -1,
	GPS_ENGINE_PORTABLE    = 0,
	GPS_ENGINE_STATIONARY  = 2,
	GPS_ENGINE_PEDESTRIAN  = 3,
	GPS_ENGINE_AUTOMOTIVE  = 4,
	GPS_ENGINE_SEA         = 5,
	GPS_ENGINE_AIRBORNE_1G = 6,
	GPS_ENGINE_AIRBORNE_2G = 7,
	GPS_ENGINE_AIRBORNE_4G = 8
};










#define GPS_FILTER
#define LAT  0
#define LON  1
#define COPRD_DIVIDER				10000000.0F
#define GPS_TIMEOUT					6000			// 6 seconds
#define GPS_FILTER_VECTOR_LENGTH	5			// GPS data filtering - moving average filter vector length

typedef struct
{
    uint8_t		DgpsValid;
	uint32_t	PosllhTime;                                  // GPS msToW
	uint8_t		GpsFixType;
	uint8_t		NumSats;
	uint16_t	Hdop;					// horizontal dilution of precision in cm
    uint16_t	vdop;                   // vertical dilution of precision in cm
	uint16_t	GroundSpeed;			// ground speed from gps m/s*100
	uint32_t	Altitude;				// gps altitude
	uint16_t	GroundCourse;			// GPS ground course
	long		HeadingAcu;				// Course / Heading Accuracy Estimate
	long		SpeedAccuracy;			// Speed Accuracy Estimate
    float		horizontal_accuracy;
    float		vertical_accuracy;
    bool		HaveVerticalVelocity:1;      // does this GPS give vertical velocity?
    bool		have_speed_accuracy:1;
    bool		have_horizontal_accuracy:1;
    bool		have_vertical_accuracy:1;
	uint32_t	time_week_ms;				// GPS time (milliseconds from start of GPS week)
	uint16_t	time_week;					// GPS week number
	uint32_t	LastGpsTimeMs;			// the system time of the last GPS timestamp, milliseconds
	bool		Update;						// toggle to show updated
	Vector3f velocity;						// 3D velocitiy in m/s, in NED format


}GpsReadings_t;


typedef struct
{
	int32_t nLatitude;
	int32_t nLongitude;
	float Latitude;
	float Longitude;
}GpsPositionReadings_t;

typedef struct
{
	unsigned char hour;
	unsigned char minute;
	unsigned char seconds;
	unsigned int year;
	unsigned char month;
	unsigned char day;
	uint16_t milliseconds;
	bool	DataValid;
}GpsTimeValues_t;




typedef struct  
{
	int32_t Latitude;
	int32_t Longitude;
}Coords_t;

typedef enum{START_CHECK,GPS_ONLINE,GPS_MALFUNCTION}eGpsState_t;

class CGps
{
//variables
public:
	//This sets the minimum elevation of satellites above the horizon for them to be used for navigation. Setting this to -100 leaves the minimum elevation set to the GPS modules default.
	//Range: -100 90  Units: Degrees
	int8_t m_MinElevation;
    
	// Bitmask for what GNSS system to use on the first GPS (all unchecked or zero to leave GPS as configured)
    // 1:GPS-NoSBAS, 3:GPS+SBAS, 4:Galileo-NoSBAS, 6:Galileo+SBAS, 8:Beidou, 
	// 51:GPS+IMES+QZSS+SBAS (Japan Only), 64:GLONASS, 66:GLONASS+SBAS, 67:GPS+GLONASS+SBAS
    // Bitmask: 0:GPS,1:SBAS,2:Galileo,3:Beidou,4:IMES,5:QZSS,6:GLOSNASS
    uint8_t  m_GnssMode;
  
  // This sets the SBAS (satellite based augmentation system) mode if available on this GPS. If set to 2 then the SBAS mode is not changed in the GPS. Otherwise the GPS will be reconfigured to enable/disable SBAS. Disabling SBAS may be worthwhile in some parts of the world where an SBAS signal is available but the baseline is too long to be useful.
  // Values: 0:Disabled,1:Enabled,2:NoChange
  uint8_t m_SbasMode;



	int8_t m_Navfilter;
	// moving average filter variables
	uint8_t GPS_filter_index;
	int32_t GPS_filter[2][GPS_FILTER_VECTOR_LENGTH];
	int32_t GPS_filter_sum[2];
	int32_t GPS_filtered[2];
	int32_t GPS_degree[2];    //the lat lon degree without any decimals (lat/10 000 000)
	GpsTimeValues_t m_GpsTimeValues;
	GpsReadings_t m_GpsReadings;
	GpsPositionReadings_t m_GpsPositionReadings;
	bool m_Debug;
	Coords_t m_Coords;
	CUart *  m_Uart;
	bool m_Update;					// used by GUI
	GPS_Status_t m_GpsStatus;
protected:
private:
    uint32_t	m_LastConfigTime;
	uint8_t		m_CanSaveConfig ;
	bool		m_ConfigSaved;
//functions
public:
	CGps(CUart * Uart);
	~CGps();
	void Init();
	void Update();
	void CheckGpsLock();
	GPS_Status_t GetStatus();
	bool IsGpsUsable();
	uint32_t GroundSpeed();
	int32_t GroundCourseCD();
	int32_t GroundCourse();
	void DoConfig();
	bool SaveConfig();
protected:
private:
	CGps( const CGps &c );
	CGps& operator=( const CGps &c );


}; //CGpsModule

#endif //__CGPSMODULE_H__

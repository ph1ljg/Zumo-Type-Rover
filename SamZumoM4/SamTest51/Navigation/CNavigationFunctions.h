/* 
* CNavigationUtils.h
*
* Created: 26/04/2017 17:27:29
* Author: phil
*/


#ifndef __CNAVIGATIONUTILS_H__
#define __CNAVIGATIONUTILS_H__
#include "CNavigation.h"


// convert a longitude or latitude point to meters or centimeters.
// Note: this does not include the longitude scaling which is dependent upon location
#define LATLON_TO_M     0.01113195f
#define LATLON_TO_CM    1.113195f
#define DEGX100 5729.57795f		// Centi-degrees to radians


typedef struct
{
	int32_t	NextWpDistanceCm;			// distance to active coordinates  (calculated) in cm
	int16_t	NextWpBearing;			// direction to active coordinates (calculated)
	int16_t	XTE_Distance;			// cross track error distance in cm
	int16_t	BoatToHomeBearing;		// 1deg = 10
	float	DistanceToHome;			// distance to home in cm
	int32_t nPresentLatitude;
	int32_t nPresentLongitude;
	int16_t CrosstrackAngle;
	int16_t RelativeBearing;
	float	XteBearing;
	uint16_t m_XtrkError;
	//	unsigned char Checksum;
}Bearings_t;

typedef struct
{
	int32_t	nLatitude;
	int32_t	nLongitude;
}Location_t;


#define WP_VALID				128


//===============================================================================
// 	LeadFilter.cpp - GPS lag remover
//===============================================================================
class LeadFilter
{
	public:
	LeadFilter() :
	m_LastVelocity(0) {}
	float 	GetPosition(float pos, int16_t vel, float GPSlagInSeconds);
	private:
	int16_t		m_LastVelocity;
};



class CNavigationFunctions 
{
//variables
public:
	int16_t x_actual_speed = 0;
	int16_t y_actual_speed = 0;
	LeadFilter xLeadFilter;									// Long GPS lag filter
	LeadFilter yLeadFilter;									// Lat  GPS lag filter
	bool NavProcessorReady;
	uint32_t	 nav_loopTimer;							// used to track the elapsed time between GPS reads
	NavigationParams_t NavigationParams;
	static float  dTNav;								// Delta Time in milliseconds for navigation computations, updated with every good GPS read



	int32_t  GPS_WP_latitude,GPS_WP_longitude;		//Currently used WP
	uint8_t  GPS_WP_flags;

	int32_t last_longitude ;
	int32_t last_latitude ;
	float GPS_lead_latitude, GPS_lead_longitude;
	float  dTnav;									// Delta Time in milliseconds for navigation computations, updated with every good GPS read
	int16_t ActualSpeed = 0;
	bool m_AtHomePosition;

	float RetVal;
	double WayDist;
	float distance;
	bool m_BearingsLoaded;
	Bearings_t m_Bearings;


protected:
private:

//functions
public:
	CNavigationFunctions();
	~CNavigationFunctions();
	void Init();
	void GPS_calc_longitude_scaling(int32_t lat);
	bool check_missed_wp(Bearings_t &Bearings,NextWaypoint_t NextWaypoint);
	uint32_t GPS_distance_cm(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2);
	void CalcGpsVelocity( Bearings_t Bearings);
	void GPS_calc_location_error( int32_t target_lat, int32_t target_lng, int32_t gps_lat, int32_t gps_lng );
	void GPS_calc_poshold(int x_error, int y_error);
	void GPS_calc_nav_rate(int max_speed);
	void GetCrosstrackError(NextWaypoint_t NextWaypoint);
	int16_t GetBearingRate();
	int16_t GPS_calc_desired_speed(unsigned int max_speed, bool _slow);
	void GPS_reset_nav();
	int32_t wrap_18000(int32_t error);
	int32_t wrap_36000(int32_t angle);
	void UpdateBearings(NextWaypoint_t NextWaypoint);
	void UpdateHomeBearings( );
	bool LoadHomeWaypoint();
	float longitude_scale(int32_t Latitude) const;
	bool get_vector_from_origin_NEU(Vector3f &vec_neu,Location_t Location) const;
	uint32_t DistanceCm(int32_t StartLatitude,int32_t StartLongitude,int32_t EndLatitude,int32_t EndLongitude);
	int32_t GetBearing(int32_t FromLatitude, int32_t FromLongitude,int32_t TopLatitude, int32_t ToLongitude);
	uint32_t DistanceMeter1(float NowLat,float NowLon,float WaypointLat,float WaypointLon);
	uint32_t DistanceMeter(float NowLat,float NowLon,float WaypointLat,float WaypointLon);
	float get_distance(const Location_t &loc1,const Location_t &loc2) const;
	float GetHomeDistance(const Location_t &loc1) const;
	bool GetHomeLocation(Location_t &Loc1);
	int32_t get_bearing_to(Location_t Loc1,const Location_t &loc2) const;
	bool same_latlon_as(Location_t Loc1,const Location_t &loc2) const;
	void offset_bearing(Location_t &Loc,float bearing, float distance);
	void offset(Location_t &Loc,float ofs_north, float ofs_east);
	Vector2f get_distance_NE(const Location_t &loc1,const Location_t &loc2) const;
	bool get_relative_position_NE_home(Vector2f &Loc);
	int  GetTrackingWithXTEerror(NextWaypoint_t NextWaypoint);
	int GetTrackingError(int16_t TargetBearing,int16_t Heading);
	void MovePoint(int32_t &latitude, int32_t &longitude, double distanceInMetres, double bearing);
	void DebugMoveAlongCourse(Bearings_t &Bearings,double Bearing,uint16_t Speed);
	bool get_vector_xy_from_origin_NE(Vector2f &vec_ne,Location_t Location) const;
	float GetRelativeBearing(float TargetBearing,float Heading);
	int16_t GetHeadingError90(int16_t Target, int16_t ActualHeading);
	int16_t GetHeadingError180(int16_t Target, int16_t ActualHeading);

		
	float GetHeadingError180(float Target, float ActualHeading);
	double BiggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo);
	void TestNav();
//	bool GetGpsInfo();

protected:
private:
	CNavigationFunctions( const CNavigationFunctions &c );
	CNavigationFunctions& operator=( const CNavigationFunctions &c );
 	int32_t Wrap18000(int32_t error);
    static constexpr float LOCATION_SCALING_FACTOR = 0.011131884502145034f;		// scaling factor from 1e-7 degrees to meters at equator  == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
    static constexpr float LOCATION_SCALING_FACTOR_INV = 89.83204953368922f;	// inverse of LOCATION_SCALING_FACTOR

// 	int32_t Wrap36000(int32_t angle);
// 	int32_t wrap_180(int32_t error);
// 	int32_t wrap_360(int32_t angle);

}; //CNavigationUtils




#endif //__CNAVIGATIONUTILS_H__

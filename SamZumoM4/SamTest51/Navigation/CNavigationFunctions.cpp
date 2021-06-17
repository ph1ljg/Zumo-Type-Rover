/* 
* CNavigationUtils.cpp
*
* Created: 26/04/2017 17:27:28
* Author: phil
*/
#include "includes.h"
float NavDebug1;
float NavDebug2;
float NavDebug3;
float NavDebug4;

// default constructor
CNavigationFunctions::CNavigationFunctions()
{
	last_longitude = 0;
	last_latitude  = 0;
	
	m_AtHomePosition = false;
	m_BearingsLoaded = false;

} 

// default destructor
CNavigationFunctions::~CNavigationFunctions()
{
} //~CNavigationUtils


void CNavigationFunctions::Init()
{
	
	NavigationParams.CrosstrackGain = CROSSTRACK_GAIN;	// Cross track gain = 1
	NavigationParams.WayPointRadius = WP_RADIUS;		//cm -> 2m
}





void CNavigationFunctions::UpdateBearings(NextWaypoint_t NextWaypoint)
{
	CMyMath Math;
		
	#if defined(GPS_LEAD_FILTER)
		CalcGpsVelocity();	//calculate the current velocity based on GPS coordinates continuously to get a valid speed at the moment when we start navigating
		m_Bearings.NextWpDistanceCm = DistanceCm(GPS_lead_latitude,GPS_lead_longitude,Navigation.m_NextWaypoint.Coordinates.lat,Navigation.m_NextWaypoint.Coordinates.lon);
		m_Bearings.NextWpBearing = GetBearing(GPS_lead_latitude,GPS_lead_longitude,Navigation.m_NextWaypoint.Coordinates.lat,Navigation.m_NextWaypoint.Coordinates.lon);
	#else
		m_Bearings.NextWpDistanceCm = DistanceCm(m_Bearings.nPresentLatitude,m_Bearings.nPresentLongitude,Navigation.m_NextWaypoint.Coordinates.lat,NextWaypoint.Coordinates.lon);
		m_Bearings.NextWpBearing = GetBearing(m_Bearings.nPresentLatitude,m_Bearings.nPresentLongitude,NextWaypoint.Coordinates.lat,NextWaypoint.Coordinates.lon);
	#endif
	GetCrosstrackError(NextWaypoint);
	m_Bearings.RelativeBearing =(int16_t) GetRelativeBearing(m_Bearings.NextWpBearing,Math.Round(Ahrs.GetHeading()));
	Config.m_RunningFlags.WP_ERROR = false;
	Config.m_RunningFlags.NAV_PROCESSOR_RUNNING = true;
	m_BearingsLoaded = true;	
}


void CNavigationFunctions::UpdateHomeBearings()
{
	m_Bearings.DistanceToHome = DistanceCm(m_Bearings.nPresentLatitude,m_Bearings.nPresentLongitude,Config.m_CourseWaypoints[0].nLatitide,Config.m_CourseWaypoints[0].nLongitude);
	m_Bearings.BoatToHomeBearing = GetBearing(m_Bearings.nPresentLatitude,m_Bearings.nPresentLongitude,Config.m_CourseWaypoints[0].nLatitide,Config.m_CourseWaypoints[0].nLongitude);
	if(m_Bearings.DistanceToHome < 100)
		m_AtHomePosition = true;
	else
		m_AtHomePosition = false;
	
}

bool CNavigationFunctions::LoadHomeWaypoint()
{
	
	if(!Gps.IsGpsUsable())
	return(false);

	if(!NavigationFunctions.m_BearingsLoaded)
	return(false);



	Config.m_CourseWaypoints[0].Action = WP_RTH;
	if(Navigation.m_Debug)
	{
		m_Bearings.nPresentLatitude = 524415210;
		m_Bearings.nPresentLongitude = -19841740;
	}
	Config.m_CourseWaypoints[0].nLatitide = m_Bearings.nPresentLatitude;
	Config.m_CourseWaypoints[0].nLongitude = m_Bearings.nPresentLongitude;
	Config.m_CourseWaypoints[0].Flag |= WP_VALID;
	Config.m_RunningFlags.HOME_LOCATION_SET = true;
	return(true);
}



//================================= Waypoints ============================================================
//========================================================================================================







// longitude_scale - returns the scaler to compensate for shrinking longitude as you move north or south from the equator
// Note: this does not include the scaling to convert longitude/latitude points to meters or centimeters
float CNavigationFunctions::longitude_scale(int32_t Latitude) const
{
	float scale = cosf(Latitude * (1.0e-7f * DEG_TO_RAD));
	return MAX(scale, 0.01f);
}


bool CNavigationFunctions::get_vector_from_origin_NEU(Vector3f &vec_neu,Location_t  Location) const
{
	// convert lat, lon
	Vector2f vec_ne;
	if (!get_vector_xy_from_origin_NE(vec_ne,Location))
		return false;

	vec_neu.x = vec_ne.x;
	vec_neu.y = vec_ne.y;

	// convert altitude
	vec_neu.z = 0;

	return true;
}


bool CNavigationFunctions::get_vector_xy_from_origin_NE(Vector2f &vec_ne,Location_t  Location) const
{

	vec_ne.x = (m_Bearings.nPresentLatitude-Location.nLatitude) * LATLON_TO_CM;
	vec_ne.y = (m_Bearings.nPresentLongitude-Location.nLongitude) * LATLON_TO_CM * longitude_scale(Config.m_CourseWaypoints[0].nLatitide);
	return true;
}

//=====================================================
//  * GetRelativeBearing()
//  * Calculate the angle from the front of the
//  * boat to the target. If the target is to the
//  * right of the boat, the angle will negative.
// ====================================================
float CNavigationFunctions::GetRelativeBearing(float TargetBearing,float Heading)
{
	NavDebug3 = Heading;
	float tracking_error;
	float tracking_error_LPF = 0.999;
	static float  AVG_tracking_error =0;
	tracking_error = GetHeadingError180( TargetBearing,Heading);
	AVG_tracking_error =  tracking_error *   tracking_error_LPF+  AVG_tracking_error *  (1 - tracking_error_LPF)  ;
	return( AVG_tracking_error); 
}

//==========================================================================================
// 	tracking error is difference between course over ground and compass heading.
// 	It's purpose is to  correct drift or anything else that causes a deviation between the direction the boat
// 	is traveling and the direction the compass says it is pointing.  The results are low pass
// 	filtered to provide a time averaged result that will not change abruptly when going to a new course
// 	but recompute on a new course.
//==========================================================================================
int  CNavigationFunctions::GetTrackingError(int16_t TargetBearing,int16_t Heading)
{
	
	float tracking_error_LPF = 0.999;										// tracking error updates when GPS updates about  once per second
	float tracking_error;
	static float  AVG_tracking_error =0;
	tracking_error = TargetBearing - Heading;					
		
	if (abs(tracking_error) > 180)										// this limits error to < 180 and makes turn short way on compass + right, - left
	{
		if(TargetBearing >   Heading)
			tracking_error = tracking_error - 360;
			
		if(TargetBearing <   Heading)
			tracking_error = 360 + tracking_error;
	}
		

	AVG_tracking_error =  tracking_error *   tracking_error_LPF+  AVG_tracking_error *  (1 - tracking_error_LPF)  ;
	return(AVG_tracking_error);
}







//Function to find angle opposite to largest side of triangle
double CNavigationFunctions::BiggerAngleFind(double largestDistance, double smallDistanceOne, double smallDistanceTwo)
{
	double biggerAngle;
	biggerAngle =  pow(smallDistanceOne,2) + pow(smallDistanceTwo, 2) - pow(largestDistance,2);
	biggerAngle = fabs(biggerAngle/(2*smallDistanceOne*smallDistanceTwo));
	biggerAngle = acos(biggerAngle)* 180.0 / PI;
	return biggerAngle;
}

int16_t CNavigationFunctions::GetHeadingError90(int16_t Target, int16_t ActualHeading) 
{
	CMyMath Math; 
	int Difference;
	Difference = GetHeadingError180( Target,  ActualHeading); 
	Difference = Math.Map(Difference,-180,180,-90,90);
    return(Difference);
}


//float Angle3 = wrap_180(20.3f - 210.6f)

float CNavigationFunctions::GetHeadingError180(float Target, float ActualHeading) 
{
	NavDebug1 = Target;
	NavDebug2 = ActualHeading;
	float DifferenceAngle;
	DifferenceAngle = wrap_180(Target - ActualHeading );
    return(DifferenceAngle);
}






int16_t CNavigationFunctions::GetHeadingError180(int16_t Target, int16_t ActualHeading) 
{
	int16_t DifferenceAngle;
	DifferenceAngle = ((((ActualHeading - Target) % 360) + 540) % 360) - 180;
    return(DifferenceAngle);
}


//======================= crosstrack ==================================================================

// Formula:  dxt = asin( sin(?13) ? sin(?13??12) ) ? R
// where ?13 is (angular) distance from start point to third point
// ?13 is (initial) bearing from start point to third point
// ?12 is (initial) bearing from start point to end point
//  R is the earth’s radius
//
// JavaScript: var dXt = Math.asin(Math.sin(d13/R)*Math.sin(?13-?12)) * R;




//======================================================================================
// Calculating cross track error, this tries to keep the boat on a direct line
// when sailing to a way point.
//======================================================================================
void CNavigationFunctions::GetCrosstrackError(NextWaypoint_t NextWaypoint)
{
	float Angle;
	float XteAngle;
	if(m_Bearings.NextWpDistanceCm <WP_RADIUS)
	{
		m_Bearings.XTE_Distance = 0;
		m_Bearings.XteBearing = 0;
		return;
	}
	Angle = m_Bearings.NextWpBearing - NextWaypoint.StartBearing;
	XteAngle = Angle;
	Angle = radians(Angle);
	Angle = sin(Angle);
	m_Bearings.XTE_Distance = (int16_t)(Angle*m_Bearings.NextWpDistanceCm); // distance to true path in CM
	m_Bearings.XteBearing = XteAngle;
}


int16_t CNavigationFunctions::GetBearingRate()
{
	float KFilterRate = .3; // bearing rate low pass filter, 0 is no filtering.
	static int32_t bearingrate2_time =0;
	int16_t DeltaHeading;
	float BR2_DeltaT;
	static int16_t HeadingOld;
	float BearingRate = (Ahrs.m_AhrsValues.RawGyro.z*cos(Ahrs.GetRollD()) 
		+ Ahrs.m_AhrsValues.RawGyro.y*sin(Ahrs.GetRollD()))*cos(Ahrs.GetPitchD()) 
		- Ahrs.m_AhrsValues.RawGyro.x*sin(Ahrs.GetPitchD())*RAD_TO_DEG;  	
	static float BearingRateSmoothed =0;
	
	DeltaHeading = Ahrs.GetHeading() - HeadingOld;
	if (abs(DeltaHeading) > 180) // this limits error to < 180 
	{
		if(HeadingOld < Ahrs.GetHeading()) 
			DeltaHeading = DeltaHeading - 360;
		if(HeadingOld > Ahrs.GetHeading()) 
			DeltaHeading = 360 + DeltaHeading;
	}
	
	BR2_DeltaT = float((Core.millis()-bearingrate2_time)/1000.0);     
	BearingRate = DeltaHeading /BR2_DeltaT;
	BearingRateSmoothed = (1-KFilterRate) * BearingRateSmoothed + KFilterRate * BearingRate; // updates 50/sec, 3000/min, 10,000 = 3.3 min avg.
	BearingRate =  BearingRateSmoothed; // should remove bias but during a turn the BR smoothed may get to big
 	
	 bearingrate2_time = Core.millis();
//	OldHeading = Imu.m_ImuValues.Heading;
	
	return((int16_t)BearingRate*10);	
}



//======================================================================================
// Check if we missed the destination somehow
//======================================================================================
bool CNavigationFunctions::check_missed_wp(Bearings_t &Bearings,NextWaypoint_t NextWaypoint)
{
	int32_t temp;
	temp = Bearings.NextWpBearing - NextWaypoint.StartBearing;
	temp = wrap_18000(temp);
	return (abs(temp) > 10000);	// we passed the waypoint by 100 degrees
}



//======================================================================================
// Get distance between two points in cm
// R = Circumfrence of earth;
// Haversine formula formula:
// a = sin²(??/2) + cos ?1 ? cos ?2 ? sin²(??/2)
// c = 2 ? atan2( ?a, ?(1?a) )
// d = R * c
//======================================================================================
uint32_t CNavigationFunctions::DistanceCm(int32_t StartLatitude,int32_t StartLongitude,int32_t EndLatitude,int32_t EndLongitude)
{
	CMyMath Math;
	double R = 6371000.0 ;								// meters
	double RlatNow; //,RlonNow;
	double RlonNow; //,RlonNow;
	double RWayLat; //,RlonNow;
	double RWayLon; //,RlonNow;
	double dist_calc=0;
	double dist_calc2=0;
	double diflat=0;
	double diflon=0;
	double RWaypointLat;
	WayDist =0;
	RlatNow = (double)StartLatitude / 10000000.0F;
	RlonNow= (double)StartLongitude / 10000000.0F;
	RWayLat= (double)EndLatitude / 10000000.0F;
	RWayLon= (double)EndLongitude / 10000000.0F;

	//	RlonNow = NowLon;
	float DiffLat = (float)(RWayLat-RlatNow);
	float DiffLon = (float)(RWayLon-RlonNow);
//	DiffLat /= 10000000.0F;
//	DiffLon /= 10000000.0F;

	diflat=radians(DiffLat);						// Difference in Latitude notice it must be done in radians
	diflon=radians(DiffLon);						// Difference in longitudes to radians
	RlatNow=radians(RlatNow);						// Convert current latitude to radians
	RWaypointLat=radians(RWayLat);					// Convert Waypoint latitude to radians

	dist_calc = (sin((diflat/2.0))*sin((diflat/2.0)));
	dist_calc2= (cos(RlatNow)*cos(RWaypointLat))*sin( (diflon/2.0)*sin((diflon/2.0)) );

	dist_calc +=dist_calc2;
	
	WayDist=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
	WayDist*= R;
	WayDist *= 100;					//To give Cm
	return(Math.Round(WayDist));								
}


// Haversine formula:
// a = sin²(??/2) + cos ?1 ? cos ?2 ? sin²(??/2)
// c = 2 ? atan2( ?a, ?(1?a) )
// d = R ? c
// where
// ? is latitude, ? is longitude, R is earth’s radius (mean radius = 6,371km);
// note that angles need to be in radians to pass to trig functions!

uint32_t CNavigationFunctions::DistanceMeter(float NowLat,float NowLon,float WaypointLat,float WaypointLon)
{
	CMyMath MyMath;
	float r = 6371;  // radius of earth
	float A,B,C,D;
	A = NowLat;
	B = NowLon;
	C = WaypointLat;
	D = WaypointLon;


	float dLat;
	float dLon;
	dLat = (C - A);
	dLon = (D - B);
	dLat /= 57.29577951; // convert to radians
	dLon /= 57.29577951;
	float v_a;
	float v_c;
	

	v_a = sin(dLat/2) * sin(dLat/2) + cos(A) * cos(C) * sin(dLon/2) * sin(dLon/2);
	v_c = 2 * atan2(sqrt(v_a),sqrt(1-v_a));
	distance = r * v_c;
	distance *=10000;
	return( MyMath.Round( distance));
}


// return distance in meters between two locations
float CNavigationFunctions::get_distance(const Location_t &loc1,const Location_t &loc2) const
{
	CMyMath Math;
	float dlat = (float)(loc2.nLatitude - loc1.nLatitude);
	float dlng = ((float)(loc2.nLongitude - loc1.nLongitude)) * longitude_scale(loc2.nLatitude);
	return norm(dlat, dlng) * LOCATION_SCALING_FACTOR;
}


float CNavigationFunctions::GetHomeDistance(const Location_t &loc1) const
{
	Location_t Home;
	Home.nLatitude = Config.m_CourseWaypoints[0].nLatitide;
	Home.nLongitude = Config.m_CourseWaypoints[0].nLongitude;
	return(get_distance(Home,loc1));
}

bool CNavigationFunctions::GetHomeLocation(Location_t &Loc1)
{
	if(!Config.m_RunningFlags.HOME_LOCATION_SET)
		return(false);
	Loc1.nLatitude = Config.m_CourseWaypoints[0].nLatitide;
	Loc1.nLongitude = Config.m_CourseWaypoints[0].nLongitude;
	return(true);	
}

// return bearing in centi-degrees from location to loc2
int32_t CNavigationFunctions::get_bearing_to(Location_t Loc1,const Location_t &loc2) const
{
	const int32_t off_x = loc2.nLongitude - Loc1.nLongitude;
	const int32_t off_y = (loc2.nLongitude - Loc1.nLatitude) / longitude_scale(loc2.nLatitude);
	int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
	if (bearing < 0) 
		bearing += 36000;
	return bearing;
}


// return true if lat and lng match. Ignores altitude and options
bool CNavigationFunctions::same_latlon_as(Location_t Loc1,const Location_t &loc2) const
{
    return (Loc1.nLatitude == loc2.nLatitude) && (Loc1.nLongitude == loc2.nLongitude);
}






//  extrapolate latitude/longitude given bearing and distance Note that this function is accurate to about 1mm at a distance of
// 100m. This function has the advantage that it works in relative positions, so it keeps the accuracy even when dealing with small
// distances and floating point numbers
void CNavigationFunctions::offset_bearing(Location_t &Loc,float bearing, float distance)
{
    const float ofs_north = cosf(radians(bearing)) * distance;
    const float ofs_east  = sinf(radians(bearing)) * distance;
    offset(Loc,ofs_north, ofs_east);
}


// extrapolate latitude/longitude given distances (in meters) north and east
void CNavigationFunctions::offset(Location_t &Loc,float ofs_north, float ofs_east)
{
	const int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
	const int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale(Loc.nLatitude);
	Loc.nLatitude += dlat;
	Loc.nLongitude += dlng;
}



//  return the distance in meters in North/East plane as a N/E vector  from loc1 to loc2
Vector2f CNavigationFunctions::get_distance_NE(const Location_t &loc1,const Location_t &loc2) const
{
    return Vector2f((loc2.nLatitude - loc1.nLatitude) * LOCATION_SCALING_FACTOR,
                    (loc2.nLongitude - loc1.nLongitude) * LOCATION_SCALING_FACTOR * longitude_scale(loc1.nLatitude));
}


//  return the distance in meters To Home North/East plane as a N/E vector  from Home to loc
bool CNavigationFunctions::get_relative_position_NE_home(Vector2f &Loc)
{
	if(!Config.m_RunningFlags.HOME_LOCATION_SET)
		return(false);
	Location_t Loc1;
	Location_t Loc2;
	
	Loc1.nLatitude = Config.m_CourseWaypoints[0].nLatitide;
	Loc1.nLongitude = Config.m_CourseWaypoints[0].nLongitude;
	Loc2.nLatitude = m_Bearings.nPresentLatitude;
	Loc2.nLongitude = m_Bearings.nPresentLongitude;
	Loc =  get_distance_NE(Loc1,Loc2);
	return(true);
}



int32_t CNavigationFunctions::GetBearing(int32_t FromLatitude, int32_t FromLongitude,int32_t TopLatitude, int32_t ToLongitude)
{
	
	CMyMath MyMath;
	float flat1 = (float)FromLatitude/10000000.0F;   // present lat
	float flon1 = (float)FromLongitude/10000000.0F;
	float flat2 = (float)TopLatitude/10000000.0F;
	float flon2 = (float)ToLongitude/10000000.0F;
	float	x,y;
	

	flat1  = radians(flat1);
	flon1  = radians(flon1);
	flat2  = radians(flat2);
	flon2  = radians(flon2);
	y = sin(flon2-flon1)*cos(flat2);
	x = cos(flat1)*sin(flat2)-sin(flat1)*cos(flat2)*cos(flon2-flon1);
	RetVal = atan2(y,x);
	RetVal = degrees(RetVal);
	if (RetVal < 0)
		RetVal += 360;
	return(MyMath.Round(RetVal));

}


//======================================================================================
// calc_velocity_and_filtered_position - velocity in lon and lat directions calculated from GPS position
//       and accelerometer data
// lon_speed expressed in cm/s.  positive numbers mean moving east
// lat_speed expressed in cm/s.  positive numbers when moving north
// Note: we use gps locations directly to calculate velocity instead of asking gps for velocity because
//       this is more accurate below 1.5m/s
// Note: even though the positions are projected using a lead filter, the velocities are calculated
//       from the unaltered gps locations.  We do not want noise from our lead filter affecting velocity
// Calculate our current speed vector from gps position data 1mph = 1.46 ft/s
//======================================================================================
void CNavigationFunctions::CalcGpsVelocity( Bearings_t Bearings)
{

	static unsigned long LastTime =0;
	static int16_t x_speed_old = 0;
	static int16_t y_speed_old = 0;
	float GPS_scaleLonDown = cos(Bearings.nPresentLatitude * 1.0e-7f * 0.01745329251f);
	float tmp = 1.0/Core.millis()-LastTime;

	// initialize last_longitude and last_latitude
	if( last_longitude == 0 && last_latitude == 0 )
	{
		last_longitude = Bearings.nPresentLongitude;
		last_latitude = Bearings.nPresentLatitude;
	}

	
	x_actual_speed 	= (float)(Bearings.nPresentLongitude - last_longitude) *  GPS_scaleLonDown * tmp;
	y_actual_speed	= (float)(Bearings.nPresentLatitude  - last_latitude)  * tmp;

	if(Config.m_NavigationConfig.NavigationFlags &NAV_FLAG_GPS_LEAD_FILTER) 
	{
		x_actual_speed	= (x_actual_speed + x_speed_old) / 2;
		y_actual_speed	= (y_actual_speed + y_speed_old) / 2;
		x_speed_old 	= x_actual_speed;
		y_speed_old 	= y_actual_speed;
	}

	last_longitude 	= Bearings.nPresentLongitude;
	last_latitude 	= Bearings.nPresentLatitude;

	if(Config.m_NavigationConfig.NavigationFlags &NAV_FLAG_GPS_LEAD_FILTER)
	{
		GPS_lead_longitude = xLeadFilter.GetPosition(Bearings.nPresentLongitude,x_actual_speed,GPS_LAG);
		GPS_lead_latitude  = yLeadFilter.GetPosition(Bearings.nPresentLatitude,y_actual_speed,GPS_LAG);
	}

	LastTime = Core.millis();
}

//==========================================================================================
// 	tracking error is difference between course over ground and compass heading.
// 	It's purpose is to  correct drift or anything else that causes a deviation between the direction the boat
// 	is traveling and the direction the compass says it is pointing.  The results are low pass
// 	filtered to provide a time averaged result that will not change abruptly when going to a new course
// 	but recompute on a new course.
//==========================================================================================
int  CNavigationFunctions::GetTrackingWithXTEerror(NextWaypoint_t NextWaypoint)
{
	 CMyMath Math;
		
	if (abs(Wrap18000(m_Bearings.NextWpBearing - NextWaypoint.StartBearing)) < 4500)
	{	 // If too far off or too close don't do track following
		if(abs(m_Bearings.XTE_Distance) <30)
		return(0);
		if(abs(m_Bearings.XTE_Distance) <70)
		{
			if(m_Bearings.XTE_Distance <0)
				return(-6);
			else
				return(6);
		}
			
		if(abs(m_Bearings.XTE_Distance) >70)
		{
			if(m_Bearings.XTE_Distance <0)
				return(-10);
			else
				return(10);
		}
	}
	m_Bearings.m_XtrkError = Math.constrain_int16( m_Bearings.NextWpBearing,(int16_t) 20,(int16_t)-20);
	return(m_Bearings.m_XtrkError);
}



//================================================================================================================
// formulas
// lat2 = asin(sin(lat1)*cos(d/R) + cos(lat1)*sin(d/R)*cos(?))
// lon2 = lon1 + atan2(sin(?)*sin(d/R)*cos(lat1), cos(d/R)?sin(lat1)*sin(lat2))
//================================================================================================================
void CNavigationFunctions::MovePoint(int32_t &latitude, int32_t &longitude, double distanceInMetres, double bearing) 
{
    
	double dLat = latitude /10000000.0;
    double dLon = longitude /10000000.0;
	double brngRad = radians(bearing);
    double latRad = radians(dLat);
    double lonRad = radians(dLon);
//	distanceInMetres /= 1000;
    int earthRadiusInMetres = 6371000;
    double distFrac = distanceInMetres / earthRadiusInMetres;

    double latitudeResult = asin(sin(latRad) * cos(distFrac) + cos(latRad) * sin(distFrac) * cos(brngRad));
    double longitudeResult = lonRad+atan2(sin(brngRad) * sin(distFrac) * cos(latRad), cos(distFrac) - sin(latRad) * sin(latitudeResult));

	latitude = degrees(latitudeResult)*10000000.0F;
	longitude = degrees(longitudeResult)*10000000.0F;
//    printf("latitude: %lf longitude %lf\n",latitudeResult,longitudeResult);
}

// 1 knot = 0.514444 meters/s
void CNavigationFunctions::DebugMoveAlongCourse(Bearings_t &Bearings,double Bearing,uint16_t Speed)
{
	static uint32_t LastTime = 0;
	int32_t Latitude = Bearings.nPresentLatitude;
	int32_t Longitude = Bearings.nPresentLongitude;
	double Meter_s; 

	if((Core.millis() - LastTime) >999)
	{
		Meter_s = Speed *0.514444;
		MovePoint(Latitude,Longitude, Meter_s, Bearing);
		Bearings.nPresentLatitude = Latitude;
		Bearings.nPresentLongitude = Longitude;
		LastTime = Core.millis();
	}
	
	
} 




//======================================================================================
// Utilities
//======================================================================================


// int32_t CNavigationFunctions::wrap_180(int32_t error)
// {
// 	if (error > 180)
// 	error -= 360;
// 	if (error < -180)
// 	error += 360;
// 	return error;
// }
// int32_t CNavigationFunctions::wrap_360(int32_t angle)
// {
// 	if (angle > 360)
// 	angle -= 360;
// 	if (angle < 0)
// 		angle += 360;
// 	return angle;
// }
// 
// 
// 
// 
// 
int32_t CNavigationFunctions::wrap_18000(int32_t error)
{
	if (error > 18000)
		error -= 36000;
	if (error < -18000)
		error += 36000;
	return error;
}

// int32_t CNavigationFunctions::wrap_36000(int32_t angle)
// {
// 	if (angle > 36000)
// 		angle -= 36000;
// 	if (angle < 0)
// 		angle += 36000;
// 	return angle;
// }



void CNavigationFunctions::TestNav()
{
	// bournville 52.429402, -1.946131 to here 52.441411, -1.982395     2.798 km    Final bearing: 298 30 10
	unsigned long Distance;
	uint32_t Bearing;
	while(1)
	{
		Distance = DistanceCm(52441411,-1982395 ,52429402,-1946131);
		//		Bearing = GetBearing(52441411,-1982395,52429402,-1946131);
		Bearing = GetBearing(52429402,-1946131,52441411,-1982395);
		printf("Distance %f    bearing  %ld\n",(float)Distance/1000,Bearing);
		Core.delay(500);
		
		
	}
}




//==================== Leadfilter ===========================
float LeadFilter::GetPosition(float pos, int16_t vel, float GPSlagInSeconds)
{

	int16_t AccelContribution = (vel - m_LastVelocity) * GPSlagInSeconds * GPSlagInSeconds;
	int16_t VelContribution = vel * GPSlagInSeconds;

	// store velocity for next iteration
	m_LastVelocity = vel;

	return pos + VelContribution + AccelContribution;
}



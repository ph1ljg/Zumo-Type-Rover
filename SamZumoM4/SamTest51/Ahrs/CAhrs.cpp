/* 
* CAhrs.cpp
*
* Created: 21/08/2020 17:38:48
* Author: philg
*/


#include "includes.h"
#include "CCompass.h"

CCompass Compass;
bool GetRawValues = false;



// default constructor
CAhrs::CAhrs()
{
	m_Status = 0;
	m_HeadingVelocity  = 0;
	AverageLinAccel.zero();
	LastLinAccel.zero();

} //CAhrs

// default destructor
CAhrs::~CAhrs()
{
} //~CAhrs


void CAhrs::Update(uint32_t Period)
{
	//velocity = accel*dt (dt in seconds)   position = 0.5*accel*dt^2
	float dt =  (float)Period / 1000000.0;
	float Position_dt = 0.5 * dt * dt;

	if(!Config.m_RunningFlags.ACC_CALIBRATED || !Config.m_RunningFlags.GYRO_CALIBRATED || !Config.m_RunningFlags.MAG_CALIBRATED)
		CheckCalibration();
	
	if(UpDateCompass())
	{
		Config.m_RunningFlags.IMU_FAIL = false;
		m_Status |= AHRS_COMPASS_OK;
		m_AhrsValues.Eular.z += (Config.m_NavigationConfig.MagDeclination/10);
	}
	else
	{
		m_Status &= ~AHRS_COMPASS_OK;
		Config.m_RunningFlags.IMU_FAIL = true;
	}

	if(Gps.IsGpsUsable())
	{
		m_Status |= AHRS_GPS_OK;
		m_AhrsValues.m_VelocityNED.x = Gps.m_GpsReadings.velocity.x;
		m_AhrsValues.m_VelocityNED.y = Gps.m_GpsReadings.velocity.y;
		m_AhrsValues.m_VelocityNED.z = Gps.m_GpsReadings.velocity.z;
	}
	else
		m_Status &= ~AHRS_GPS_OK;
	
  // calc_trig(BN0055.GetDcm(),   _cos_roll, _cos_pitch, _cos_yaw,   _sin_roll, _sin_pitch, _sin_yaw);
	
	m_HeadingVelocity = Imu.m_LinAccelFiltered.y/ cos(DEG_TO_RAD * m_AhrsValues.Eular.x);
	LastLinAccel = Imu.m_LinAccelFiltered;
//	GuiFunctions.SetPidOutputValues( m_AhrsValues.Eular.y*10,GUI_PID_OUTPUT_OUTPUT);
//	GuiFunctions.SetPidOutputValues( m_AhrsValues.Eular.z*10,GUI_PID_OUTPUT_ERROR);


}

void CAhrs::Init()
{
}


bool  CAhrs::UpDateCompass()
{
	if(Compass.Update())
	{
		m_AhrsValues.Eular.z = Compass.GetHeadingDegrees();
		m_AhrsValues.Eular.x = Compass.GetPitch();
		m_AhrsValues.Eular.y = Compass.GetRoll();
		m_AhrsValues.Temperature = Compass.GetTemperature();
		m_AhrsValues.CalibrationState =  Compass.GetSysCal();
		m_AhrsValues.RawMag.zero(); //Compass.m_CmpsRawData.MagRaw;
		m_AhrsValues.RawGyro.zero(); //Compass.m_CmpsRawData.GiroRaw;
		m_AhrsValues.RawAccel.zero();//Compass.m_CmpsRawData.AccelRaw;
		Config.m_RunningFlags.IMU_FAIL = false;
		Compass.UpdateTurnRate();
		return(true);
	}
	else
		Config.m_RunningFlags.IMU_FAIL = true;
	return(false);
}


float CAhrs::GetHeadingVelocity()
{
	return(m_HeadingVelocity);
}



bool CAhrs::GetHeadingVelocity(float &Velocity)
{
	if(Imu.m_LinVelGood)
	{
		Velocity = m_HeadingVelocity;
		return(true);
	}
	return(false);
}





float CAhrs::GetHeadingRad()
{
	return(radians(m_AhrsValues.Eular.z));
}


float CAhrs::GetHeading()
{
	return(m_AhrsValues.Eular.z);
}

float CAhrs::GetRollD()
{
	return(m_AhrsValues.Eular.x);
}

float CAhrs::GetPitchD()
{
	return(m_AhrsValues.Eular.y);
}

float CAhrs::GetRollR()
{
	return(radians(m_AhrsValues.Eular.x));
}

float CAhrs::GetPitchR()
{
	return(radians(m_AhrsValues.Eular.y));
}



int32_t CAhrs::GetHeadingCD()
{
	return((uint32_t) m_AhrsValues.Eular.z*100);
}

void CAhrs::CalibrateLevel()
{
	
}


void CAhrs::CheckCalibration()
{
	Compass.UpdateCalState();
	if(Compass.GetGyroCal())		Config.m_RunningFlags.GYRO_CALIBRATED = true;	else		Config.m_RunningFlags.GYRO_CALIBRATED = false;
	if(Compass.GetAccelCal())		Config.m_RunningFlags.ACC_CALIBRATED = true;	else		Config.m_RunningFlags.ACC_CALIBRATED = false;
	if(Compass.GetMagCal())		Config.m_RunningFlags.MAG_CALIBRATED = true;	else		Config.m_RunningFlags.MAG_CALIBRATED = false;
}


// get yaw rate in earth frame in radians/sec
float CAhrs::get_yaw_rate_earth(void) const
{
	return(Compass.GetTurnRate());
}


void  CAhrs::SaveCompassCalibration()
{
	static bool CalStored = false;
	if(!CalStored)
	{
		CalStored = true;
		Compass.StoreCalibration();
	}
}



// return a ground speed estimate in m/s
Vector2f CAhrs::GetGroundspeedVector(void)
{
	Vector2f GndVelGPS;								// Generate estimate of ground speed vector

	if (Gps.IsGpsUsable())			// Generate estimate of ground speed vector using GPS
	{
		const float cog = radians(Gps.m_GpsReadings.GroundCourse*0.01f);
		GndVelGPS = Vector2f(cosf(cog), sinf(cog)) * Gps.m_GpsReadings.GroundSpeed;
		return GndVelGPS;
	}

	return Vector2f(0.0f, 0.0f);
}



bool CAhrs::GetVelocityNED(Vector3f &velocity)
{
	if(Imu.m_LinVelGood)
	{
		velocity = m_linearAccelVect;
		return(true);
	}
	return(false);
}

// rotate a 2D vector from earth frame to body frame
Vector2f CAhrs::earth_to_body2D(const Vector2f &ef) const
{
	return Vector2f(ef.x * _cos_yaw + ef.y * _sin_yaw,
	-ef.x * _sin_yaw + ef.y * _cos_yaw);
}

// rotate a 2D vector from body frame to body earth
Vector2f CAhrs::body_to_earth2D(const Vector2f &bf) const
{
	return Vector2f(bf.x * _cos_yaw - bf.y * _sin_yaw,
	bf.x * _sin_yaw + bf.y * _cos_yaw);
}

 // convert a vector from body to earth frame
Vector3f CAhrs::body_to_earth(const Vector3f &v) const 
{
   return v * get_rotation_body_to_ned();
}

    // convert a vector from earth to body frame
 Vector3f CAhrs::earth_to_body(const Vector3f &v) const 
{
        return get_rotation_body_to_ned().mul_transpose(v);
}

// return rotation matrix representing rotaton from body to earth axes
const Matrix3f CAhrs::get_rotation_body_to_ned() const  
{
	Matrix3f Rot = BN0055.GetDcm();
    return (Rot);
}

//=============================================================================
// calculate sin and cos of roll/pitch/yaw from a body_to_ned rotation matrix
//=============================================================================

void CAhrs::calc_trig(const Matrix3f &rot,float &cr, float &cp, float &cy,float &sr, float &sp, float &sy) const
{
	CMyMath Math;
	Vector2f yaw_vector(rot.a.x, rot.b.x);

	if (fabsf(yaw_vector.x) > 0 || fabsf(yaw_vector.y) > 0) 
	{
		yaw_vector.normalize();
	}
	sy = Math.constrain_float(yaw_vector.y, -1.0f, 1.0f);
	cy = Math.constrain_float(yaw_vector.x, -1.0f, 1.0f);

	// sanity checks
	if (yaw_vector.is_inf() || yaw_vector.is_nan()) 
	{
		sy = 0.0f;
		cy = 1.0f;
	}

	const float cx2 = rot.c.x * rot.c.x;
	if (cx2 >= 1.0f) 
	{
		cp = 0;
		cr = 1.0f;
	} 
	else 
	{
		cp = safe_sqrt(1 - cx2);
		cr = rot.c.z / cp;
	}
	cp = Math.constrain_float(cp, 0.0f, 1.0f);
	cr = Math.constrain_float(cr, -1.0f, 1.0f); // this relies on constrain_float() of infinity doing the right thing

	sp = -rot.c.x;

	if (!is_zero(cp)) 
	{
		sr = rot.c.y / cp;
	}

	if (is_zero(cp) || isinf(cr) || isnan(cr) || isinf(sr) || isnan(sr)) 
	{
		float r, p, y;
		rot.to_euler(&r, &p, &y);
		cr = cosf(r);
		sr = sinf(r);
	}
}


/*
// Reference from: http://www.geometrictools.com/Documentation/EulerAngles.pdf// SEE SECTION 2.1 - "Factor as RxRyRz"//
euler camadd(euler E1, euler E2) 
{    
	euler E3;    
	double sx,sy,sz,cx,cy,cz;    
	sx = sin(E1.angx);  
	sy = sin(E1.angy);  
	sz = sin(E1.angz);    
	cx = cos(E1.angx);  
	cy = cos(E1.angy);  
	cz = cos(E1.angz);    
	double a1 = cy*cz;              
	double a2 = -cy*sz;             
	double a3 = sy;    
	double b1 = cz*sx*sy + cx*sz;   
	double b2 = cx*cz-sx*sy*sz;     
	double b3 = -cy*sx;    
	double c1 = -cx*cz*sy+ sx*sz;   
	double c2 = cz*sx + cx*sy*sz;   
	double c3 = cx*cy;    
	sx = sin(E2.angx);  
	sy = sin(E2.angy);  
	sz = sin(E2.angz);    
	cx = cos(E2.angx);  
	cy = cos(E2.angy);  
	cz = cos(E2.angz);    
	double m1 = cy*cz;              
	double m2 = -cy*sz;             
	double m3 = sy;    
	double n1 = cz*sx*sy + cx*sz;   
	double n2 = cx*cz-sx*sy*sz;     
	double n3 = -cy*sx;    
	double o1 = -cx*cz*sy+ sx*sz;   
	double o2 = cz*sx + cx*sy*sz;   
	double o3 = cx*cy;        
	double r00=(a1*m1 + a2*n1 + a3*o1);     
	double r01=(a1*m2 + a2*n2 + a3*o2);     
	double r02=(a1*m3 + a2*n3 + a3*o3);     
	double r10=(b1*m1 + b2*n1 + b3*o1);    
	 double r11=(b1*m2 + b2*n2 + b3*o2);     
	 double r12=(b1*m3 + b2*n3 + b3*o3);     
	 double r20=(c1*m1 + c2*n1 + c3*o1);     
	 double r21=(c1*m2 + c2*n2 + c3*o2);    
	  double r22=(c1*m3 + c2*n3 + c3*o3);     
	  if (r02 < +1)    
	  {        if (r02 > -1)   
			  {            
				E3.angy = asin(r02);            
				E3.angx = atan2(-r12,r22);            
				E3.angz = atan2(-r01,r00);        
			 }
			 else 
			 // r02 = -1        
			 {            // Not a unique solution:	 thetaZ - thetaX = atan2(r10,r11)            
				 E3.angy = -0.5*PI;            
				 E3.angx = -atan2(r10,r11);            
				 E3.angz = 0;        
			}    
		}    
		else // r02 = +1    
		{        // Not a unique solution: 	thetaZ + thetaX = atan2(r10,r11)        
			E3.angy = 0.5*PI;       
			E3.angx = atan2(r10,r11);
			E3.angz = 0;    
		}    
		return E3:
}
*/
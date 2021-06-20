/*
* CinertialManagement.cpp
*
* Created: 24/02/2021 16:27:34
* Author: philg
*/
#include "Includes.h"
#include "CCmps12.h"
#include "CBN0055.h"
#include "CLowPassFilter2p.h"
#include "CNotchFilter.h"


extern CBN0055 BN0055;
extern CCmps12 Cmps12;


// default constructor
CImu::CImu()
{
	m_AccelClipLimit =  29.5f * GRAVITY_MSS;
	_accel_clip_count =0;
	 _accel_filter_cutoff = DEFAULT_ACCEL_FILTER;
	GyroFilterCutoff= DEFAULT_GYRO_FILTER;
	_gyro_cal_timing =1;
    _accel_raw_sample_rates = ACCEL_BACKEND_SAMPLE_RATE;
    _gyro_raw_sample_rates =  GYRO_BACKEND_SAMPLE_RATE;
	_last_raw_gyro.zero();
	m_NewGyroData = false;
	calibrating = false;
	LastAccelerationFilterHz = 0;
	lastLinAcceFilterhz = 0;
} //CinertialManagement

// default destructor
CImu::~CImu()
{
} //~CinertialManagement


void CImu::Init()
{
// 	if(!BN0055.Init())
// 	{
// 	 	Config.m_RunningFlags.IMU_FAIL = false;
// 		return;
// 	}
	Core.delay(1000);
	LinAccelLastSample.zero();
	Update(1000000);
	InitGyro();
	//Test();
}



		
	//	xPos = xPos + ACCEL_POS_TRANSITION * linearAccelVect.x;
	//	yPos = yPos + ACCEL_POS_TRANSITION * linearAccelVect.y;

	// velocity of sensor in the direction it's facing
	






void CImu::Update(uint32_t Period)
{
	m_AccelGood = false;
	m_AngularVelGood = false;
	m_LinVelGood = false;

	float dt = 	Period/1000000;	// to seconds
//	if(GetAccelerometerData())
//		m_AccelGood = true;
	
	if(GetGyroData())
		m_AngularVelGood = true;
	
	if(GetLinAccelerometerData())
		m_LinVelGood = true;
	
	if(m_AccelGood && m_AngularVelGood&&m_LinVelGood)
		m_Status |= IMU_BN0055_OK;
	else
		m_Status &= ~IMU_BN0055_OK;



	// clear accumulators
	DeltaAngVelocityAcc.zero();
	DeltaVelocityAccAccel_dt = 0;
	DeltaAngleAccGyro.zero();
	DeltaAngleAccGyro_dt = 0;

    // possibly update filter frequency
    if (LastAccelerationFilterHz != 10) 
	{
        AccelerationFilter.set_cutoff_frequency(20,_accel_filter_cutoff);
        LastAccelerationFilterHz = 10;
    }

   // possibly update filter frequency
   if (lastLinAcceFilterhz != 10)
   {
	   m_LinAccelerationFilter.set_cutoff_frequency(20,4);
	   lastLinAcceFilterhz = 10;
   }


   // possibly update filter frequency
   if (LastGyroFilterHz != GyroFilterCutoff|| sensors_converging()) 
   {
	   GyroFilter.set_cutoff_frequency(_gyro_raw_sample_rates, GyroFilterCutoff);
	   LastGyroFilterHz = GyroFilterCutoff;
   }

   // possibly update the notch filter parameters
   if (!is_equal(LastNotchCenterFreqHz,GyroNotchFilter.NotchParams.CenterFreqHz)  || !is_equal(LastNotchBandwidthHz, GyroNotchFilter.NotchParams.BandwidthHz)
				|| !is_equal(LastNotchAttenuation_dB, GyroNotchFilter.NotchParams.Attenuation_dB) ||sensors_converging())
	{
 	   GyroNotchFilter.init(_gyro_raw_sample_rates, GyroNotchFilter.NotchParams.CenterFreqHz,  GyroNotchFilter.NotchParams.BandwidthHz, GyroNotchFilter.NotchParams.Attenuation_dB);
	   LastNotchCenterFreqHz = GyroNotchFilter.NotchParams.CenterFreqHz;
	   LastNotchBandwidthHz =  GyroNotchFilter.NotchParams.BandwidthHz;
	   LastNotchAttenuation_dB = GyroNotchFilter.NotchParams.Attenuation_dB;
	}
}


void CImu::InitGyro()
{
    Vector3f LastAverage, BestAvg;
    Vector3f NewGyroOffset;
    float best_diff;
    bool converged;
 
  
    // remove existing gyro offsets
	NewGyroOffset.zero();
	best_diff = -1.f;
	LastAverage.zero();
	converged = false;
	uint32_t LastTime;

    for(int8_t c = 0; c < 4; c++) 
	{
		 LastTime = Core.micros();
        Core.delay(10);
        Update(Core.micros() - LastTime);
    }

    // the strategy is to average 50 points over 0.5 seconds, then do it again and see if the 2nd average is within a small margin of the first
    // Try to get a good calibration estimate for up to 30 seconds if the gyros are stable, we should get it in 1 second
    for (int16_t j = 0; j <= 30*2; j++) 
	{
		Vector3f gyro_sum, gyro_avg, gyro_diff;
		Vector3f accel_start;
		float diff_norm;
		uint8_t i;
		diff_norm = 0;
		gyro_sum.zero();
		accel_start = m_AccelFiltered;

        for (i=0; i<50; i++) 
		{
			 LastTime = Core.micros();
            Core.delay(10);
			GetGyroData();
	        GetAccelerometerData();
             gyro_sum += m_GyroFiltered;
        }

        Vector3f accel_diff = m_AccelFiltered - accel_start;
        if (accel_diff.length() > 0.2f) 
		{
            // the accelerometer changed during the gyro sum. Skip this sample. This copes with doing gyro cal on a
            // steadily moving platform. The value 0.2 corresponds with around 5 degrees/second of rotation.
            continue;
        }

        gyro_avg = gyro_sum / i;
        gyro_diff = LastAverage - gyro_avg;
        diff_norm = gyro_diff.length();

        if (best_diff < 0) 
		{
            best_diff = diff_norm;
            BestAvg = gyro_avg;
        } 
		else  
		{
			float DiffLen = gyro_diff.length();
            if (DiffLen < radians(GYRO_INIT_MAX_DIFF_DPS))
			{
				LastAverage = (gyro_avg * 0.5f) + (LastAverage * 0.5f);					// The average should  be within 0.1 bit, which is 0.04 degrees/s
				if (!converged || LastAverage.length() < NewGyroOffset.length()) 
					NewGyroOffset = LastAverage;

				if (!converged) 
				{
					converged = true;
					break;
				}
			}
			else if (diff_norm < best_diff)
			{
				best_diff = diff_norm;
				BestAvg = (gyro_avg * 0.5f) + (LastAverage * 0.5f);
			}
			LastAverage = gyro_avg;
		} 
	}

   //  waited long enough - use the best pair  found so far
	if (!converged) 
	{
		DebugDisplay.Printf("gyro did not converge: diff=%f dps (expected < %f)\n",(double)degrees(best_diff),	(double)GYRO_INIT_MAX_DIFF_DPS);
		_gyro_offset = BestAvg;
		// flag calibration as failed for this gyro
		Config.m_RunningFlags.GYRO_CALIBRATED = false;
	} 
	else 
	{
		Config.m_RunningFlags.GYRO_CALIBRATED = true;
		_gyro_offset = NewGyroOffset;
	}

}


Vector3f CImu::GetLinAcceleration()
{
	return(m_LinAccelFiltered);
}


bool CImu::GetLinAccelerometerData()
{
// 	float dt;
// 	Vector3f LinAccel;
// 	if(	BN0055.GetVector(LinAccel, VECTOR_LINEARACCEL))
// 	{
// 		
// 		uint32_t  NowTime = Core.micros();
// 		dt = (NowTime - LinAccelLastSample_us) * 1.0e-6f; //change to sec
// 		LinAccelLastSample_us = NowTime;
// 
// 		CalcVibrationAndClippingLin( LinAccel, dt);
// 
// 		m_LinAccelFiltered = m_LinAccelerationFilter.apply(LinAccel);
// 		if (m_LinAccelFiltered.is_nan() || m_LinAccelFiltered.is_inf())
// 			m_LinAccelerationFilter.reset();
// 		SetLinAccelPeakHold( m_LinAccelFiltered);
// 	
// 		m_LinVelocity += m_LinAccelFiltered*dt;
		
// 	   Vector3f LinAccelDiff = LinAccel - LinAccelLastSample;
//        if (LinAccelDiff.length() > 0.2f) // The value 0.2 corresponds with around 5 degrees/second of rotation. to stop build up
//             m_LinVelocity.zero();

// 		LinAccelLastSample = LinAccel;
// 		
// 		return(true);
// 	}
 	return(false);
}

bool CImu::GetAccelerometerData()
{
// 	uint32_t lastSampleTime = _accel_last_sample_us;
// 	uint32_t  NowTime = Core.micros();
// 	Vector3f Accel;
// 	float dt;
// 	if(BN0055.GetVector(Accel, VECTOR_ACCELEROMETER))
// 	{
// 		
// 		dt = (NowTime - _accel_last_sample_us) * 1.0e-6f; //change to s
// 		_accel_last_sample_us = NowTime;
// 
// 		calc_vibration_and_clipping( Accel, dt);
// 
// 		if (NowTime - lastSampleTime > 100000U)
// 		{
// 			// zero accumulator if sensor was unhealthy for 0.1s
// 			DeltaAngVelocityAcc.zero();
// 			DeltaVelocityAccAccel_dt = 0;
// 			dt = 0;
// 		}
// 		// delta velocity
// 		DeltaAngVelocityAcc += Accel * dt;
// 		DeltaVelocityAccAccel_dt += dt;
// 		m_AccelFiltered = AccelerationFilter.apply(Accel);
// 		if (m_AccelFiltered.is_nan() || m_AccelFiltered.is_inf())
// 			AccelerationFilter.reset();
// 		set_accel_peak_hold( m_AccelFiltered);
// 		return(true);
// 	}
	return(false);
}


bool CImu::GetGyroData()
{
// 	float dt;
// 	Vector3f Gyro;
// 
// 	if( BN0055.GetVector(Gyro, VECTOR_GYROSCOPE))
// 	{
// 		uint32_t lastSampleTime = _gyro_last_sample_us;
// 		uint32_t  NowTime = Core.micros();
// 		m_GyroFiltered = Gyro;
// 		return(true);
// 
// 		if (NowTime != 0 && _gyro_last_sample_us != 0) 
// 		{
// 			dt = (NowTime - _gyro_last_sample_us) * 1.0e-6f;
// 			_gyro_last_sample_us = NowTime;
// 		} 
// 		else 
// 		{
// 			_gyro_last_sample_us = NowTime;
// 				return(true);
// 		}
// 		// compute delta angle
// 		Vector3f delta_angle = (Gyro + _last_raw_gyro) * 0.5f * dt;
// 
// 		// compute coning correction
// 		Vector3f delta_coning = (DeltaAngleAccGyro +_last_delta_angle * (1.0f / 6.0f));
// 		delta_coning = delta_coning % delta_angle;
// 		delta_coning *= 0.5f;
// 
// 		uint64_t now = Core.micros();
// 
// 		if (now - lastSampleTime > 100000U) 
// 		{
// 			// zero accumulator if sensor was unhealthy for 0.1s
// 			DeltaAngleAccGyro.zero();
// 			DeltaAngleAccGyro_dt = 0;
// 			dt = 0;
// 			delta_angle.zero();
// 		}
// 		// integrate delta angle accumulator the angles and coning corrections are accumulated separately in the
// 		// referenced paper, but in simulation little difference was found between integrating together and integrating separately (see examples/coning.py)
// 		DeltaAngleAccGyro += delta_angle + delta_coning;
// 		DeltaAngleAccGyro_dt += dt;
// 
// 		// save previous delta angle for coning correction
// 		_last_delta_angle = delta_angle;
// 		_last_raw_gyro = Gyro;
// 
// 		Vector3f gyro_filtered = Gyro;
// 
// 		// apply the notch filter
//  		if (GyroNotchFilter.NotchParams._enable) 
//  		{
//  			gyro_filtered = GyroNotchFilter.apply(gyro_filtered);
//  		}
// 		// apply the low pass filter last to attenuate any notch induced noise
// 		gyro_filtered = GyroFilter.apply(gyro_filtered);
// 
// 		// if the filtering failed in any way then reset the filters and keep the old value
// 		if (gyro_filtered.is_nan() || gyro_filtered.is_inf()) 
// 		{
// 			GyroFilter.reset();
// 			GyroNotchFilter.reset();
// 		} 
// 		else 
// 		  m_GyroFiltered = gyro_filtered;
// 
// 		m_NewGyroData = true;
// 			return(true); 
// 	}
	return(false);
}



// peak hold detector for slower mechanisms to detect spikes
void CImu::SetLinAccelPeakHold( const Vector3f &accel)
{
	uint32_t now = Core.millis();

	// negative x peak(min) hold detector
	if (accel.x < LinPeakHoldState.AccelPeakHoldNeg_x ||	LinPeakHoldState.AccelPeakHoldNegXAge <= now) 
	{
		LinPeakHoldState.AccelPeakHoldNeg_x = accel.x;
		LinPeakHoldState.AccelPeakHoldNegXAge = now + INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS;
	}
}




// peak hold detector for slower mechanisms to detect spikes
void CImu::set_accel_peak_hold( const Vector3f &accel)
{
	uint32_t now = Core.millis();

	// negative x peak(min) hold detector
	if (accel.x < _peak_hold_state.accel_peak_hold_neg_x ||	_peak_hold_state.accel_peak_hold_neg_x_age <= now) 
	{
		_peak_hold_state.accel_peak_hold_neg_x = accel.x;
		_peak_hold_state.accel_peak_hold_neg_x_age = now + INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS;
	}
}

// calculate vibration levels and check for accelerometer clipping (called by a backends)
void CImu::CalcVibrationAndClippingLin( const Vector3f &Accel, float dt)
{

	if (fabsf(Accel.x) > m_AccelClipLimit ||  fabsf(Accel.y) >  m_AccelClipLimit ||  fabsf(Accel.z) > m_AccelClipLimit)
	_accel_clip_count++;

	// filter accel at 5hz
	Vector3f accel_filt = m_LinAccelVibeFlooFilter.apply(Accel, dt);

	// calc difference from this sample and 5hz filtered value, square and filter at 2hz
	Vector3f accel_diff = (Accel - accel_filt);
	accel_diff.x *= accel_diff.x;
	accel_diff.y *= accel_diff.y;
	accel_diff.z *= accel_diff.z;
	m_LinAccelVibeFilter.apply(accel_diff, dt);
}






// calculate vibration levels and check for accelerometer clipping (called by a backends)
void CImu::calc_vibration_and_clipping( const Vector3f &Accel, float dt)
{

    if (fabsf(Accel.x) > m_AccelClipLimit ||  fabsf(Accel.y) >  m_AccelClipLimit ||  fabsf(Accel.z) > m_AccelClipLimit)
	    _accel_clip_count++;

	// filter accel at 5hz
	Vector3f accel_filt = _accel_vibe_floor_filter.apply(Accel, dt);

	// calc difference from this sample and 5hz filtered value, square and filter at 2hz
	Vector3f accel_diff = (Accel - accel_filt);
	accel_diff.x *= accel_diff.x;
	accel_diff.y *= accel_diff.y;
	accel_diff.z *= accel_diff.z;
	_accel_vibe_filter.apply(accel_diff, dt);
}




bool CImu::GetAccelerationVect(Vector3f &Vect)
{
	
	//return(BN0055.GetVector(Vect, VECTOR_LINEARACCEL));
	return(true);
}

bool CImu::OrientationVect(Vector3f& Vect)
{

	//return(BN0055.GetVector(Vect, VECTOR_EULER));
	return(true);
}

bool CImu::AngVelocityVect(Vector3f& Vect)
{

	//return(BN0055.GetVector(Vect, VECTOR_GYROSCOPE));
	return(true);
}

bool CImu::AccelVect(Vector3f& Vect)
{

	//return(BN0055.GetVector(Vect, VECTOR_ACCELEROMETER));
	return(true);
}


float CImu::GetForwardSpeed()
{
	return( WheelEncoder.GetMetersSec(BOTH_WHEEL_SENSORS));
}

bool  CImu::GetForwardVelocity(float* Vel)
{
//	bool RetVal = true;
	Vector3f Vect,OVect;
	float FVel =0;
	float Rate = WheelEncoder.GetRate(BOTH_WHEEL_SENSORS);
	
	if(GetAccelerationVect(Vect))
	{
		if(OrientationVect(OVect))
		{
			double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
			FVel = ACCEL_VEL_TRANSITION * Vect.x / cos(DEG_TO_RAD * OVect.x);	// velocity of sensor in the direction it's facing (front of vehicle)
		}
	}
	*Vel = max(Rate,FVel);
	return(true);
}



void CImu::Test()
{
		float Giro;
		Vector3f Giro2;
		Vector3f Giro1;
		//	uint32_t Task;
		static uint32_t UpdateLastTime = 0;
		static uint32_t DisplayLastTime = 0;
		uint32_t Period;
		while(1)
		{
			Period = Core.millis()-UpdateLastTime;
			if(Period >10)
			{
				Imu.Update(1000);
				Cmps12.GetGyroRaw();
				Giro = Ahrs.get_yaw_rate_earth();
				Period = Core.millis()-DisplayLastTime;
				if(Period >300)
				{	Vector3f GyroAd = m_GyroFiltered - _gyro_offset;
					DebugDisplay.Printf(10,10,1,"Yaw Rate %f      Bn  X %f   Y %f   Z %f",Giro,GyroAd.x, GyroAd.y,GyroAd.z);
					DebugDisplay.Printf(11,10,1,"X %f   Y %f   Z %f",radians(Giro2.x), radians(Giro2.y),Giro2.z);
					DebugDisplay.Printf(12,10,1,"%d",Period);
					DisplayLastTime = Core.millis();;
				}
				UpdateLastTime = Core.millis();
			}
		}
	
}
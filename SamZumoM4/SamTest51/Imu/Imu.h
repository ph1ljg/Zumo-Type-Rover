/*
* CinertialManagement.h
*
* Created: 24/02/2021 16:27:34
* Author: philg
*/


#ifndef __CINERTIALMANAGEMENT_H__
#define __CINERTIALMANAGEMENT_H__
#include "CLowPassFilter2p.h"
#include "CLowPassFilter.h"
#include "CNotchFilter.h"

#define  IMU_BN0055_OK			(1<<2)


// Gyro and Accelerometer calibration criteria
#define AP_INERTIAL_SENSOR_ACCEL_TOT_MAX_OFFSET_CHANGE  4.0f
#define AP_INERTIAL_SENSOR_ACCEL_MAX_OFFSET             250.0f
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FLOOR_FILT_HZ     5.0f    // accel vibration floor filter hz
#define AP_INERTIAL_SENSOR_ACCEL_VIBE_FILT_HZ           2.0f    // accel vibration filter hz
#define INERTIAL_SENSOR_ACCEL_PEAK_DETECT_TIMEOUT_MS 500     // peak-hold detector timeout
#define GYRO_INIT_MAX_DIFF_DPS 0.1f


#define MAG_CALIBRATION_MASK (3<<0)
#define ACC_CALIBRATION_MASK (3<<2)
#define GIR_CALIBRATION_MASK (3<<4)



#define DEFAULT_GYRO_FILTER  4
#define DEFAULT_ACCEL_FILTER 10
#define DEFAULT_STILL_THRESH 0.1f

#define ACCEL_BACKEND_SAMPLE_RATE   50 //2000
#define GYRO_BACKEND_SAMPLE_RATE    50 //2000


class CImu
{
	//variables
public:
	// filtering frequency (0 means default)
	int16_t    _accel_filter_cutoff;
	int16_t    GyroFilterCutoff;
	int8_t     _gyro_cal_timing;
    
	// support for updating filter at runtime
    uint16_t lastLinAcceFilterhz;
    uint16_t LastAccelerationFilterHz;
    uint16_t LastGyroFilterHz;
    float LastNotchCenterFreqHz;
    float LastNotchBandwidthHz;
    float LastNotchAttenuation_dB;
	bool m_NewGyroData;

    // accelerometer and gyro raw sample rate in units of Hz
    float  _accel_raw_sample_rates;
    float  _gyro_raw_sample_rates;

	uint8_t _num_calculated_harmonic_notch_frequencies = 1;
    // accelerometer scaling and offsets
	Vector3f _accel_scale;
	Vector3f _accel_offset;
	Vector3f _gyro_offset;
	Vector3f m_AccelFiltered;
	Vector3f m_LinAccelFiltered;
	Vector3f m_GyroFiltered;
	Vector3f m_LinVelocity;
	bool m_AccelGood;
	bool m_LinVelGood;
	bool m_AngularVelGood;
	uint8_t m_Status;

protected:
	uint32_t _gyro_last_sample_us;
	uint32_t _accel_last_sample_us;
	uint32_t LinAccelLastSample_us;
	Vector3f LinAccelLastSample;
	float m_AccelClipLimit;

	    
private:

   // Low Pass filters for gyro and accel
	LowPassFilter2pVector3f  AccelerationFilter;
	LowPassFilter2pVector3f GyroFilter;
	NotchFilterVector3f GyroNotchFilter;
	LowPassFilter2pVector3f m_LinAccelerationFilter;

	// vibration and clipping
	uint32_t _accel_clip_count;
	LowPassFilterVector3f _accel_vibe_floor_filter;
	LowPassFilterVector3f _accel_vibe_filter;
	LowPassFilterVector3f m_LinAccelVibeFlooFilter;
	LowPassFilterVector3f m_LinAccelVibeFilter;
	// delta velocity accumulators
	Vector3f DeltaAngVelocityAcc;
	Vector3f DeltaLinVelocityAcc;
	Vector3f DeltaVelocityAcc;
	// time accumulator for delta velocity accumulator
	float DeltaVelocityAccAccel_dt;


	// peak hold detector state for primary accel
	struct PeakHoldState 
	{
		float accel_peak_hold_neg_x;
		uint32_t accel_peak_hold_neg_x_age;
	} _peak_hold_state;

	// peak hold detector state for Lin accel
	struct LinPeakHoldState 
	{
		float AccelPeakHoldNeg_x;
		uint32_t AccelPeakHoldNegXAge;
	} LinPeakHoldState;


	Vector3f _last_raw_gyro;
	Vector3f DeltaAngleAccGyro;
	Vector3f _last_delta_angle;
	// time accumulator for delta angle accumulator
	float DeltaAngleAccGyro_dt;
	bool calibrating;
	//functions
public:
	CImu();
	~CImu();
	void Init();
	void Update(uint32_t Period);
	void InitGyro();
	Vector3f GetLinAcceleration();
	bool GetLinAccelerometerData();
	bool GetAccelerometerData();
	bool GetGyroData();
	void SetLinAccelPeakHold( const Vector3f &accel);
	Vector3f GetRawGiro();
	void set_accel_peak_hold( const Vector3f &accel);
	void CalcVibrationAndClippingLin( const Vector3f &Accel, float dt);
	void calc_vibration_and_clipping( const Vector3f &accel, float dt);
	void CheckCalibration();	
	bool GetAccelerationVect(Vector3f & Vect);
	bool OrientationVect(Vector3f& Vect);
	bool AngVelocityVect(Vector3f& Vect);
	bool AccelVect(Vector3f& Vect);
	float GetForwardSpeed();
	bool GetForwardVelocity(float* Vel);
	void Test();
	bool CalibrateGiro();
protected:
private:
	CImu(const CImu& c);
	CImu& operator=(const CImu& c);
    // return true if the sensors are still converging and sampling rates could change significantly
    bool sensors_converging() const { return Core.millis() < 30000; }

}; //CinertialManagement

#endif //__CINERTIALMANAGEMENT_H__

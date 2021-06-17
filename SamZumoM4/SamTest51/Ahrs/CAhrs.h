/* 
* CAhrs.h
*
* Created: 21/08/2020 17:38:48
* Author: philg
*/


#ifndef __CAHRS_H__
#define __CAHRS_H__

#define MAG_CALIBRATION_MASK (3<<0)
#define ACC_CALIBRATION_MASK (3<<2)
#define GIR_CALIBRATION_MASK (3<<4)


typedef struct
{
	uint16_t RollOffset;
	uint16_t PitchOffset;
}AttitudeOffsets_t;

typedef struct
{
	Vector3f AccelRawValues;
	Vector3f GyroRawValues;
	Vector3f MagRawValues;
}AhrsRawValues_t;

typedef struct
{
//	float HeadingDeg;
//	int8_t Roll;
//	int8_t Pitch;
	Vector3f Eular;
	Vector3f RawAccel;
	Vector3f RawGyro;
	Vector3f RawMag;
	uint16_t Temperature;
	uint8_t CalibrationState;
	float Altitude;   // in cm
	Vector3f m_VelocityNED;
	
}AhrsValues_t;

typedef struct
{
	float HeadingDeg;
	int8_t Roll;
	int8_t Pitch;
}AhrsRadianValues_t;

#define AHRS_COMPASS_OK			(1<<0)
#define  AHRS_GPS_OK			(1<<1)
#define AHRS_GOOD			AHRS_NO_COMPASS|AHRS_GPS_OK



class CAhrs
{
//variables
public:
	Vector3f m_OrientationVect;
	Vector3f m_AvgVelocityVect;
	Vector3f m_linearAccelVect;
	AttitudeOffsets_t m_AhrsOffsets;
	AhrsValues_t m_AhrsValues;
	uint8_t m_Status;
    // helper trig variables
    float _cos_roll, _cos_pitch, _cos_yaw;
    float _sin_roll, _sin_pitch, _sin_yaw;

protected:
private:
	bool m_EularVectorGood;
	float m_HeadingVelocity;
	Vector3f AverageLinAccel;
	Vector3f LastLinAccel;
//functions
public:
	CAhrs();
	~CAhrs();
	void Update(uint32_t Period);
	void Init();
	bool UpDateCompass();
	float GetHeadingVelocity();
	bool GetHeadingVelocity(float &Velocity);
	float GetHeadingRad();
	float GetHeading();
	float GetRollD();
	float GetPitchD();
	float GetRollR();
	float GetPitchR();
	int32_t GetHeadingCD();
	void CalibrateLevel();
	void CheckCalibration();
	void SaveCompassCalibration();
	Vector2f GetGroundspeedVector(void);
	bool GetVelocityNED(Vector3f &velocity);
	Vector2f earth_to_body2D(const Vector2f &ef) const;
	Vector2f body_to_earth2D(const Vector2f &bf) const;
	Vector3f body_to_earth(const Vector3f &v) const;
	Vector3f earth_to_body(const Vector3f &v) const;
	const Matrix3f get_rotation_body_to_ned() const;
	void calc_trig(const Matrix3f &rot,float &cr, float &cp, float &cy,float &sr, float &sp, float &sy) const;
	float GetRoll();
	float GetPitch();
	float GetYawR();
	float get_yaw_rate_earth(void) const;
protected:
private:
	CAhrs( const CAhrs &c );
	CAhrs& operator=( const CAhrs &c );
}; //CAhrs
#endif //__CAHRS_H__

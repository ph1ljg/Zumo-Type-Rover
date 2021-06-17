/* 
* CCompass.h
*
* Created: 11/03/2021 18:41:08
* Author: philg
*/


#ifndef __CCOMPASS_H__
#define __CCOMPASS_H__

#define HAL_COMPASS_FILTER_DEFAULT 0

class CCompass :public CCmps12
{
//variables
public:
	uint32_t _last_update_ms = 0;
	uint32_t _last_update_usecs =0;
	float _mean_field_length;			 // mean field length for range filter
	uint32_t _error_count = 0;
	float m_AngularVelocity =0;
	struct mag_state
	{
		bool        Healthy;
		uint32_t    _last_update_ms;         // when we last got data
		uint32_t    _last_update_usec;
		Vector3f accum;                 // accumulated samples
		uint32_t accum_count;
	};


protected:
private:
	Vector3f accum;
	struct mag_state  CompassState;
	uint32_t accum_count;
	float  m_Declination = 0;	    //  The angle to compensate between the true north and magnetic north Range: -3.142 3.142 Units: rad Increment: 0.01
	bool _cal_thread_started;
	bool init_done;
	bool m_InitialDeclinationSet;
    int8_t _filter_range = HAL_COMPASS_FILTER_DEFAULT;	// This sets the range around the average value that new samples must be within to be accepted. This can help reduce the impact of noise on sensors that are on long I2C cables.

//functions
public:
	CCompass();
	~CCompass();
	void init();
	bool UpDate(void);
	bool SaveCalibration();
	bool UpdateCalState();
	float GetHeadingDegrees(void);
	uint16_t GetHeading();
	float GetPitch();
	float GetRoll();
	uint16_t GetTemperature();
	float GetTurnRate();
	float UpdateTurnRate();
	bool IsCalibrating();
	bool GetGyroCal();
	bool GetAccelCal();
	bool GetMagCal();
	bool GetSysCal();
	// return last update time in microseconds
	uint32_t last_update_usecs() const { return CompassState._last_update_usec; }
	uint32_t last_update_ms() const { return CompassState._last_update_ms; }
	uint8_t get_filter_range() const { return uint8_t(_filter_range); }
    bool healthy() const { return CompassState.Healthy; }

protected:
private:
	CCompass( const CCompass &c );
	CCompass& operator=( const CCompass &c );

}; //CCompass

#endif //__CCOMPASS_H__

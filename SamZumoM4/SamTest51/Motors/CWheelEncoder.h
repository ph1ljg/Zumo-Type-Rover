/* 
* CWheelEncoder.h
*
* Created: 29/10/2020 12:18:56
* Author: philg
*/


#ifndef __CWHEELENCODER_H__
#define __CWHEELENCODER_H__

#define  COUNTS_PER_REVOLUTION	150
//#define RADS_PER_CLICK			0.041887902


typedef enum{ RIGHT_WHEEL, LEFT_WHEEL, BOTH_WHEELS} WheelType_t;


typedef enum
{
	MPH,
	RADS_PER_SEC,
	DISTANCE
}WheelsensorData_t;

typedef enum
{
	RIGHT_WHEEL_SENSOR,
	LEFT_WHEEL_SENSOR,
	BOTH_WHEEL_SENSORS
}Wheelsensor_t;


typedef enum
{
	READING_VALID,
	IN_PROGRESS,
	DATA_FAIL
}WheelsensorState_t;



typedef struct
{
	uint32_t	DistanceCount;
	uint32_t	TotalCount;
}WheelSensor_t;






class CWheelEncoder
{
//variables
public:
	WheelSensor_t m_RightWheelSensor;
	WheelSensor_t m_LeftWheelSensor;
	uint32_t m_RightTotalCount;
	uint32_t m_LeftTotalCount;
	float m_RightWheelMeters_Sec;
	float m_LeftWheelMeters_Sec;
	float m_RightWheelRate;
	float m_LeftWheelRate;
	float m_RightDistance;
	float m_LeftDistance;
	uint32_t m_SampleTime;
	uint16_t m_RightDistanceCount;
	uint16_t m_LeftDistanceCount;
	float m_MPH_Left;
	float m_MPH_Right;
protected:
private:
	bool m_NewData = false;
//functions
public:
	CWheelEncoder();
	~CWheelEncoder();
	void Init();
	void UpDate();
	void IncrementRightWheel();
	void IncrementLeftWheel();
	void CopyIrqState();
	void ResetWheelSensor(Wheelsensor_t Wheel);
	float GetDeltaAngle(Wheelsensor_t Sensor) const;
	float GetRps(Wheelsensor_t Sensor) const;
	float GetDistance(Wheelsensor_t Sensor) const;
	float GetMetersSec(Wheelsensor_t Sensor) ;
	float GetMph(Wheelsensor_t Sensor ) ;
	float GetRate(Wheelsensor_t Wheel) const;
	uint32_t GetLastSample_ms(Wheelsensor_t Wheel) const;
	void CalibrateWheelSensors2();
	bool GetWheelState();
	void CalibrateRightWheelSensor();
protected:
private:
	CWheelEncoder( const CWheelEncoder &c );
	CWheelEncoder& operator=( const CWheelEncoder &c );

}; //CWheelEncoder

#endif //__CWHEELENCODER_H__

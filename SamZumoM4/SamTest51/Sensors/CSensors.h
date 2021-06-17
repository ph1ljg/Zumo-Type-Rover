/* 
* CSensors.h
*
* Created: 15/06/2016 12:23:53
* Author: phil
*/


#ifndef __CSENSORS_H__
#define __CSENSORS_H__
#include "stdio.h"




typedef struct
{
	float	Temperature;
	uint16_t RightLidar;
	uint16_t RearLidar;
	uint16_t LeftLidar;
	uint16_t FrontLidar;
	uint16_t FrontLidarStrength;
	uint16_t RearLidarStrength;
	uint16_t RightLidarStrength;
	uint16_t LeftLidarStrength;
	float VoltsBattery_1;               // motor battery voltage  
//	float VoltsBattery_2;               // Controller battery voltage
	uint8_t BatteryRemaining;
	float MahUsed;
	float Current;
} SensorValues_t;



class CSensors
{
//variables
public:
	SensorValues_t m_SensorValues;
	uint8_t m_BatteryCount;
protected:
private:
	 uint32_t m_RwDataValues[4];

//functions
public:
	CSensors();
	~CSensors();
	void Init(void);
	bool GetFrontDistance();
	void UpDate();
	void GetMotorCurrent();
	bool AreAllPathsClear();
	bool GetSensorMpuMotorCurrentValues();
	void GetBatteryVolts();
	void InitLidar();
	void UpdateLidarValues();
	void GetLaserDistance();
protected:
private:
	CSensors( const CSensors &c );
	CSensors& operator=( const CSensors &c );

}; //CSensors

#endif //__CSENSORS_H__

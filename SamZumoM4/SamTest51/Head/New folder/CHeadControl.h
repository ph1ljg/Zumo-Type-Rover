/* 
* CHeadControl.h
*
* Created: 13/07/2020 18:48:42
* Author: philg
*/


#ifndef __CHEADCONTROL_H__
#define __CHEADCONTROL_H__


#define TF_MINI_ADDRESS 0x10
#define FRONT_SECTOR_WIDTH_DEG      10.0f   // width of sectors in degrees

#define NUM_FRONT_SECTORS           9       // number of sectors must be odd no
#define PROXIMITY_BOUNDARY_DIST_MIN 0.6f    // minimum distance for a boundary point.  This ensures the object avoidance code doesn't think we are outside the boundary.
#define PROXIMITY_BOUNDARY_DIST_DEFAULT 100 // if we have no data for a sector, boundary is placed 100m out



#define RADAR_LOWPASS_F_FACTOR 1 //  LPF of 16

#define LEFT_PATH 0
#define RIGHT_PATH 1


//#define SERVO_VERTICAL_HEAD 0
typedef struct __attribute__ ((packed))
{
	uint8_t TriggerDone;
	uint8_t Zero;
	uint8_t DistanceLsb;
	uint8_t DistanceMsb;
	uint8_t StrengthLsb;
	uint8_t StrengthMsb;
	uint8_t Mode;

}LidarRx_t;

typedef enum
{
	HEAD_FAR_RIGHT	= 0,
	HEAD_MID_RIGHT	= 1,
	NEAR_RIGHT		= 2,
	HEAD_CENTER		= 3,
	HEAD_NEAR_LEFT	= 4,
	HEAD_MID_LEFT	= 5,
	HEAD_FAR_LEFT	= 6,
}HeadPosition_t;



typedef struct
{
	uint16_t Distance;
	uint16_t LidarStrength;
	float Angle;
	bool DistanceValid;
	uint8_t SectorNo;
} Sector_t;


typedef struct
{
	uint16_t Distance;
	bool Valid;
} MinimumLiveRange_t;


class CHeadControl
{
//variables
public:
	Sector_t m_Sectors[NUM_FRONT_SECTORS];
	Sector_t m_SectorsSort[NUM_FRONT_SECTORS];
	Sector_t m_FrontSector;
	uint16_t m_PwmValue;
	bool m_FrontScanActive;
	bool  m_IsScanStart;
	uint16_t m_MinimumRange;
	uint8_t m_HeadPosition;
	uint16_t m_Temperature;
	uint16_t m_BestPath;
protected:
private:
	bool m_DataRead;
	uint8_t m_VerticalServoNo;
	uint8_t m_HorizontalServoNo;
//functions
public:
	CHeadControl();
	~CHeadControl();
	uint16_t GetMaxMinRange();
	void Init();
	void Update();
	bool GetSafeCourse(Sector_t &Sector);
	bool GetLidarRange(uint16_t &Distance,uint16_t &Strength,uint16_t &Temperature);
	bool GetLidarRange(uint16_t &Distance);
	void StartFrontScan();
	void ScanFrontPath();
	int8_t GetBestDistance(float& angle_deg, float &distance,uint8_t StartSector,uint8_t EndSector) const;
	int8_t GetClosestObject(float& angle_deg, float &distance,uint8_t StartSector,uint8_t EndSector) const;
	int8_t GetBestCourse() ;
	uint8_t GetBestDirection();
	void SortFrontDistance();
	bool GetObjectAngleAndDistance(uint8_t object_number, float& angle_deg, float &distance) const;
	uint8_t ConverAngleToSector(float angle_degrees) const;
//	void Test();
	void LookForwardPath();
	void LookUp();
	void LookDown();
	void SetVerticalHead(int16_t Degrees,bool Imediate);
	void Test();
protected:
private:
	CHeadControl( const CHeadControl &c );
	CHeadControl& operator=( const CHeadControl &c );

}; //CHeadControl

#endif //__CHEADCONTROL_H__

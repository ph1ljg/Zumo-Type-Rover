/* 
* CHeadControl.cpp
*
* Created: 13/07/2020 18:48:42
* Author: philg
*/
//     Head	
// Right	= 1000
// Left		= 2000
// Up		= 1000
// Down		= 2000
//    Husky
// 00 top left
// 320, 0 top right
// centre = vert 120 horiz 160
// 
#include "Includes.h"
//#include "CTFMini.h"

//CTFMini TFMini;
CTFMiniPlus TFMiniPlus;
CServo VertServo;
CServo HorizServo;

volatile	float Anglet;
volatile	uint8_t Sector;


// default constructor
CHeadControl::CHeadControl()
{

} //CHeadControl

// default destructor
CHeadControl::~CHeadControl()
{
} //~CHeadControl

//extern CHeadControl HeadControl;


void CHeadControl::Init()
{
// 	m_VerticalServoNo = VertServo.Attach(PORTA,3,Config.m_MainConfig.ServoConf[SERVO_HEAD_VERT].min,Config.m_MainConfig.ServoConf[SERVO_HEAD_VERT].max);
// 	m_HorizontalServoNo = HorizServo.Attach(PORTA,6,Config.m_MainConfig.ServoConf[SERVO_HEAD_HORZ].min,Config.m_MainConfig.ServoConf[SERVO_HEAD_HORZ].max);
// 	SetVerticalHead(0,true);
// 	HorizServo.SetDegrees45(0);
// 	m_HeadPosition = HEAD_CENTER;
// 	TFMiniPlus.Init();
// //	Test();
// 	m_FrontSector.Angle= 0;
// 	m_FrontSector.SectorNo = 10;
// 	m_DataRead = true;
}


void CHeadControl::Update()
{
// 	if(m_FrontScanActive)
// 	{
// 			m_BestPath = GetBestCourse();
// 			m_FrontScanActive = false;
// 	}
// 	else
// 	{
// 		GetLidarRange(m_FrontSector.Distance,m_FrontSector.LidarStrength,m_Temperature);
// 		
// 		if(m_FrontSector.Distance <Config.m_MainConfig.SafeFrontDistance)
// 		{
// 			if(m_DataRead)
// 			{
// 				m_DataRead = false;
// 				StartFrontScan();
// 			}
// 		}
// 		else
// 			Config.m_RunningFlags.FRONT_DISTANCE_ALARM = false;
// 	}
}

bool  CHeadControl::GetSafeCourse(Sector_t &Sector)
{
// 	if(m_FrontScanActive)
// 		return(false);
// 	
// 	if(Config.m_RunningFlags.FRONT_DISTANCE_ALARM)
// 	{	
// 		Sector.Angle = m_Sectors[m_BestPath].Angle;
// 		Sector.Distance = m_Sectors[m_BestPath].Distance;
// 		Sector.DistanceValid = m_Sectors[m_BestPath].DistanceValid;
// 		Sector.LidarStrength = m_Sectors[m_BestPath].LidarStrength;
// 		Sector.SectorNo = m_Sectors[m_BestPath].SectorNo;
// 	}
// 	else
// 	{
// 			Sector.Angle = m_FrontSector.Angle;
// 			Sector.Distance = m_FrontSector.Distance;
// 			Sector.DistanceValid = m_FrontSector.DistanceValid;
// 			Sector.LidarStrength = m_FrontSector.LidarStrength;
// 			Sector.SectorNo = m_FrontSector.SectorNo;
// 	}
// 	m_DataRead = true;
// 	return(true);
}





bool CHeadControl::GetLidarRange(uint16_t &Distance)
{
// 	uint16_t Strength;
// 	uint16_t Temperature;
// 	
// 	return(GetLidarRange(Distance,Strength,Temperature));
}

bool CHeadControl::GetLidarRange(uint16_t &Distance,uint16_t &Strength,uint16_t &Temperature)
{
// 	TfminiPlusData_t Data;
// 	
// 	if(TFMiniPlus.GetRange(Data))
// 	{
// 		Strength = Data.Strength;
// 		Distance	= Data.Distance;
// 		Temperature	= Data.Temperature;
// 	}
// 	else
// 	{
// 		Config.m_RunningFlags.HEAD_LIDAR_FAIL = true;
// 		return(false);
// 	}
// 	Config.m_RunningFlags.HEAD_LIDAR_FAIL = false;
// 	return(true);
// }
// 
// void CHeadControl::StartFrontScan()
// {
// 	m_IsScanStart = true;
// 	VertServo.SetDegrees45(5);
// 	m_FrontScanActive = true;
// 	Config.m_RunningFlags.FRONT_DISTANCE_ALARM = true;
// 	TaskManager.EnableTask(TASK_HEAD_UPDATE,false);
// 	TaskManager.EnableTask(TASK_FRONT_SCAN,true);
// 	
}

void CHeadControl::ScanFrontPath()
{
// 	static int16_t Degrees = -45;
// 	uint8_t Sector =0;
// 	uint16_t Distance;
// 	static uint8_t Count =FRONT_SECTOR_WIDTH_DEG;
// 	uint16_t Strength;
// 	uint16_t Temperature;
// 	HorizServo.SetDegrees45(Degrees);
// 	if(m_IsScanStart)
// 	{
// 		m_IsScanStart = false;
// 		return;
// 	}
// 	Count++;
// 	if(Count >=FRONT_SECTOR_WIDTH_DEG)
// 	{
// 			Sector = ConverAngleToSector(Degrees+45);
// 			if(GetLidarRange(Distance,Strength,Temperature))
// 			{
// 				m_Sectors[Sector].Angle = Degrees;
// 				m_Sectors[Sector].Distance = Distance;
// 				m_Sectors[Sector].LidarStrength= Strength;
// 				m_Sectors[Sector].DistanceValid = true;
// 				m_Sectors[Sector].SectorNo = Sector;
// 			}
// 			else
// 				m_Sectors[Sector].DistanceValid = false;
// 			Count = 0;	
// 	}
// 	Degrees++;
// 	
// 	if(Degrees >45)
// 	{
// 				TaskManager.EnableTask(TASK_FRONT_SCAN,false);
				TaskManager.EnableTask(TASK_HEAD_UPDATE,true);
// 				HorizServo.SetDegrees45(0);
// 				Degrees = -45;
// 				Count =0;
// 	}
}

// get distance and angle to furthest  object    returns sector
int8_t CHeadControl::GetBestDistance(float& angle_deg, float &distance,uint8_t StartSector,uint8_t EndSector) const
{
// 	bool sector_found = false;
// 	uint8_t sector = 0;
// 
// 	// check all sectors for shorter distance
// 	for (uint8_t i=StartSector; i<EndSector; i++)
// 	{
// 		if (m_Sectors[i].DistanceValid)
// 		{
// 			if (!sector_found || (m_Sectors[i].Distance > m_Sectors[sector].Distance))
// 			{
// 				sector = i;
// 				sector_found = true;
// 			}
// 		}
// 	}
// 
// 	if (sector_found)
// 	{
// 		angle_deg = m_Sectors[sector].Angle;
// 		distance = m_Sectors[sector].Distance;
// 		return( sector);
// 	}
// 	return(-1);
}



// get distance and angle to closest object    returns true on success, false if no valid readings
int8_t CHeadControl::GetClosestObject(float& angle_deg, float &distance,uint8_t StartSector,uint8_t EndSector) const
{
// 	bool sector_found = false;
// 	uint8_t sector = 0;
// 
// 	// check all sectors for shorter distance
// 	for (uint8_t i=StartSector; i<EndSector; i++) 
// 	{
// 		if (m_Sectors[i].DistanceValid) 
// 		{
// 			if (!sector_found || (m_Sectors[i].Distance < m_Sectors[sector].Distance)) 
// 			{
// 				sector = i;
// 				sector_found = true;
// 			}
// 		}
// 	}
// 
// 	if (sector_found) 
// 	{
// 		angle_deg = m_Sectors[sector].Angle;
// 		distance = m_Sectors[sector].Distance;
// 		return( sector);
// 	}
// 	return(-1);
}


// get distance and angle to closest object    returns true on success, false if no valid readings
int8_t CHeadControl::GetBestCourse() 
{
// 	float Angle;
// 	float Distance;
// 	uint8_t Path;
// 	//GetClosestObject(Angle, Dist);
// 
// 	Path = GetBestDirection();
// 	if(Path == RIGHT_PATH)
// 	{
// 		return(GetBestDistance(Angle, Distance,4,9)); 
// 	}
// 	return(GetBestDistance(Angle, Distance,0,6));
}

uint8_t CHeadControl::GetBestDirection()
{
// 	uint16_t WeightRight = 1;
// 	uint16_t WeightLeft = 1;
// 	uint8_t i;
// 	uint8_t lower = (NUM_FRONT_SECTORS/2);
// 	uint8_t upper = (NUM_FRONT_SECTORS-lower);
// 	for(i=0;i<lower+1;i++)
// 	{
// 		if(m_Sectors[i].Distance > Config.m_MainConfig.SafeFrontDistance)
// 			WeightLeft = (1<<i);
// 	}
// 
// 	for(i=NUM_FRONT_SECTORS-1;i>upper;i--)
// 	{
// 		if(m_Sectors[i].Distance > Config.m_MainConfig.SafeFrontDistance)
// 			WeightRight <<= 1;
// 	}
// 
// 	if(WeightRight >WeightLeft)
// 		return(RIGHT_PATH);
// 	
// 	return(LEFT_PATH);	
}


	
void CHeadControl::SortFrontDistance()
{	
// 	bool sector_found = false;
// 	uint8_t sector = 0;
// 	uint16_t  Distance1,Distance2;
// 	uint8_t i,x;
// 	uint16_t Distance;
// 	uint16_t LidarStrength;
// 	float Angle;
// 	bool DistanceValid;
// 	uint8_t SectorNo;
// 
// 
// 	for(i=0;i<NUM_FRONT_SECTORS;i++)
// 	{
// 		for(x=0;x<NUM_FRONT_SECTORS-i;x++)
// 		{
// 			Distance1 = m_Sectors[x].Distance;
// 			Distance2 = m_Sectors[x+1].Distance;
// 			if(Distance1< Distance2)
// 			{
// 				// swap
// 
// 				Distance			= m_Sectors[x+1].Distance;
// 				LidarStrength	= m_Sectors[x+1].LidarStrength;
// 				Angle				= m_Sectors[x+1].Angle;
// 				DistanceValid		= m_Sectors[x+1].DistanceValid;
// 				SectorNo			= m_Sectors[x+1].SectorNo;
// 
// 				m_Sectors[x+1].Distance			= m_Sectors[x].Distance;
// 				m_Sectors[x+1].LidarStrength	= m_Sectors[x].LidarStrength;
// 				m_Sectors[x+1].Angle				= m_Sectors[x].Angle;
// 				m_Sectors[x+1].DistanceValid	= m_Sectors[x].DistanceValid;
// 				m_Sectors[x+1].SectorNo			= m_Sectors[x].SectorNo;
// 				
// 
// 				m_Sectors[x].Distance = Distance;
// 				m_Sectors[x].LidarStrength = LidarStrength;
// 				m_Sectors[x].Angle = Angle;
// 				m_Sectors[x].DistanceValid =DistanceValid ;
// 				m_Sectors[x].SectorNo = SectorNo;
// 			}
// 		}
// 	}
}



// Get an object's angle and distance, used for non-GPS avoidance returns false if no angle or distance could be returned for some reason
bool CHeadControl::GetObjectAngleAndDistance(uint8_t object_number, float& angle_deg, float &distance) const
{
// 	if (object_number < NUM_FRONT_SECTORS ) 
// 	{
// 		angle_deg = m_Sectors[object_number].Angle;
// 		distance = m_Sectors[object_number].Distance;
// 		return true;
// 	}
// 	return false;
}


// Find which sector a given angle falls into 0- 360
uint8_t CHeadControl::ConverAngleToSector(float angle_degrees) const
{
// 	float RetVal = wrap_360(angle_degrees + (FRONT_SECTOR_WIDTH_DEG * 0.5f)) / FRONT_SECTOR_WIDTH_DEG;
// 
// 	return((uint8_t)RetVal); 
}



void CHeadControl::LookUp()
{
//	SetVerticalHead(4000,false);
}

void CHeadControl::LookDown()
{
//	SetVerticalHead(-3000,false);
}

//Imediate 
void CHeadControl::SetVerticalHead(int16_t Degrees,bool Imediate)
{
// 	CMyMath Math;
// 	static int16_t OldValue = 0;
// 	if(Imediate)
// 	{
// 			VertServo.SetDegrees45(Degrees);
// 			return;
// 	}
// 	
// 	if(Degrees >OldValue)
// 	{
// 		while(Degrees > OldValue)
// 		{
// 			OldValue++;
// 			VertServo.SetDegrees45(OldValue);
// 			Core.delay(2);
// 		}
// 	}
// 	
// 	if(Degrees < OldValue)
// 	{
// 		while(Degrees < OldValue)
// 		{
// 			OldValue--;
// 			VertServo.SetDegrees45(OldValue);
// 			Core.delay(2);
// 		}
// 	}
// 
// 	Core.delay(10);
}




void CHeadControl::Test()
{
// 	float CourseAngle;
// uint16_t Distance;
// while(1)
// 	{
// 		if(m_FrontScanActive)
// 		{
// 			CourseAngle = (float)GetBestCourse();
// 			m_FrontScanActive = false;
// 		}
// 		else
// 		{
// 			GetLidarRange(Distance);
// 			if(Distance <Config.m_MainConfig.SafeFrontDistance)
// 			StartFrontScan();
// 		}
// 		Core.delay(500);
// 		
// 	}
}





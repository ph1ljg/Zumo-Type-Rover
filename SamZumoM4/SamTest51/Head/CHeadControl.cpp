/* 
* CHeadControl.cpp
*
* Created: 02/04/2021 11:16:54                ORIGINAL
* Author: philg
*/

#include "Includes.h"
#include "CHeadControl.h"
#include "CToshibaLedDriver.h"
CTFMiniPlus TFMiniPlus;
CServo VertServo;
CServo HorizServo;

volatile	float Anglet;
volatile	uint8_t Sector;
//extern CToshibaLedDriver ToshibaLedDriver;
extern CLedDriver LedDriver;
// default constructor
CHeadControl::CHeadControl()
{

} //CHeadControl

// default destructor
CHeadControl::~CHeadControl()
{
	m_ScanInhibit = false;
} //~CHeadControl

//extern CHeadControl HeadControl;


void CHeadControl::Init()
{
	m_VerticalServoNo = VertServo.Attach(PORTA,3,Config.m_MainConfig.ServoConf[SERVO_HEAD_VERT].min,Config.m_MainConfig.ServoConf[SERVO_HEAD_VERT].max);
	m_HorizontalServoNo = HorizServo.Attach(PORTA,6,Config.m_MainConfig.ServoConf[SERVO_HEAD_HORZ].min,Config.m_MainConfig.ServoConf[SERVO_HEAD_HORZ].max);
	SetVerticalHead(0,true);
	HorizServo.SetDegrees45(0);
	m_HeadPosition = HEAD_CENTER;
	TFMiniPlus.Init();
//	Test();
	m_BestCourse.Angle= 0;
	m_BestCourse.SectorNo = 10;
	m_DataRead = true;
	m_FrontDistance.Angle = 0;
}

// from sequencer
void CHeadControl::Update()
{
	if(m_FrontScanActive)
	{
			m_BestPath = GetBestCourse();
			m_FrontScanActive = false;
	}
	else
	{
		GetLidarRange(m_FrontDistance.Distance,m_FrontDistance.LidarStrength,m_Temperature);
//Test();
		if(m_FrontDistance.Distance <Config.m_MainConfig.SafeFrontDistance)
		{
			CheckForScanTimeout();
			m_FrontDistance.DistanceValid = false;
			if(!m_ScanInhibit)
			{
				m_ScanInhibit = true;
				StartFrontScan(); //==============================================================================
			}
		}
		else
		{
			Config.m_RunningFlags.FRONT_DISTANCE_ALARM = false;
			m_FrontDistance.DistanceValid = true;
			m_ScanInhibit = false;
		}
	}
}


void CHeadControl::Test3()
{
	
	while(1)
	{
		HorizServo.SetDegrees45(45);
		Core.delay(5000);
		HorizServo.SetDegrees45(0);
		Core.delay(5000);
		HorizServo.SetDegrees45(-45);
		Core.delay(5000);
		
	
	}
}


bool  CHeadControl::GetSafeCourse(Sector_t &Sector)
{
//	m_DataRead = true;
	if(m_FrontDistance.DistanceValid )
	{
		Sector.Angle = m_FrontDistance.Angle;
		Sector.Distance = m_FrontDistance.Distance;
		Sector.DistanceValid = m_FrontDistance.DistanceValid;
		Sector.LidarStrength = m_FrontDistance.LidarStrength;
		return(true);
	}
	Sector.Angle = m_Sectors[m_BestPath].Angle;
	Sector.Distance = m_Sectors[m_BestPath].Distance;
	Sector.DistanceValid = m_Sectors[m_BestPath].DistanceValid;
	Sector.LidarStrength = m_Sectors[m_BestPath].LidarStrength;
	Sector.SectorNo = m_Sectors[m_BestPath].SectorNo;
	return(false);
}

uint16_t CHeadControl::GetFrontDistance()
{
	return(m_BestCourse.Distance);
}


void CHeadControl::CheckForScanTimeout()
{
	static uint32_t LastTime =0;
	
	if(!m_ScanInhibit)
	{
		LastTime = Core.millis();
		return;
	}
	
	if((Core.millis()- LastTime) > 5000 )
	{
		m_ScanInhibit = false;
		LastTime = Core.millis();
	}
}


bool CHeadControl::GetLidarRange(uint16_t &Distance)
{
	uint16_t Strength;
	uint16_t Temperature;

	return(GetLidarRange(Distance,Strength,Temperature));
}

bool CHeadControl::GetLidarRange(uint16_t &Distance,uint16_t &Strength,uint16_t &Temperature)
{
	TfminiPlusData_t Data;

	if(TFMiniPlus.GetRange(Data))
	{
		Strength = Data.Strength;
		Distance	= Data.Distance;
		Temperature	= Data.Temperature;
	}
	else
	{
		Config.m_RunningFlags.HEAD_LIDAR_FAIL = true;
		return(false);
	}
	Config.m_RunningFlags.HEAD_LIDAR_FAIL = false;
	return(true);
	}

	void CHeadControl::StartFrontScan()
	{
	m_IsScanStart = true;
	VertServo.SetDegrees45(5);
	m_FrontScanActive = true;
	Config.m_RunningFlags.FRONT_DISTANCE_ALARM = true;
	TaskManager.EnableTask(TASK_HEAD_UPDATE,false);
	TaskManager.EnableTask(TASK_FRONT_SCAN,true);

}

void CHeadControl::ScanFrontPath()
{
	static int16_t Degrees		=-45;
	uint8_t Sector				= 0;
	static uint8_t DelayCount	= 0;
	uint16_t Distance           = 0;
	static uint8_t Count		= 0;
	uint16_t Strength;
	uint16_t Temperature;
	HorizServo.SetDegrees45(Degrees);
	static bool ScanEnd = false;
	
	if(m_IsScanStart) 
	{
		//
		DelayCount++;
		if (DelayCount >25)
		{	
			m_IsScanStart = false;
			DelayCount=0;
		}
		return;
	}
	if(ScanEnd)
	{
		DelayCount++;
		if(DelayCount< 10)   // Allow head to return to center
			 return;
		TaskManager.EnableTask(TASK_FRONT_SCAN,false);
		TaskManager.EnableTask(TASK_HEAD_UPDATE,true);
		Degrees = -45;
		DelayCount =0;
		ScanEnd = false;
		Count =0;
		return;	 
	}
	
	Count++;
	if(Count >=FRONT_SECTOR_WIDTH_DEG || Degrees == -40)
	{
			Sector = ConverAngleToSector(Degrees+45);
			
			if(GetLidarRange(Distance,Strength,Temperature))
			{

				m_Sectors[Sector].Angle = Degrees;
				m_Sectors[Sector].Distance = Distance/10;      // PSB I think that the units are wrong?
				m_Sectors[Sector].LidarStrength= Strength;
				m_Sectors[Sector].DistanceValid = true;
				m_Sectors[Sector].SectorNo = Sector;
			}
			else
				m_Sectors[Sector].DistanceValid = false;
			Count = 0;
	}

	if(++Degrees > 45)
	{
		ScanEnd = true;
		Degrees = 0;
	//	HorizServo.SetDegrees45(0);

	}
}

 // get distance and angle to furthest  object    returns sector
int8_t CHeadControl::GetBestDistance(float& angle_deg, uint16_t  &distance,uint8_t StartSector,uint8_t EndSector) const
{
	bool sector_found = false;
	uint8_t sector = 0;

	// check all sectors for shorter distance
	for (uint8_t i=StartSector; i<EndSector; i++)
	{
		if (m_Sectors[i].DistanceValid)
		{
			if (!sector_found || (m_Sectors[i].Distance > m_Sectors[sector].Distance))
			{
				sector = i;
				sector_found = true;
			}
		}
	}

	if (sector_found)
	{
		angle_deg = m_Sectors[sector].Angle;
		distance = m_Sectors[sector].Distance;
		return( sector);
	}
	return(-1);
}



// get distance and angle to closest object    returns true on success, false if no valid readings
int8_t CHeadControl::GetClosestObject(float& angle_deg, uint16_t &distance,uint8_t StartSector,uint8_t EndSector) const
{
	bool sector_found = false;
	uint8_t sector = 0;

	// check all sectors for shorter distance
	for (uint8_t i=StartSector; i<EndSector; i++)
	{
		if (m_Sectors[i].DistanceValid)
		{
			if (!sector_found || (m_Sectors[i].Distance < m_Sectors[sector].Distance))
			{
				sector = i;
				sector_found = true;
			}
		}
	}

	if (sector_found)
	{
		angle_deg = m_Sectors[sector].Angle;
		distance = m_Sectors[sector].Distance;
		return( sector);
	}
	return(-1);
}


// get distance and angle to closest object    returns true on success, false if no valid readings
int8_t CHeadControl::GetBestCourse()
{
	uint8_t Path;
	//GetClosestObject(Angle, Dist);

	Path = GetBestDirection();
	if(Path == RIGHT_PATH)
		return(GetBestDistance(m_BestCourse.Angle, m_BestCourse.Distance,4,9));

	return(GetBestDistance(m_BestCourse.Angle, m_BestCourse.Distance,0,6));
}

uint8_t CHeadControl::GetBestDirection()
{
	uint16_t WeightRight = 1;
	uint16_t WeightLeft = 1;
	uint8_t i;
	uint8_t lower = (NUM_FRONT_SECTORS/2);
	uint8_t upper = (NUM_FRONT_SECTORS-lower);
	for(i=0;i<lower+1;i++)
	{
		if(m_Sectors[i].Distance > Config.m_MainConfig.SafeFrontDistance)
			WeightLeft = (1<<i);
	}

	for(i=NUM_FRONT_SECTORS-1;i>upper;i--)
	{
		if(m_Sectors[i].Distance > Config.m_MainConfig.SafeFrontDistance)
			WeightRight <<= 1;
	}

	if(WeightRight >WeightLeft)
		return(RIGHT_PATH);

	return(LEFT_PATH);
}



void CHeadControl::SortFrontDistance()
{
	uint16_t  Distance1,Distance2;
	uint8_t i,x;
	uint16_t Distance;
	uint16_t LidarStrength;
	float Angle;
	bool DistanceValid;
	uint8_t SectorNo;

	for(i=0;i<NUM_FRONT_SECTORS;i++)	{		for(x=0;x<NUM_FRONT_SECTORS-i;x++)		{			Distance1 = m_Sectors[x].Distance;			Distance2 = m_Sectors[x+1].Distance;			if(Distance1< Distance2)			{				// swap				Distance			= m_Sectors[x+1].Distance;				LidarStrength	= m_Sectors[x+1].LidarStrength;				Angle				= m_Sectors[x+1].Angle;				DistanceValid		= m_Sectors[x+1].DistanceValid;				SectorNo			= m_Sectors[x+1].SectorNo;				m_Sectors[x+1].Distance			= m_Sectors[x].Distance;				m_Sectors[x+1].LidarStrength	= m_Sectors[x].LidarStrength;				m_Sectors[x+1].Angle				= m_Sectors[x].Angle;				m_Sectors[x+1].DistanceValid	= m_Sectors[x].DistanceValid;				m_Sectors[x+1].SectorNo			= m_Sectors[x].SectorNo;				m_Sectors[x].Distance = Distance;				m_Sectors[x].LidarStrength = LidarStrength;				m_Sectors[x].Angle = Angle;				m_Sectors[x].DistanceValid =DistanceValid ;				m_Sectors[x].SectorNo = SectorNo;			}		}	}}



// Get an object's angle and distance, used for non-GPS avoidance returns false if no angle or distance could be returned for some reason
bool CHeadControl::GetObjectAngleAndDistance(uint8_t object_number, float& angle_deg, float &distance) const
{
	if (object_number < NUM_FRONT_SECTORS )
	{
		angle_deg = m_Sectors[object_number].Angle;
		distance = m_Sectors[object_number].Distance;
		return true;
	}
	return false;
}


// Find which sector a given angle falls into 0- 360
uint8_t CHeadControl::ConverAngleToSector(float angle_degrees) const
{
	float RetVal = wrap_360(angle_degrees + (FRONT_SECTOR_WIDTH_DEG * 0.5f)) / FRONT_SECTOR_WIDTH_DEG;

	return((uint8_t)RetVal);
}



void CHeadControl::LookUp()
{
	SetVerticalHead(4000,false);
}

void CHeadControl::LookDown()
{
	SetVerticalHead(-3000,false);
}

//Imediate
void CHeadControl::SetVerticalHead(int16_t Degrees,bool Imediate)
{
	CMyMath Math;
	static int16_t OldValue = 0;
	if(Imediate)
	{
			VertServo.SetDegrees45(Degrees);
			return;
	}

	if(Degrees >OldValue)
	{
		while(Degrees > OldValue)
		{
			OldValue++;
			VertServo.SetDegrees45(OldValue);
			Core.delay(2);
		}
	}

	if(Degrees < OldValue)
	{
		while(Degrees < OldValue)
		{
			OldValue--;
			VertServo.SetDegrees45(OldValue);
			Core.delay(2);
		}
	}

	Core.delay(10);
}




void CHeadControl::Test2()
{
	uint16_t Distance;
	uint16_t Strength;
	uint16_t Temperature;
	while(1)
	{
		GetLidarRange(Distance,Strength,Temperature);
		Core.delay(1000);
	}

}

void CHeadControl::Test()
{
	uint16_t Distance;
	uint16_t LidarStrength;
	uint16_t Temperature;
	
	GetLidarRange(Distance,LidarStrength,Temperature);

	m_Sectors[0].Distance = 50;
	m_Sectors[1].Distance = 100;
	m_Sectors[2].Distance = 150;
	m_Sectors[3].Distance = 200;
	m_Sectors[4].Distance = Distance;
	m_Sectors[5].Distance = 200;
	m_Sectors[6].Distance = 150;
	m_Sectors[7].Distance = 100;
	m_Sectors[8].Distance = 50;
}



/*
void CHeadControl::ScanFrontPath()
{
	static int16_t Degrees = -45;
	static uint8_t DelayCount = 0;
	static bool ScanEnd = false;
	uint8_t Sector =0;
	uint16_t Distance;
	static uint8_t Count =0;
	uint16_t Strength;
	uint16_t Temperature;
	static uint16_t ScanDelay = 0;
	HorizServo.SetDegrees45(Degrees);
	//	Core.delay(20);
	if(m_IsScanStart  || ScanEnd)
	{
		if(m_IsScanStart)
		{
			if( ++DelayCount > 25)										// start delay
				m_IsScanStart = false;
		}
		else
		{
			if(++DelayCount >10)										// End Delay
			{
				Degrees = -45;												// reset param for next count
				Count =0;
				ScanEnd = false;
				DelayCount =0;
				TaskManager.EnableTask(TASK_FRONT_SCAN,false);
				TaskManager.EnableTask(TASK_HEAD_UPDATE,true);
			}
		}
		return;
	}
	else
	{
		if(++ScanDelay <5)										// scan delay
		return;
	}

	Count++;
	if(Count >=FRONT_SECTOR_WIDTH_DEG || Degrees == -45)
	{
		Sector = ConverAngleToSector(Degrees+45);
		GetLidarRange(Distance,Strength,Temperature);
		if(GetLidarRange(Distance,Strength,Temperature))
		{
			m_Sectors[Sector].Angle = Degrees;
			m_Sectors[Sector].Distance = Distance;
			m_Sectors[Sector].LidarStrength= Strength;
			m_Sectors[Sector].DistanceValid = true;
			m_Sectors[Sector].SectorNo = Sector;
		}
		else
		m_Sectors[Sector].DistanceValid = false;
		Count = 0;
		ScanDelay = 0;
	}

	if(Degrees++ >45)
	{
		Degrees = 0;
		HorizServo.SetDegrees45(0);
		DelayCount = 0;
		ScanEnd = true;
	}
}


*/
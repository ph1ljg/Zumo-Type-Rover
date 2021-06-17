/* 
* CObsticalAvoidance.h
*
* Created: 04/06/2019 19:00:57
* Author: phil
*/


#ifndef __COBSTICALAVOIDANCE_H__
#define __COBSTICALAVOIDANCE_H__
#include "CHeadControl.h"
#include "CFilters.h"
#define AC_AVOID_MIN_BACKUP_BREACH_DIST     10.0f
#define AC_AVOID_ACCEL_TIMEOUT_MS           200     // stored velocity used to calculate acceleration will be reset if avoidance is active after this many ms
#define AC_AVOID_ACCEL_CMSS_MAX         100.0f  // maximum acceleration/deceleration in cm/s/s used to avoid hitting fence

typedef enum
{
	RANGE_OK,
	RANGE_DIST_FAIL,
	RANGE_INVALID,
	RANGER_SENSOR_FAIL
}eRangeResult_t;


#define RADAR_ADDRESS 0x70

#define SAFE_LIDAR_DISTANCE		150

typedef enum
{
	SENSOR_LIDAR,
	SENSOR_SONAR,
	SENSOR_BOTH
}RangerSensor_t;


typedef enum
{
	FRONT_RANGE,
	FRONT_SCAN
}RangerDataset_t;

typedef enum
{
	SET_OVERIDE,
	SETPOSITION,
	CLEAR_OVERIDE
}SetRadarPos_t;


typedef enum
{
	FAR_RIGHT_PATH	= 0,
	MID_RIGHT_PATH	= 1,
	NEAR_RIGHT_PATH = 2,
	CENTER_PATH		= 3,
	NEAR_LEFT_PATH  = 4,
	MID_LEFT_PATH	= 5,
	FAR_LEFT_PATH	= 6,
	REVERSE_PATH	= 7,
	NO_COURSE			= 8
}ForwardrRangePosition_t;




class CAvoidance
{
//variables
public:
	uint8_t m_RadarPosition;
	uint16_t m_RearDistance;
	uint32_t m_ObstacleFailTime = 0;
	float _accel_max;
	float _margin;
	bool m_AvoidActive;
	Sector_t Sector;
 protected:

private:
    Vector3f _prev_avoid_vel;       // copy of avoidance adjusted velocity
    uint32_t _last_limit_time =0;      // the last time a limit was active
	//Sector_t m_SafePath;
//functions
public:
	CAvoidance();
	~CAvoidance();
	void Init();
	void Update();
	bool GetCourse(float &Steer);
	//void AdjustSpeed(float kP, float accel, float heading, float &speed, float dt);
	void AdjustSpeed( float &Speed, float dt);
	void AdjustVelocity(Vector3f &desired_vel_cms, bool &backing_up, float kP, float accel_cmss, float dt);
	void AdjustVelocityProximity(float kP, float accel_cmss, Vector3f &desired_vel_cms, Vector3f &backup_vel, float dt);
	void limit_accel(const Vector3f &original_vel, Vector3f &modified_vel, float dt);
	float get_stopping_distance(float kP, float accel_cmss, float speed_cms) const;
	void calc_backup_velocity_3D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cms, Vector3f limit_direction, float kp_z, float accel_cmss_z, float back_distance_z, float& min_z_vel, float& max_z_vel, float dt);
	void calc_backup_velocity_2D(float kP, float accel_cmss, Vector2f &quad1_back_vel_cms, Vector2f &quad2_back_vel_cms, Vector2f &quad3_back_vel_cms, Vector2f &quad4_back_vel_cms, float back_distance_cm, Vector2f limit_direction, float dt);
	void find_max_quadrant_velocity_3D(Vector3f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel, float &max_z_vel, float &min_z_vel);
	void find_max_quadrant_velocity(Vector2f &desired_vel, Vector2f &quad1_vel, Vector2f &quad2_vel, Vector2f &quad3_vel, Vector2f &quad4_vel);
	float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) ;
//	float get_max_speed(float kP, float accel_cmss, float distance_cm, float dt) const;
	void limit_velocity_3D(float kP, float accel_cmss, Vector3f &desired_vel_cms, const Vector3f& obstacle_vector, float margin_cm, float kP_z, float accel_cmss_z, float dt);
	void limit_velocity_2D(float kP, float accel_cmss, Vector2f &desired_vel_cms, const Vector2f& limit_direction, float limit_distance_cm, float dt);
	float sqrt_controller(float error, float p, float second_ord_lim, float dt);
	void CheckFrontRanger();
	void CheckRearRanger();
	bool SailSafeCourse();
	bool HandleNavFrontObstruction();
	void CheckRCFrontObstruction();
	bool GetSpinRoom();
	bool SetSafeCourse();
	bool HandleRearObstruction();
	void TestSpin();
protected:
private:
	CAvoidance( const CAvoidance &c );
	CAvoidance& operator=( const CAvoidance &c );
	void RadarposTest();
	CMovingAverage DistanceAverage;
}; //CObsticalAvoidance
#endif //__COBSTICALAVOIDANCE_H__

/* 
* CProximity.cpp
*
* Created: 14/03/2021 16:59:16
* Author: philg
*/

#include "Includes.h"

// default constructor
CProximity::CProximity()
{
} //CProximity

// default destructor
CProximity::~CProximity()
{
} //~CProximity


uint8_t CProximity::get_obstacle_count()
{
	return(0);
}


uint8_t CProximity::GetObstacle(uint8_t ObstacleNum, Vector3f& VecToObstacle)
{
	
}

float CProximity::distance_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const
{
	
}

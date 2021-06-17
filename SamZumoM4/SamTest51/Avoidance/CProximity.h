/* 
* CProximity.h
*
* Created: 14/03/2021 16:59:16
* Author: philg
*/


#ifndef __CPROXIMITY_H__
#define __CPROXIMITY_H__


class CProximity
{
//variables
public:
protected:
private:

//functions
public:
	CProximity();
	~CProximity();
	uint8_t get_obstacle_count();
	uint8_t GetObstacle(uint8_t ObstacleNum, Vector3f& VecToObstacle);
	float distance_to_obstacle(uint8_t obstacle_num, const Vector3f& seg_start, const Vector3f& seg_end, Vector3f& closest_point) const;
protected:
private:
	CProximity( const CProximity &c );
	CProximity& operator=( const CProximity &c );

}; //CProximity

#endif //__CPROXIMITY_H__

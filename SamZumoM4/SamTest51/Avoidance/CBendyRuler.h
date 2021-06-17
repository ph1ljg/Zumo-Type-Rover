/* 
* CBendyRuler.h
*
* Created: 28/10/2020 17:56:27
* Author: philg
*/


#ifndef __CBENDYRULER_H__
#define __CBENDYRULER_H__

const int16_t OA_BR_BEARING_INC_XY = 5;            // check every 5 degrees around vehicle
const float OA_BR_LOOKAHEAD_STEP2_MIN = 2.0f;   // step2 checks at least this many meters past step1's location
const float OA_BENDYRULER_LOOKAHEAD_PAST_DEST = 2.0f;   // lookahead length will be at least this many meters past the destination
const float OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO = 1.0f; // step2's lookahead length as a ratio of step1's lookahead length
const float OA_BENDYRULER_LOW_SPEED_SQUARED = (0.2f * 0.2f);    // when ground course is below this speed squared, vehicle's heading will be used

class CBendyRuler
{
//variables
public:
protected:
private:
    float _current_lookahead;       // distance (in meters) ahead of the vehicle we are looking for obstacles
    float _lookahead;            // object avoidance will look this many meters ahead of vehicle
    float _margin_max;              // object avoidance will ignore objects more than this many meters from vehicle
    Location_t _destination_prev;     // previous destination, to check if there has been a change in destination
    float _bearing_prev;            // stored bearing in degrees
    float _bendy_ratio;          // object avoidance will avoid major directional change if change in margin ratio is less than this param
    int16_t _bendy_angle;          // object avoidance will try avoiding change in direction over this much angle

//functions
public:
	CBendyRuler();
	~CBendyRuler();
	bool update(const Location_t& current_loc, const Location_t& destination, const Vector2f &ground_speed_vec, Location_t &origin_new, Location_t &destination_new, bool proximity_only);
	bool search_xy_path(const Location_t &current_loc, const Location_t& destination, float ground_course_deg, Location_t &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only);
	bool resist_bearing_change(const Location_t &destination, const Location_t &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location_t &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const;
	float calc_avoidance_margin(const Location_t &start, const Location_t &end, bool proximity_only) const;
	bool calc_margin_from_object_database(const Location_t &start, const Location_t &end, float &margin) const;
	bool calc_margin_from_circular_fence(const Location_t &start, const Location_t &end, float &margin) const;
    void set_config(float margin_max) { _margin_max = MAX(margin_max, 0.0f); }			// send configuration info stored in front end parameters

protected:
private:
	CBendyRuler( const CBendyRuler &c );
	CBendyRuler& operator=( const CBendyRuler &c );

}; //CBendyRuler

#endif //__CBENDYRULER_H__

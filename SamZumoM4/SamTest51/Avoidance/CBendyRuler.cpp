/* 
* CBendyRuler.cpp
*
* Created: 28/10/2020 17:56:27
* Author: philg
*/


#include "Includes.h"
// default constructor
CBendyRuler::CBendyRuler()
{
  _bearing_prev = FLT_MAX;

} //CBendyRuler

// default destructor
CBendyRuler::~CBendyRuler()
{
} //~CBendyRuler




// run background task to find best path and update avoidance_results
// returns true and updates origin_new and destination_new if a best path has been found
bool CBendyRuler::update(const Location_t& current_loc, const Location_t& destination, const Vector2f &ground_speed_vec, Location_t &origin_new, Location_t &destination_new, bool proximity_only)
{
	CMyMath Math;
	origin_new = current_loc;																				// bendy ruler always sets origin to current_loc
	const float bearing_to_dest = NavigationFunctions.get_bearing_to(current_loc,destination) * 0.01f;		// calculate bearing and distance to final destination
	const float distance_to_dest = NavigationFunctions.get_distance(current_loc,destination);

	_lookahead = MAX(_lookahead,1.0f);																		// make sure user has set a meaningful value for _lookahead

	_current_lookahead = Math.constrain_float(_current_lookahead, _lookahead * 0.5f, _lookahead);				// lookahead distance is adjusted dynamically based on avoidance results

	// calculate lookahead dist and time for step1.  distance can be slightly longer than	// the distance to the destination to allow room to dodge after reaching the destination
	const float lookahead_step1_dist = MIN(_current_lookahead, distance_to_dest + OA_BENDYRULER_LOOKAHEAD_PAST_DEST);
	
	const float lookahead_step2_dist = _current_lookahead * OA_BENDYRULER_LOOKAHEAD_STEP2_RATIO;			// calculate lookahead dist for step2

	
	float ground_course_deg;																				// get ground course
	if (ground_speed_vec.length_squared() < OA_BENDYRULER_LOW_SPEED_SQUARED)
		ground_course_deg =Ahrs.m_AhrsValues.Eular.z;													// get ground course
	else
		ground_course_deg = degrees(ground_speed_vec.angle());
	
	bool ret;
	ret = search_xy_path(current_loc, destination, ground_course_deg, destination_new, lookahead_step1_dist, lookahead_step2_dist, bearing_to_dest, distance_to_dest, proximity_only);
	
	return ret;
}



// Search for path in the horizontal directions
bool CBendyRuler::search_xy_path(const Location_t &current_loc, const Location_t& destination, float ground_course_deg, Location_t &destination_new, float lookahead_step1_dist, float lookahead_step2_dist, float bearing_to_dest, float distance_to_dest, bool proximity_only)
{
	CMyMath Math;
	// check OA_BEARING_INC definition allows checking in all directions
	static_assert(360 % OA_BR_BEARING_INC_XY == 0, "check 360 is a multiple of OA_BEARING_INC");

	// search in OA_BR_BEARING_INC_XY degree increments around the vehicle alternating left and right. For each direction check if vehicle would avoid all obstacles
	float best_bearing = bearing_to_dest;
	bool have_best_bearing = false;
	float best_margin = -FLT_MAX;
	float best_margin_bearing = best_bearing;

	for (uint8_t i = 0; i <= (170 / OA_BR_BEARING_INC_XY); i++)
	{
		for (uint8_t bdir = 0; bdir <= 1; bdir++)
		{
			// skip duplicate check of bearing straight towards destination
			if ((i==0) && (bdir > 0))
			{
				continue;
			}
			// bearing that we are probing
			const float bearing_delta = i * OA_BR_BEARING_INC_XY * (bdir == 0 ? -1.0f : 1.0f);
			const float bearing_test = wrap_180(bearing_to_dest + bearing_delta);

			// ToDo: add effective groundspeed calculations speed
			// ToDo: add prediction of vehicle's position change as part of turn to desired heading

			// test location is projected from current location at test bearing
			Location_t test_loc;
			test_loc.nLatitude = current_loc.nLatitude;
			test_loc.nLongitude = current_loc.nLongitude;
			NavigationFunctions.offset_bearing(test_loc,bearing_test, lookahead_step1_dist);

			// calculate margin from obstacles for this scenario
			float margin = calc_avoidance_margin(current_loc, test_loc, proximity_only);
			if (margin > best_margin)
			{
				best_margin_bearing = bearing_test;
				best_margin = margin;
			}
			if (margin > _margin_max)
			{
				// this bearing avoids obstacles out to the lookahead_step1_dist
				// now check in there is a clear path in three directions towards the destination
				if (!have_best_bearing)
				{
					best_bearing = bearing_test;
					have_best_bearing = true;
				}
				else if (fabsf(wrap_180(ground_course_deg - bearing_test)) <fabsf(wrap_180(ground_course_deg - best_bearing)))
				{
					// replace bearing with one that is closer to our current ground course
					best_bearing = bearing_test;
				}

				// perform second stage test in three directions looking for obstacles
				const float test_bearings[] { 0.0f, 45.0f, -45.0f };
				const float bearing_to_dest2 = NavigationFunctions.get_bearing_to(test_loc,destination) * 0.01f;
				float distance2 = Math.constrain_float(lookahead_step2_dist, OA_BR_LOOKAHEAD_STEP2_MIN, NavigationFunctions.get_distance(test_loc,destination));
				for (uint8_t j = 0; j < ARRAY_SIZE(test_bearings); j++)
				{
					float bearing_test2 = wrap_180(bearing_to_dest2 + test_bearings[j]);
					Location_t test_loc2 = test_loc;
					NavigationFunctions.offset_bearing(test_loc2,bearing_test2, distance2);

					// calculate minimum margin to fence and obstacles for this scenario
					float margin2 = calc_avoidance_margin(test_loc, test_loc2, proximity_only);
					if (margin2 > _margin_max)
					{
						// if the chosen direction is directly towards the destination avoidance can be turned off
						// i == 0 && j == 0 implies no deviation from bearing to destination
						const bool active = (i != 0 || j != 0);
						float final_bearing = bearing_test;
						float final_margin = margin;
						// check if we need ignore test_bearing and continue on previous bearing
//						const bool ignore_bearing_change = resist_bearing_change(destination, current_loc, active, bearing_test, lookahead_step1_dist, margin, _destination_prev,_bearing_prev, final_bearing, final_margin, proximity_only);

						// all good, now project in the chosen direction by the full distance
						destination_new = current_loc;
						NavigationFunctions.offset_bearing(destination_new,final_bearing, distance_to_dest);
						_current_lookahead = MIN(_lookahead, _current_lookahead * 1.1f);
						return active;
					}
				}
			}
		}
	}

	float chosen_bearing;
	if (have_best_bearing)
	{
		// none of the directions tested were OK for 2-step checks. Choose the direction that was best for the first step
		chosen_bearing = best_bearing;
		_current_lookahead = MIN(_lookahead, _current_lookahead * 1.05f);
	}
	else
	{
		// none of the possible paths had a positive margin. Choose
		// the one with the highest margin
		chosen_bearing = best_margin_bearing;
		_current_lookahead = MAX(_lookahead * 0.5f, _current_lookahead * 0.9f);
	}

	// calculate new target based on best effort
	destination_new = current_loc;
	NavigationFunctions.offset_bearing(destination_new,chosen_bearing, distance_to_dest);
	return true;
}





// This function is called when BendyRuler has found a bearing which is obstacles free at at least lookahead_step1_dist and  then lookahead_step2_dist from the present location
// In many situations, this new bearing can be either left or right of the obstacle, and BendyRuler can have a tough time deciding between the two.
// It has the tendency to move the vehicle back and forth, if the margin obtained is even slightly better in the newer iteration.
// Therefore, this method attempts to avoid changing direction of the vehicle by more than _bendy_angle degrees,
// unless the new margin is at least _bendy_ratio times better than the margin with previously calculated bearing.
// We return true if we have resisted the change and will follow the last calculated bearing.
bool CBendyRuler::resist_bearing_change(const Location_t &destination, const Location_t &current_loc, bool active, float bearing_test, float lookahead_step1_dist, float margin, Location_t &prev_dest, float &prev_bearing, float &final_bearing, float &final_margin, bool proximity_only) const
{
	bool resisted_change = false;
	// see if there was a change in destination, if so, do not resist changing bearing
	bool dest_change = false;
	if (!NavigationFunctions.same_latlon_as(destination,prev_dest))
	{
		dest_change = true;
		prev_dest = destination;
	}
	
	// check if we need to resist the change in direction of the vehicle. If we have a clear path to destination, go there any how
	if (active && !dest_change && is_positive(_bendy_ratio))
	{
		// check the change in bearing between freshly calculated and previous stored BendyRuler bearing
		if ((fabsf(wrap_180(prev_bearing-bearing_test)) > _bendy_angle) && (!is_equal(prev_bearing,FLT_MAX)))
		{
			// check margin in last bearing's direction
			Location_t test_loc_previous_bearing;
			test_loc_previous_bearing.nLatitude = current_loc.nLatitude;
			test_loc_previous_bearing.nLongitude = current_loc.nLongitude;
			NavigationFunctions.offset_bearing(test_loc_previous_bearing,wrap_180(prev_bearing), lookahead_step1_dist);
			float previous_bearing_margin = calc_avoidance_margin(current_loc,test_loc_previous_bearing, proximity_only);

			if (margin < (_bendy_ratio * previous_bearing_margin))
			{
				// don't change direction abruptly. If margin difference is not significant, follow the last direction
				final_bearing = prev_bearing;
				final_margin  = previous_bearing_margin;
				resisted_change = true;
			}
		}
	}
	else
	{
		// reset stored bearing if BendyRuler is not active or if WP has changed for unnecessary resistance to path change
		prev_bearing = FLT_MAX;
	}
	if (!resisted_change)
	prev_bearing = bearing_test; // we are not resisting the change, hence store BendyRuler's presently calculated bearing for future iterations

	return resisted_change;
}




// calculate minimum distance between a segment and any obstacle
float CBendyRuler::calc_avoidance_margin(const Location_t &start, const Location_t &end, bool proximity_only) const
{
	float margin_min = FLT_MAX;

	float latest_margin;
	
	if (calc_margin_from_object_database(start, end, latest_margin))
	margin_min = MIN(margin_min, latest_margin);
	
	if (proximity_only)
	return margin_min;			// only need margin from proximity data
	
	if (calc_margin_from_circular_fence(start, end, latest_margin))
	{
		margin_min = MIN(margin_min, latest_margin);
	}

	// return smallest margin from any obstacle
	return margin_min;
}


// calculate minimum distance between a path and proximity sensor obstacles  on success returns true and updates margin
bool CBendyRuler::calc_margin_from_object_database(const Location_t &start, const Location_t &end, float &margin) const
{
	// exit immediately if db is empty
	CObjAvoidDbase *oaDb = oadatabase();
	if (oaDb == nullptr || !oaDb->healthy())
	return false;

	// convert start and end to offsets (in cm) from EKF origin
	Vector3f start_NEU,end_NEU;
	if (!NavigationFunctions.get_vector_from_origin_NEU(start_NEU,start) || !NavigationFunctions.get_vector_from_origin_NEU(end_NEU,end))
	return false;

	if (start_NEU == end_NEU)
	{
		return false;
	}

	// check each obstacle's distance from segment
	float smallest_margin = FLT_MAX;
	for (uint16_t i=0; i<oaDb->database_count(); i++)
	{
		const CObjAvoidDbase::OA_DbItem& item = oaDb->get_item(i);
		const Vector3f point_cm = item.pos * 100.0f;
		// margin is distance between line segment and obstacle minus obstacle's radius
		const float m = Vector3f::closest_distance_between_line_and_point(start_NEU, end_NEU, point_cm) * 0.01f - item.radius;
		if (m < smallest_margin)
		smallest_margin = m;
	}

	// return smallest margin
	if (smallest_margin < FLT_MAX)
	{
		margin = smallest_margin;
		return true;
	}

	return false;
}




// calculate minimum distance between a path and the circular fence (centered on home) on success returns true and updates margin
bool CBendyRuler::calc_margin_from_circular_fence(const Location_t &start, const Location_t &end, float &margin) const
{
	// exit immediately if polygon fence is not enabled
	const CFence *fence = CFence::get_singleton();
	if (fence == nullptr)
	{
		return false;
	}
	if ((fence->get_enabled_fences() & AC_FENCE_TYPE_CIRCLE) == 0)
	{
		return false;
	}

	// calculate start and end point's distance from home
	Location_t Loc;
	if(!NavigationFunctions.GetHomeLocation(Loc))
	return(false);
	const float start_dist_sq = NavigationFunctions.get_distance_NE(Loc,start).length_squared();
	const float end_dist_sq = NavigationFunctions.get_distance_NE(Loc,end).length_squared();

	// get circular fence radius + margin
	const float fence_radius_plus_margin = fence->get_radius() - fence->get_margin();

	// margin is fence radius minus the longer of start or end distance
	margin = fence_radius_plus_margin - sqrtf(MAX(start_dist_sq, end_dist_sq));
	return true;
}




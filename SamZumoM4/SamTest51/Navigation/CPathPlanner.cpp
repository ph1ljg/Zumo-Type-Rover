/* 
* CPathPlanner.cpp
*
* Created: 28/10/2020 17:20:08
* Author: philg
*/

#include "Includes.h"

// default constructor
CPathPlanner::CPathPlanner()
{
    _singleton = this;
	_type = OA_PATHPLAN_DISABLED;
} //CPathPlanner

// default destructor
CPathPlanner::~CPathPlanner()
{
} //~CPathPlanner




// perform any required initialisation
void CPathPlanner::init()
{
	// run background task looking for best alternative destination
	switch (_type)
	{
	case OA_PATHPLAN_DISABLED:
		// do nothing
		return;
	case OA_PATHPLAN_BENDYRULER:
		if (_oabendyruler == nullptr) 
		{
			_oabendyruler = new CBendyRuler();
//			AP_Param::load_object_from_eeprom(_oabendyruler, CBendyRuler::var_info);
		}
		break;
	}

	_oadatabase.init();
	start_thread();
}

// return type of BendyRuler in use
// CPathPlanner::get_bendy_type() const
// {
// 	if (_oabendyruler == nullptr) {
// 		return AP_OABendyRuler::OABendyType::OA_BENDY_DISABLED;
// 	}
// 	return _oabendyruler->get_type();
// }

// pre-arm checks that algorithms have been initialised successfully
bool CPathPlanner::pre_arm_check() const
{
	// check if initialisation has succeeded
	switch (_type) 
	{
		case OA_PATHPLAN_DISABLED:
		// do nothing
		break;
		case OA_PATHPLAN_BENDYRULER:
		if (_oabendyruler == nullptr) 
		{
			GuiFunctions.Printf( "BendyRuler OA requires reboot ");
			return false;
		}
		break;
	}
	return true;
}

bool CPathPlanner::start_thread()
{
	if (_type == OA_PATHPLAN_DISABLED) 
	{
		return false;
	}

	// create the avoidance thread as low priority. It should soak up spare CPU cycles to fill in the avoidance_result structure based
	// on requests in avoidance_request
	TaskManager.EnableTask(TASK_PATH_PLANNER,true);
	_thread_created = true;
	return true;
}



// provides an alternative target location if path planning around obstacles is required
// returns true and updates result_loc with an intermediate location
uint8_t CPathPlanner::mission_avoidance(const Location_t &current_loc,const Location_t &origin,const Location_t &destination,Location_t &result_origin,Location_t &result_destination)
{
	// exit immediately if disabled or thread is not running from a failed init
	if (_type == OA_PATHPLAN_DISABLED || !_thread_created) 
	{
		return OA_NOT_REQUIRED;
	}

	const uint32_t now =Core.millis();

	// place new request for the thread to work on
	avoidance_request.current_loc = current_loc;
	avoidance_request.origin = origin;
	avoidance_request.destination = destination;
	avoidance_request.ground_speed_vec =Ahrs.GetGroundspeedVector();
	avoidance_request.request_time_ms = now;

	// check result's destination matches our request
	const bool destination_matches = (destination.nLatitude == avoidance_result.destination.nLatitude) && (destination.nLongitude == avoidance_result.destination.nLongitude);

	// check results have not timed out
	const bool timed_out = now - avoidance_result.result_time_ms > OA_TIMEOUT_MS;

	// return results from background thread's latest checks
	if (destination_matches && !timed_out) 
	{
		// we have a result from the thread
		result_origin = avoidance_result.origin_new;
		result_destination = avoidance_result.destination_new;
		return avoidance_result.ret_state;
	}

	// if timeout then path planner is taking too long to respond
	if (timed_out) 
		return OA_ERROR;

	// background thread is working on a new destination
	return OA_PROCESSING;
}

// avoidance thread that continually updates the avoidance_result structure based on avoidance_request
void CPathPlanner::avoidance_thread()
{
	// require ekf origin to have been set
	bool origin_set = false;
	while (!origin_set) 
	{
		Core.delay(500);
		 Location_t ekf_origin;
		origin_set = NavigationFunctions.GetHomeLocation(ekf_origin);
	}

	while (true) 
	{
		// if database queue needs attention, service it faster
// 		if (_oadatabase.process_queue()) 
// 		{
// 			hal.scheduler->delay(1);
// 			} else {
// 			hal.scheduler->delay(20);
// 		}

		const uint32_t now = Core.millis();
		if (now - avoidance_latest_ms < OA_UPDATE_MS) 
			continue;

		avoidance_latest_ms = now;

		_oadatabase.Update();

		Location_t origin_new;
		Location_t destination_new;
		
		if (now - avoidance_request.request_time_ms > OA_TIMEOUT_MS) 
			continue;		// this is a very old request, don't process it

		// copy request to avoid conflict with main thread
		avoidance_request2 = avoidance_request;

		// store passed in origin and destination so we can return it if object avoidance is not required
			origin_new = avoidance_request.origin;
			destination_new = avoidance_request.destination;

		// run background task looking for best alternative destination
		OA_RetState res = OA_NOT_REQUIRED;
		switch (_type) 
		{
		case OA_PATHPLAN_DISABLED:
			continue;
		case OA_PATHPLAN_BENDYRULER:
			if (_oabendyruler == nullptr) 
				continue;
			_oabendyruler->set_config(_margin_max);
			if (_oabendyruler->update(avoidance_request2.current_loc, avoidance_request2.destination, avoidance_request2.ground_speed_vec, origin_new, destination_new, false)) 
			{
				res = OA_SUCCESS;
			}
			break;
		}
		{
			// give the main thread the avoidance result
			avoidance_result.destination = avoidance_request2.destination;
			avoidance_result.origin_new = (res == OA_SUCCESS) ? origin_new : avoidance_result.origin_new;
			avoidance_result.destination_new = (res == OA_SUCCESS) ? destination_new : avoidance_result.destination;
			avoidance_result.result_time_ms = Core.millis();
			avoidance_result.ret_state = res;
		}
	}
}

// singleton instance
CPathPlanner *CPathPlanner::_singleton;

	CPathPlanner *ap_oapathplanner()
	{
		return CPathPlanner::get_singleton();
	}


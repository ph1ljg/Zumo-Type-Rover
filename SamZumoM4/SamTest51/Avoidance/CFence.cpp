/* 
* CFence.cpp
*
* Created: 28/10/2020 11:11:17
* Author: philg
*/


#include "Includes.h"
// default constructor
CFence::CFence()
{
    _singleton = this;

} //CFence

// default destructor
CFence::~CFence()
{
} //~CFence


// enable the Fence code generally; a master switch for all fences
void CFence::enable(bool value)
{
	if(value)
		Config.m_RunningFlags.FENCE_ENABLE = true;
	else
	{	
		Config.m_RunningFlags.FENCE_ENABLE = false;
		clear_breach(AC_FENCE_TYPE_ALT_MAX | AC_FENCE_TYPE_CIRCLE | AC_FENCE_TYPE_POLYGON);
	}
}

// get_enabled_fences - returns Bitmask of enabled fences
uint8_t CFence::get_enabled_fences() const
{
	if (Config.m_RunningFlags.FENCE_ENABLE) 
		return 0;
	return _enabled_fences;
}

// additional checks for the polygon fence:
// bool CFence::pre_arm_check_polygon(const char* &fail_msg) const
// {
// 	if (!(_enabled_fences & AC_FENCE_TYPE_POLYGON)) 
// 		return true;			 // not enabled; all good
// 
// 	if (! _poly_loader.loaded()) 
// 	{
// 		fail_msg = "Fences invalid";
// 		return false;
// 	}
// 
// 	if (!_poly_loader.check_inclusion_circle_margin(_margin)) 
// 	{
// 		fail_msg = "Margin is less than inclusion circle radius";
// 		return false;
// 	}
// 
// 	return true;
// }

// additional checks for the circle fence:
bool CFence::pre_arm_check_circle(const char* &fail_msg) const
{
	if (_circle_radius < 0) 
	{
		fail_msg = "Invalid FENCE_RADIUS value";
		return false;
	}
	if (_circle_radius < _margin) 
	{
		fail_msg = "FENCE_MARGIN is less than FENCE_RADIUS";
		return false;
	}

	return true;
}

// additional checks for the alt fence:
bool CFence::pre_arm_check_alt(const char* &fail_msg) const
{
	if (_alt_max < 0.0f) 
	{
		fail_msg = "Invalid FENCE_ALT_MAX value";
		return false;
	}
	return true;
}


/// pre_arm_check - returns true if all pre-takeoff checks have completed successfully
bool CFence::pre_arm_check(const char* &fail_msg) const
{
	fail_msg = nullptr;

	
	if (!Config.m_RunningFlags.FENCE_ENABLE || !_enabled_fences) 
		return true;		// if not enabled or not fence set-up always return true

	// if we have horizontal limits enabled, check we can get a relative position from the AHRS
	if ((_enabled_fences & AC_FENCE_TYPE_CIRCLE) ||	(_enabled_fences & AC_FENCE_TYPE_POLYGON)) 
	{
		if (Config.m_RunningFlags.HOME_LOCATION_SET)
		 {
			fail_msg = "Fence requires position";
			return false;
		}
	}

	if (!pre_arm_check_polygon(fail_msg)) 
	{
		return false;
	}

	if (!pre_arm_check_circle(fail_msg)) 
	{
		return false;
	}

	if (!pre_arm_check_alt(fail_msg))
	 {
		return false;
	}

	// check no limits are currently breached
	if (_breached_fences) 
	{
		fail_msg =  "vehicle outside fence";
		return false;
	}

	// if we got this far everything must be ok
	return true;
}


// check_fence_polygon - returns true if the poly fence is freshly breached.  That includes being inside exclusion zones and outside inclusions zones
// bool CFence::check_fence_polygon()
// {
// 	const bool was_breached = _breached_fences & AC_FENCE_TYPE_POLYGON;
// 	const bool breached = ((_enabled_fences & AC_FENCE_TYPE_POLYGON) &&
// 	_poly_loader.breached());
// 	if (breached)
// 	{
// 		if (!was_breached) 
// 		{
// 			record_breach(AC_FENCE_TYPE_POLYGON);
// 			return true;
// 		}
// 		return false;
// 	}
// 	if (was_breached) 
// 	{
// 		clear_breach(AC_FENCE_TYPE_POLYGON);
// 	}
// 	return false;
// }

// check_fence_circle - returns true if the circle fence (defined via parameters) has been freshly breached.  May also set up a backup
// fence outside the fence and return a fresh breach if that backup fence is breached.
bool CFence::check_fence_circle()
{
	if (!(_enabled_fences & AC_FENCE_TYPE_CIRCLE)) 
	{
		// not enabled; no breach
		return false;
	}

	Vector2f home;
	
	if (NavigationFunctions.get_relative_position_NE_home(home)) 
		_home_distance = home.length();		// we (may) remain breached if we can't update home

	// check if we are outside the fence
	if (_home_distance >= _circle_radius) 
	{

		// record distance outside the fence
		_circle_breach_distance = _home_distance - _circle_radius;

		// check for a new breach or a breach of the backup fence
		if (!(_breached_fences & AC_FENCE_TYPE_CIRCLE) ||		(!is_zero(_circle_radius_backup) && _home_distance >= _circle_radius_backup)) 
		{
			// new breach
			// create a backup fence 20m further out
			record_breach(AC_FENCE_TYPE_CIRCLE);
			_circle_radius_backup = _home_distance + AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE;
			return true;
		}
		return false;
	}

	// not currently breached

	// clear circle breach if present
	if (_breached_fences & AC_FENCE_TYPE_CIRCLE) 
	{
		clear_breach(AC_FENCE_TYPE_CIRCLE);
		_circle_radius_backup = 0.0f;
		_circle_breach_distance = 0.0f;
	}

	return false;
}


/// check - returns bitmask of fence types breached (if any)
uint8_t CFence::check()
{
	uint8_t ret = 0;

	
	if (!Config.m_RunningFlags.FENCE_ENABLE || !_enabled_fences)
		return 0;	// return immediately if disabled

	// clear any breach from a non-enabled fence
	clear_breach(~_enabled_fences);

	// check if pilot is attempting to recover manually
	if (_manual_recovery_start_ms != 0) 
	{
		// we ignore any fence breaches during the manual recovery period which is about 10 seconds
		if ((Core.millis() - _manual_recovery_start_ms) < AC_FENCE_MANUAL_RECOVERY_TIME_MIN) {
			return 0;
		}
		// recovery period has passed so reset manual recovery time
		// and continue with fence breach checks
		_manual_recovery_start_ms = 0;
	}


	// circle fence check
	if (check_fence_circle()) 
	{
		ret |= AC_FENCE_TYPE_CIRCLE;
	}

	// polygon fence check
	if (check_fence_polygon()) 
	{
		ret |= AC_FENCE_TYPE_POLYGON;
	}

	// return any new breaches that have occurred
	return ret;
}

// returns true if the destination is within fence (used to reject waypoints outside the fence)
bool CFence::check_destination_within_fence(const Location_t &loc)
{
	// Circular fence check
	if ((get_enabled_fences() & AC_FENCE_TYPE_CIRCLE)) 
	{
		if (NavigationFunctions.GetHomeDistance(loc) > _circle_radius) 
			return false;
	}

// 	// polygon fence check
// 	if ((get_enabled_fences() & AC_FENCE_TYPE_POLYGON)) 
// 	{
// 		if (_poly_loader.breached(loc)) 
// 			return false;
// 	}

	return true;
}

// record_breach - update breach Bitmask, time and count
void CFence::record_breach(uint8_t fence_type)
{
	if (!_breached_fences)		// if we haven't already breached a limit, update the breach time
		_breach_time = Core.millis();

	if (_breach_count < 65500)		// update breach count
		_breach_count++;
	
	_breached_fences |= fence_type;		// update Bitmask
}

// clear_breach - update breach Bitmask, time and count
void CFence::clear_breach(uint8_t fence_type)
{
	_breached_fences &= ~fence_type;
}

// get_breach_distance - returns maximum distance in meters outside of the given fences.  fence_type is a Bitmask here.
float CFence::get_breach_distance(uint8_t fence_type) const
{
	float max = 0.0f;
	if (fence_type & AC_FENCE_TYPE_ALT_MAX) 
		max = MAX(_alt_max_breach_distance, max);

	if (fence_type & AC_FENCE_TYPE_CIRCLE) 
		max = MAX(_circle_breach_distance, max);
	return max;
}

/// manual_recovery_start - caller indicates that pilot is re-taking manual control so fence should be disabled for 10 seconds  has no effect if no breaches have occurred
void CFence::manual_recovery_start()
{
	// return immediate if we haven't breached a fence
	if (!_breached_fences) 
		return;

	// record time pilot began manual recovery
	_manual_recovery_start_ms = Core.millis();
}


// AC_PolyFence_loader &AC_Fence::polyfence() {
// 	return _poly_loader;
// }
// const AC_PolyFence_loader &AC_Fence::polyfence() const {
// 	return _poly_loader;
// }
// 
// singleton instance

CFence *CFence::_singleton;

	CFence *fence()
	{
		return CFence::get_singleton();
	}




/* 
* CFence.h
*
* Created: 28/10/2020 11:11:18
* Author: philg
*/


#ifndef __CFENCE_H__
#define __CFENCE_H__


// bit masks for enabled fence types.  Used for TYPE parameter
#define AC_FENCE_TYPE_ALT_MAX                       1       // high alt fence which usually initiates an RTL
#define AC_FENCE_TYPE_CIRCLE                        2       // circular horizontal fence (usually initiates an RTL)
#define AC_FENCE_TYPE_POLYGON                       4       // polygon horizontal fence

// valid actions should a fence be breached
#define AC_FENCE_ACTION_REPORT_ONLY                 0       // report to GCS that boundary has been breached but take no further action
#define AC_FENCE_ACTION_RTL_AND_LAND                1       // return to launch and, if that fails, land
#define AC_FENCE_ACTION_ALWAYS_LAND                 2       // always land
#define AC_FENCE_ACTION_SMART_RTL                   3       // smartRTL, if that fails, RTL, it that still fails, land
#define AC_FENCE_ACTION_BRAKE                       4       // brake, if that fails, land

// default boundaries
#define AC_FENCE_ALT_MAX_DEFAULT                    100.0f  // default max altitude is 100m
#define AC_FENCE_ALT_MIN_DEFAULT                    -10.0f  // default maximum depth in meters
#define AC_FENCE_CIRCLE_RADIUS_DEFAULT              300.0f  // default circular fence radius is 300m
#define AC_FENCE_ALT_MAX_BACKUP_DISTANCE            20.0f   // after fence is broken we recreate the fence 20m further up
#define AC_FENCE_CIRCLE_RADIUS_BACKUP_DISTANCE      20.0f   // after fence is broken we recreate the fence 20m further out
#define AC_FENCE_MARGIN_DEFAULT                     2.0f    // default distance in meters that autopilot's should maintain from the fence to avoid a breach

// give up distance
#define AC_FENCE_GIVE_UP_DISTANCE                   100.0f  // distance outside the fence at which we should give up and just land.  Note: this is not used by library directly but is intended to be used by the main code
#define AC_FENCE_MANUAL_RECOVERY_TIME_MIN           10000   // pilot has 10seconds to recover during which time the autopilot will not attempt to re-take control



class CFence
{
//variables
public:
protected:
private:
    uint8_t		_enabled;               // top level enable/disable control
    uint8_t     _enabled_fences;        // bit mask holding which fences are enabled
    uint8_t     _action;                // recovery action specified by user
    float		_alt_max;               // altitude upper limit in meters
    float       _alt_min;               // altitude lower limit in meters
    float       _circle_radius;         // circle fence radius in meters
    float       _margin;                // distance in meters that autopilot's should maintain from the fence to avoid a breach
    uint8_t      _total;                 // number of polygon points saved in eeprom

    // backup fences
    float           _alt_max_backup;        // backup altitude upper limit in meters used to refire the breach if the vehicle continues to move further away
    float           _circle_radius_backup;  // backup circle fence radius in meters used to refire the breach if the vehicle continues to move further away

    // breach distances
    float           _alt_max_breach_distance;   // distance above the altitude max
    float           _circle_breach_distance;    // distance beyond the circular fence

    // other internal variables
    float           _home_distance;         // distance from home in meters (provided by main code)
    float _curr_alt;


    // breach information
    uint8_t         _breached_fences;       // bitmask holding the fence type that was breached (i.e. AC_FENCE_TYPE_ALT_MIN, AC_FENCE_TYPE_CIRCLE)
    uint32_t        _breach_time;           // time of last breach in milliseconds
    uint16_t        _breach_count;          // number of times we have breached the fence

    uint32_t        _manual_recovery_start_ms;  // system time in milliseconds that pilot re-took manual control

    static CFence *_singleton;

//functions
public:
	CFence();
	~CFence();
	void enable(bool value);
	uint8_t get_enabled_fences() const;
	bool pre_arm_check_polygon(const char* &fail_msg) const;
	bool pre_arm_check_circle(const char* &fail_msg) const;
	bool pre_arm_check_alt(const char* &fail_msg) const;
	bool pre_arm_check(const char* &fail_msg) const;
	bool check_fence_polygon();
	bool check_fence_circle();
	uint8_t check();
	bool check_destination_within_fence(const Location_t &loc);
	void record_breach(uint8_t fence_type);
	void clear_breach(uint8_t fence_type);
	float get_breach_distance(uint8_t fence_type) const;
	void manual_recovery_start();
    static CFence *get_singleton() { return _singleton; }		// get singleton instance
    float get_radius() const { return _circle_radius; }		// get_radius - returns the fence radius in meters
    float get_margin() const { return _margin; }		// get_margin - returns the fence margin in meters

protected:
private:
	CFence( const CFence &c );
	CFence& operator=( const CFence &c );

}; //CFence

#endif //__CFENCE_H__

/* 
* CPathPlanner.h
*
* Created: 28/10/2020 17:20:08
* Author: philg
*/


#ifndef __CPATHPLANNER_H__
#define __CPATHPLANNER_H__

#define OA_TIMEOUT_MS  3000     // results over 3 seconds old are ignored
#define OA_UPDATE_MS  1000      // path planning updates run at 1hz

class CPathPlanner
{
//variables
public:
    // enumerations for _TYPE parameter
    enum OAPathPlanTypes 
	{
	    OA_PATHPLAN_DISABLED	= 0,
	    OA_PATHPLAN_BENDYRULER	= 1,
    };
	// object avoidance processing return status enum
	enum OA_RetState : uint8_t 
	{
		OA_NOT_REQUIRED = 0,            // object avoidance is not required
		OA_PROCESSING,                  // still calculating alternative path
		OA_ERROR,                       // error during calculation
		OA_SUCCESS                      // success
	};
    float _margin_max;           // object avoidance will ignore objects more than this many meters from vehicle

protected:
private:
    uint8_t _type;                  // avoidance algorithm to be used
    CObjAvoidDbase _oadatabase;      // Database of dynamic objects to avoid
	CBendyRuler *_oabendyruler;		// Bendy Ruler algorithm
    bool _thread_created;           // true once background thread has been created
    // an avoidance request from the navigation code
    struct avoidance_info 
	{
	    Location_t current_loc;
	    Location_t origin;
	    Location_t destination;
	    Vector2f ground_speed_vec;
	    uint32_t request_time_ms;
    } avoidance_request, avoidance_request2;
    // an avoidance result from the avoidance thread
    struct
	 {
	    Location_t destination;       // destination vehicle is trying to get to (also used to verify the result matches a recent request)
	    Location_t origin_new;        // intermediate origin.  The start of line segment that vehicle should follow
	    Location_t destination_new;   // intermediate destination vehicle should move towards
	    uint32_t result_time_ms;    // system time the result was calculated (used to verify the result is recent)
	    OA_RetState ret_state;      // OA_SUCCESS if the vehicle should move along the path from origin_new to destination_new
    } avoidance_result;
    uint32_t avoidance_latest_ms;   // last time Dijkstra's or BendyRuler algorithms ran

//functions
public:
	CPathPlanner();
	~CPathPlanner();
	
	void init();
	bool pre_arm_check() const;
	bool start_thread();
	uint8_t mission_avoidance(const Location_t &current_loc,const Location_t &origin,const Location_t &destination,Location_t &result_origin,Location_t &result_destination);
	void avoidance_thread();
	// Do not allow copies 
	CPathPlanner(const CPathPlanner &other) = delete;
	CPathPlanner &operator=(const CPathPlanner&) = delete;
	    
	static CPathPlanner *get_singleton()			// get singleton instance 
	{
		return _singleton;
	}


protected:
private:
//	CPathPlanner( const CPathPlanner &c );
//	CPathPlanner& operator=( const CPathPlanner &c );
    static CPathPlanner *_singleton;

}; //CPathPlanner

  CPathPlanner *ap_oapathplanner();

#endif //__CPATHPLANNER_H__

/* 
* CObjAvoidDbase.h
*
* Created: 27/10/2020 10:44:27
* Author: philg
*/


#ifndef __COBJAVOIDDBASE_H__
#define __COBJAVOIDDBASE_H__


class CObjAvoidDbase
{
//variables
public:
    enum OA_DbItemImportance { Low, Normal, High };
    struct OA_DbItem
    {
	    Vector3f pos;           // position of the object as an offset in meters from the EKF origin
	    uint32_t timestamp_ms;  // system time that object was last updated
	    float radius;           // objects radius in meters
	    OA_DbItemImportance importance;
    };

protected:
private:
	struct 
	{
		CObjectBuffer<OA_DbItem> *items;                     // thread safe incoming queue of points from proximity sensor to be put into database
		uint16_t        size;                               // cached value of _queue_size_param.
	} _queue;

   struct 
   {
		OA_DbItem       *items;                             // array of objects in the database
		uint16_t        count;                              // number of objects in the items array
		uint16_t        size;                               // cached value of _database_size_param that sticks after initialized
   } _database;

//functions
public:
	CObjAvoidDbase();
	~CObjAvoidDbase();

	void init();
	void Update();
	void queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance);
	void init_queue();
	void init_database();
	bool process_queue();
	void database_item_add(const OA_DbItem &item);
	void database_item_remove(const uint16_t index);
	void database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius);
	void database_items_remove_all_expired();
	bool is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const;
	    // get singleton instance
	static CObjAvoidDbase *get_singleton() { return _singleton; }

    // returns true if database is healthy
    bool healthy() const { return (_queue.items != nullptr) && (_database.items != nullptr); }

    // fetch an item in database. Undefined result when i >= _database.count.
    const OA_DbItem& get_item(uint32_t i) const { return _database.items[i]; }

    // get number of items in the database
    uint16_t database_count() const { return _database.count; }




protected:
private:
	    CObjAvoidDbase(const CObjAvoidDbase &other) = delete;
	    CObjAvoidDbase &operator=(const CObjAvoidDbase&) = delete;
// 	CObjAvoidDbase( const CObjAvoidDbase &c );
// 	CObjAvoidDbase& operator=( const CObjAvoidDbase &c );
	int16_t        _queue_size_param;                      // queue size
	int16_t        _database_size_param;                   // db size
	uint8_t         _database_expiry_seconds;               // objects expire after this timeout
	uint8_t         _output_level;                          // controls which items should be sent to GCS
	float       _beam_width;                            // beam width used when converting lidar readings to object radius
	float        _radius_min;                            // objects minimum radius (in meters)
	float        _dist_max;                              // objects maximum distance (in meters)
//	float        _min_alt;                               // OADatabase minimum vehicle height check (in meters)

	float dist_to_radius_scalar;                            // scalar to convert the distance and beam width to an object radius
 
   
   
   
    static CObjAvoidDbase *_singleton;
}; //CObjAvoidDbase

CObjAvoidDbase *oadatabase();


#endif //__COBJAVOIDDBASE_H__

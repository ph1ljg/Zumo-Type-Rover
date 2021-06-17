/* 
* CObjAvoidDbase.cpp
*
* Created: 27/10/2020 10:44:27
* Author: philg
*/

#include "Includes.h"

// default constructor
CObjAvoidDbase::CObjAvoidDbase()
{
} //CObjAvoidDbase

// default destructor
CObjAvoidDbase::~CObjAvoidDbase()
{
} //~CObjAvoidDbase


void CObjAvoidDbase::init()
{
	init_database();
	init_queue();

	// initialise scalar using beam width of at least 1deg
	dist_to_radius_scalar = tanf(radians(MAX(_beam_width, 1.0f)));

	if (!healthy())
	{
		GuiFunctions.Printf("DB init failed . Sizes queue:%u, db:%u", (unsigned int)_queue.size, (unsigned int)_database.size);
		delete _queue.items;
		delete[] _database.items;
		return;
	}
}

void CObjAvoidDbase::Update()
{
	if (!healthy()) 
		return;

	process_queue();
	database_items_remove_all_expired();
}



// push a location into the database
void CObjAvoidDbase::queue_push(const Vector3f &pos, uint32_t timestamp_ms, float distance)
{
	if (!healthy()) 
		return;

	
	// ignore objects that are far away
	if ((_dist_max > 0.0f) && (distance > _dist_max)) 
		return;

	const OA_DbItem item = {pos, timestamp_ms, MAX(_radius_min, distance * dist_to_radius_scalar), CObjAvoidDbase::OA_DbItemImportance::Normal};
	{
		_queue.items->push(item);
	}
}

void CObjAvoidDbase::init_queue()
{
	_queue.size = _queue_size_param;
	if (_queue.size == 0) 
		return;

	_queue.items = new CObjectBuffer<OA_DbItem>(_queue.size);
}

void CObjAvoidDbase::init_database()
{
	_database.size = _database_size_param;
	if (_database_size_param == 0) 
		return;
	_database.items = new OA_DbItem[_database.size];
}


// returns true when there's more work in the queue to do
bool CObjAvoidDbase::process_queue()
{
	if (!healthy()) 
		return false;

	// processing queue by moving those entries into the database Using a for with fixed size is better than while(!empty) because the
	// while could get us stuck here longer than expected if we're getting a lot of values pushing into it while we're trying to empty it. With
	// the for we know we will exit at an expected time
	const uint16_t queue_available = MIN(_queue.items->available(), 100U);
	if (queue_available == 0) 
		return false;

	for (uint16_t queue_index=0; queue_index<queue_available; queue_index++) 
	{
		OA_DbItem item;

		bool pop_success;
		pop_success = _queue.items->pop(item);

		if (!pop_success) 
			return false;


		// compare item to all items in database. If found a similar item, update the existing, else add it as a new one
		bool found = false;
		for (uint16_t i=0; i<_database.count; i++) 
		{
			if (is_close_to_item_in_database(i, item)) 
			{
				database_item_refresh(i, item.timestamp_ms, item.radius);
				found = true;
				break;
			}
		}

		if (!found) 
		{
			database_item_add(item);
		}
	}
	return (_queue.items->available() > 0);
}

void CObjAvoidDbase::database_item_add(const OA_DbItem &item)
{
	if (_database.count >= _database.size)
		return;

	_database.items[_database.count] = item;
	_database.count++;
}

void CObjAvoidDbase::database_item_remove(const uint16_t index)
{
	if (index >= _database.count || _database.count == 0)
		return;			// index out of range


	_database.count--;
	if (_database.count == 0) 
		return;

	if (index != _database.count) 
	{
		// copy last object in array over expired object
		_database.items[index] = _database.items[_database.count];
	}
}

void CObjAvoidDbase::database_item_refresh(const uint16_t index, const uint32_t timestamp_ms, const float radius)
{
	if (index >= _database.count) 
		return;			// index out of range

	const bool is_different =	(!is_equal(_database.items[index].radius, radius)) ||	(timestamp_ms - _database.items[index].timestamp_ms >= 500);

	if (is_different) 
	{
		// update timestamp and radius on close object so it stays around longer 
		_database.items[index].timestamp_ms = timestamp_ms;
		_database.items[index].radius = radius;
	}
}

void CObjAvoidDbase::database_items_remove_all_expired()
{
	// calculate age of all items in the _database

	if (_database_expiry_seconds <= 0) 
	{
		// zero means never expire. This is not normal behavior but perhaps you could send a static environment once that you don't want to have to constantly update
		return;
	}

	const uint32_t now_ms = Core.millis();
	const uint32_t expiry_ms = (uint32_t)_database_expiry_seconds * 1000;
	uint16_t index = 0;
	while (index < _database.count)
	 {
		if (now_ms - _database.items[index].timestamp_ms > expiry_ms) 
			database_item_remove(index);
		else 
			index++;
	}
}

// returns true if a similar object already exists in database. When true, the object timer is also reset
bool CObjAvoidDbase::is_close_to_item_in_database(const uint16_t index, const OA_DbItem &item) const
{
	if (index >= _database.count) 
		return false;			// index out of range

	const float distance_sq = (_database.items[index].pos - item.pos).length_squared();
	return ((distance_sq < sq(item.radius)) || (distance_sq < sq(_database.items[index].radius)));
}

// singleton instance
CObjAvoidDbase *CObjAvoidDbase::_singleton;

//namespace AP {
	CObjAvoidDbase *oadatabase()
	{
		return CObjAvoidDbase::get_singleton();
	}

//}

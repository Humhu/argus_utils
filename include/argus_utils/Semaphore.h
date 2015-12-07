#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>

namespace argus_utils {

/*! \brief A standard semaphore class. */
class Semaphore {
public:

	Semaphore( int startCounter = 0 );
	
	void Increment( int i = 1 );
	
	void Decrement( int i = 1 );

	// TODO IncrementWait and DecrementWait
	
	/*! \brief Returns how many counters are available. */
	int Query() const;
	
protected:

	typedef boost::shared_mutex Mutex;
	typedef boost::condition_variable_any Condition;
	
	mutable Mutex mutex;
	int counter;
	Condition hasCounters;
	
};

}


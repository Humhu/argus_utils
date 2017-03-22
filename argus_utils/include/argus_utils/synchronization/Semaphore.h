#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus 
{

/*! \brief A standard semaphore class. */
class Semaphore 
{
public:

	Semaphore( int startCounter = 0 );
	
	void Increment( int i = 1 );
	
	void Decrement( int i = 1 );

	// TODO IncrementWait and DecrementWait
	
	/*! \brief Returns how many counters are available. */
	int Query() const;
	
protected:

	mutable Mutex mutex;
	int counter;
	ConditionVariable hasCounters;
	
};

}


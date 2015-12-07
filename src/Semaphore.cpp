#include "argus_utils/Semaphore.h"

namespace argus_utils
{

Semaphore::Semaphore( int startCounter )
	: counter( startCounter )
{}

void Semaphore::Increment( int i )
{
	boost::unique_lock<Mutex> lock( mutex );
	counter += i;
	hasCounters.notify_all(); // TODO all or one?
}

void Semaphore::Decrement( int i )
{
	boost::unique_lock<Mutex> lock( mutex );
	
	while( counter < i )
	{
		hasCounters.wait( lock );
	}
	counter = counter - i;
}

// TODO IncrementWait and DecrementWait

/*! \brief Returns how many counters are available. */
int Semaphore::Query() const
{
	boost::shared_lock<Mutex> lock( mutex );
	return counter;
}
	
}

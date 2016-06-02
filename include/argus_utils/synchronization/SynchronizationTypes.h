#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

namespace argus
{
	// TODO Move semaphore into here?
	
	typedef boost::shared_mutex Mutex;
	typedef boost::shared_lock<Mutex> ReadLock;
	typedef boost::unique_lock<Mutex> WriteLock;
	
	typedef boost::condition_variable_any ConditionVariable;
	
}

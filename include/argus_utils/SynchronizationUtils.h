#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>

namespace argus_utils
{
	// TODO Move semaphore into here?
	
	typedef boost::shared_mutex Mutex;
	typedef boost::shared_lock<Mutex> ReadLock;
	typedef boost::unique_lock<Mutex> WriteLock;
	
}

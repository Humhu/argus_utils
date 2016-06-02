#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#include "argus_utils/synchronization/Semaphore.h"

#include <queue>

namespace argus 
{

/*! \brief An asynchronous worker thread pool. Threads are not created
 * until specified, so construction is fast. */
class WorkerPool {
public:

	typedef std::shared_ptr<WorkerPool> Ptr;
	typedef boost::function<void()> Job;

	/*! \brief Creates a pool with the specified target number of workers. 
	 * Workers are not created until StartWorkers() is called. */
	WorkerPool( unsigned int n = 4 );
		
	/*! \brief Stops all threads and waits for them to return. */
	~WorkerPool();
	
	/*! \brief Sets the number of workers to initialize. */
	void SetNumWorkers( unsigned int n );
	
	/*! \brief Adds a job to the worker queue and wakes workers. */
	void EnqueueJob( Job job );

	/*! \brief Creates the target number of worker threads and assigns
	 * them to the task queue. */
	void StartWorkers();
	
	/*! \brief Stops all workers and waits for them to return. */
	void StopWorkers();

	/*! \brief Block until all current jobs are complete. */
	void WaitOnJobs();
	
protected:

	typedef boost::unique_lock< boost::shared_mutex > Lock;

	boost::shared_mutex _mutex;
	unsigned int _numWorkers;
	std::queue<Job> _jobQueue;
	
	unsigned int _activeThreads;
	boost::condition_variable_any _hasJobs;
	boost::condition_variable_any _threadsDone;
	
	boost::thread_group _workerThreads;
	
	void WorkerLoop();

};

} // end namespace argus

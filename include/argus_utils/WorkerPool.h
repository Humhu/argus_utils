#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#include <queue>

namespace argus_utils {

/*! \brief An asynchronous worker thread pool. */
class WorkerPool {
public:

	typedef std::shared_ptr<WorkerPool> Ptr;
	typedef boost::function<void()> Job;

	WorkerPool();
		
	WorkerPool( unsigned int n );
		
	~WorkerPool();
	
	void SetNumWorkers( unsigned int n );
	
	void EnqueueJob( Job job );

	void StartWorkers();
	
	void StopWorkers();

	void WaitOnJobs();
	
protected:

	typedef boost::unique_lock< boost::shared_mutex > Lock;

	boost::shared_mutex mutex;
	unsigned int numWorkers;
	std::queue<Job> jobQueue;
	boost::condition_variable_any hasJobs;
	boost::condition_variable_any hasNoJobs;
	boost::thread_group workerThreads;
	
	void WorkerLoop();

};

} // end namespace argus_utils

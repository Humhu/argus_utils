#include "argus_utils/WorkerPool.h"

namespace argus_utils
{

WorkerPool::WorkerPool()
: numWorkers( 4 ) {}
	
WorkerPool::WorkerPool( unsigned int n )
: numWorkers( n ) {}
	
WorkerPool::~WorkerPool()
{
	StopWorkers();
}

void WorkerPool::SetNumWorkers( unsigned int n )
{
	numWorkers = n;
}

void WorkerPool::EnqueueJob( Job job )
{
	jobQueue.push( job );
	hasJobs.notify_one();
}

void WorkerPool::StartWorkers()
{
	for( unsigned int i = 0; i < numWorkers; i++) {
		workerThreads.create_thread( boost::bind( &WorkerPool::WorkerLoop, this ) );
	}
}

void WorkerPool::StopWorkers()
{
	workerThreads.interrupt_all();
	workerThreads.join_all();
}

void WorkerPool::WaitOnJobs()
{
	Lock lock( mutex );
	while( !jobQueue.empty() )
	{
		hasNoJobs.wait( lock );
	}
}

void WorkerPool::WorkerLoop()
{
	try {
		while( true ) 
		{
			boost::this_thread::interruption_point();

			Lock lock( mutex );
			while( jobQueue.empty() )
			{
				hasJobs.wait( lock );
			}
			
			Job job = jobQueue.front();
			jobQueue.pop();
			if( jobQueue.size() == 0 )
			{
				hasNoJobs.notify_all();
			}
			
			lock.unlock();
			
			job();
		}
	}
	catch( boost::thread_interrupted e ) { return; }
}

}

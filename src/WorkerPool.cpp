#include "argus_utils/WorkerPool.h"

namespace argus_utils
{

WorkerPool::WorkerPool()
: numWorkers( 4 ), activeThreads( 0 ) {}
	
WorkerPool::WorkerPool( unsigned int n )
: numWorkers( n ), activeThreads( 0 ) {}
	
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
	while( !jobQueue.empty() || activeThreads > 0 )
	{
		threadsDone.wait( lock );
	}
}

void WorkerPool::WorkerLoop()
{
	try {
		while( true ) 
		{
			boost::this_thread::interruption_point();

			// Reacquire lock to allow other threads a chance
			Lock lock( mutex );
			
			// Wait on a job here
			while( jobQueue.empty() )
			{
				hasJobs.wait( lock );
			}
			
			activeThreads++;
			Job job = jobQueue.front();
			jobQueue.pop();
			lock.unlock();
			
			job();
			
			lock.lock();
			activeThreads--;
			if( activeThreads == 0 )
			{
				threadsDone.notify_all();
			}
		}
	}
	catch( boost::thread_interrupted e ) { return; }
}

}

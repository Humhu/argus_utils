#include "argus_utils/synchronization/WorkerPool.h"

namespace argus
{

WorkerPool::WorkerPool( unsigned int n )
: _numWorkers( n ), _activeThreads( 0 ) {}
	
WorkerPool::~WorkerPool()
{
	StopWorkers();
}

void WorkerPool::SetNumWorkers( unsigned int n )
{
	_numWorkers = n;
}

void WorkerPool::EnqueueJob( Job job )
{
	_jobQueue.push( job );
	_hasJobs.notify_one();
}

void WorkerPool::StartWorkers()
{
	for( unsigned int i = 0; i < _numWorkers; i++) 
	{
		_workerThreads.create_thread( boost::bind( &WorkerPool::WorkerLoop, 
		                                           this ) );
	}
}

void WorkerPool::StopWorkers()
{
	_workerThreads.interrupt_all();
	_workerThreads.join_all();
}

void WorkerPool::WaitOnJobs()
{
	Lock lock( _mutex );
	while( !_jobQueue.empty() || _activeThreads > 0 )
	{
		_threadsDone.wait( lock );
	}
}

void WorkerPool::WorkerLoop()
{
	try {
		while( true ) 
		{
			boost::this_thread::interruption_point();

			// Reacquire lock to allow other threads a chance
			Lock lock( _mutex );
			
			// Wait on a job here
			while( _jobQueue.empty() )
			{
				_hasJobs.wait( lock );
			}
			
			_activeThreads++;
			Job job = _jobQueue.front();
			_jobQueue.pop();
			lock.unlock();
			
			job();
			
			lock.lock();
			_activeThreads--;
			if( _activeThreads == 0 )
			{
				_threadsDone.notify_all();
			}
		}
	}
	catch( boost::thread_interrupted e ) { return; }
}

}

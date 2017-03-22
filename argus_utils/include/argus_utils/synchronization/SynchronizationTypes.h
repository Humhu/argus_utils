#pragma once

#include <boost/thread/thread.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lockable_adapter.hpp>

namespace argus
{

// TODO Move semaphore into here?

typedef boost::shared_mutex Mutex;
typedef boost::shared_lock<Mutex> ReadLock;
typedef boost::unique_lock<Mutex> WriteLock;

typedef boost::condition_variable_any ConditionVariable;

template<template<typename> class Lock, typename Lockable>
void CheckLockOwnership( const Lock<Lockable>& lock, const Lockable* lockable )
{
	if( lock.mutex() != lockable )
	{
		throw std::runtime_error( "Lock ownership error!" );
	}
}

}

#pragma once

#include <memory>
#include <deque>

#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus 
{
	
/*! \class ThreadsafeContainer ThreadsafeQueue.h
* \brief is a mutex-wrapped container. */
template <class T, template<typename,typename> class Container = std::deque >
class ThreadsafeQueue {
public:

	typedef T DataType;
	typedef Container<T, typename std::allocator<T> > ContainerType;

	ThreadsafeQueue( size_t initSize )
	{
		_items.resize( initSize );
		_items.clear();
	}

	void PushFront( const T& item )
	{
		WriteLock lock( _mutex );
		_items.push_front( item );
		_hasContents.notify_one();
	}
	
	void PushBack( const T& item)
	{
		WriteLock lock( _mutex );
		_items.push_back( item );
		_hasContents.notify_one();
	}
	
	void WaitPopFront( T& item )
	{
		WriteLock lock( _mutex );
		while( _items.empty() )
		{
			_hasContents.wait( lock );
		}
		item = _items.front();
		_items.pop_front();
		
		if( _items.empty() )
		{
			_isEmpty.notify_all();
		}
	}
	
	void WaitPopBack( T& item )
	{
		WriteLock lock( _mutex );
		while( _items.empty() )
		{
			_hasContents.wait( lock );
		}
		item = _items.back();
		_items.pop_back();	
		
		if( _items.empty() )
		{
			_isEmpty.notify_all();
		}
	}
	
	bool TryPopFront( T& item )
	{
		WriteLock lock( _mutex );
		if( _items.empty() )
		{
			return false;
		}
		item = _items.front();
		_items.pop_front();
		
		if( _items.empty() )
		{
			_isEmpty.notify_all();
		}
		return true;
	}
	
	void TryPopBack( T& item )
	{
		WriteLock lock( _mutex );
		while( _items.empty() )
		{
			return false;
		}
		item = _items.back();
		_items.pop_back();	
		
		if( _items.empty() )
		{
			_isEmpty.notify_all();
		}
		return true;
	}
	
	size_t Size() const
	{
		WriteLock lock( _mutex );
		return _items.size();
	}
	
	bool IsEmpty() const
	{
		return Size() == 0;
	}
	
	void Clear()
	{
		WriteLock lock( _mutex );
		_items.clear();
		_isEmpty.notify_all();
	}

	/*! \brief Wait for this queue to be empty. */
	void WaitEmpty()
	{
		WriteLock lock( _mutex );
		while( !_items.empty() )
		{
			_isEmpty.wait( lock );
		}
		
	}

protected:

	mutable Mutex _mutex;

	ContainerType _items;
	ConditionVariable _hasContents;
	ConditionVariable _isEmpty;

};

}

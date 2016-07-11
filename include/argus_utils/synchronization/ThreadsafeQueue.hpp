#pragma once

#include <memory>
#include <deque>

#include "argus_utils/synchronization/SynchronizationTypes.h"

namespace argus
{

/*! \brief A mutex-wrapped container that supports size limiting. */
template <class T,
          template<typename,typename> class Container = std::deque >
class ThreadsafeQueue
{
public:

	typedef T DataType;
	typedef Container<T, typename std::allocator<T> > ContainerType;

	ThreadsafeQueue( size_t maxSize = std::numeric_limits<size_t>::max() )
	: _maxSize( maxSize ) {}

	void SetMaxSize( size_t maxSize )
	{
		_maxSize = maxSize;
	}

	template< class... Args>
	void EmplaceFront( Args&&... args )
	{
		WriteLock lock( _mutex );
		_items.emplace_front( args... );
		if( _maxSize > 0 && _items.size() > _maxSize ) { _items.pop_back(); }
		_hasContents.notify_one();
	}

	template< class... Args>
	void EmplaceBack( Args&&... args )
	{
		WriteLock lock( _mutex );
		_items.emplace_back( args... );
		if( _maxSize > 0 && _items.size() > _maxSize ) { _items.pop_front(); }
		_hasContents.notify_one();
	}

	void PushFront( const T& item )
	{
		WriteLock lock( _mutex );
		_items.push_front( item );
		if( _maxSize > 0 && _items.size() > _maxSize ) { _items.pop_back(); }
		_hasContents.notify_one();
	}
	
	void PushBack( const T& item)
	{
		WriteLock lock( _mutex );
		_items.push_back( item );
		if( _maxSize > 0 && _items.size() > _maxSize ) { _items.pop_front(); }
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

	void WaitHasContents()
	{
		WriteLock lock( _mutex );
		while( _items.empty() )
		{
			_hasContents.wait( lock );
		}
	}

protected:

	mutable Mutex _mutex;

	size_t _maxSize;
	ContainerType _items;
	ConditionVariable _hasContents;
	ConditionVariable _isEmpty;

};

}

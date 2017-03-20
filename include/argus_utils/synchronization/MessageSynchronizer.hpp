#pragma once

#include <boost/foreach.hpp>
#include <sstream>

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "argus_utils/utils/MapUtils.hpp"

namespace argus
{

template<typename Msg, typename Key = std::string>
class MessageSynchronizer
{
public:

	typedef std::tuple<Key, double, Msg> KeyedStampedData;

	MessageSynchronizer()
	{
		SetBufferLength( 10 );
		SetMaxDt( 0.1 );
		SetMinSyncNum( 0 );
	}

	void SetBufferLength( double buffLen )
	{
		if( buffLen < 0 )
		{
			throw std::invalid_argument( "Buffer length must be non-negative." );
		}
		_maxBufferLen = buffLen;
	}

	void SetMaxDt( double dt )
	{
		if( dt < 0 )
		{
			throw std::invalid_argument( "Max dt must be non-negative." );
		}
		_maxDt = dt;
	}

	void SetMinSyncNum( unsigned int num )
	{
		_minSyncNum = num;
	}


	void RegisterSource( const Key& key )
	{
		WriteLock lock( _registryMutex );
		CheckStatus( key, false, lock );
		_registry[key];
	}

	void BufferData( const Key& key, double stamp, const Msg& msg )
	{
		ReadLock lock( _registryMutex );
		CheckStatus( key, true, lock );
		WriteLock regLock( _registry.at( key ).mutex );
		_registry.at( key ).buffer.emplace( std::piecewise_construct,
		                                    std::forward_as_tuple( stamp ),
		                                    std::forward_as_tuple( msg ) );
	}

	bool GetOutput( double now, std::vector<KeyedStampedData>& out )
	{
		out.clear();
		WriteLock lock( _registryMutex );

		unsigned int minSync = (_minSyncNum == 0) ? _registry.size() : _minSyncNum;

		double earliest;
		while( FindEarliestOverspan( earliest, lock ) )
		{
			unsigned int numReady = FindDataAtTime( earliest, out, false, lock );
			if( numReady >= minSync )
			{
				FindDataAtTime( earliest, out, true, lock );
				return true;
			}
			RemoveBeforeInclusive( earliest, lock );
		}
		return false;
	}

private:

	struct SourceRegistration
	{
		mutable Mutex mutex;
		std::map<double, Msg> buffer;
	};

	typedef std::map<Key, SourceRegistration> SourceRegistry;
	mutable Mutex _registryMutex;     // Locks all access to the registry
	SourceRegistry _registry;

	// Parameters
	double _maxBufferLen;
	double _maxDt;
	unsigned int _minSyncNum;

	bool FindEarliestOverspan( double& earliest,
	                           WriteLock& lock ) const
	{
		CheckLockOwnership( lock, &_registryMutex );

		typedef typename SourceRegistry::value_type Item;
		BOOST_FOREACH( const Item & item, _registry )
		{
			const SourceRegistration& reg = item.second;
			ReadLock regLock( reg.mutex );
			// if( reg.buffer.size() > _maxBufferLen )
			// {
			// 	earliest = get_lowest_key( reg.buffer );
			// 	return true;
			// }
			earliest = get_lowest_key( reg.buffer );
			double latest = get_highest_key( reg.buffer );
			if( (latest - earliest) > _maxBufferLen ) { return true; }
		}
		return false;
	}

	unsigned int FindDataAtTime( double t,
	                             std::vector<KeyedStampedData>& out,
	                             bool retrieve,
	                             const WriteLock& lock )
	{
		CheckLockOwnership( lock, &_registryMutex );

		unsigned int count = 0;
		typename std::map<double, Msg>::const_iterator iter;
		typedef typename SourceRegistry::value_type Item;
		BOOST_FOREACH( Item & item, _registry )
		{
			const Key& key = item.first;
			SourceRegistration& reg = item.second;
			WriteLock regLock( reg.mutex );
			if( !get_closest_eq( reg.buffer, t, iter ) ) { continue; }
			double closest = iter->first;
			if( std::abs( closest - t ) > _maxDt ) { continue; }

			++count;
			if( retrieve )
			{
				out.push_back( KeyedStampedData( key, iter->first, iter->second ) );
				reg.buffer.erase( iter );
			}
		}
		return count;
	}

	/*! \brief Removes all stamped data with time before and including specified t. */
	void RemoveBeforeInclusive( double t, const WriteLock& lock )
	{
		CheckLockOwnership( lock, &_registryMutex );

		typedef typename SourceRegistry::value_type Item;
		BOOST_FOREACH( Item & item, _registry )
		{
			SourceRegistration& reg = item.second;
			while( !reg.buffer.empty() && get_lowest_key( reg.buffer ) <= t )
			{
				remove_lowest( reg.buffer );
			}
		}
	}


	template<typename Lock>
	void CheckStatus( const std::string& key, bool expect_reg,
	                  const Lock& lock )
	{
		CheckLockOwnership( lock, &_registryMutex );

		bool is_reg = _registry.count( key ) > 0;
		if( expect_reg != is_reg )
		{
			std::stringstream ss;
			ss << "Source: " << key << ( expect_reg ? " not registered!" : " already_registered" );
			throw std::invalid_argument( ss.str() );
		}
	}

};


}
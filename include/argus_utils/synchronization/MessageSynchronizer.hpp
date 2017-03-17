#pragma once

#include <deque>
#include <boost/foreach.hpp>
#include <Eigen/Dense>
#include <sstream>
#include <stdexcept>

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "argus_utils/utils/MapUtils.hpp"

namespace argus
{

/*! \brief Synchronizes buffers of timestamped data within some amount of tolerance.
 * NOTE: Assumes that data arrives in temporal order.
 * NOTE: Accesses to the output buffer are synchronized, but parameter setting and registration
 * is not
 */
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

	// NOTE Does not change buffer length of existing buffers!
	void SetBufferLength( unsigned int buffLen )
	{
		_bufferLen = buffLen;
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

	void BufferData( const Key& key,
	                 double t,
	                 const Msg& m )
	{
		ReadLock lock( _registryMutex );
		CheckStatus( key, true, lock );

		SourceRegistration& reg = _registry.at( key );
		WriteLock buffLock( reg.bufferMutex );

		if( !reg.buffer.empty() )
		{
			double lastTime = get_highest_key( reg.buffer );
			if( t < lastTime )
			{
				std::stringstream ss;
				ss << "Time " << t << " predecdes last time " << lastTime;
				throw std::invalid_argument( ss.str() );
			}
		}

		reg.buffer[t] = m;

		// Prune down to size
		while( reg.buffer.size() > _bufferLen )
		{
			remove_lowest( reg.buffer );
		}
	}

	bool GetOutput( std::vector<KeyedStampedData>& out )
	{
		out.clear();

		// Nobody else can interact with registry here
		WriteLock lock( _registryMutex );

		// Quick fail if we have nothing to check!
		if( _registry.size() == 0 ) { return false; }

		while( !AnyBuffersEmpty( lock ) )
		{
			double earliest = GetEarliestTime( lock );
			if( EnoughContain( earliest, lock ) )
			{
				RetrieveAndRemoveData( earliest, out, lock );
				// Don't remove b/c there may be more synchronized sets
				return true;
			}
			else
			{
				// Trim off all data up to and including, and try again
				RemoveBeforeInclusive( earliest, lock );
			}
		}
		return false;
	}

private:

	struct SourceRegistration
	{
		mutable Mutex bufferMutex;
		std::map<double, Msg> buffer;
	};

	typedef std::map<Key, SourceRegistration> SourceRegistry;
	mutable Mutex _registryMutex; // Locks all access to the registry
	SourceRegistry _registry;

	// Parameters
	unsigned int _bufferLen;
	double _maxDt;
	unsigned int _minSyncNum;

	bool AnyBuffersEmpty( const WriteLock& lock ) const
	{
		CheckLockOwnership( lock, &_registryMutex );

		typedef typename SourceRegistry::value_type Item;
		BOOST_FOREACH( const Item &item, _registry )
		{
			const SourceRegistration& reg = item.second;
			if( reg.buffer.empty() ) { return true; }
		}
		return false;
	}

	double GetEarliestTime( const WriteLock& lock ) const
	{
		CheckLockOwnership( lock, &_registryMutex );

		typedef typename SourceRegistry::value_type Item;
		double earliest = std::numeric_limits<double>::infinity();
		BOOST_FOREACH( const Item &item, _registry )
		{
			const SourceRegistration& reg = item.second;
			double buffEarliest = get_lowest_key( reg.buffer );
			if( buffEarliest < earliest )
			{
				earliest = buffEarliest;
			}
		}
		return earliest;
	}

	bool EnoughContain( double t, const WriteLock& lock )
	{
		CheckLockOwnership( lock, &_registryMutex );

		typename std::map<double, Msg>::const_iterator iter;
		typedef typename SourceRegistry::value_type Item;
		unsigned int count = 0;
		BOOST_FOREACH( const Item &item, _registry )
		{
			const SourceRegistration& reg = item.second;
			if( !get_closest_eq( reg.buffer, t, iter ) ) { continue; }
			double closest = iter->first;
			if( std::abs( closest - t ) <= _maxDt ) { ++count; }
		}

		if( _minSyncNum == 0 ) { return count == _registry.size(); }
		else { return count >= _minSyncNum; }
	}

	void RetrieveAndRemoveData( double t,
	                            std::vector<KeyedStampedData>& out,
	                            const WriteLock& lock )
	{
		CheckLockOwnership( lock, &_registryMutex );

		typename std::map<double, Msg>::const_iterator iter;
		typedef typename SourceRegistry::value_type Item;
		BOOST_FOREACH( Item & item, _registry )
		{
			const Key& key = item.first;
			SourceRegistration& reg = item.second;
			if( !get_closest_eq( reg.buffer, t, iter ) ) { continue; }
			double closest = iter->first;
			if( std::abs( closest - t ) > _maxDt ) { continue; }

			out.push_back( KeyedStampedData( key, iter->first, iter->second ) );
			reg.buffer.erase( iter );
		}
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
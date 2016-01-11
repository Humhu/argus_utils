#pragma once

#include <vector>
#include <map>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

namespace argus_utils 
{

/*! \brief Find the closest item in a map whose key is lower than the specified.
 * Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_lower( std::map<Index, Content>& m, const Index& ind,
                        typename std::map<Index, Content>::iterator& iter )
{
	// Returns first key not before ind, so only failure when it is the first item
	iter = m.lower_bound( ind );
	if( iter == m.begin() ) { return false; }
	iter--;
	return true;
}

/*! \brief Find the closest item in a map whose key is higher than the specified.
 * Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_upper( std::map<Index, Content>& m, const Index& ind,
                        typename std::map<Index, Content>::iterator& iter )
{
	// Returns first after ind, so only failure when it is non-item
	iter = m.upper_bound( ind );
	if( iter == m.end() ) { return false; }
	return true;
}

template <typename Index, typename Content>
Index get_highest_key( const std::map<Index, Content>& m )
{
	typename std::map<Index, Content>::const_iterator iter = m.end();
	iter--;
	return iter->first;
}

template <typename Index, typename Content>
Index get_lowest_key( const std::map<Index, Content>& m )
{
	return m.begin()->first;
}

template <typename Key>
std::set<Key> make_set( const std::vector<Key>& vec )
{
	std::set<Key> span;
	BOOST_FOREACH( const Key& k, vec )
	{
		span.insert( k );
	}
	return span;
}

} // end namespace argus_utils

#pragma once

#include <vector>
#include <map>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/foreach.hpp>

namespace argus 
{

/*! \brief Find the closest item in a map whose key is higher or equal than 
 * the specified. Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_greater_eq( std::map<Index,Content>& m, const Index& ind,
                             typename std::map<Index,Content>::iterator& iter )
{
	iter = m.lower_bound( ind );
	if( iter == m.end() ) { return false; }
	return true;
}
template <typename Index, typename Content>
bool get_closest_greater_eq( const std::map<Index,Content>& m, const Index& ind,
                             typename std::map<Index,Content>::const_iterator& iter )
{
	iter = m.lower_bound( ind );
	if( iter == m.end() ) { return false; }
	return true;
}

/*! \brief Find the closest item in a map whose key is strictly higher than 
 * the specified. Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_greater( std::map<Index,Content>& m, const Index& ind,
                          typename std::map<Index,Content>::iterator& iter )
{
	iter = m.upper_bound( ind );
	if( iter == m.end() ) { return false; }
	return true;
}
template <typename Index, typename Content>
bool get_closest_greater( const std::map<Index,Content>& m, const Index& ind,
                          typename std::map<Index,Content>::const_iterator& iter )
{
	iter = m.upper_bound( ind );
	if( iter == m.end() ) { return false; }
	return true;
}

/*! \brief Find the closest item in a map whose key is lower than or equal
 * to the specified. Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_lesser_eq( std::map<Index,Content>& m, const Index& ind,
                            typename std::map<Index,Content>::iterator& iter )
{
	if( m.empty() ) { return false; }
	iter = m.lower_bound( ind );
	if( iter == m.end() ) 
	{ 
		--iter;
		return true;
	}
	if( iter->first == ind ) { return true; }
	if( iter == m.begin() ) { return false; }
	--iter;
	return true;
}
template <typename Index, typename Content>
bool get_closest_lesser_eq( const std::map<Index,Content>& m, const Index& ind,
                            typename std::map<Index,Content>::const_iterator& iter )
{
	if( m.empty() ) { return false; }
	iter = m.lower_bound( ind );
	if( iter == m.end() ) 
	{ 
		--iter;
		return true;
	}
	if( iter->first == ind ) { return true; }
	if( iter == m.begin() ) { return false; }
	--iter;
	return true;
}

/*! \brief Find the closest item in a map whose key is strictly lower than 
 * to the specified. Returns true if an item is found, false if not. */
template <typename Index, typename Content>
bool get_closest_lesser( std::map<Index,Content>& m, const Index& ind,
                         typename std::map<Index,Content>::iterator& iter )
{
	if( m.empty() ) { return false; }
	iter = m.lower_bound( ind );
	if( iter == m.end() ) 
	{ 
		--iter;
		return true;
	}
	if( iter->first == ind ) { return false; }
	if( iter == m.begin() ) { return false; }
	--iter;
	return true;
}
template <typename Index, typename Content>
bool get_closest_lesser( const std::map<Index,Content>& m, const Index& ind,
                         typename std::map<Index,Content>::const_iterator& iter )
{
	if( m.empty() ) { return false; }
	iter = m.lower_bound( ind );
	if( iter == m.end() ) 
	{ 
		--iter;
		return true;
	}
	if( iter->first == ind ) { return false; }
	if( iter == m.begin() ) { return false; }
	--iter;
	return true;
}

template <typename Index, typename Content>
bool get_closest( std::map<Index,Content>& m, const Index& ind,
                  typename std::map<Index,Content>::iterator& iter )
{
	typename std::map<Index,Content>::iterator lower, upper;
	bool hasLower = get_closest_lesser( m, ind, lower );
	bool hasUpper = get_closest_greater( m, ind, upper );
	if( hasLower && !hasUpper ) { iter = lower; }
	else if( !hasLower && hasUpper ) { iter = upper; }
	else if( !hasLower && !hasUpper ) { return false; }
	else { iter = ( (upper->first - ind) > (ind - lower->first) ) ? lower : upper; }
	return true;
}
template <typename Index, typename Content>
bool get_closest( const std::map<Index,Content>& m, const Index& ind,
                  typename std::map<Index,Content>::const_iterator& iter )
{
	typename std::map<Index,Content>::const_iterator lower, upper;
	bool hasLower = get_closest_lesser( m, ind, lower );
	bool hasUpper = get_closest_greater( m, ind, upper );
	if( hasLower && !hasUpper ) { iter = lower; }
	else if( !hasLower && hasUpper ) { iter = upper; }
	else if( !hasLower && !hasUpper ) { return false; }
	else { iter = ( (upper->first - ind) > (ind - lower->first) ) ? lower : upper; }
	return true;
}

template <typename Index, typename Content>
bool get_closest_eq( std::map<Index,Content>& m, const Index& ind,
                     typename std::map<Index,Content>::iterator& iter )
{
	typename std::map<Index,Content>::iterator lower, upper;
	bool hasLower = get_closest_lesser_eq( m, ind, lower );
	bool hasUpper = get_closest_greater_eq( m, ind, upper );
	if( hasLower && !hasUpper ) { iter = lower; }
	else if( !hasLower && hasUpper ) { iter = upper; }
	else if( !hasLower && !hasUpper ) { return false; }
	else { iter = ( (upper->first - ind) > (ind - lower->first) ) ? lower : upper; }
	return true;
}
template <typename Index, typename Content>
bool get_closest_eq( const std::map<Index,Content>& m, const Index& ind,
                     typename std::map<Index,Content>::const_iterator& iter )
{
	typename std::map<Index,Content>::const_iterator lower, upper;
	bool hasLower = get_closest_lesser_eq( m, ind, lower );
	bool hasUpper = get_closest_greater_eq( m, ind, upper );
	if( hasLower && !hasUpper ) { iter = lower; }
	else if( !hasLower && hasUpper ) { iter = upper; }
	else if( !hasLower && !hasUpper ) { return false; }
	else { iter = ( (upper->first - ind) > (ind - lower->first) ) ? lower : upper; }
	return true;
}

template <typename Index, typename Content>
Index get_highest_key( const std::map<Index,Content>& m )
{
	typename std::map<Index,Content>::const_iterator iter = m.end();
	iter--;
	return iter->first;
}

template <typename Index, typename Content>
Index get_lowest_key( const std::map<Index,Content>& m )
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

} // end namespace argus

#pragma once

#include "argus_utils/utils/ParsersCommon.h"

namespace argus
{

// Vectors of signed ints
template <typename S, typename T>
typename std::enable_if< std::is_integral<T>::value &&
                         std::is_signed<T>::value, bool>::type
GetParam( const S& src, 
          const std::string& name, 
          std::vector<T>& t )
{
	std::vector<int> temp;
	if( !FieldRetrieval<S>::get( src, name, temp ) ) { return false; }

	t.clear();
	t.reserve( temp.size() );
	for( unsigned int i = 0; i < temp.size(); i++ )
	{
		t.emplace_back( t[i] );
	}
	return true;
}

// Vectors of unsigned ints
template <typename S, typename T>
typename std::enable_if< std::is_integral<T>::value &&
                         !std::is_signed<T>::value, bool>::type
GetParam( const S& src, 
          const std::string& name, 
          std::vector<T>& t )
{
	std::vector<typename std::make_signed<T>::type> temp;
	if( !FieldRetrieval<S>::get( src, name, temp ) ) { return false; }

	t.clear();
	t.reserve( temp.size() );
	for( unsigned int i = 0; i < temp.size(); i++ )
	{
		if( temp[i] < 0 )
		{
			throw std::runtime_error( "Attempted to parse value " + 
			                          std::to_string(temp[i]) +
			                          " as unsigned." );
		}
		t.emplace_back( t[i] );
	}
	return true;
}

// Containers of floats
template <typename S, typename T>
typename std::enable_if< std::is_floating_point<T>::value, bool>::type
GetParam( const S& src, 
          const std::string& name, 
          std::vector<T>& t )
{
	if( FieldRetrieval<S>::get( src, name, t ) ) { return true; }

	std::vector<std::string> temp;
	if( !FieldRetrieval<S>::get( src, name, temp ) ) { return false; }

	t.clear();
	t.reserve( temp.size() );
	for( unsigned int i = 0; i < temp.size(); i++ )
	{
		try
		{
			t.emplace_back( std::stod( temp[i] ) );
		}
		catch( std::exception e )
		{
			return false;
		}
	}
	return true;
}

// Containers of everything else
template <typename S, typename T>
typename std::enable_if< !std::is_arithmetic<T>::value, bool>::type
GetParam( const S& src, 
          const std::string& name, 
          std::vector<T>& t )
{
	return FieldRetrieval<S>::get( src, name, t );
}

}
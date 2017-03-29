#pragma once

#include "argus_utils/utils/ParsersCommon.h"

namespace argus
{

// For all unsigned ints
template <typename T, typename S>
typename std::enable_if< std::is_unsigned<T>::value && 
                         !std::is_same<T,bool>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	typename std::make_signed<T>::type tSigned;
	// if( !nh.getParam( name, tSigned ) ) 
	if( !FieldRetrieval<S>::get( src, name, tSigned ) )
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false;
	}
	// Make sure we're above 0
	if( tSigned < 0 )
	{
		throw std::runtime_error( "Attempted to parse value " +
		                          std::to_string(tSigned) +
		                          " as unsigned." );
	}
	t = (T) tSigned;
	return true;
}

// For all floats
template <typename T, typename S>
typename std::enable_if< std::is_floating_point<T>::value, bool >::type
GetParam( const S& src, const std::string& name, T& t )
{
	// First see if normal retrieval works
	// if( !nh.getParam( name, t ) )
	if( !FieldRetrieval<S>::get( src, name, t ) )
	{
		// If not, see if it's a string that we can convert
		std::string valS;
		if( !FieldRetrieval<S>::get( src, name, valS ) ) { return false; }
		try
		{
			t = std::stod( valS );
		}
		catch( std::exception e ) 
		{
			return false;
		}
	}
	return true;
}

// For strings
template <typename T, typename S>
typename std::enable_if< std::is_same<std::string,T>::value,
                         bool>::type
GetParam( const S& src, const std::string& name, T& t )
{
	return FieldRetrieval<S>::get( src, name, t );
}

// Need full specializations for YAML::Node since implementation differs
// between ROS and YAML sources
bool GetParam( const ros::NodeHandle& nh, const std::string& name,
               XmlRpc::XmlRpcValue& t );
bool GetParam( const ros::NodeHandle& nh, const std::string& name,
               YAML::Node& t );
bool GetParam( const YAML::Node& node, const std::string& name,
               YAML::Node& t );

}
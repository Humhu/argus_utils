#pragma once

#include <ros/ros.h>
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/geometry/GeometryUtils.h"

#include "argus_utils/utils/CoreParsers.h"
#include "argus_utils/utils/ArrayParsers.h"
#include "argus_utils/utils/MatrixParsers.h"
#include "argus_utils/utils/GeometryParsers.h"

namespace argus
{

template <typename S>
bool HasParam( const S& src, const std::string& name )
{
	return FieldRetrieval<S>::has( src, name );
}

/*! \brief Retrieve a parameter from the ROS parameter server. Print an error
 * if the parameter retrieval fails. Has a specialization for unsigned ints
 * that checks to make sure the int value is > 0. */
template <typename T, typename S>
void GetParamRequired( const S& src, const std::string& name, T& t )
{
	if( !GetParam( src, name, t ) )
	{ 
		std::string base = FieldRetrieval<S>::qualify( src );
		throw std::runtime_error( "Could not retrieve required parameter: " + base + "/" + name );
	}
}

/*! \brief Retreive a parameter from the ROS parameter server. Returns the
 * specified default if the parameter retrieval fails. Has a specialization
 * for unsigned ints that checks to make sure the int value is > 0. */
template <typename T, typename S>
void GetParam( const S& src, const std::string& name, T& t, 
               const T& def )
{
	if( !GetParam( src, name, t ) ) 
	{ 
		std::string base = FieldRetrieval<S>::qualify( src );
		ROS_WARN_STREAM( "Parameter: " << base << "/" << name << " will use default: " << def );
		t = def;
	}
}

/*! \brief Get/set a parameter YAML object out of a node handle by combining calls
 * to get XmlRpc and convert it. Returns success. */
void SetYamlParam( const ros::NodeHandle& nh, const std::string& name, 
                   const YAML::Node& node );

}



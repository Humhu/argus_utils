#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <argus_utils/utils/YamlUtils.h>

namespace argus
{


/*! \brief Retrieve a parameter from the ROS parameter server. Print an error
 * if the parameter retrieval fails. Has a specialization for unsigned ints
 * that checks to make sure the int value is > 0. */
template <typename T>
bool GetParam( ros::NodeHandle& nh, const std::string& name, T& t )
{
	if( !nh.getParam( name, t ) ) 
	{ 
		ROS_ERROR_STREAM( "Could not retrieve parameter: " << name );
		return false; 
	}
	return true;
}

template <>
bool GetParam<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                             unsigned int& t );

/*! \brief Retreive a parameter from the ROS parameter server. Returns the
 * specified default if the parameter retrieval fails. Has a specialization
 * for unsigned ints that checks to make sure the int value is > 0. */
template <typename T>
void GetParamDefault( ros::NodeHandle& nh, const std::string& name, T& t, 
                      const T& def )
{
	if( !nh.getParam( name, t ) ) 
	{ 
		ROS_ERROR_STREAM( "Could not retrieve parameter: " << name
		                  << ". Using default value: " << def );
		t = def;
	}
}

template <>
void GetParamDefault<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                                    unsigned int& t, const unsigned int& def );

/*! \brief Get/set a parameter YAML object out of a node handle by combining calls
 * to get XmlRpc and convert it. Returns success. */
bool GetYamlParam( ros::NodeHandle& nh, const std::string& name, 
                   YAML::Node& node );

void SetYamlParam( ros::NodeHandle& nh, const std::string& name, 
                   const YAML::Node& node );

template <typename Scalar, typename Derived>
bool GetMatrixParam( ros::NodeHandle& nh, const std::string& name, 
                     Eigen::DenseBase<Derived>& mat )
{
	std::vector<Scalar> values;
	if( !GetParam( nh, name, values ) );
	if( !ParseMatrix( values, mat ) )
	{
		ROS_ERROR_STREAM( "Could not parse values from " << name
		                  << " into " << mat.rows() << " by " << mat.cols()
		                  << " matrix." );
		return false;
	}
	return true;
}
	
}



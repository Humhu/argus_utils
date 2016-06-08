#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <argus_utils/utils/YamlUtils.h>

namespace argus
{


template <typename T>
bool GetParam( ros::NodeHandle& nh, const std::string& name, T& t )
{
	if( !nh.getParam( name, t ) ) 
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false;
	}
	return true;
}

template <>
bool GetParam<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                             unsigned int& t );

template <>
bool GetParam<double>( ros::NodeHandle& nh, const std::string& name, 
                       double& t );

/*! \brief Retrieve a parameter from the ROS parameter server. Print an error
 * if the parameter retrieval fails. Has a specialization for unsigned ints
 * that checks to make sure the int value is > 0. */
template <typename T>
void GetParamRequired( ros::NodeHandle& nh, const std::string& name, T& t )
{
	if( !GetParam( nh, name, t ) ) 
	{ 
		throw std::runtime_error( "Could not retrieve required parameter: " + name );
	}
}

/*! \brief Retreive a parameter from the ROS parameter server. Returns the
 * specified default if the parameter retrieval fails. Has a specialization
 * for unsigned ints that checks to make sure the int value is > 0. */
template <typename T>
void GetParam( ros::NodeHandle& nh, const std::string& name, T& t, 
               const T& def )
{
	if( !GetParam( nh, name, t ) ) 
	{ 
		ROS_WARN_STREAM( "Parameter: " << name << " will use default: " << def );
		t = def;
	}
}

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
	if( !GetParam( nh, name, values ) ) { return false; }
	if( !ParseMatrix( values, mat ) )
	{
		ROS_WARN_STREAM( "Could not parse values from " << name
		                 << " into " << mat.rows() << " by " << mat.cols()
		                 << " matrix." );
		return false;
	}
	return true;
}

template <typename Scalar, typename Derived>
bool GetDiagonalParam( ros::NodeHandle& nh, const std::string& name, 
                       Eigen::DenseBase<Derived>& mat )
{
	std::vector<Scalar> values;
	if( !GetParam( nh, name, values ) ) { return false; }
	unsigned int minDim = std::min( mat.rows(), mat.cols() );
	if( values.size() != minDim ) 
	{ 
	  ROS_WARN_STREAM( "Could not parse values from " << name
			   << " into " << minDim << " diagonal matrix." );
	  return false;
	}
	
	mat.setZero();
	for( unsigned int ind = 0; ind < minDim; ++ind )
	{
		mat(ind,ind) = values[ind];
	}
	return true;
}

}



#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

namespace argus_utils
{

template <typename T>
bool GetParam( ros::NodeHandle& nh, const std::string& name, T& t );

template <typename T>
void GetParamDefault( ros::NodeHandle& nh, const std::string& name, T& t, const T& def )
{
	if( !GetParam<T>( nh, name, t ) ) { t = def; }
}

/*! \brief Get/set a parameter YAML object out of a node handle by combining calls
 * to get XmlRpc and convert it. Returns success. */
bool GetYamlParam( ros::NodeHandle& nh, const std::string& name, YAML::Node& node );
void SetYamlParam( ros::NodeHandle& nh, const std::string& name, const YAML::Node& node );
	
}

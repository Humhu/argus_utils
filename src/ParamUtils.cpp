#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

template <>
void GetParam<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                             unsigned int& t )
{
	int val;
	if( !nh.getParam( name, val ) ) 
	{ 
		ROS_ERROR_STREAM( "Could not retrieve parameter: " << name );
		throw std::runtime_error( "Could not retrieve parameter: " + name );
	}
	if( val < 0 )
	{
		ROS_ERROR_STREAM( "Attempted to parse value " << val << " as unsigned int." );
		throw std::runtime_error( "Attempted to parse negative value as unsigned int." );
	}
	t = static_cast<unsigned int>( val );
}

template <>
void GetParamDefault<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                                    unsigned int& t, const unsigned int& def )
{
	int val;
	if( !nh.getParam( name, val ) ) 
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name
		                 << ". Using default value: " << def );
		t = def;
		return;
	}
	if( val < 0 )
	{
		ROS_ERROR_STREAM( "Attempted to parse value " << val << " as unsigned int." );
		t = def;
		return;
	}
	t = static_cast<unsigned int>( val );
}

bool GetYamlParam( ros::NodeHandle& nh, const std::string& name, 
                          YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml;
	if( !nh.getParam( name, xml ) ) { return false; }
	node = XmlToYaml( xml );
	return true;
}

void SetYamlParam( ros::NodeHandle& nh, const std::string& name, 
                          const YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml = YamlToXml( node );
	nh.setParam( name, xml );
}

}
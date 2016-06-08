#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

template <>
bool GetParam<unsigned int>( ros::NodeHandle& nh, const std::string& name, 
                             unsigned int& t )
{
	int val;
	if( !GetParam<int>( nh, name, val ) ) 
	{ 
		ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
		return false; 
	}

	if( val < 0 )
	{
		ROS_WARN_STREAM( "Attempted to parse value " << val << " as unsigned int." );
		return false;
	}
	t = static_cast<unsigned int>( val );
	return true;
}

template <>
bool GetParam<double>( ros::NodeHandle& nh, const std::string& name,
                       double& t )
{
	// First see if normal double retrieval works
	if( !nh.getParam( name, t ) )
	{
		// If not, see if it's a string that we can convert
		std::string valS;
		if( !GetParam<std::string>( nh, name, valS ) )
		{
			ROS_WARN_STREAM( "Could not retrieve parameter: " << name );
			return false;
		}
		try
		{
			t = std::stod( valS );
		}
		catch( std::exception e ) 
		{
			ROS_WARN_STREAM( "Parameter " << valS << " could not be interpreted as a double." );
			return false;
		}
	}
	return true;
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
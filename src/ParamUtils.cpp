#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"

namespace argus_utils
{

template <>
bool GetParam<unsigned int>( ros::NodeHandle& nh, const std::string& name, unsigned int& t )
{
	int val;
	if( !nh.getParam( name, val ) ) { return false; }
	if( val < 0 )
	{
		ROS_WARN_STREAM( "Attempted to parse value " << val << " as unsigned int." );
		return false;
	}
	t = static_cast<unsigned int>( val );
	return true;
}

bool GetYamlParam( ros::NodeHandle& nh, const std::string& name, YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml;
	if( !nh.getParam( name, xml ) ) { return false; }
	node = XmlToYaml( xml );
	return true;
}

void SetYamlParam( ros::NodeHandle& nh, const std::string& name, const YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml = YamlToXml( node );
	nh.setParam( name, xml );
}

}

#include "argus_utils/utils/ParsersCommon.h"
#include "argus_utils/utils/YamlUtils.h"

namespace argus
{

bool GetParam( const ros::NodeHandle& nh, 
               const std::string& name,
               XmlRpc::XmlRpcValue& t )
{
	if( !nh.getParam( name, t ) ) { return false; }
	return true;
}

bool GetParam( const ros::NodeHandle& nh, 
               const std::string& name,
               YAML::Node& t )
{
	XmlRpc::XmlRpcValue xml;
	if( !GetParam( nh, name, xml ) ) { return false; }
	t = XmlToYaml( xml );
	return true;
}

bool GetParam( const YAML::Node& node,
               const std::string& name,
               YAML::Node& t )
{
	if( !node[name] ) { return false; }
	t = node[name];
	return true;
}

}
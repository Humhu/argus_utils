#include "argus_utils/ParamUtils.h"
#include "argus_utils/YamlUtils.h"

namespace argus_utils
{

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

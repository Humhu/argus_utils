#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

void SetYamlParam( const ros::NodeHandle& nh, 
                   const std::string& name, 
                   const YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml = YamlToXml( node );
	nh.setParam( name, xml );
}

}
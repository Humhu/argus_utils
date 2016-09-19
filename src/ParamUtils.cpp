#include "argus_utils/utils/ParamUtils.h"

namespace argus
{

template <>
bool GetParam<std::vector<unsigned int>, ros::NodeHandle>( const ros::NodeHandle& src, 
                                                           const std::string& name, 
                                                           std::vector<unsigned int>& t )
{
	std::vector<int> vals;
	if( !FieldRetrieval<ros::NodeHandle>::get( src, name, vals ) ) { return false; }

	t.clear();
	t.reserve( vals.size() );
	for( unsigned int i = 0; i < vals.size(); i++ )
	{
		if( vals[i] < 0 )
		{
			ROS_WARN_STREAM( "Attempted to parse value " << vals[i] << " as unsigned int." );
			return false;
		}
		t.push_back( static_cast<unsigned int>( vals[i] ) );
	}
	return true;
}

template <>
bool GetParam<YAML::Node,ros::NodeHandle>( const ros::NodeHandle& nh, 
                                           const std::string& name,
                                           YAML::Node& t )
{
	XmlRpc::XmlRpcValue xml;
	if( !GetParam<XmlRpc::XmlRpcValue>( nh, name, xml ) ) { return false; }
	t = XmlToYaml( xml );
	return true;
}

template <>
bool GetParam<YAML::Node,YAML::Node>( const YAML::Node& node,
                                      const std::string& name,
                                      YAML::Node& t )
{
	if( !node[name] ) { return false; }
	t = node[name];
	return true;
}

void SetYamlParam( const ros::NodeHandle& nh, 
                   const std::string& name, 
                   const YAML::Node& node )
{
	XmlRpc::XmlRpcValue xml = YamlToXml( node );
	nh.setParam( name, xml );
}

}
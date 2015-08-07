#include <ros/ros.h>
#include "argus_utils/YamlUtils.h"
#include <yaml-cpp/yaml.h>

using namespace argus_utils;

int main( int argc, char** argv )
{

	ros::init( argc, argv, "yaml_test" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	nh.setParam( "a/b/c", 5 );
	nh.setParam( "a/b/z", "hello" );
	
	XmlRpc::XmlRpcValue xml;
	nh.getParam( "a", xml );
	std::cout << "Got xml from param server: " << std::endl;
	xml.write( std::cout );
	
	YAML::Node yaml = XmlToYaml( xml );
	std::cout << "Converted to yaml: " << std::endl;
	std::cout << yaml << std::endl;
	
	xml = YamlToXml( yaml );
	std::cout << "Converted back to xml: " << std::endl;
	xml.write( std::cout );
	
	std::cout << std::endl;
	
}

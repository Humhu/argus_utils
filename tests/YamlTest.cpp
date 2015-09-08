#include <ros/ros.h>
#include "argus_utils/YamlUtils.h"
#include <yaml-cpp/yaml.h>

using namespace argus_utils;

int main( int argc, char** argv )
{

	ros::init( argc, argv, "yaml_test" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	std::vector<bool> boolArray = {false, true, false};
	std::vector<double> doubleArray = {5, 4, 3.14};
	std::vector<int> intArray = {-1, 0, 2};
	std::vector< std::string > stringArray = { "hello", "world" };
	
	nh.setParam( "test/arrays/bools", boolArray );
	nh.setParam( "test/arrays/doubles", doubleArray );
	nh.setParam( "test/arrays/ints", intArray );
	nh.setParam( "test/arrays/strings", stringArray );
	
	nh.setParam( "test/singles/bool", true );
	nh.setParam( "test/singles/double", 2.714 );
	nh.setParam( "test/singles/ints", -254 );
	nh.setParam( "test/singles/string", "string" );
	
	XmlRpc::XmlRpcValue xml;
	nh.getParam( "test", xml );
	std::cout << "Got xml from param server: " << std::endl;
	xml.write( std::cout );
	std::cout << std::endl;
	
	YAML::Node yaml = XmlToYaml( xml );
	std::cout << "Converted to yaml: " << std::endl;
	std::cout << yaml << std::endl;
	
	xml = YamlToXml( yaml );
	std::cout << "Converted back to xml: " << std::endl;
	xml.write( std::cout );
	
	std::cout << std::endl;
	
}

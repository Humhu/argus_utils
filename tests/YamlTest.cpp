#include <ros/ros.h>
#include "argus_utils/utils/YamlUtils.h"
#include <yaml-cpp/yaml.h>

using namespace argus;

void ConversionTest()
{
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

int main( int argc, char** argv )
{

	ros::init( argc, argv, "yaml_test" );
	
	ConversionTest();
	
	YAML::Node a, b, temp;
	a["1"] = "a";
	temp["1"] = "b";
	a["2"] = temp;
	
	b["3"] = "c";
	
	std::cout << "YAML a:" << std::endl << a << std::endl;
	std::cout << "YAML b:" << std::endl << b << std::endl;
	
	std::cout << "Merging a and b..." << std::endl;
	YAML::Node merged = MergeYaml( a, b );
	
	std::cout << "YAML a:" << std::endl << a << std::endl;
	std::cout << "YAML b:" << std::endl << b << std::endl;
	
	std::cout << "merged:" << std::endl << merged << std::endl;
}

#include <ros/ros.h>

#include "extrinsics_array/ExtrinsicsInterface.h"

#include <argus_utils/geometry/GeometryUtils.h>
#include <argus_utils/utils/ParamUtils.h>
#include <unordered_map>

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "extrinsics_publisher_node" );
	ros::NodeHandle nh, ph( "~" );
	
	ExtrinsicsInterface interface( nh );

	YAML::Node transforms;
	GetParamRequired( ph, "", transforms );
	YAML::Node::const_iterator iter;
	for( iter = transforms.begin(); iter != transforms.end(); ++iter )
	{
		// TODO For some reason this segfaults if it's a const &
		YAML::Node info = iter->second;
		std::string child = iter->first.as<std::string>();
		std::string parent;
		PoseSE3 pose;
		GetParamRequired( info, "parent_id", parent );
		GetParamRequired( info, "pose", pose );
		
		ROS_INFO_STREAM( "Publishing extrinsics for: " << child << 
		                 " relative to " << parent <<
		                 " of " << pose );
		interface.SetStaticExtrinsics( child, parent, pose );
	}

	ros::spin();

	return 0;
}
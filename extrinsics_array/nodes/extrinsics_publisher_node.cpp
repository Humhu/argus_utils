#include <ros/ros.h>

#include "extrinsics_array/ExtrinsicsInterface.h"
#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"

#include <argus_utils/geometry/GeometryUtils.h>
#include <argus_utils/utils/ParamUtils.h>
#include <unordered_map>
#include <boost/foreach.hpp>

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "extrinsics_publisher_node" );
	ros::NodeHandle nh, ph( "~" );
	
	ExtrinsicsInterface interface( nh );

	std::vector<RelativePose> poses;
	YAML::Node transforms;
	GetParamRequired( ph, "", transforms );
	if( !ParseExtrinsicsCalibration( transforms, poses ) )
	{
		ROS_ERROR_STREAM( "Could not parse extrinsics!" );
		return -1;
	}

	BOOST_FOREACH( const RelativePose& pose, poses )
	{
		ROS_INFO_STREAM( "Publishing extrinsics for: " << pose.childID << 
		                 " relative to " << pose.parentID <<
		                 " of " << pose.pose );
		interface.SetStaticExtrinsics( pose );
	}
	ros::spin();

	return 0;
}

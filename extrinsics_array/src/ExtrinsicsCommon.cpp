#include "extrinsics_array/ExtrinsicsCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{

ExtrinsicsException::ExtrinsicsException( const std::string& msg )
	: _msg( msg )
{
}

const char*ExtrinsicsException::what() const throw( )
{
	return ( "ExtrinsicsException: " + _msg ).c_str();
}

RelativePose::RelativePose() 
{
}

RelativePose::RelativePose( const std::string& parent, const std::string& child,
                            const PoseSE3& p, const ros::Time& t )
{
	time = t;
	parentID = parent;
	childID = child;
	pose = p;
}

RelativePose::RelativePose( const geometry_msgs::TransformStamped& msg )
{
	time = msg.header.stamp;
	parentID = msg.header.frame_id;
	childID = msg.child_frame_id;
	pose = TransformToPose( msg.transform );
}

geometry_msgs::TransformStamped RelativePose::ToTransformMsg() const
{
	geometry_msgs::TransformStamped msg;
	msg.header.stamp = time;
	msg.header.frame_id = parentID;
	msg.child_frame_id = childID;
	msg.transform = PoseToTransform( pose );
	return msg;
}

}
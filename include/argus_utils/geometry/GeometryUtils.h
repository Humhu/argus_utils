#pragma once

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <tf/LinearMath/Transform.h> // From tf

#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/ArgusTypes.h"

#include <Eigen/Geometry>
#include <iostream>

namespace argus
{

// Convert between Eigen quaternions and yaw-pitch-roll Euler angles
EulerAngles QuaternionToEuler( const QuaternionType& quat );
QuaternionType EulerToQuaternion( const EulerAngles& eul );

// Convert between PoseSE3 and tf Transform objects
tf::Transform PoseToTf( const PoseSE3& pose );
PoseSE3 TfToPose( const tf::Transform& tf );

// Convert between geometry_msgs::Quaternion message and QuaternionType
QuaternionType MsgToQuaternion( const geometry_msgs::Quaternion& msg );
geometry_msgs::Quaternion QuaternionToMsg( const QuaternionType& q );

// Convert between geometry_msgs::Pose message and PoseSE3 objects
geometry_msgs::Pose PoseToMsg( const PoseSE2& pose );
geometry_msgs::Pose PoseToMsg( const PoseSE3& pose );
PoseSE3 MsgToPose( const geometry_msgs::Pose& msg );

// Convert between geometry_msgs::Twist message and PoseSE3::TangentVector
geometry_msgs::Twist TangentToMsg( const PoseSE2::TangentVector& tan );
geometry_msgs::Twist TangentToMsg( const PoseSE3::TangentVector& tan );
PoseSE3::TangentVector MsgToTangent( const geometry_msgs::Twist& twist );

// Convert between geometry_msgs::Vector3 message and FixedVectorType<3>
template <typename Derived>
geometry_msgs::Vector3 Vector3ToMsg( const Eigen::DenseBase<Derived>& vec )
{
	if( vec.size() != 3 ) { throw std::invalid_argument( "Vector must have 3 elements." ); }
	geometry_msgs::Vector3 msg;
	msg.x = vec(0);
	msg.y = vec(1);
	msg.z = vec(2);
	return msg;
}

FixedVectorType<3> MsgToVector3( const geometry_msgs::Vector3& msg );


// Print an Euler angle in "Y: yaw P: pitch R: roll" format
std::ostream& operator<<( std::ostream& os, const EulerAngles& eul );
	
} // end namespace argus

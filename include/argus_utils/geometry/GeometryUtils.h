#pragma once

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf/LinearMath/Transform.h> // From tf

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
geometry_msgs::Pose PoseToMsg( const PoseSE3& pose );
PoseSE3 MsgToPose( const geometry_msgs::Pose& msg );

// Convert between geometry_msgs::Twist message and PoseSE3::TangentVector
geometry_msgs::Twist TangentToMsg( const PoseSE3::TangentVector& tan );
PoseSE3::TangentVector MsgToTangent( const geometry_msgs::Twist& twist );

// Print an Euler angle in "Y: yaw P: pitch R: roll" format
std::ostream& operator<<( std::ostream& os, const EulerAngles& eul );
	
} // end namespace argus
